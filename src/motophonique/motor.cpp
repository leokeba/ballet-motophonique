#include "motophonique/motor.h"

#include <math.h>

namespace motophonique {

using tessermotion::Guard;

void Motor::setSpeed(float rpm) {
    Guard g(lock());
    speed_ = rpm;
    if (state_ == State::Halted) state_ = State::Stopped;
}

float Motor::speed() {
    Guard g(lock());
    return speed_;
}

Motor::State Motor::state() {
    Guard g(lock());
    return state_;
}

float Motor::rpm() {
    Guard g(lock());
    float perRpm = microstepsPerSecond(1);
    return perRpm != 0 ? drive_.actualVelocity() / perRpm : 0;
}

float Motor::microstepsPerSecond(float rpm) const {
    float ratio = settings.ratio > 0 ? settings.ratio : 1;
    float motorTurnsPerSecond = rpm / 60.0f / ratio;
    return motorTurnsPerSecond * static_cast<float>(settings.stepsPerTurn) *
           static_cast<float>(drive_.microsteps());
}

float Motor::target() const {
    float limit = fabsf(settings.limit);
    return speed_ > limit ? limit : speed_ < -limit ? -limit : speed_;
}

void Motor::begin() { drive_.claim(owner_); }

void Motor::update(uint64_t nowUs) {
    // The driver's ramp follows the motor's acceleration, in its units.
    drive_.ramp.acceleration = fabsf(microstepsPerSecond(settings.acceleration));
    float to = target();
    bool turn = to != 0;
    if (!booted_) {
        booted_ = true;
        if (chip_) {
            tessermotion::TmcFound found = chip_->status().found;
            if (found == tessermotion::TmcFound::Turning && drive_.enabled()) {
                // Adopted at the speed it turned at: down to a stop on the
                // ramp, then held (a motor boots at speed 0), unless a speed
                // was written first.
                state_ = turn ? State::Running : State::Stopping;
            } else if (found == tessermotion::TmcFound::Coasting) {
                coastUntilUs_ = nowUs + static_cast<uint64_t>(settings.coastMs) * 1000;
            }
        } else {
            // A step and direction drive can't tell a coasting rotor from a
            // still one: wait before enabling it, as for a coasting chip.
            coastUntilUs_ = nowUs + static_cast<uint64_t>(settings.coastMs) * 1000;
        }
    }
    switch (state_) {
        case State::Halted:
            return;
        case State::Stopped:
        case State::Absent:
            if (!turn) {
                state_ = State::Stopped;
                return;
            }
            if (!drive_.present()) {
                state_ = State::Absent;
                return;
            }
            if (nowUs < coastUntilUs_) return;  // still coasting
            drive_.enable(true);
            sinceUs_ = nowUs;
            state_ = State::Settling;
            return;
        case State::Settling:
            if (!drive_.enabled()) {
                state_ = State::Halted;
                return;
            }
            if (!turn) {
                state_ = State::Holding;  // enabled: 0 holds
                return;
            }
            if (nowUs - sinceUs_ >= static_cast<uint64_t>(settings.settleMs) * 1000) {
                state_ = State::Running;
                drive_.setVelocity(microstepsPerSecond(to));
            }
            return;
        case State::Running:
        case State::Stopping:
        case State::Holding:
            if (!drive_.enabled()) {
                state_ = State::Halted;
                return;
            }
            if (turn) {
                // Every tick, so the deadman never runs out while turning.
                state_ = State::Running;
                drive_.setVelocity(microstepsPerSecond(to));
                return;
            }
            if (state_ == State::Holding) return;
            state_ = State::Stopping;
            drive_.hold();  // ramps down, then stays enabled at standstill
            if (drive_.actualVelocity() == 0) state_ = State::Holding;
            return;
    }
}

void Motor::stop() {
    drive_.enable(false);
    state_ = State::Halted;
}

}  // namespace motophonique
