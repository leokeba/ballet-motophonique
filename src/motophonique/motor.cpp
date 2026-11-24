#include "motophonique/motor.h"

#include <math.h>

namespace motophonique {

using tessermotion::Guard;

void Motor::setSpeed(float rpm) {
    Guard g(lock());
    if (program_ != Program::None && stepper_) stepper_->hold();
    program_ = Program::None;
    segmentStarted_ = false;
    speed_ = rpm;
    if (state_ == State::Halted) state_ = State::Stopped;
}

bool Motor::moveDegrees(float degrees, float maxRpm, float accelerationRpmPerSecond) {
    Guard g(lock());
    return startProgram(degrees, maxRpm, accelerationRpmPerSecond, Program::Move);
}

bool Motor::pingPong(float spanDegrees, float maxRpm, float accelerationRpmPerSecond) {
    Guard g(lock());
    if (!isfinite(spanDegrees) || spanDegrees <= 0) return false;
    return startProgram(spanDegrees, maxRpm, accelerationRpmPerSecond, Program::PingPong);
}

void Motor::onProgramWritten(std::function<void()> callback) {
    Guard g(lock());
    programWritten_ = std::move(callback);
}

bool Motor::startProgram(float degrees, float maxRpm, float accelerationRpmPerSecond, Program program) {
    if (!stepper_ || program_ != Program::None || !isfinite(degrees) || !isfinite(maxRpm) ||
        !isfinite(accelerationRpmPerSecond) || maxRpm <= 0 || accelerationRpmPerSecond <= 0 ||
        speed_ != 0 || !stepper_->commandedPositionValid()) {
        return false;
    }
    float limit = fabsf(settings.limit);
    if (!isfinite(limit) || limit <= 0) return false;
    if (maxRpm > limit) maxRpm = limit;

    int32_t steps = 0;
    if (!microstepsForDegrees(degrees, steps)) return false;
    if (program == Program::PingPong && steps == 0) return false;
    float maxSpeed = microstepsPerSecond(maxRpm);
    float acceleration = microstepsPerSecond(accelerationRpmPerSecond);
    float speedLimit = stepper_->settings.maxSpeed;
    if (!isfinite(maxSpeed) || !isfinite(acceleration) || !isfinite(speedLimit) || speedLimit <= 0 ||
        maxSpeed > speedLimit || acceleration > 4000000.0f) {
        return false;
    }

    program_ = program;
    programDegrees_ = degrees;
    programMaxRpm_ = maxRpm;
    programAccelerationRpmPerSecond_ = accelerationRpmPerSecond;
    programDirection_ = 1;
    segmentStarted_ = false;
    if (state_ == State::Halted) state_ = State::Stopped;
    if (programWritten_) programWritten_();
    return true;
}

bool Motor::startProgramSegment() {
    if (!stepper_) return false;
    float motorLimit = fabsf(settings.limit);
    if (!isfinite(motorLimit) || motorLimit <= 0) return false;
    if (programMaxRpm_ > motorLimit) programMaxRpm_ = motorLimit;

    int32_t steps = 0;
    float maxSpeed = microstepsPerSecond(programMaxRpm_);
    float acceleration = microstepsPerSecond(programAccelerationRpmPerSecond_);
    if (!isfinite(maxSpeed) || !isfinite(acceleration)) return false;
    float speedLimit = stepper_->settings.maxSpeed;
    if (!isfinite(speedLimit) || speedLimit <= 0 || maxSpeed <= 0 || maxSpeed > speedLimit || acceleration <= 0 ||
        acceleration > 4000000.0f ||
        !microstepsForDegrees(programDegrees_ * static_cast<float>(programDirection_), steps) ||
        !stepper_->moveRelative(steps, maxSpeed, acceleration, acceleration)) {
        return false;
    }
    segmentStarted_ = true;
    state_ = program_ == Program::PingPong ? State::PingPong : State::Moving;
    return true;
}

float Motor::position() {
    Guard g(lock());
    if (!stepper_) return 0;
    float ratio = settings.ratio > 0 ? settings.ratio : 1;
    return static_cast<float>(stepper_->commandedPosition256()) * 360.0f * ratio /
           (static_cast<float>(tessermotion::StepDirMotor::kPositionUnitsPerFullStep) *
            static_cast<float>(settings.stepsPerTurn));
}

bool Motor::positionValid() {
    Guard g(lock());
    return stepper_ && stepper_->commandedPositionValid();
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

bool Motor::microstepsForDegrees(float degrees, int32_t& microsteps) const {
    Guard g(lock());
    if (!isfinite(degrees) || !isfinite(settings.ratio) || settings.ratio <= 0 ||
        settings.stepsPerTurn == 0) {
        return false;
    }
    uint16_t resolution = drive_.microsteps();
    if (resolution == 0) return false;
    float requested = degrees * static_cast<float>(settings.stepsPerTurn) * static_cast<float>(resolution) /
                      (360.0f * settings.ratio);
    if (!isfinite(requested)) return false;
    requested = roundf(requested);
    constexpr float kMinSteps = static_cast<float>(INT32_MIN);
    constexpr float kMaxStepsExclusive = -kMinSteps;
    if (requested < kMinSteps || requested >= kMaxStepsExclusive) return false;
    microsteps = static_cast<int32_t>(requested);
    return true;
}

float Motor::target() const {
    float limit = fabsf(settings.limit);
    return speed_ > limit ? limit : speed_ < -limit ? -limit : speed_;
}

void Motor::begin() { drive_.claim(owner_); }

void Motor::update(uint64_t nowUs) {
    if (program_ != Program::None && stepper_) {
        float limit = fabsf(settings.limit);
        if (!isfinite(limit) || limit <= 0) {
            if (segmentStarted_) stepper_->hold();
            program_ = Program::None;
            segmentStarted_ = false;
        } else if (programMaxRpm_ > limit) {
            if (segmentStarted_) {
                stepper_->hold();
                program_ = Program::None;
                segmentStarted_ = false;
            } else {
                programMaxRpm_ = limit;
            }
        }
    }
    // The driver's ramp follows the motor's acceleration, in its units.
    drive_.ramp.acceleration = fabsf(microstepsPerSecond(settings.acceleration));
    float to = target();
    bool turn = to != 0 || program_ != Program::None;
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
                program_ = Program::None;
                segmentStarted_ = false;
                state_ = State::Halted;
                return;
            }
            if (!turn) {
                state_ = State::Holding;  // enabled: 0 holds
                return;
            }
            if (nowUs - sinceUs_ >= static_cast<uint64_t>(settings.settleMs) * 1000) {
                if (program_ != Program::None) {
                    if (!startProgramSegment()) {
                        program_ = Program::None;
                        drive_.hold();
                        state_ = drive_.enabled() ? State::Holding : State::Halted;
                    }
                    return;
                }
                state_ = State::Running;
                drive_.setVelocity(microstepsPerSecond(to));
            }
            return;
        case State::Moving:
        case State::PingPong:
        case State::Running:
        case State::Stopping:
        case State::Holding:
            if (!drive_.enabled()) {
                program_ = Program::None;
                segmentStarted_ = false;
                state_ = State::Halted;
                return;
            }
            if (program_ != Program::None) {
                if (!segmentStarted_) {
                    if (!startProgramSegment()) {
                        program_ = Program::None;
                        drive_.hold();
                        state_ = drive_.enabled() ? State::Holding : State::Halted;
                    }
                    return;
                }
                const auto moveState = stepper_->moveState();
                if (moveState == tessermotion::StepDirMotor::MoveState::Running ||
                    moveState == tessermotion::StepDirMotor::MoveState::Stopping) {
                    state_ = program_ == Program::PingPong ? State::PingPong : State::Moving;
                    return;
                }
                if (moveState == tessermotion::StepDirMotor::MoveState::Complete) {
                    if (program_ == Program::PingPong) {
                        programDirection_ = -programDirection_;
                        segmentStarted_ = false;
                        if (!startProgramSegment()) {
                            program_ = Program::None;
                            state_ = State::Halted;
                        }
                    } else {
                        program_ = Program::None;
                        segmentStarted_ = false;
                        state_ = State::Holding;
                    }
                    return;
                }
                if (moveState == tessermotion::StepDirMotor::MoveState::Failed ||
                    moveState == tessermotion::StepDirMotor::MoveState::Interrupted ||
                    moveState == tessermotion::StepDirMotor::MoveState::Cancelled) {
                    program_ = Program::None;
                    segmentStarted_ = false;
                    drive_.hold();
                    state_ = drive_.enabled() ? State::Holding : State::Halted;
                    return;
                }
                return;
            }
            if (stepper_) {
                const auto moveState = stepper_->moveState();
                if (moveState == tessermotion::StepDirMotor::MoveState::Running ||
                    moveState == tessermotion::StepDirMotor::MoveState::Stopping) {
                    return;
                }
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
    program_ = Program::None;
    segmentStarted_ = false;
    drive_.enable(false);
    state_ = State::Halted;
}

}  // namespace motophonique
