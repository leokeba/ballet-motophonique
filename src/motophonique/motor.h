#pragma once

// A stepper motor turned at a signed speed (docs/DESIGN.md section 5.1): a
// TesserMotion part driving a TMC2209 at a velocity (VACTUAL, the chip's own
// step generator), in turns per minute of what it turns, through the
// driver's ramp. One number says everything: a speed other than 0 enables
// the driver, waits at standstill, then turns; 0 ramps down to a stop and
// disables it. The velocity is renewed every tick, so the driver's deadman
// never stops a turning motor and a master writing the speed needn't renew
// it. The motor claims its driver: the driver tree's controls are readouts.
//
// Added to Motion before its driver, so the velocity it sets reaches the
// chip in the same tick. Built on Fluante's spinner (fluante-firmware).

#include <stdint.h>

#include "tessermotion/motion.h"
#include "tessermotion/tmc2209.h"

namespace motophonique {

class Motor : public tessermotion::Part {
public:
    enum class State : uint8_t {
        Stopped,   // speed 0, or never asked
        Absent,    // asked to turn, the driver doesn't answer
        Settling,  // enabled, at standstill before turning
        Running,   // turning, ramping to the speed
        Stopping,  // ramping down to 0, then disabled
        Halted,    // the driver was disabled underneath (Motion::stopAll(),
                   // a reset of the chip): waits for a speed to be written
    };

    // The fastest speed the tree takes, either way, rpm.
    static constexpr float kMaxRpm = 1000;

    struct Settings {
        float acceleration = 60;  // rpm/s, up and down; 0 jumps
        float limit = 120;        // rpm: faster speeds are turned at this
        // Turns of what the motor turns per motor turn (a pulley's or a
        // gearbox's ratio); 1 for the shaft itself.
        float ratio = 1;
        uint16_t stepsPerTurn = 200;  // the motor's full steps
        // At standstill after enabling, before turning: stealthChop's
        // automatic tuning wants 130 ms at least.
        uint32_t settleMs = 300;
        // At boot, after a driver found coasting (its power stage was off
        // through the ESP32's reset while the motor turned): the wait before
        // enabling it.
        uint32_t coastMs = 10000;
    };

    // `owner` names the motor in the driver's claim ("driven by <owner>").
    explicit Motor(tessermotion::Tmc2209& driver, const char* owner = "motor") : driver_(driver), owner_(owner) {}

    // Under lock() once added to Motion (the tree does).
    Settings settings;

    // From any task: these take Motion's lock.
    // The speed asked, rpm, signed: 0 stops. Writing it leaves a halt.
    void setSpeed(float rpm);
    float speed();
    State state();
    float rpm();  // the speed now, as the driver ramps it

    // The driver's velocity for `rpm`, in µsteps/s, at the microsteps in
    // effect (also rpm/s to µsteps/s²): the driver keeps the speed through
    // a change. Under the lock.
    float microstepsPerSecond(float rpm) const;
    // The speed turned at: the speed asked, within ±limit. Under the lock.
    float target() const;

    tessermotion::Tmc2209& driver() { return driver_; }

    // Part
    void begin() override;
    void update(uint64_t nowUs) override;
    // Motion::stopAll(): disabled at once, halted; the speed keeps what was
    // asked.
    void stop() override;

private:
    tessermotion::Tmc2209& driver_;
    const char* owner_;
    float speed_ = 0;  // never stored: a motor boots stopped
    State state_ = State::Stopped;
    uint64_t sinceUs_ = 0;  // entered Settling
    bool booted_ = false;   // the first update, which sees what the driver found
    uint64_t coastUntilUs_ = 0;
};

}  // namespace motophonique
