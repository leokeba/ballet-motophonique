#pragma once

// A stepper motor turned at a signed speed (docs/DESIGN.md section 5.1): a
// TesserMotion part driving a MotionDrive at a velocity (a TMC2209's VACTUAL,
// or a step and direction generator's pulses for a TMC5160), in turns per
// minute of what it turns, through the drive's ramp. One number says everything: a speed other than 0 enables
// the driver, waits at standstill, then turns; 0 ramps down to a stop and
// holds there, the driver enabled at its hold current (Leo, 2026-11-12). The
// velocity is renewed every tick, so the driver's deadman never stops a
// turning motor and a master writing the speed needn't renew it. The motor claims its drive, and its chip too: their trees' controls are readouts.
//
// Added to Motion before its driver, so the velocity it sets reaches the
// chip in the same tick. Built on Fluante's spinner (fluante-firmware).

#include <stdint.h>

#include <functional>
#include <utility>

#include "tessermotion/motion.h"
#include "tessermotion/step_dir.h"
#include "tessermotion/tmc2209.h"

namespace motophonique {

class Motor : public tessermotion::Part {
public:
    enum class State : uint8_t {
        Stopped,   // never asked to turn since boot or a halt: the driver off
        Absent,    // asked to turn, the driver doesn't answer
        Settling,  // enabled, at standstill before turning
        Running,   // turning, ramping to the speed
        Stopping,  // ramping down to 0, then holding
        Holding,   // speed 0: at standstill, the driver enabled (its hold current)
        Moving,    // completing a finite relative move
        PingPong,  // moving between two points until stopped
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
        // through the ESP32's reset while the motor turned), or after any
        // reset of a step and direction drive, which can't tell a coasting
        // rotor from a still one (Leo, 2026-11-15): the wait before enabling.
        uint32_t coastMs = 10000;
    };

    // `owner` names the motor in the drive's claim ("driven by <owner>").
    // `chip`: the TMC2209 behind `drive`, to adopt a chip found turning at
    // boot; null for a step and direction drive, which has no found state.
    Motor(tessermotion::MotionDrive& drive, tessermotion::TmcDriver* chip, const char* owner = "motor",
          tessermotion::StepDirMotor* stepper = nullptr)
        : drive_(drive), chip_(chip), stepper_(stepper), owner_(owner) {}

    // Under lock() once added to Motion (the tree does).
    Settings settings;

    // From any task: these take Motion's lock.
    // The speed asked, rpm, signed: 0 stops and holds. Writing it leaves a
    // halt.
    void setSpeed(float rpm);
    // Starts a relative degree move, or ping-pongs across a total span, at
    // the given output speed and acceleration.
    bool moveDegrees(float degrees, float maxRpm, float accelerationRpmPerSecond);
    bool pingPong(float spanDegrees, float maxRpm, float accelerationRpmPerSecond);
    bool zeroPosition();
    void onProgramWritten(std::function<void()> callback);
    float position();
    bool positionValid();
    float speed();
    State state();
    float rpm();  // the speed now, as the driver ramps it

    // The driver's velocity for `rpm`, in µsteps/s, at the microsteps in
    // effect (also rpm/s to µsteps/s²): the driver keeps the speed through
    // a change. Under the lock.
    float microstepsPerSecond(float rpm) const;
    // Converts an output rotation into signed µsteps at the current
    // resolution, rejecting non-finite and out-of-range commands.
    bool microstepsForDegrees(float degrees, int32_t& microsteps) const;
    // The speed turned at: the speed asked, within ±limit. Under the lock.
    float target() const;

    // Part
    void begin() override;
    void update(uint64_t nowUs) override;
    // Motion::stopAll(): disabled at once, halted; the speed keeps what was
    // asked.
    void stop() override;

private:
    enum class Program : uint8_t { None, Move, PingPong };
    bool startProgramSegment();
    bool startProgram(float degrees, float maxRpm, float accelerationRpmPerSecond, Program program);

    tessermotion::MotionDrive& drive_;
    tessermotion::TmcDriver* chip_;
    tessermotion::StepDirMotor* stepper_;
    const char* owner_;
    float speed_ = 0;  // never stored: a motor boots stopped
    State state_ = State::Stopped;
    uint64_t sinceUs_ = 0;  // entered Settling
    bool booted_ = false;   // the first update, which sees what the chip found
    uint64_t coastUntilUs_ = 0;
    Program program_ = Program::None;
    float programDegrees_ = 0;
    float programMaxRpm_ = 0;
    float programAccelerationRpmPerSecond_ = 0;
    int programDirection_ = 1;
    bool segmentStarted_ = false;
    std::function<void()> programWritten_;
};

}  // namespace motophonique
