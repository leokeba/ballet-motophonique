// The motor (docs/DESIGN.md section 5.1) on TesserMotion's fake TMC2209: the
// units, booting stopped, the start, the driver's ramp, the deadman, speed
// changes through 0, stopping and holding, the limit, halts, an absent driver, a chip
// found turning or coasting at boot.
#pragma once

#include "rig.h"

using motophonique::Motor;
using State = motophonique::Motor::State;

namespace {

struct StepDirMotorRig {
    fake::Gpio gpio;
    fake::Spi bus;
    fake::Tmc5160 chip;
    tessermotion::Tmc5160 driver{bus, tessermotion::Tmc5160Config{5, -1}, &gpio};
    fake::PulseOutput output;
    tessermotion::StepDirGroup group{22, &gpio};
    tessermotion::StepDirMotor stepper{output, driver, group, tessermotion::StepDirConfig{12, 13}, &gpio};
    Motor motor{stepper, &driver, "motor", &stepper};
    motophonique::Motors motors{&motor, 1};
    tessermotion::Motion motion;
    tesser::Api api;
    uint64_t now = 1000000;

    StepDirMotorRig() {
        chip.stepDir = true;
        chip.attach(bus, 5);
        driver.settings.microsteps = 256;
        CHECK(group.add(stepper));
        motion.add(group);
        motion.add(driver);
        motion.add(motor);
        motion.add(stepper);
        motophonique::mountTree(api, motors, motion,
                                [this](tesser::Object& o, int) { tessermotion::describe(o, driver); },
                                [this](tesser::Object& o, int) { tessermotion::describe(o, stepper); });
        motor.settings.coastMs = 0;
        motor.settings.settleMs = 0;
        fake::now = now;
        motion.begin();
    }

    void tick(int count = 1) {
        for (int i = 0; i < count; ++i) {
            now += 20000;
            fake::now = now;
            motion.step(now);
        }
    }

    tesser::Status call(const char* path, const char* body) {
        JsonDocument doc;
        deserializeJson(doc, body);
        tesser::Request request;
        request.op = tesser::Op::Set;
        request.path = path;
        request.body = doc.as<JsonVariantConst>();
        request.client.authenticated = true;
        request.client.access = tesser::Access::User;
        tesser::StringReply reply;
        api.handle(request, reply);
        return reply.status;
    }
};

}  // namespace

TEST(motor_units) {
    Rig r;
    // 60 rpm of the shaft: 1 turn/s, 200 steps of 256 microsteps.
    CHECK(near(r.motor[0].microstepsPerSecond(60), 51200, 0.01));
    CHECK(near(r.motor[0].microstepsPerSecond(-60), -51200, 0.01));
    // Through a 1:3 reduction (a third of a turn per motor turn), at 16.
    r.motor[0].settings.ratio = 1.0f / 3;
    r.driver[0].setMicrosteps(16);
    CHECK(near(r.motor[0].microstepsPerSecond(60), 9600, 0.01));
}

TEST(motor_degrees_convert_to_signed_microsteps) {
    Rig r;
    int32_t steps = 0;
    CHECK(r.motor[0].microstepsForDegrees(90, steps));
    CHECK_EQ(steps, 12800);
    CHECK(r.motor[0].microstepsForDegrees(-90, steps));
    CHECK_EQ(steps, -12800);

    r.motor[0].settings.ratio = 0.5f;
    r.driver[0].setMicrosteps(16);
    CHECK(r.motor[0].microstepsForDegrees(90, steps));
    CHECK_EQ(steps, 1600);
}

TEST(motor_degrees_reject_invalid_or_unrepresentable_commands) {
    Rig r;
    int32_t steps = 17;
    CHECK(!r.motor[0].microstepsForDegrees(NAN, steps));
    CHECK(!r.motor[0].microstepsForDegrees(1e30f, steps));
    CHECK_EQ(steps, 17);
}

TEST(motor_tree_runs_relative_degree_moves) {
    StepDirMotorRig r;
    CHECK(r.motor.positionValid());
    CHECK(r.call("/motors/1/move", "{\"degrees\":90,\"maxSpeed\":30,\"acceleration\":60}") ==
          tesser::Status::Ok);
    r.tick(3);
    CHECK(r.output.running());
    CHECK_EQ(r.output.pump(20000), 12800u);
    r.tick(2);
    CHECK(r.motor.state() == State::Holding);
    CHECK(r.motor.positionValid());
    CHECK(near(r.motor.position(), 90, 0.001));

    CHECK(r.call("/motors/1/move", "{\"degrees\":-45,\"maxSpeed\":30,\"acceleration\":60}") ==
          tesser::Status::Ok);
    r.tick(2);
    CHECK(r.output.running());
    CHECK_EQ(r.output.pump(20000), 6400u);
    r.tick(2);
    CHECK(near(r.motor.position(), 45, 0.001));
}

TEST(motor_relative_moves_obey_the_saved_speed_limit) {
    StepDirMotorRig r;
    r.motor.settings.limit = 10;
    CHECK(r.call("/motors/1/move", "{\"degrees\":90,\"maxSpeed\":30,\"acceleration\":60}") ==
          tesser::Status::Ok);
    r.tick(3);
    CHECK_EQ(r.output.pump(20000), 12800u);
    uint32_t shortestPeriod = UINT32_MAX;
    for (uint32_t period : r.output.periods) {
        if (period < shortestPeriod) shortestPeriod = period;
    }
    CHECK(shortestPeriod >= 1172u);  // 10 rpm at 256 microsteps, not the requested 30
}

TEST(motor_tree_ping_pongs_across_span_and_stop_cancels_cleanly) {
    StepDirMotorRig r;
    CHECK(r.call("/motors/1/pingPong", "{\"span\":90,\"maxSpeed\":30,\"acceleration\":60}") ==
          tesser::Status::Ok);
    r.tick(3);
    CHECK(r.output.running());
    CHECK_EQ(r.output.pump(20000), 12800u);
    r.tick(3);
    CHECK(near(r.motor.position(), 90, 0.001));
    CHECK(r.motor.state() == State::PingPong);
    CHECK(r.output.running());
    CHECK(r.output.pump(10) > 0);

    CHECK(r.call("/motors/stop", "null") == tesser::Status::Ok);
    CHECK_EQ(r.output.pump(20000) > 0, true);
    r.tick(2);
    CHECK(r.motor.state() == State::Holding);
    CHECK(r.motor.positionValid());
    CHECK(r.motor.position() < 90);
    CHECK(r.motor.position() > 0);
}

TEST(motor_lowering_speed_limit_cancels_ping_pong_safely) {
    StepDirMotorRig r;
    CHECK(r.call("/motors/1/pingPong", "{\"span\":90,\"maxSpeed\":30,\"acceleration\":60}") ==
          tesser::Status::Ok);
    r.tick(3);
    CHECK(r.output.running());
    CHECK(r.output.pump(10) > 0);
    r.motor.settings.limit = 10;
    r.tick();
    CHECK(r.output.pump(20000) > 0);
    r.tick(2);
    CHECK(r.motor.state() == State::Holding);
    CHECK(r.motor.positionValid());
    CHECK(r.motor.position() > 0);
    CHECK(r.motor.position() < 90);
}

TEST(motor_boots_stopped_and_claims_its_driver) {
    Rig r;
    for (int i = 0; i < 2; i++) {
        CHECK(r.motor[i].state() == State::Stopped);
        CHECK_EQ(r.motor[i].speed(), 0.0f);
        CHECK(!r.driver[i].enabled());
        CHECK_EQ(r.written(r22::vactual, i), 0u);
    }
    CHECK_EQ(r.driver[0].owner(), std::string("motor 1"));
    CHECK_EQ(r.driver[1].owner(), std::string("motor 2"));
    CHECK(r.set("/drivers/1/enabled", "true").status == tesser::Status::Busy);
    CHECK(r.set("/drivers/2/velocity", "1000").status == tesser::Status::Busy);
    CHECK(!r.driver[0].enabled());
}

TEST(motor_starts_settles_and_ramps) {
    Rig r;
    r.motor[0].settings.acceleration = 30;
    r.motor[0].setSpeed(60);
    r.tick();
    CHECK(r.motor[0].state() == State::Settling);
    CHECK(r.driver[0].enabled());
    CHECK(!r.driver[1].enabled());  // the other motor stays off
    // The driver's ramp, in its units: 30 rpm/s.
    CHECK(near(r.driver[0].ramp.acceleration, 25600, 0.5));
    r.run(0.25f);
    CHECK(r.motor[0].state() == State::Settling);
    CHECK_EQ(r.vactual(), 0.0f);
    r.run(0.1f);
    CHECK(r.motor[0].state() == State::Running);
    float rpm = r.motor[0].rpm();
    CHECK(rpm < 6);
    r.run(1);
    CHECK(near(r.motor[0].rpm(), rpm + 30, 0.7));
    r.run(2);
    CHECK(near(r.motor[0].rpm(), 60, 0.01));
    CHECK(near(r.vactual(), 51200, 30));
}

TEST(motor_outlives_the_deadman) {
    Rig r;
    r.motor[0].settings.acceleration = 0;
    r.motor[0].setSpeed(30);
    r.run(1);
    CHECK(r.motor[0].state() == State::Running);
    r.run(5);  // five deadman times, the speed written once
    CHECK(r.driver[0].velocity() != 0);
    CHECK(r.vactual() > 0);
}

TEST(motor_reverses_through_zero_on_a_ramp) {
    Rig r;
    r.motor[0].settings.acceleration = 60;
    r.motor[0].setSpeed(60);
    r.run(2);
    CHECK(near(r.motor[0].rpm(), 60, 0.01));
    r.motor[0].setSpeed(-60);
    r.run(1);
    CHECK(near(r.motor[0].rpm(), 0, 1.5));
    CHECK(r.motor[0].state() == State::Running);  // not stopped on the way
    CHECK(r.driver[0].enabled());
    r.run(1.1f);
    CHECK(near(r.motor[0].rpm(), -60, 0.01));
    CHECK(r.vactual() < 0);
}

TEST(motor_stops_on_a_ramp_and_holds_at_zero) {
    Rig r;
    r.motor[0].settings.acceleration = 60;
    r.motor[0].setSpeed(60);
    r.run(2);
    r.motor[0].setSpeed(0);
    r.tick();
    CHECK(r.motor[0].state() == State::Stopping);
    CHECK(r.driver[0].enabled());
    r.run(0.5f);
    CHECK(near(r.motor[0].rpm(), 30, 1.5));
    r.run(0.6f);
    CHECK(r.motor[0].state() == State::Holding);
    CHECK(r.driver[0].enabled());  // held at the hold current, not free
    CHECK(r.written(r22::toff) != 0u);
    CHECK_EQ(r.motor[0].rpm(), 0.0f);
    CHECK_EQ(r.vactual(), 0.0f);
    r.run(5);  // past the deadman: still holding
    CHECK(r.motor[0].state() == State::Holding);
    CHECK(r.driver[0].enabled());
}

TEST(motor_turns_from_holding_without_settling) {
    Rig r;
    r.motor[0].settings.acceleration = 0;
    r.motor[0].setSpeed(30);
    r.run(1);
    r.motor[0].setSpeed(0);
    r.run(0.1f);
    CHECK(r.motor[0].state() == State::Holding);
    r.motor[0].setSpeed(-30);
    r.tick();
    CHECK(r.motor[0].state() == State::Running);
    CHECK(near(r.motor[0].rpm(), -30, 0.01));
}

TEST(motor_holds_when_zero_comes_while_settling) {
    Rig r;
    r.motor[0].setSpeed(30);
    r.tick();
    CHECK(r.motor[0].state() == State::Settling);
    r.motor[0].setSpeed(0);
    r.tick();
    CHECK(r.motor[0].state() == State::Holding);
    CHECK(r.driver[0].enabled());
    CHECK_EQ(r.vactual(), 0.0f);
}

TEST(motor_turns_again_while_stopping) {
    Rig r;
    r.motor[0].settings.acceleration = 60;
    r.motor[0].setSpeed(60);
    r.run(2);
    r.motor[0].setSpeed(0);
    r.run(0.5f);
    r.motor[0].setSpeed(45);
    r.tick();
    CHECK(r.motor[0].state() == State::Running);  // no settle: still enabled
    r.run(1);
    CHECK(near(r.motor[0].rpm(), 45, 0.01));
}

TEST(motor_turns_at_its_limit) {
    Rig r;
    r.motor[0].settings.acceleration = 0;
    r.motor[0].settings.limit = 50;
    r.motor[0].setSpeed(-200);
    r.run(1);
    CHECK_EQ(r.motor[0].speed(), -200.0f);  // as asked
    CHECK(near(r.motor[0].rpm(), -50, 0.01));
    r.motor[0].settings.limit = 0;  // nothing turns
    r.run(1.1f);
    CHECK(r.motor[0].state() == State::Holding);
    CHECK_EQ(r.vactual(), 0.0f);
}

TEST(motor_halts_on_stop_all_and_keeps_its_speed) {
    Rig r;
    r.motor[0].setSpeed(30);
    r.run(2);
    r.motion.stopAll();
    r.tick();
    CHECK(r.motor[0].state() == State::Halted);
    CHECK(!r.driver[0].enabled());
    CHECK_EQ(r.motor[0].speed(), 30.0f);
    r.run(2);
    CHECK(r.motor[0].state() == State::Halted);
    r.motor[0].setSpeed(30);  // written again
    r.tick();
    CHECK(r.motor[0].state() == State::Settling);
}

TEST(motor_halts_when_disabled_underneath) {
    Rig r;
    r.motor[0].setSpeed(30);
    r.tick();
    r.driver[0].enable(false);  // code isn't refused by the claim
    r.tick();
    CHECK(r.motor[0].state() == State::Halted);
    r.run(1);
    CHECK(!r.driver[0].enabled());
    CHECK(r.set("/motors/1/speed", "30").status == tesser::Status::Ok);
    r.tick();
    CHECK(r.motor[0].state() == State::Settling);
}

TEST(motor_waits_for_an_absent_driver) {
    Rig r;
    r.chip[0].powered = false;
    r.run(1);
    CHECK(!r.driver[0].present());
    r.motor[0].setSpeed(30);
    r.tick();
    CHECK(r.motor[0].state() == State::Absent);
    CHECK(!r.driver[0].enabled());
    r.chip[0].powered = true;
    r.run(3);  // found again within the driver's 2 s retry
    CHECK(r.driver[0].present());
    CHECK(r.motor[0].state() == State::Settling || r.motor[0].state() == State::Running);
}

TEST(motor_halts_when_the_chip_resets) {
    Rig r;
    r.motor[0].setSpeed(30);
    r.run(2);
    CHECK(r.motor[0].state() == State::Running);
    r.chip[0].powerOn();  // lost power: the driver configures it again, disabled
    r.run(1);
    CHECK(r.motor[0].state() == State::Halted);
    CHECK(!r.driver[0].enabled());
}

TEST(motor_keeps_its_speed_through_a_microsteps_change) {
    for (const char* to : {"\"16\"", "\"1\"", "\"256\""}) {
        Rig r;
        r.motor[0].setSpeed(60);
        r.run(2.5f);
        CHECK(near(r.motor[0].rpm(), 60, 0.01));
        CHECK(r.set("/drivers/1/config/microsteps", to).status == tesser::Status::Ok);
        for (int i = 0; i < 25; i++) {
            r.tick();
            CHECK(near(r.motor[0].rpm(), 60, 0.1));
        }
        // What the chip runs, in full steps: 60 rpm is 200/s.
        uint32_t mres = r.written(r22::mres);
        CHECK(near(r.vactual() / static_cast<float>(256u >> mres), 200, 0.5));
    }
}

namespace {

// A TMC2209 left turning by a reset of the ESP32 alone: powered (no reset
// flag), its stage on, VACTUAL at 120 rpm (TSTEP 117: 400 full steps/s) at
// 256 microsteps, its datagrams 1.1 ms apart; MSCNT advancing `perRead` per
// read (113 at that speed). The driver tells the direction from the count
// the speed and the time predict.
void turningChip(fake::Tmc2209& c, int32_t perRead) {
    c.regs[r22::GSTAT] = 0;
    c.regs[r22::DRV_STATUS] = r22::cs_actual.set(0, 31);
    c.regs[r22::TSTEP] = 117;
    c.regs[r22::CHOPCONF] = r22::mres.set(0x10000053, 0);
    c.mscntPerRead = perRead;
    c.usPerDatagram = 1100;
}

}  // namespace

TEST(motor_ramps_a_chip_found_turning_down_to_a_stop) {
    Rig r(0, [](fake::Tmc2209& c) { turningChip(c, 113); });
    CHECK(r.driver[0].status().found == tessermotion::TmcFound::Turning);
    r.tick();
    CHECK(r.motor[0].state() == State::Stopping);  // a motor boots at speed 0
    CHECK(r.driver[0].enabled());                  // not braked at once
    CHECK(r.motor[0].rpm() > 100);
    r.run(2.2f);  // 120 rpm at 60 rpm/s
    CHECK(r.motor[0].state() == State::Holding);
    CHECK(r.driver[0].enabled());
}

TEST(motor_goes_on_from_a_chip_found_turning_when_asked_first) {
    Rig r(0, [](fake::Tmc2209& c) { turningChip(c, 113); });
    r.motor[0].setSpeed(90);  // written before Motion's first tick
    r.tick();
    CHECK(r.motor[0].state() == State::Running);  // no settle
    for (int i = 0; i < 50; i++) {
        r.tick();
        CHECK(r.driver[0].enabled());  // never stopped
    }
    CHECK(near(r.motor[0].rpm(), 90, 0.01));
}

TEST(motor_waits_for_a_chip_found_coasting) {
    Rig r(0, [](fake::Tmc2209& c) { turningChip(c, 0); });  // its direction can't be told
    CHECK(r.driver[0].status().found == tessermotion::TmcFound::Coasting);
    CHECK(!r.driver[0].enabled());
    r.motor[0].settings.coastMs = 3000;
    r.motor[0].setSpeed(30);
    r.run(2.9f);
    CHECK(r.motor[0].state() == State::Stopped);
    CHECK(!r.driver[0].enabled());
    r.run(0.2f);
    CHECK(r.motor[0].state() == State::Settling);
    CHECK(r.driver[0].enabled());
}

TEST(motor_without_a_chip_waits_the_coast_time_at_boot) {
    // A step and direction drive: no found state, so a reset can't tell a
    // coasting rotor from a still one. The motor waits coastMs before it
    // enables, as for a chip found coasting.
    Rig r;
    motophonique::Motor sd(r.driver[0], nullptr, "step and direction");
    sd.settings.coastMs = 4000;
    sd.setSpeed(30);
    sd.update(1000000);  // the first update: boot
    CHECK(sd.state() == State::Stopped);
    CHECK(!r.driver[0].enabled());
    sd.update(1000000 + 3900000);
    CHECK(sd.state() == State::Stopped);
    sd.update(1000000 + 4100000);
    CHECK(sd.state() == State::Settling);
    CHECK(r.driver[0].enabled());
}
