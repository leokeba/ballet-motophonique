// The /motors and /drivers trees (docs/DESIGN.md section 6).
#include "motophonique/tree.h"

#include "tessermotion/tree_setting.h"
#include "tessermotion/trees.h"

namespace motophonique {

namespace {

using tesser::Access;
using tesser::Object;
using tessermotion::setting;

const char* const kNames[Motors::kMax] = {"1", "2", "3", "4", "5", "6", "7", "8"};
const char* const kLabels[Motors::kMax] = {"Motor 1", "Motor 2", "Motor 3", "Motor 4",
                                           "Motor 5", "Motor 6", "Motor 7", "Motor 8"};

}  // namespace

void describe(Object& o, MoveRequest& request) {
    o.value("degrees", request.degrees).range(-3600, 3600).unit("°").doc("Relative output rotation");
    o.value("maxSpeed", request.maxSpeed).range(0.1, Motor::kMaxRpm).unit("rpm");
    o.value("acceleration", request.acceleration).range(0.1, 6000).unit("rpm/s");
}

void describe(Object& o, PingPongRequest& request) {
    o.value("span", request.span).range(0.1, 3600).unit("°").doc("Total endpoint-to-endpoint distance");
    o.value("maxSpeed", request.maxSpeed).range(0.1, Motor::kMaxRpm).unit("rpm");
    o.value("acceleration", request.acceleration).range(0.1, 6000).unit("rpm/s");
}

void Motors::stopAll() {
    for (int i = 0; i < count; i++) items[i].setSpeed(0);
}

void describe(Object& o, Motor& m) {
    o.writeAccess(Access::User);
    Motor::Settings& s = m.settings;
    o.value("speed", [&m] { return m.speed(); }, [&m](float v) { m.setSpeed(v); })
        .range(-Motor::kMaxRpm, Motor::kMaxRpm)
        .step(0.1)
        .unit("rpm")
        .watch()
        .doc("Turns per minute, negative the other way, 0 stops and holds. Not stored: the motor boots "
             "stopped, its driver off. Faster than the limit turns at the limit");
    o.value("rpm", [&m] { return m.rpm(); })
        .unit("rpm")
        .ui("precision", 1)
        .watch()
        .doc("The speed now, as ramped. Approximate: the driver's clock isn't calibrated");
    o.value("state", [&m] { return m.state(); },
            {"stopped", "absent", "settling", "running", "stopping", "holding", "moving", "pingPong", "halted"})
        .watch()
        .doc("stopped: the driver off, as at boot. holding: at speed 0, the driver on at its hold current. absent: "
             "the driver doesn't answer. moving: a finite relative move. pingPong: moving between endpoints until "
             "stopped. halted: the driver was disabled underneath; write a speed again. stopped "
             "with a speed: waiting for the motor to coast down at boot");
    o.value("position", [&m] { return m.position(); })
        .unit("°")
        .watch()
        .doc("Open-loop commanded output position, relative to boot; valid only if positionValid is true");
    o.value("positionValid", [&m] { return m.positionValid(); })
        .watch()
        .doc("False if untracked continuous motion or an interruption made the commanded position uncertain");
    o.action("move", [&m](const MoveRequest& request) -> tesser::Check {
         if (m.moveDegrees(request.degrees, request.maxSpeed, request.acceleration)) return tesser::Check::ok();
         return tesser::Check::fail(
             tesser::Status::NotAllowed,
             "move refused: requires an idle step/dir motor, valid position, and supported settings");
     })
        .label("Move")
        .writeAccess(Access::User)
        .doc("Move by a relative number of degrees at maxSpeed, with the given acceleration");
    o.action("pingPong", [&m](const PingPongRequest& request) -> tesser::Check {
         if (m.pingPong(request.span, request.maxSpeed, request.acceleration)) return tesser::Check::ok();
         return tesser::Check::fail(
             tesser::Status::NotAllowed,
             "ping-pong refused: requires an idle step/dir motor, valid position, and supported settings");
     })
        .label("Ping-pong")
        .writeAccess(Access::User)
        .doc("Start at the current position and alternate across span degrees until stopped");
    o.action("zeroPosition", [&m, &o] {
         if (!m.zeroPosition()) {
             return tesser::Check::fail(
                 tesser::Status::NotAllowed,
                 "re-zero refused: requires an idle step/dir motor held at zero speed");
         }
         o.changed();
         return tesser::Check::ok();
     })
        .label("Re-zero position")
        .writeAccess(Access::Admin)
        .doc("Declare the current held position to be 0°; does not move or home the motor");
    setting(o, "acceleration", m, s.acceleration)
        .range(0, 6000)
        .step(1)
        .unit("rpm/s")
        .persist()
        .doc("Up and down; 0 jumps. Sets the driver's ramp");
    setting(o, "limit", m, s.limit)
        .range(0, Motor::kMaxRpm)
        .step(0.1)
        .unit("rpm")
        .persist()
        .doc("The fastest the motor turns, either way, whatever speed is asked");

    auto& mech = o.object("mechanics")
                     .label("Mechanics")
                     .persist()
                     .writeAccess(Access::Admin)
                     .ui("collapsed");
    setting(mech, "ratio", m, s.ratio)
        .range(0.001, 1000)
        .step(0.001)
        .doc("Turns of what the motor turns per motor turn (pulleys, a gearbox): speeds are counted there. 1 for "
             "the shaft");
    setting(mech, "stepsPerTurn", m, s.stepsPerTurn).range(1, 1000).doc("The motor's full steps per turn");

    auto& start = o.object("start")
                      .label("Start")
                      .persist()
                      .writeAccess(Access::Admin)
                      .doc("What starting does before turning")
                      .ui("collapsed");
    setting(start, "settle", m, s.settleMs).range(0, 10000).unit("ms").doc("At standstill after enabling");
    setting(start, "coast", m, s.coastMs)
        .range(0, 60000)
        .unit("ms")
        .doc("At boot, when the driver was found coasting (off through a reset of the ESP32 alone): the wait for "
             "the motor to stop before enabling it");
}

void describe(Object& o, Motors& motors) {
    o.writeAccess(Access::User);
    o.action("stop", [&motors, &o] {
         motors.stopAll();
         o.changed();
     })
        .label("Stop all")
        .doc("Every speed to 0: each motor ramps down and holds")
        .ui("style", "danger")
        .ui("icon", "power");
    for (int i = 0; i < motors.count && i < Motors::kMax; i++) {
        describe(o.object(kNames[i]).label(kLabels[i]), motors.items[i]);
    }
}

void mountTree(tesser::Api& api, Motors& motors, tessermotion::Motion& motion, const DriveTree& describeDrive,
               const MotorDriveTree& describeMotorDrive) {
    auto& tree = api.mount("motors", motors).label("Motors").ui("icon", "motor");
    // A speed's writer is remembered (TesserAPI's trackWriter()): when that
    // device is lost (api.writerLost(), from TesserKIT's peers.onDeviceLost())
    // the motor stops on its ramp. A speed written without a device (this
    // controller's page, its serial API) has no writer to lose.
    for (int i = 0; i < motors.count && i < Motors::kMax; i++) {
        auto* motor = static_cast<Object*>(tree.child(kNames[i]));
        if (!motor) continue;
        if (describeMotorDrive) describeMotorDrive(*motor, i);
        tesser::Node* speed = motor->child("speed");
        if (!speed) continue;
        Motor& m = motors.items[i];
        m.onProgramWritten([&api, speed, motor] {
            api.wrote(*speed);
            motor->changed();
        });
        api.trackWriter(*speed, [&m](tesser::Node& node, tesser::PeerLoss) {
            m.setSpeed(0);
            node.changed();
        });
    }
    auto& drivers = api.object("drivers").label("Motor drivers").ui("icon", "motor").ui("group", "Hardware");
    for (int i = 0; i < motors.count && i < Motors::kMax; i++) {
        describeDrive(drivers.object(kNames[i]).label(kLabels[i]), i);
    }
    api.mount("motion", motion).label("Control loop").ui("icon", "chart").ui("group", "Hardware");
}

}  // namespace motophonique
