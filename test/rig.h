// A motor controller on the host: two motors on TesserMotion's fake TMC2209s,
// each behind its own fake UART, the tree, and a clock ticked at 50 Hz as the
// Motion task would.
#pragma once

#include <math.h>

#include <stdint.h>

#include <functional>
#include <string>
#include <string_view>

#include <ArduinoJson.h>

#include "Motophonique.h"
#include "TesserAPI.h"
#include "TesserMotion.h"
#include "check.h"
#include "fake_hw.h"
#include "fake_tmc.h"
#include "tessermotion/tmc/tmc2209_map.h"

namespace {

namespace r22 = tessermotion::tmc2209;

constexpr uint32_t kTickUs = 20000;
constexpr int kEnablePins[2] = {13, 14};

struct Rig {
    fake::Gpio gpio;
    fake::Uart uart[2];
    fake::Tmc2209 chip[2];
    tessermotion::Tmc2209 driver[2] = {tessermotion::Tmc2209{uart[0], config(0), &gpio},
                                       tessermotion::Tmc2209{uart[1], config(1), &gpio}};
    motophonique::Motor motor[2] = {motophonique::Motor{driver[0], &driver[0], "motor 1"},
                                    motophonique::Motor{driver[1], &driver[1], "motor 2"}};
    motophonique::Motors motors{motor, 2};
    tessermotion::Motion motion;
    tesser::Api api;
    uint64_t now = 1000000;

    static tessermotion::Tmc2209Config config(int i) {
        tessermotion::Tmc2209Config c;
        c.enable = kEnablePins[i];
        return c;
    }

    // `idle` seconds of ticks, so the drivers have read every register.
    // `prepare` sets the first chip up before its driver looks for it.
    explicit Rig(float idle = 0.5f, std::function<void(fake::Tmc2209&)> prepare = nullptr) {
        if (prepare) prepare(chip[0]);
        for (int i = 0; i < 2; i++) {
            chip[i].attach(uart[i]);
            driver[i].settings.microsteps = 256;
        }
        for (auto& m : motor) motion.add(m);  // before the drivers: a velocity reaches the chip in the same tick
        for (auto& d : driver) motion.add(d);
        motophonique::mountTree(api, motors, motion, [this](tesser::Object& o, int i) {
            tessermotion::describe(o, driver[i]);
        });
        motion.begin();
        run(idle);
    }

    void tick(int n = 1) {
        for (int i = 0; i < n; i++) {
            now += kTickUs;
            fake::now = now;
            motion.step(now);
        }
    }
    void run(float seconds) { tick(static_cast<int>(seconds * 1e6f / kTickUs + 0.5f)); }

    // The velocity chip `i` was last given, in µsteps/s.
    float vactual(int i = 0) const {
        return static_cast<float>(r22::vactual.get(chip[i].written[r22::VACTUAL])) * 12e6f / 16777216.0f;
    }
    uint32_t written(tessermotion::TmcField f, int i = 0) const {
        return static_cast<uint32_t>(f.get(chip[i].written[f.reg]));
    }

    struct Reply {
        tesser::Status status;
        std::string body;
    };
    Reply request(tesser::Op op, std::string_view path, std::string_view body = {},
                  tesser::Access level = tesser::Access::Admin) {
        JsonDocument doc;
        if (!body.empty()) deserializeJson(doc, body.data(), body.size());
        tesser::Request req;
        req.op = op;
        req.path = path;
        req.body = doc.as<JsonVariantConst>();
        req.client.authenticated = level != tesser::Access::Public;
        req.client.access = level;
        tesser::StringReply reply;
        api.handle(req, reply);
        return {reply.status, reply.body};
    }
    Reply get(std::string_view path) { return request(tesser::Op::Get, path); }
    Reply set(std::string_view path, std::string_view body, tesser::Access level = tesser::Access::Admin) {
        return request(tesser::Op::Set, path, body, level);
    }
    // An action, as the transports call one: a set.
    Reply call(std::string_view path) { return request(tesser::Op::Set, path); }
};

bool near(double a, double b, double tolerance) { return fabs(a - b) <= tolerance; }

}  // namespace
