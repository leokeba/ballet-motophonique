// A host node serving the app's real tree (one step/dir motor on a TMC5160,
// as in StepDirMotorRig) over stdin/stdout, so the UI can be driven and
// screenshotted without a board (DESIGN section 10): TesserUI's
// web/scripts/device.mjs serves it with --exe. device.mjs runs it with
// --clients: each line is "<client> <envelope>", and client 0 carries the
// HTTP requests (TesserUI's test/demo_main.cpp does the same). The fake
// pulse output is pumped each tick, as the platform's chunked refill of the
// pulse queue would be on a board.
#include <errno.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <map>
#include <memory>
#include <string>

#include "Motophonique.h"
#include "TesserAPI.h"
#include "TesserMotion.h"
#include "fake_hw.h"
#include "fake_tmc.h"
#include "tesserui/views.h"

namespace {

struct Client {
    Client(tesser::Api& api, long id)
        : prefix(std::to_string(id) + " "), line(api, [this](const char* data, size_t len) {
              out.append(data, len);
              size_t nl;
              while ((nl = out.find('\n')) != std::string::npos) {
                  fwrite(prefix.data(), 1, prefix.size(), stdout);
                  fwrite(out.data(), 1, nl + 1, stdout);
                  out.erase(0, nl + 1);
              }
              fflush(stdout);
          }) {}
    std::string prefix;
    std::string out;
    tesser::LineTransport line;
};

}  // namespace

int main(int argc, char** argv) {
    bool clients = false;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--clients") == 0) clients = true;
    }

    static fake::Gpio gpio;
    static fake::Spi bus;
    static fake::Tmc5160 chip;
    static tessermotion::Tmc5160 driver{bus, tessermotion::Tmc5160Config{5, -1}, &gpio};
    static fake::PulseOutput output;
    static tessermotion::StepDirGroup group{22, &gpio};
    static tessermotion::StepDirMotor stepper{output, driver, group, tessermotion::StepDirConfig{12, 13}, &gpio};
    static motophonique::Motor motor{stepper, &driver, "motor", &stepper};
    static motophonique::Motors motors{&motor, 1};
    static tessermotion::Motion motion;
    static tesser::Api api;

    chip.stepDir = true;
    chip.attach(bus, 5);
    driver.settings.microsteps = 256;
    if (!group.add(stepper)) return 1;
    motion.add(group);
    motion.add(driver);
    motion.add(motor);
    motion.add(stepper);
    // driver and stepper are statics: no capture needed.
    motophonique::mountTree(api, motors, motion,
                            [](tesser::Object& o, int) { tessermotion::describe(o, driver); },
                            [](tesser::Object& o, int) { tessermotion::describe(o, stepper); });
    static tesserui::Views views(api);
    motophonique::addViews(views, motors);
    motor.settings.coastMs = 0;
    motor.settings.settleMs = 0;
    motion.begin();

    fprintf(stderr, "app demo node: one step/dir motor on a TMC5160%s\n", clients ? ", --clients" : "");

    std::map<long, std::unique_ptr<Client>> byId;
    std::string pending;
    auto dispatch = [&](const std::string& in) {
        char* rest = nullptr;
        long id = strtol(in.c_str(), &rest, 10);
        if (rest == in.c_str() || *rest != ' ') return;
        std::string body(rest + 1);
        if (body == "close") {
            byId.erase(id);
            return;
        }
        auto& c = byId[id];
        if (!c) c.reset(new Client(api, id));
        body += '\n';
        c->line.feed(body.data(), body.size());
    };

    char buf[4096];
    uint32_t nowMs = 1000;
    for (;;) {
        struct pollfd pfd = {0, POLLIN, 0};
        int ready = poll(&pfd, 1, 20);
        if (ready < 0 && errno != EINTR) break;
        if (ready > 0) {
            ssize_t n = read(0, buf, sizeof(buf));
            if (n == 0) break;
            if (n < 0 && errno != EINTR && errno != EAGAIN) break;
            if (n > 0) {
                if (!clients) {
                    // One client with no prefix.
                    auto& c = byId[0];
                    if (!c) c.reset(new Client(api, 0));
                    std::string body(buf, static_cast<size_t>(n));
                    size_t nl;
                    while ((nl = body.find('\n')) != std::string::npos) {
                        c->line.feed(body.data(), nl + 1);
                        body.erase(0, nl + 1);
                    }
                } else {
                    pending.append(buf, static_cast<size_t>(n));
                    size_t nl;
                    while ((nl = pending.find('\n')) != std::string::npos) {
                        dispatch(pending.substr(0, nl));
                        pending.erase(0, nl + 1);
                    }
                }
            }
        }
        nowMs += 20;
        fake::now = static_cast<uint64_t>(nowMs) * 1000;
        motion.step(fake::now);
        output.pump(8192);  // the platform's chunked refill of the pulse queue
        api.poll(nowMs);
    }
    byId.clear();
    return 0;
}
