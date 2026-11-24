// Ballet Motophonique (docs/DESIGN.md): stepper motors at a signed speed,
// under /motors, played by Ballet Photophonique's master through its
// controller mappings; TesserMotion for the TMC5160 and step/dir output; TesserKIT for Wi-Fi,
// updates from this repository's releases, files, accounts, the log and the
// API over UDP (so the master finds it under its /peers) under /sys.
//
// Flash it, join the "ballet-motophonique-xxxx" network from a phone, and
// the interface opens; or reach it from the launcher over USB, or by its
// address. A fresh controller boots with every motor stopped, and every
// controller boots that way: a speed is never stored. Check the motor's
// current (/drivers/n/config/runCurrent) before writing a speed.
//
// The ESP32/TMC5160 wiring is described in board.h. Each motor boots stopped;
// check each driver's current before requesting motion.
//
// For development, a gitignored main/secrets.h may define WIFI_SSID and
// WIFI_PASSWORD, and WIFI_TX_POWER (dBm) for boards with a weak supply.
#include "Motophonique.h"
#include "TesserKIT.h"
#include "TesserMotion.h"
#include "board.h"
#include "esp_err.h"
#include "tessermotion/platform/esp/pulse_rmt.h"
#include "tessermotion/step_dir.h"

#if __has_include("secrets.h")
#include "secrets.h"
#endif

using namespace tessermotion;

namespace {

tesserkit::Device dev("Ballet Motophonique");
tesserkit::System sys(dev);
tesserkit::Network net(dev);
tesserkit::NowTp espNow(dev);  // the controllers nearby over ESP-NOW, for testing (after Network)
tesserkit::Firmware firmware(dev);
tesserkit::Files files(dev);  // LittleFS: the settings
tesserkit::Auth auth(dev);    // open until an admin account exists
tesserkit::UdpApi udp(dev);   // the master reaches the motors over UDP
tesserkit::Peers peers(dev);  // how the other controllers are reached (UDP, then NowTP)
tesserkit::Log logs(dev);     // the firmware's log, as events and a file
// The launcher's USB way: envelopes on the console's UART, one per line, with
// Admin access (physical access). Logs stay at warnings, so that few land in
// the middle of a reply (sdkconfig.defaults).
tesser::UartTransport serialApi(dev.api(), UART_NUM_0);

SpiBus spi({.sclk = board::kTmcSck, .mosi = board::kTmcMosi, .miso = board::kTmcMiso});
static_assert(board::kMotors <= board::kMaxMotors);
Tmc5160 drivers[board::kMaxMotors] = {
    Tmc5160(spi, {.cs = board::kMotorPins[0].cs, .enable = -1}),
    Tmc5160(spi, {.cs = board::kMotorPins[1].cs, .enable = -1}),
};
RmtPulseOutput pulseOutputs[board::kMaxMotors];
StepDirGroup stepDirGroup(board::kTmcEnable);
StepDirMotor stepDirMotors[board::kMaxMotors] = {
    StepDirMotor(pulseOutputs[0], drivers[0], stepDirGroup,
                 {.step = board::kMotorPins[0].step, .dir = board::kMotorPins[0].dir}),
    StepDirMotor(pulseOutputs[1], drivers[1], stepDirGroup,
                 {.step = board::kMotorPins[1].step, .dir = board::kMotorPins[1].dir}),
};
Motion motion;

motophonique::Motor motor[board::kMaxMotors] = {
    motophonique::Motor(stepDirMotors[0], &drivers[0], "motor 1", &stepDirMotors[0]),
    motophonique::Motor(stepDirMotors[1], &drivers[1], "motor 2", &stepDirMotors[1]),
};
motophonique::Motors motors{motor, board::kMotors};

}  // namespace

extern "C" void app_main() {
#if defined(WIFI_SSID) && defined(WIFI_PASSWORD)
    net.defaultNetwork(WIFI_SSID, WIFI_PASSWORD);
#endif
#if defined(WIFI_TX_POWER)
    net.txPower(WIFI_TX_POWER);
#endif
    // The master's writes (docs/DESIGN.md section 5.2): over UDP, handled in
    // UDP's own task (in TesserKIT's they wait about 10 ms more); NowTP for
    // testing, off until turned on.
    espNow.enable(false).maxCalls(10);
    udp.maxCalls(10).ownTask(true);
    peers.order("udp/ethernet, nowtp");
    // The show's discovery group, as photophonique's: its master mounts only
    // the peers sharing one (docs/DESIGN.md section 5.2). The factory default
    // of /sys/peers/group (Admin, persisted); empty mounts every node.
    peers.group("ballet");
    // Over UDP a lost master's route waits for its 5 s probe before its
    // writes go again: a motor isn't stopped by a short loss of its master
    // (as photophonique's, which sets the same).
    peers.grace(6000);
    // A master gone from every transport, or restarted: the motors whose
    // speed it wrote last stop on their ramps (docs/DESIGN.md section 5.3).
    peers.onDeviceLost([](const tesser::PeerAddress& device, tesser::PeerLoss reason) {
        dev.api().writerLost(device, reason);
    });
    // Updates from this repository's releases (.github/workflows/release.yml),
    // and from any URL.
    firmware.github("leokeba/ballet-motophonique", "ballet-motophonique-{target}.bin");

    for (int i = 0; i < board::kMotors; ++i) {
        ESP_ERROR_CHECK(stepDirGroup.add(stepDirMotors[i]) ? ESP_OK : ESP_ERR_INVALID_STATE);
    }

    // The TMC5160 supplies the driver's resolution; the ESP32 generates STEP.
    for (int i = 0; i < board::kMotors; ++i) drivers[i].settings.microsteps = 256;

    // The application sets the requested velocity before the step generator
    // consumes it; the group owns the shared active-low enable line.
    motion.add(stepDirGroup);
    for (int i = 0; i < board::kMotors; ++i) motion.add(drivers[i]);
    for (int i = 0; i < board::kMotors; ++i) motion.add(motor[i]);
    for (int i = 0; i < board::kMotors; ++i) motion.add(stepDirMotors[i]);

    auto& api = dev.api();
    motophonique::mountTree(
        api, motors, motion,
        [](tesser::Object& o, int i) { tessermotion::describe(o, drivers[i]); },
        [](tesser::Object& o, int i) { tessermotion::describe(o, stepDirMotors[i]); });
    static tesserui::Views views(api);
    motophonique::addViews(views, motors);

    // Nothing turns while new firmware is written.
    dev.onState([](tesserkit::State s) {
        if (s == tesserkit::State::Updating) motion.stopAll();
    });

    dev.cors("https://leokeba.github.io");  // the launchers, reaching the device by its address
    dev.begin();  // loads the stored settings
    serialApi.begin();
    motion.start(50);
}
