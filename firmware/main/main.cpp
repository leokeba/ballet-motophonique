// Ballet Motophonique (docs/DESIGN.md): stepper motors at a signed speed,
// under /motors, played by Ballet Photophonique's master through its
// controller mappings; TesserMotion for the TMC2209s; TesserKIT for Wi-Fi,
// updates from this repository's releases, files, accounts, the log and the
// API over UDP (so the master finds it under its /peers) under /sys.
//
// Flash it, join the "ballet-motophonique-xxxx" network from a phone, and
// the interface opens; or reach it from the launcher over USB, or by its
// address. A fresh controller boots with every motor stopped, and every
// controller boots that way: a speed is never stored. Check the motor's
// current (/drivers/n/config/runCurrent) before writing a speed.
//
// The board isn't known yet (board.h): don't flash this onto a board with a
// motor before board.h describes it.
//
// For development, a gitignored main/secrets.h may define WIFI_SSID and
// WIFI_PASSWORD, and WIFI_TX_POWER (dBm) for boards with a weak supply.
#include "Motophonique.h"
#include "TesserKIT.h"
#include "TesserMotion.h"
#include "board.h"

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

UartPort tmcUart({.rx = board::kTmcRx, .tx = board::kTmcTx, .baud = 115200, .port = board::kTmcUart});
Tmc2209 drivers[board::kMotors] = {Tmc2209(tmcUart, {.address = 0, .enable = board::kTmcEnable})};  // 0.11 Ω
Motion motion;

motophonique::Motor motor[board::kMotors] = {motophonique::Motor(drivers[0], "motor 1")};
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
    // A master gone from every transport, or restarted: the motors whose
    // speed it wrote last stop on their ramps (docs/DESIGN.md section 5.3).
    peers.onDeviceLost([](const tesser::PeerAddress& device, tesser::PeerLoss reason) {
        dev.api().writerLost(device, reason);
    });
    // Updates from this repository's releases (.github/workflows/release.yml),
    // and from any URL.
    firmware.github("leokeba/ballet-motophonique", "ballet-motophonique-{target}.bin");

    // The board's pins the libraries don't drive.
    Gpio& gpio = defaultGpio();
    gpio.output(board::kTmcMs1, false);
    gpio.output(board::kTmcMs2, false);
    gpio.output(board::kTmcStep, false);
    gpio.output(board::kTmcDir, false);

    // Factory defaults, until the stored settings say otherwise: 256
    // microsteps (the chip makes the steps: the smoothest costs nothing).
    for (auto& d : drivers) d.settings.microsteps = 256;

    // Each motor sets its driver's velocity before the driver acts on it.
    for (auto& m : motor) motion.add(m);
    for (auto& d : drivers) motion.add(d);

    auto& api = dev.api();
    motophonique::mountTree(api, motors, motion);
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
