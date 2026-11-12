# Ballet Motophonique design

Status: phase 0 (2026-11-12): the motor (a signed speed, ramped, the driver's deadman handled inside), the `/motors` tree, the first page and 23 host cases on TesserMotion's fake TMC2209; the firmware builds for a classic ESP32 without its board (§3: not known yet; ESP-IDF 6.1 in CI's image, 1.64 MB, 11 % of the 1.75 MB slot free) and has never run on one. Nothing is pushed: the GitHub repository doesn't exist yet (§8). This is the specification the code follows; when the two disagree, fix one of them. Sections marked *(later)* are committed to but not built yet.

Ballet Motophonique's controller turns stepper motors at speeds that Ballet Photophonique's master plays from a MIDI keyboard and its recorder. It is an application of [TesserKIT](https://github.com/leokeba/TesserKIT) and [TesserMotion](https://github.com/leokeba/TesserMotion) on [TesserAPI](https://github.com/leokeba/TesserAPI) and [TesserUI](https://github.com/leokeba/TesserUI), built like [tiny-heliostat](https://github.com/leokeba/tiny-heliostat) and, for the motor, like [Fluante](https://github.com/leokeba/fluante-firmware)'s spinner.

## 1. Goals

- **A speed per motor, and nothing else to play.** One signed number, in turns per minute: the motor turns that way at that speed, through a ramp, and 0 stops it. A master writes it once; the motor keeps turning until another value comes (the driver's deadman is renewed here, §5.1).
- **Played by Ballet Photophonique's master** (Leo, 2026-11-12): a knob's control change, live or replayed by the master's recorder, mapped to a motor's speed through an ensemble slot (photophonique's DESIGN §5.5, §5.7). Both directions on one knob: a "centred" mapping, 64 stopped (§5.2).
- **An ordinary TesserKIT node.** Wi-Fi, updates, files, accounts, the API over UDP and NowTP are TesserKIT's, so the master finds this controller under its `/peers` and writes to it like any other; the motor drivers are TesserMotion's. Only what is specific to the motors lives here (§9).
- **Safe by default.** Every motor boots stopped, whatever was played before the restart: a speed is never stored. A motor whose master is lost ramps down and stops *(later, §5.3)*.
- **Host-tested.** The motor, its tree and a master's writes run on the host with sanitizers, against TesserMotion's fake TMC2209 (§10).

### Non-goals

- **MIDI.** Nothing of the MIDI side runs here: no key map, no controller mappings, no recorder. The master has them (photophonique DESIGN §13, 2026-11-12); they move to a library only once a second firmware must play MIDI by itself.
- **Positions** (turning to an angle, counting turns) and **several motors in step**: not asked. Timing across controllers is photophonique's §5.8, planned there (§5.4 here).
- Fades, envelopes or sequences on the device: the master's recorder sequences the speeds.

## 2. Glossary

| Term | Meaning |
|---|---|
| Master | The Ballet Photophonique controller whose slots name this one (its `/ensemble/n/device`, this controller's station MAC), and whose controller mappings write its motors' speeds. |
| Speed | A motor's turns per minute, signed, of what it turns (§5.1, `ratio`). 0 stops. |
| Ratio | Turns of what the motor turns per motor turn (pulleys, a gearbox); 1 for the shaft. |
| VACTUAL | The TMC2209's velocity register: microsteps per 2^24 cycles of its 12 MHz clock, which isn't calibrated. |
| Settle | The time at standstill after enabling the driver, before turning: stealthChop tunes itself then. |
| Halted | The driver was disabled underneath the motor (an update, its own tree, a reset of the chip): the motor waits for a speed to be written. |
| Centred mapping | A photophonique controller mapping where 64 writes the middle of its range: `min` −v at 0, `max` +v at 127, a deadband around 64. |

## 3. Hardware

**Not known yet.** Leo sends the boards' details once he finds them (2026-11-12). Until then `firmware/main/board.h` holds Fluante's control board's pins (a classic ESP32, one TMC2209 on UART 2: RX 26, TX 25, EN 13, address 0) so the firmware builds; **it is not to be flashed onto a board with a motor** before `board.h` describes that board.

What the design assumes, to check against the boards (§12):

- A classic ESP32 (or any TesserKIT target) with 4 MB of flash, TesserKIT's partition table with a filesystem.
- **TMC2209s on their single-wire UART**, turning the motors by themselves (VACTUAL), as Fluante's: up to four per UART at addresses 0 to 3. A TMC5160 (SPI) is TesserMotion's too and would need little here (its ramp's fields differ). **Step and direction drivers** (an external driver's STEP/DIR inputs) need a step generator, which is on TesserMotion's roadmap (its §13), not built: asked of TesserMotion's agent then, not written here (§9).
- One or more motors per board (`board::kMotors`, up to 8 in the tree).

## 4. Architecture

Only what is specific to the motors is here; the rest comes from the libraries, pinned in `deps.lock`:

- **TesserKIT**: `Device`, `System` (`/sys/name`), `Network` (Wi-Fi, the access point, mDNS), `Firmware` (uploads, GitHub releases, rollback), `Files` (LittleFS: the settings), `Auth`, `UdpApi` (the API over UDP: the master's way here), `NowTp` (ESP-NOW, off by default: testing, and time later), `Peers` (`/sys/peers`, and when a device is lost), `Log`; TesserAPI's serial transport on the console's UART (the launcher's USB way).
- **TesserMotion**: `Tmc2209` on a `UartPort`, `Motion` (the control task, 50 Hz).
- **TesserUI**: the interface, and `Views` for the first page.

### 4.1 Pieces of this repository

| Piece | Does | Uses |
|---|---|---|
| `Motor` | a TesserMotion `Part`: a signed speed, the start, the ramp, stopping, halts; the speed in rpm (§5.1) | `Tmc2209` |
| `Motors` | the board's motors, Stop all | `Motor` |
| `tree.cpp`, `views.cpp` | the `/motors` and `/drivers` trees, the sections' order and labels, the first page | TesserAPI, TesserMotion's driver trees, TesserUI |

There is no platform code here (TesserMotion and TesserKIT have it); any would go in `src/motophonique/platform/`.

### 4.2 Tasks and locks

- The motors and their drivers run in TesserMotion's control task, under Motion's lock (recursive). The tree's motor nodes take that lock. Each motor is added to Motion before the drivers, so a velocity it sets reaches its chip in the same tick.
- The master's writes arrive in UDP's own task (`udp.ownTask(true)`, as photophonique's nodes) and go through the API lock, then Motion's.

## 5. Behaviour

### 5.1 Motor

- **Speed** (`/motors/n/speed`, rpm, −1000 to 1000, not stored): from stopped, a speed other than 0 waits for the driver to answer (`absent`), enables it, waits `settle` at standstill (`settling`), then turns at the speed through the driver's ramp (`running`). The velocity is renewed every tick, so the driver's deadman (1 s) never stops a turning motor: a speed written once holds until another is written. Changes, the sign included, go through the ramp without stopping on the way.
- **0** asks the driver to hold, which ramps down, then disables it (`stopping`, then `stopped`): the motor turns freely. A speed written while stopping turns again from where the ramp is, with no settle.
- **Limit** (`limit`, stored, 120 rpm by default): a faster speed turns at the limit, either way; the speed shows what was asked. 0 keeps every motor still.
- **The driver's ramp** follows `acceleration` (rpm/s, stored, 60 by default; 0 jumps), written to the driver in µsteps/s² every tick. The driver's velocity is `rpm / 60 / ratio · stepsPerTurn · microsteps` µsteps/s, at the microsteps in effect, so a change of microsteps keeps the speed (TesserMotion 3fc5cf5); `rpm` reads the driver's ramped velocity back. Approximate: the TMC2209's clock isn't calibrated (±5 % or so).
- **The claim:** each motor claims its driver at begin, so `/drivers/n/enabled`, `velocity` and `stop` are readouts, refused with `busy`.
- **Halts:** the driver disabled underneath (Motion's `stopAll()` during an update, the chip reset by a power loss, code) halts the motor; the speed keeps what was asked. Writing a speed, any speed, leaves the halt. A master's mapping writes only when its value changes, so after a halt the motor waits for the knob to move (§12).
- **Stop all** (`/motors/stop`): every speed to 0.
- **At boot** every speed is 0. A reset of the ESP32 alone (its USB port, an update, a crash) leaves the TMC2209 on its supply: a chip found still turning (`/drivers/n/status/found` `turning`) is adopted at its measured speed, still enabled, and ramped down to a stop, unless a speed was written before the first tick; one found `coasting` (moving with its stage off, or its direction unknown) is left disabled for `start/coast` (10 s) before a speed may enable it, so a coasting rotor isn't braked (Fluante's lesson, its §10.7).
- Factory defaults: 60 rpm/s, a 120 rpm limit, ratio 1, 200 steps, a 300 ms settle (stealthChop's automatic tuning wants 130 ms at standstill), 10 s of coasting; the drivers at 256 microsteps (with VACTUAL the chip makes the steps: the smoothest costs nothing) and TesserMotion's defaults (0.5 A run, 0.25 A hold, stealthChop). The currents are the motors', to set once the motors are known (§12).

### 5.2 Played by a master

- **Found:** TesserKIT's `UdpApi` is on, so this controller advertises itself on the LAN and the master mounts it under `/peers` (Ethernet, then NowTP, then Wi-Fi in photophonique's order; `peers.order("udp/ethernet, nowtp")` here too, for the day this controller plays something itself). The master's slot names it by its station MAC (`/sys/info`, or the master's `/peers` list by name).
- **Access:** UDP peers are `User` without a token (TesserKIT's `peerAccess`): they write `speed`, `acceleration`, `limit` and Stop all; `mechanics/` and `start/` are `Admin` and refused (§6). Once a controller has an admin account, the master signs in with its `/sys/peers` account (photophonique §5.7).
- **The writes** are TesserAPI's `RemoteNode::stream()`: never waiting, one in flight per node so they can't be reordered, what waits merged into the next. The latest value wins.
- **A mapping**, on the master (photophonique §5.5): `{cc: 74, slot: 1, path: "/motors/1/speed", min: -60, max: 60, centred: true, deadband: 2}`. No `renew`: the speed holds by itself (§5.1). 7 bits give 63 steps a side: about 0.95 rpm each at ±60. `rest` (written at all notes off and at the recorder's stop) may be `0` to stop the motor then.
- **Persistence:** the speed isn't stored (§13), so a knob moving it never rewrites this controller's settings. A mapping onto a stored node (`acceleration`, `limit`) doesn't either: since TesserAPI 8285ae6 (pinned) streamed sets carry "not saved" (`Query::persist` false), and photophonique's master sends its mappings' sets so; the value stored before comes back at the next boot.

### 5.3 A lost master *(later)*

When the device that last wrote a motor's speed is lost (gone from every transport, or restarted: TesserKIT's `peers.onDeviceLost(device, reason)`, 0.3 s after a goodbye or a restart, 15 s after it falls silent), that motor's speed goes to 0, on its ramp; a speed written without a device (this controller's page, its serial API) is kept. Photophonique does the same for its outputs in application code (`Outputs::trackWriters()`/`release()`, its §5.7); TesserAPI plans it generically (§9): a value's `onWriterLost(fn)`, `api.writerLost(mac)` wired from TesserKIT's `peers.onDeviceLost()`, `api.wrote(node)` for writes made outside a set. Here it will be the speed node's `onWriterLost`, setting the speed to 0; nothing is copied meanwhile. Until it lands a motor keeps turning when its master is lost: **no show with motors before then** (§11).

### 5.4 Timing across controllers *(later)*

Photophonique's DESIGN §5.8 plans how the ensemble plays in time (NowTP's network time, measured first). The motors follow whatever it decides: NowTP is a component of this firmware (off by default), as it is there.

## 6. The tree

Persisted nodes are marked *(p)*. Writes to `/motors` need `User`; TesserKIT's Auth leaves everyone Admin until an admin account exists. UDP peers are `User` (§5.2).

```
/motors                 label "Motors", ui.icon motor
  stop                  action "Stop all": every speed to 0, ui.style danger
  1 … n/                label "Motor n"
    speed               −1000–1000 rpm, step 0.1, watched; not stored: 0 at boot
    rpm                 the speed now, read-only, watched
    state               stopped | absent | settling | running | stopping | halted, watched
    acceleration (p)    0–6000 rpm/s (60)
    limit (p)           0–1000 rpm (120)
    mechanics/ (p)      Admin, collapsed: ratio (0.001–1000, 1), stepsPerTurn (1–1000, 200)
    start/ (p)          Admin, collapsed: settle (0–10000 ms, 300), coast (0–60000 ms, 10000)
/drivers                label "Motor drivers", group "Hardware"
  1 … n/                TesserMotion's TMC2209 tree (config, chopper, pwm, ramp, status, registers, map);
                        claimed by the motor: enabled, velocity and stop are readouts
/motion                 the control loop's statistics, group "Hardware"
/peers                  the controllers nearby (UDP, NowTP)
/sys                    TesserKIT
```

## 7. Interface

TesserUI's interface, served by the device, its first page from `Views` (`views.cpp`): Motors (each motor's speed as a slider, the speed now, its state; Stop all). The rest is TesserUI's tree view.

## 8. Persistence, first flash and updates

- Settings are stored by TesserKIT (LittleFS, `Files`), restored by `Device::begin()`, before Motion starts. The speeds aren't among them.
- The partition table is TesserKIT's `tesserkit_4mb_files.csv`: two 1.75 MB OTA slots, LittleFS, a core dump. A new image rolls back unless it confirms itself.
- Updates from this repository's GitHub releases (`ballet-motophonique-{target}.bin`), checked daily, or uploaded. During an update, `motion.stopAll()` halts every motor.
- **The repository isn't on GitHub yet** (2026-11-12): `leokeba/ballet-motophonique` is assumed. Once Leo creates it, each push to `main` releases (`.github/workflows/release.yml`, as photophonique's: host tests, TesserKIT's `firmware.yml` on ESP-IDF 6.1, the macOS bridge, the launcher on Pages).

## 9. Requests to other layers

A gap goes to the lowest layer, asked of its agent; no workarounds here.

| Request | Layer | For | Status |
|---|---|---|---|
| Streamed sets that aren't saved: `Query::persist` false carried by `RemoteNode::stream()` to the node | TesserAPI | a master's mapping onto a stored node (`acceleration`, `limit`); the speed isn't stored, so it doesn't need it (§5.2) | done, TesserAPI 8285ae6 (pinned; asked 2026-11-12 by ballet-photophonique of tesserapi-5a) |
| What a node does when the device that wrote a value is lost, generic: photophonique's `Outputs::trackWriters()` and `release()` | TesserAPI | a lost master's motors stop (§5.3) | planned by tesserapi-5a (asked 2026-11-12 by ballet-photophonique): `onWriterLost(fn)` on a value or object, `api.writerLost(mac)` from TesserKIT's `peers.onDeviceLost()`, `api.wrote(node)`; not built yet |
| A step and direction generator (RMT or MCPWM), with the drivers' velocity, ramp and deadman | TesserMotion (its §13) | step and direction drivers, if Leo's boards have them (§3) | not asked: waits for the hardware |

## 10. Testing

1. **Host** (`test/`, `cmake -S . -B build && cmake --build build -j && ./build/motophonique_tests`, ASan and UBSan): two motors on TesserMotion's fake TMC2209s. The units (ratio, microsteps); booting stopped, the drivers claimed; the start, the settle and the ramp; a speed outliving five deadman times; reversing through 0 on the ramp without stopping; stopping on the ramp, the stage off; turning again while stopping; the limit; halts on `stopAll()`, on the driver disabled, on a reset of the chip, and leaving them; an absent driver; microsteps changed while turning (16, 1, 256) keeping 60 rpm within 0.1; a chip found turning at boot ramped down to a stop, or on to a speed written before the first tick; a chip found coasting waited for. The tree: declared cleanly, validated, access (`User` for the speed, `Admin` for the mechanics), Stop all, the settings stored and restored and the speed not. A master on an in-memory link (TesserAPI's datagram endpoints, the node at `User` as UdpApi gives): a streamed speed turning a motor for 5 s from one write, then 0 stopping it; a stream to the mechanics refused.
2. **Firmware build** with ESP-IDF 6.1, against clean copies of `deps.lock`'s commits (AGENTS.md). 2026-11-12: in `espressif/idf:release-v6.1` (v6.1-1055) 1,637,040 bytes, 194 KB free in the slot, and the host tests with its GCC 13.3, 23 of 23; on this Mac's 6.1-dev, 1,615,696 bytes.
3. *(later)* **On the board**, with Leo: the driver found, a motor turning at a speed written from the page, then from photophonique's master over UDP; a reset of the ESP32 alone while turning; the master lost.

## 11. Roadmap

| Phase | Content | Acceptance |
|---|---|---|
| 0 *(host-tested)* | The repository, the motor, its tree and page, the firmware without its board | Host tests; the firmware builds with ESP-IDF 6.1 |
| 1 | The board (§3): `board.h`, the drivers Leo's boards have (step and direction through TesserMotion if so), the motors' currents | A motor turning, ramping, reversing and stopping from the page, with Leo |
| 2 | Played by photophonique's master: a centred mapping over UDP (Ethernet or Wi-Fi), live and replayed; a lost master's motors stop (§5.3, §9) | A knob turning a motor both ways; the master unplugged, the motor stopping |
| 3 | Releases and the launcher, once Leo creates the GitHub repository | A release installed from `/sys/firmware/github` |
| 4 *(later)* | Timing across controllers (photophonique §5.8) | As decided there |

## 12. Open questions

For Leo:

- **The boards:** which ESP32, which drivers (TMC2209 on UART, TMC5160 on SPI, or step and direction drivers), how many motors per board, Ethernet or Wi-Fi, the motors (steps per turn, rated current) and their supply, what they turn (a ratio).
- **Units:** rpm of what the motor turns (the shaft by default) is assumed. Turns per second, degrees per second or step rates instead? The tree's ±1000 rpm and the 120 rpm limit and 60 rpm/s are guesses until the motors are known.
- **At speed 0:** the driver off, the motor free (as Fluante, assumed), or held at the hold current? Holding keeps the position against a load and restarts without the settle (300 ms of delay after a knob leaves the middle); free is silent and cool.
- **A lost master:** stop on the ramp (assumed, §5.3), or keep turning?
- **After a halt** (a reset of the motor supply), the motor waits for a speed to be written; the master's knob, unmoved, writes nothing. Turn again at the speed asked once the driver is back instead?
- **A speed at power-up** (Fluante's stored `running`): assumed not wanted; every motor boots stopped.

## 13. Decisions

- **Motors in a firmware of their own** (Leo, 2026-11-12, photophonique DESIGN §13): an ordinary TesserKIT node with TesserMotion's drivers, on other boards, played by photophonique's master through its controller mappings; nothing of the MIDI side here.
- **One signed speed, not stored** (2026-11-12): 0 stops, so a master needs one mapping per motor and no `renew`; never stored, so a motor always boots stopped (TesserAPI 8285ae6 would keep a stored speed's NVS quiet too, but a motor turning at power-up is a choice for Leo, §12). Built on Fluante's spinner (its start, ramp, halts, adoption at boot) with its `running` and `speed` merged.
- **Siblings pinned at their GitHub heads** (2026-11-12): TesserKIT 4dafe71, TesserAPI 8285ae6 (streamed sets not saved, §9), TesserUI e2dcc48, NowTP d0af51b, TesserMotion 8a4f137.
