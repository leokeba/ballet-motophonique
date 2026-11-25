# Ballet Motophonique design

Status (2026-11-24): relative degree moves, ping-pong, and stopped-only position re-zero are implemented and host-tested. The firmware now configures two TMC5160 Step/Dir channels; the current connected board has one installed driver, and the next board will use both (§3). The two-channel firmware has not yet been tested on the dual-motor board. This is the specification the code follows; when the two disagree, fix one of them. Sections marked *(later)* are committed to but not built yet.

Ballet Motophonique's controller turns stepper motors at speeds that Ballet Photophonique's master plays from a MIDI keyboard and its recorder. It is an application of [TesserKIT](https://github.com/leokeba/TesserKIT) and [TesserMotion](https://github.com/leokeba/TesserMotion) on [TesserAPI](https://github.com/leokeba/TesserAPI) and [TesserUI](https://github.com/leokeba/TesserUI), built like [tiny-heliostat](https://github.com/leokeba/tiny-heliostat) and, for the motor, like [Fluante](https://github.com/leokeba/fluante-firmware)'s spinner.

## 1. Goals

- **A speed per motor, with finite moves and ping-pong too.** One signed number, in turns per minute, keeps the motor turning through a ramp; 0 stops and holds. A relative move turns a requested number of output degrees, and ping-pong alternates over a total degree span (§5.1).
- **Played by Ballet Photophonique's master** (Leo, 2026-11-12): a knob's control change, live or replayed by the master's recorder, mapped to a motor's speed through an ensemble slot (photophonique's DESIGN §5.5, §5.7). Both directions on one knob: a "centred" mapping, 64 stopped (§5.2).
- **An ordinary TesserKIT node.** Wi-Fi, updates, files, accounts, the API over UDP and NowTP are TesserKIT's, so the master finds this controller under its `/peers` and writes to it like any other; the motor drivers are TesserMotion's. Only what is specific to the motors lives here (§9).
- **Safe by default.** Every motor boots stopped, whatever was played before the restart: a speed is never stored. A motor whose master is lost ramps down and stops (§5.3).
- **Host-tested.** The motor, its tree and a master's writes run on the host with sanitizers, against TesserMotion's fake TMC2209 (§10).

### Non-goals

- **MIDI.** Nothing of the MIDI side runs here: no key map, no controller mappings, no recorder. The master has them (photophonique DESIGN §13, 2026-11-12); they move to a library only once a second firmware must play MIDI by itself.
- **Absolute position and homing.** Finite-move position is open-loop, counted from boot, and cannot detect missed steps; there is no encoder or homing.
- Fades, envelopes or sequences on the device: the master's recorder sequences the speeds.

## 2. Glossary

| Term | Meaning |
|---|---|
| Master | The Ballet Photophonique controller whose slots name this one (its `/ensemble/n/device`, this controller's station MAC), and whose controller mappings write its motors' speeds. |
| Speed | A motor's turns per minute, signed, of what it turns (§5.1, `ratio`). 0 stops and holds. |
| Commanded position | An open-loop count from boot, converted to output degrees; it is trustworthy only after finite moves, not after untracked continuous motion or an uncertain interruption. |
| Holding | At speed 0, the driver enabled at its hold current: the motor keeps its position and turns again without settling. |
| Step and direction | A driver whose STEP input advances the motor a microstep per pulse, DIR choosing the way: the pulses come from the ESP32, not from the driver's own motion controller. |
| Ratio | Turns of what the motor turns per motor turn (pulleys, a gearbox); 1 for the shaft. |
| VACTUAL | The TMC2209's velocity register: microsteps per 2^24 cycles of its 12 MHz clock, which isn't calibrated. |
| Settle | The time at standstill after enabling the driver, before turning: stealthChop tunes itself then. |
| Halted | The driver was disabled underneath the motor (an update, its own tree, a reset of the chip): the motor waits for a speed to be written. |
| Centred mapping | A photophonique controller mapping where 64 writes the middle of its range: `min` −v at 0, `max` +v at 127, a deadband around 64. |

## 3. Hardware

Leo, 2026-11-12: **classic ESP32 devkits** (4 MB, no Ethernet: Wi-Fi), and **TMC5160 drivers driven through their STEP pin**: on these modules the driver's own motion controller (its velocity ramp, which TesserMotion's `Tmc5160` drives over SPI) isn't usable, so the ESP32 makes the steps and DIR the direction. TesserMotion's `Tmc5160` handles a chip strapped for step and direction (SD_MODE high, read in `IOIN`): configured and read over SPI, its velocity refused. Its RMT step generator and pulse-output diagnostics are integrated from TesserMotion ba87f6e. A pre-fix hardware test accepted speed but StepDir velocity stayed at 0; TesserMotion fixed a same-tick deadman timestamp underflow in ba87f6e. Post-fix hardware motion verification is pending.

Pins (Leo, 2026-11-15; confirmed 2026-11-19): SPI SCK GPIO18, MOSI GPIO23, MISO GPIO19; CS1 GPIO5; STEP1 GPIO21, DIR1 GPIO17 (TX2); shared EN GPIO22. For the second driver, Leo selected CS2 GPIO16 (RX2), STEP2 GPIO4, DIR2 GPIO15 on 2026-11-24. The firmware exposes both channels; a single-driver board leaves motor 2 absent. SPI is wired. The motor is reported as 3.0 A; verify each driver's sense resistor and each motor's current rating before increasing the factory 0.5 A run current. Both motors are 200 full steps, with no gearing.

`firmware/main/board.h` describes the two GPIO sets. The app pairs each TMC5160 with its own RMT output and TesserMotion `StepDirMotor`; the pulse readiness and rejected start count are exposed on each motor's tree. Both channels share SPI and EN. On a single-driver board, motor 2 remains in the tree but cannot turn without its TMC5160.

## 4. Architecture

Only what is specific to the motors is here; the rest comes from the libraries, pinned in `deps.lock`:

- **TesserKIT**: `Device`, `System` (`/sys/name`), `Network` (Wi-Fi, the access point, mDNS), `Firmware` (uploads, GitHub releases, rollback), `Files` (LittleFS: the settings), `Auth`, `UdpApi` (the API over UDP: the master's way here), `NowTp` (ESP-NOW, on by default — the primary way Leo configures the boards, 2026-11-25 — testing, and time later), `Peers` (`/sys/peers`, and when a device is lost), `Log`; TesserAPI's serial transport on the console's UART (the launcher's USB way).
- **TesserMotion**: `Motion` (the control task, 50 Hz); on the board `Tmc5160` on an `SpiBus` plus `StepDirMotor` on RMT, with the TMC5160 strapped for STEP/DIR.
- **TesserUI**: the interface, and `Views` for the first page.

### 4.1 Pieces of this repository

| Piece | Does | Uses |
|---|---|---|
| `Motor` | a TesserMotion `Part`: a signed speed, the start, the ramp, stopping and holding, halts; the speed in rpm (§5.1) | `MotionDrive` (TesserMotion e4e12d6: a TMC2209 or a step and direction motor, §3), and the chip behind it for adoption (null for step and direction) |
| `Motors` | the board's motors, Stop all | `Motor` |
| `tree.cpp`, `views.cpp` | the `/motors` and `/drivers` trees, the sections' order and labels, the first page | TesserAPI, TesserMotion's driver trees, TesserUI |

There is no platform code here (TesserMotion and TesserKIT have it); any would go in `src/motophonique/platform/`.

### 4.2 Tasks and locks

- The motors and their drivers run in TesserMotion's control task, under Motion's lock (recursive). The tree's motor nodes take that lock. Each motor is added to Motion before the drivers, so a velocity it sets reaches its chip in the same tick.
- The master's writes arrive in UDP's own task (`udp.ownTask(true)`, as photophonique's nodes) and go through the API lock, then Motion's.

## 5. Behaviour

### 5.1 Motor

- **Speed** (`/motors/n/speed`, rpm, −1000 to 1000, not stored): from stopped (the driver off, as at boot), a speed other than 0 waits for the driver to answer (`absent`), enables it, waits `settle` at standstill (`settling`), then turns at the speed through the driver's ramp (`running`). The velocity is renewed every tick, so the driver's deadman (1 s) never stops a turning motor: a speed written once holds until another is written. Changes, the sign included, go through the ramp without stopping on the way.
- **0 holds** (Leo, 2026-11-12): the driver ramps down and stays enabled at standstill (`stopping`, then `holding`), at its hold current, so the motor keeps its position against a load. A speed written while stopping or holding turns again at once, from where the ramp is, with no settle; 0 written while settling holds too. The driver is off only at boot (`stopped`) and after a halt: once a motor has been asked to turn, it holds until the next reset or halt.
- **Limit** (`limit`, stored, 120 rpm by default; Leo, 2026-11-12: the defaults are fine): a faster speed turns at the limit, either way; the speed shows what was asked. 0 keeps the motor still (holding).
- **The driver's ramp** follows `acceleration` (rpm/s, stored, 60 by default; 0 jumps), written to the driver in µsteps/s² every tick. The driver's velocity is `rpm / 60 / ratio · stepsPerTurn · microsteps` µsteps/s, at the microsteps in effect, so a change of microsteps keeps the speed (TesserMotion 3fc5cf5); `rpm` reads the driver's ramped velocity back. Approximate: the TMC2209's clock isn't calibrated (±5 % or so).
- **Relative move** (`/motors/n/move`): an action with `degrees`, `maxSpeed` (rpm) and `acceleration` (rpm/s). Degrees are output rotation; the controller converts them to the driver's current microstep resolution and asks TesserMotion for an exact finite move with the same acceleration and deceleration. `maxSpeed` is capped by the motor's stored `limit`; lowering that limit during a move cancels it with a controlled deceleration. The driver reports a commanded position in canonical 1/256-full-step units, so changing microsteps between moves preserves position. It is an open-loop count from boot, not a sensor reading; continuous speed mode or an interruption that loses the pulse count marks it invalid, and later degree moves are refused.
- **Ping-pong** (`/motors/n/pingPong`): `span` is the full endpoint-to-endpoint distance in output degrees. It starts at the current position, moves by that span, then alternates between endpoints at the requested max speed and acceleration until `/motors/stop` or a speed write cancels it. Cancellation decelerates and drains queued pulses; a master loss cancels it through the tracked speed node. It needs a valid open-loop position and step/direction output.
- **Re-zero** (`/motors/n/zeroPosition`, Admin): while the step/direction motor is held at zero speed, this action declares its current physical position to be 0°. It does not move or home the motor; the operator must first place it at the intended reference. This restores position validity after an untracked move, but cannot verify that the physical reference is correct.
- **The claim:** each motor claims its driver at begin, so `/drivers/n/enabled`, `velocity` and `stop` are readouts, refused with `busy`.
- **Step and direction drives** (TesserMotion e4e12d6, its §4.9): no found state, so a reset of such a drive is treated as coasting: the motor waits `start/coast` (10 s, Leo's choice, 2026-11-15) before a speed may enable it, and never adopts a rotor. `/motors/n/pulseOutputReady` reports RMT setup success and `/motors/n/pulseStartFailures` counts rejected pulse starts; RMT setup, transmit and stop errors are also logged.
- **Halts:** the driver disabled underneath (Motion's `stopAll()` during an update, the chip reset by a power loss, code) halts the motor; the speed keeps what was asked. Writing a speed, any speed, leaves the halt. No resume (Leo, 2026-11-12): a master's mapping writes only when its value changes, so after a halt the motor waits for the knob to move.
- **Stop all** (`/motors/stop`): every speed to 0, each motor holding.
- **At boot** every speed is 0. A reset of the ESP32 alone (its USB port, an update, a crash) leaves the TMC2209 on its supply: a chip found still turning (`/drivers/n/status/found` `turning`) is adopted at its measured speed, still enabled, and ramped down to a stop and held, unless a speed was written before the first tick; one found `coasting` (moving with its stage off, or its direction unknown) is left disabled for `start/coast` (10 s) before a speed may enable it, so a coasting rotor isn't braked (Fluante's lesson, its §10.7).
- Factory defaults: 60 rpm/s, a 120 rpm limit, ratio 1, 200 steps, a 300 ms settle (stealthChop's automatic tuning wants 130 ms at standstill), 10 s of coasting; the placeholder TMC2209 at 256 microsteps (with VACTUAL the chip makes the steps: the smoothest costs nothing) and TesserMotion's defaults (0.5 A run, 0.25 A hold, stealthChop). With step and direction the microsteps set the pulse rate: 120 rpm of a 200-step motor is 102 400 pulses/s at 256, 6 400 at 16 (the TMC5160 interpolates to 256 either way); the choice is the step generator's limits' (§9). The currents are the motors', to set once the motors are known (§12): the hold current is what holding costs in heat.

### 5.2 Played by a master

- **Found:** TesserKIT's `UdpApi` is on, so this controller advertises itself on the LAN and the master mounts it under `/peers` (Ethernet, then NowTP, then Wi-Fi in photophonique's order; `peers.order("udp/ethernet, nowtp")` here too, for the day this controller plays something itself). The master's slot names it by its station MAC (`/sys/info`, or the master's `/peers` list by name). **Its group:** since TesserKIT 7425872 discovery has groups, and a controller mounts under `/peers` only the peers sharing one of its own (or declared in `/sys/peers/nodes`); one advertising none is left out. This firmware sets `peers.group("ballet")`, as photophonique's master does (its b910627): the factory default of `/sys/peers/group` (`Admin`, persisted; empty mounts every node), applied to UdpApi and NowTP at once and advertised as `group=` in mDNS. Controllers on a release from before 2026-11-13 advertise no group, so a show's boards are updated together, or the old ones declared in the master's `/sys/peers/nodes`. **Its grace:** `peers.grace(6000)`, as photophonique's (its ballet-photophonique-9d request, 2026-11-14): over UDP a lost master's route waits for its 5 s probe before its writes go again, so a short loss of the master doesn't stop a motor (§5.3 stops it only after the grace).
- **Access:** UDP peers are `User` without a token (TesserKIT's `peerAccess`): they write `speed`, `acceleration`, `limit` and Stop all; `mechanics/` and `start/` are `Admin` and refused (§6). Once a controller has an admin account, the master signs in with its `/sys/peers` account (photophonique §5.7).
- **The writes** are TesserAPI's `RemoteNode::stream()`: never waiting, one in flight per node so they can't be reordered, what waits merged into the next. The latest value wins.
- **A mapping**, on the master (photophonique §5.5): `{cc: 74, slot: 1, path: "/motors/1/speed", min: -60, max: 60, centred: true, deadband: 2}`. No `renew`: the speed holds by itself (§5.1). 7 bits give 63 steps a side: about 0.95 rpm each at ±60. `rest` (written at all notes off and at the recorder's stop) may be `0` to stop the motor then.
- **Persistence:** the speed isn't stored (§13), so a knob moving it never rewrites this controller's settings. A mapping onto a stored node (`acceleration`, `limit`) doesn't either: since TesserAPI 8285ae6 (pinned) streamed sets carry "not saved" (`Query::persist` false), and photophonique's master sends its mappings' sets so; the value stored before comes back at the next boot.

### 5.3 A lost master

When the device that last wrote a motor's speed is lost (gone from every transport, or restarted: TesserKIT's `peers.onDeviceLost(device, reason)`, 0.3 s after a goodbye or a restart, 15 s after it falls silent), that motor's speed goes to 0, on its ramp, and it holds; a speed written without a device (this controller's page, its serial API) is kept. TesserAPI does it (5a991e4, its §14.4; §9): `mountTree()` tracks each speed node's writer (`api.trackWriter()`, the device of the client whose set last changed it, none for a client without one), and the firmware wires TesserKIT's `peers.onDeviceLost()` to `api.writerLost()`, which calls the speed's release, under the API lock, once: the speed to 0, announced. Only the speed is tracked, so a master writing a motor's `acceleration` doesn't make its speed the master's. Stop all is an action and claims nothing; a speed of 0 released again is harmless. Photophonique's outputs are on the same mechanism (its §5.7). Not yet tried with two boards (§10).

### 5.4 Timing across controllers *(later)*

Photophonique's DESIGN §5.8 plans how the ensemble plays in time (NowTP's network time, measured first). The motors follow whatever it decides: NowTP is a component of this firmware (on by default), as it is there.

## 6. The tree

Persisted nodes are marked *(p)*. Writes to `/motors` need `User`, except `zeroPosition`, which needs `Admin`; TesserKIT's Auth leaves everyone Admin until an admin account exists. UDP peers are `User` (§5.2).

```
/motors                 label "Motors", ui.icon motor
  stop                  action "Stop all": every speed to 0 (holding), ui.style danger
  1 … n/                label "Motor n"
    speed               −1000–1000 rpm, step 0.1, watched; not stored: 0 at boot; 0 holds
    rpm                 the speed now, read-only, watched
    position            open-loop commanded output degrees relative to boot; read-only, watched
    positionValid       whether the open-loop position is still trustworthy; read-only, watched
    move                action: relative degrees, maxSpeed rpm, acceleration rpm/s
    pingPong            action: full endpoint span in degrees, maxSpeed rpm, acceleration rpm/s
    hold                action "Stop": to 0 — ends a move or ping-pong, ramps down and holds
    zeroPosition        Admin action: assign the held shaft's current position to 0°
    state               stopped | absent | settling | running | stopping | holding | moving | pingPong | halted, watched
    acceleration (p)    0–6000 rpm/s (60)
    limit (p)           0–1000 rpm (120)
    pulseOutputReady    whether the step pulse output initialized, read-only
    pulseStartFailures  number of rejected pulse starts, read-only
    mechanics/ (p)      Admin, collapsed: ratio (0.001–1000, 1), stepsPerTurn (1–1000, 200)
    start/ (p)          Admin, collapsed: settle (0–10000 ms, 300), coast (0–60000 ms, 10000)
/drivers                label "Motor drivers", group "Hardware"
  1 … n/                TesserMotion's driver tree: config, chopper, pwm, ramp, status, registers and map;
                        claimed by the motor at begin(): enabled, velocity, target and stop are readouts, the ramp hidden
/motion                 the control loop's statistics, group "Hardware"
/peers                  the controllers nearby (UDP, NowTP)
/sys                    TesserKIT
```

## 7. Interface

TesserUI's interface, served by the device, its first page from `Views` (`views.cpp`): Motors (each motor's speed as a slider, the speed and position now, its state; Stop all). The rest is TesserUI's tree view.

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
| What a node does when the device that wrote a value is lost, generic: photophonique's `Outputs::trackWriters()` and `release()` | TesserAPI | a lost master's motors stop (§5.3) | done, TesserAPI 5a991e4 (pinned; asked 2026-11-12 by ballet-photophonique of tesserapi-5a): `api.trackWriter(node, release)`, `api.wrote(node)`, `api.writerLost(device, reason)` from TesserKIT's `peers.onDeviceLost()` |
| Discovery groups: a controller mounting only its project's peers, so the master's `/peers` and slots hold the show's controllers alone | TesserAPI, TesserKIT | the master finding these controllers once its own discovery has a group (§5.2) | done, TesserAPI 9073227, TesserKIT 3d8d2fc (pinned 2026-11-14, as photophonique's; before: 1334b5f, 8061847; asked 2026-11-12 by ballet-photophonique-6e of tesserapi-5a and tesserkit-80; this firmware's `peers.group("ballet")` at ballet-photophonique-74's ask, 2026-11-13) |
| A step and direction generator for the classic ESP32: pulses on STEP at a signed velocity in µsteps/s, DIR set before the first pulse of each way, with the `MotionDrive` interface and RMT output; pulse readiness, rejected starts and RMT errors are observable | TesserMotion §13, ba87f6e | Leo's boards (§3): each motor | integrated in Ballet Motophonique; post-fix hardware motion verification pending |
| Exact relative finite moves for step/dir pulses, with observable commanded position and completion; canonical position across microstep-resolution changes, graceful deceleration/drain on cancellation, and invalidation only when a hard interruption makes emitted steps unknowable | TesserMotion §13 | a degrees-based relative move and a ping-pong mode in Ballet Motophonique: total endpoint span, speed in rpm, acceleration in rpm/s; open-loop position resets at boot and cannot detect missed steps | done, TesserMotion 42de010 (pinned; moves d8757fd and stopped zeroing f78f64d, asked 2026-11-24) |
| An action's params carry each field's default in the schema (`default`, the value in the default-constructed argument), so a form starts from what the device would use | TesserAPI §4.2, TesserUI | the move and pingPong forms offered at their defaults (90°, 30 rpm, 60 rpm/s) instead of empty inputs (§6) | done, TesserAPI 8b6294a and TesserUI e90523c (pinned; asked 2026-11-25 in the motor/driver UI review) |
| A tree's claim visible to the UI: a watched `claimed` boolean, its claim-checked controls hinted disabled while it's true | TesserMotion §4.3, §9 | a page seeing a claimed driver's or axis's controls grey out instead of failing on write (§6) | done, TesserMotion 42de010 (pinned; asked 2026-11-25 in the motor/driver UI review) |

## 10. Testing

1. **Host** (`test/`, `cmake -S . -B build && cmake --build build -j && ./build/motophonique_tests`, ASan and UBSan): two motors on TesserMotion's fake TMC2209s. The units (ratio, microsteps); booting stopped, the drivers claimed; the start, the settle and the ramp; a speed outliving five deadman times; reversing through 0 on the ramp without stopping; stopping on the ramp, then holding (enabled past the deadman); turning again while stopping, and from holding without a settle; 0 while settling holding; the limit; halts on `stopAll()`, on the driver disabled, on a reset of the chip, and leaving them; an absent driver; microsteps changed while turning (16, 1, 256) keeping 60 rpm within 0.1; a chip found turning at boot ramped down to a stop, or on to a speed written before the first tick; a chip found coasting waited for. The tree: declared cleanly, validated, access (`User` for the speed, `Admin` for the mechanics), Stop all, the settings stored and restored and the speed not. A master on an in-memory link (TesserAPI's datagram endpoints, the node at `User` as UdpApi gives): a streamed speed turning a motor for 5 s from one write, then 0 stopping it; a stream to the mechanics refused; the master lost (`forgetPeer`, the endpoint's `onPeerLost` wired to `api.writerLost()` as the firmware wires TesserKIT's): the motor it turned stops on its ramp, while a speed the node's own page wrote, or wrote after the master, is kept, and a master writing a motor's acceleration doesn't take its speed. A host node (`test/demo_node.cpp`, built as `motophonique_demo`) serves the same tree over stdin/stdout, so the UI can be driven and screenshotted without a board through TesserUI's `web/scripts/device.mjs --exe`.
2. **Firmware build** with ESP-IDF 6.1, against clean copies of `deps.lock`'s commits (AGENTS.md). 2026-11-12: in `espressif/idf:release-v6.1` (v6.1-1055) 1,637,040 bytes, 194 KB free in the slot, and the host tests with its GCC 13.3, 23 of 23; on this Mac's 6.1-dev, 1,615,696 bytes. With the lost master (TesserAPI 5a991e4): 1,638,352 bytes in the image, 192 KB free; GCC 13.3, 25 of 25. With holding at 0: GCC 13.3, 27 of 27, the image builds. With the group "ballet" (TesserKIT 8061847, TesserAPI 1334b5f, TesserUI 67286b1, 2026-11-13): GCC 13.3, 27 of 27, the image 1,644,288 bytes (186 KB free, 5.9 KB more than with the lost master). The finite-move and ping-pong commands are host-tested against a fake TMC5160 and pulse output, with a hold ending a move (ramped down to its position, holding there, the driver still enabled) and the step/dir motor claiming its chip (its tree's controls refused); the board has not been flashed or moved. With a motor's hold and its claimed chip (TesserAPI 8b6294a, TesserUI e90523c, TesserMotion 42de010, 2026-11-25): GCC 12.2, 38 of 38, the image 1,723,968 bytes (108 KB free).
3. *(later)* **On the board**, with Leo: the master's `/peers` listing it with the group set, and leaving out a controller in another group; the TMC5160 found over SPI (if wired), the step generator's pulses on a scope, a motor turning at a speed written from the page, holding at 0, then from photophonique's master over UDP; a reset of the ESP32 alone while turning; the master lost.

## 11. Roadmap

| Phase | Content | Acceptance |
|---|---|---|
| 0 *(host-tested)* | The repository, the motor, its tree and page, the firmware without its board | Host tests; the firmware builds with ESP-IDF 6.1 |
| 1 | The board (§3): `board.h`, the TMC5160s on TesserMotion's step generator (§9), the motors' currents | A motor turning, ramping, reversing, stopping and holding from the page, with Leo |
| 2 *(the lost master host-tested)* | Played by photophonique's master: a centred mapping over UDP (Ethernet or Wi-Fi), live and replayed; a lost master's motors stop (§5.3, §9) | A knob turning a motor both ways; the master unplugged, the motor stopping |
| 3 | Releases and the launcher, once Leo creates the GitHub repository | A release installed from `/sys/firmware/github` |
| 4 *(later)* | Timing across controllers (photophonique §5.8) | As decided there |

## 12. Open questions

For Leo:

- **The TMC5160 modules' SPI:** wired to the ESP32 (SCK, SDI, SDO, CS each), so TesserMotion configures and reads them (currents, microsteps, stealthChop, faults, the chip found turning at a reset), or not (the module's pins and potentiometer, and the EN pin only)? Which module (its sense resistor: 0.075 Ω on most)?
- **The pins and the count:** motors per devkit; each one's STEP, DIR, EN (and CS); the SPI bus's pins.
- **The motors:** steps per turn, rated current, the supply's voltage; what they turn (a ratio), if not the shaft.
- **Holding's heat:** the hold current (TesserMotion's default 0.25 A, or the motor's) is drawn whenever a motor has turned since boot. A way to let go without a halt (an action, or holding only for a while) if that proves too warm?
- **A lost master:** stop and hold (assumed, built, §5.3), or keep turning?

## 13. Decisions

- **Motors in a firmware of their own** (Leo, 2026-11-12, photophonique DESIGN §13): an ordinary TesserKIT node with TesserMotion's drivers, on other boards, played by photophonique's master through its controller mappings; nothing of the MIDI side here.
- **One signed speed, not stored** (2026-11-12): 0 stops, so a master needs one mapping per motor and no `renew`; never stored, so a motor always boots stopped (TesserAPI 8285ae6 would keep a stored speed's NVS quiet too, but Leo wants no speed at power-up). Built on Fluante's spinner (its start, ramp, halts, adoption at boot) with its `running` and `speed` merged.
- **Leo's answers** (2026-11-12): the boards are classic ESP32 devkits with TMC5160s driven through STEP and DIR (their motion controller not usable), so the step generator is TesserMotion's to build (§9); the limits and defaults are fine (rpm, ±1000, 120 rpm, 60 rpm/s); speed 0 holds (§5.1); no resume after a halt; no speed at power-up (every motor boots stopped, the driver off).
- **Motor on `MotionDrive`** (2026-11-15): `Motor` takes the base, so the same motor runs on a TMC2209 or a step and direction drive; the boot rule for step and direction is a time wait (`start/coast`), agreed with TesserMotion and Leo, no sensor and no standstill call (its stst is the chip's own step timing, not the rotor's)
- **Siblings pinned at their GitHub heads** (2026-11-12): TesserKIT 4dafe71, TesserAPI 5a991e4 (streamed sets not saved since 8285ae6; a lost writer's release, §9), TesserUI e2dcc48, NowTP d0af51b, TesserMotion 8a4f137.
- **Discovery group "ballet"** (2026-11-13, at ballet-photophonique-74's ask; Leo's ask of TesserKIT 7425872): this firmware advertises and mounts the group `ballet`, as photophonique's master does, or the master leaves it out of `/peers` and its slots (§5.2). Siblings pinned at photophonique's: TesserKIT 8061847, TesserAPI 1334b5f, TesserUI 67286b1 (ahead of TesserKIT's a08af93), NowTP d0af51b; TesserMotion stays at 8a4f137.
