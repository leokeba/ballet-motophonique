# Ballet Motophonique

Firmware for Ballet Motophonique's motor controller: stepper motors turned at a speed that [Ballet Photophonique](https://github.com/leokeba/ballet-photophonique)'s master plays from a MIDI keyboard and its recorder, set from a web interface served by the device itself, or from any [TesserAPI](https://github.com/leokeba/TesserAPI) client.

It is an application of four libraries:

- [TesserKIT](https://github.com/leokeba/TesserKIT): Wi-Fi with a captive portal, OTA updates from this repository's releases, files, users, and the API over UDP and ESP-NOW, under `/sys`.
- [TesserMotion](https://github.com/leokeba/TesserMotion): the TMC2209 drivers and the control loop.
- [TesserAPI](https://github.com/leokeba/TesserAPI) and [TesserUI](https://github.com/leokeba/TesserUI): the device's tree, reachable over HTTP, WebSocket, USB serial and UDP, drawn in the browser without UI code.

This repository holds only the motors: `/motors/1` to `/motors/n`, each with `speed` (rpm, signed: 0 stops; never stored, so every motor boots stopped), `rpm` (the speed now), `state`, `acceleration` and `limit`.

> **Status:** the motor, its tree and host tests; the firmware builds, but its board isn't known yet and it has never run on one. The specification is [docs/DESIGN.md](docs/DESIGN.md).

## Building

```sh
cmake -S . -B build && cmake --build build -j && ./build/motophonique_tests   # host tests (ASan, UBSan)
. ~/esp/esp-idf/export.sh
idf.py -C firmware -B build/idf set-target esp32 build                         # the firmware
```

TesserAPI, TesserUI, TesserKIT, TesserMotion and NowTP are expected next to this repository (`../TesserAPI`, ...), or through `TESSERAPI_PATH`, `TESSERUI_PATH`, `TESSERKIT_PATH`, `TESSERMOTION_PATH` and `NOWTP_PATH`; `deps.lock` pins the commits releases build. A gitignored `firmware/main/secrets.h` may define `WIFI_SSID` and `WIFI_PASSWORD` for development; without it, a fresh controller opens an access point named `ballet-motophonique-xxxx` with a captive portal.

## Playing the motors from Ballet Photophonique

On the master, name this controller in an ensemble slot (`/ensemble/1/device`, its MAC, offered from `/peers` by name), then map a knob to a motor's speed, both ways on one knob:

```json
{"cc": 74, "slot": 1, "path": "/motors/1/speed", "min": -60, "max": 60, "centred": true, "deadband": 2}
```

64 stops the motor, 0 turns it at −60 rpm, 127 at +60. The speed holds until the next value: no `renew` is needed.

## Controlling the motors

```sh
curl -X PUT http://<ip>/api/motors/1/speed -d 30      # 30 rpm, ramped
curl -X PUT http://<ip>/api/motors/stop -d null       # every motor to 0
```
