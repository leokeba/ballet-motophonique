#pragma once

// The connected classic ESP32 devkit and TMC5160 (docs/DESIGN.md section 3).

namespace board {

// One TMC5160 module is installed and wired for SPI configuration and
// step/direction control. GPIO numbers, not header labels.
constexpr int kMotors = 1;
constexpr int kTmcSck = 18, kTmcMosi = 23, kTmcMiso = 19;
constexpr int kTmcCs = 5;
constexpr int kTmcEnable = 22;  // EN, active low
constexpr int kTmcStep = 21, kTmcDir = 17;  // DIR is TX2

}  // namespace board
