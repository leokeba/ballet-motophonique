#pragma once

// The connected classic ESP32 devkit and TMC5160 (docs/DESIGN.md section 3).

namespace board {

// Two TMC5160 channels use the shared SPI bus and enable line. Boards with
// one module leave motor 2 absent. GPIO numbers, not header labels.
constexpr int kMaxMotors = 2;
constexpr int kMotors = 2;
constexpr int kTmcSck = 18, kTmcMosi = 23, kTmcMiso = 19;
constexpr int kTmcEnable = 22;  // EN, active low

struct MotorPins {
    int cs;
    int step;
    int dir;
};

constexpr MotorPins kMotorPins[kMaxMotors] = {
    {5, 21, 17},  // motor 1
    {16, 4, 15},  // motor 2
};

}  // namespace board
