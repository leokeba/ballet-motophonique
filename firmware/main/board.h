#pragma once

// The motor controller's board (docs/DESIGN.md section 3): A PLACEHOLDER.
// Leo's boards are classic ESP32 devkits with TMC5160s driven through STEP
// and DIR, whose step generator TesserMotion doesn't have yet; their pins
// aren't known. Until both are, these are Fluante's control board's pins (a
// classic ESP32 with one TMC2209 on its single-wire UART), so the firmware
// builds; don't flash it onto a board with a motor.

namespace board {

// The motors, each a TMC2209 on one UART at its own address (MS1 and MS2:
// 0 to 3; up to four per UART).
constexpr int kMotors = 1;

// TMC2209s on their single-wire UART: UART 2 at 115200 baud.
constexpr int kTmcRx = 26, kTmcTx = 25;
constexpr int kTmcUart = 2;
constexpr int kTmcEnable = 13;  // EN, active low
// MS1 and MS2 set the UART address: both low, 0.
constexpr int kTmcMs1 = 14, kTmcMs2 = 27;
// STEP and DIR, unused: the chip turns the motor by itself (VACTUAL). Held
// low.
constexpr int kTmcStep = 33, kTmcDir = 32;

}  // namespace board
