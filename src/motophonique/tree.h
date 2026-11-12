#pragma once

// The application's trees (docs/DESIGN.md section 6), found by
// argument-dependent lookup: api.mount("motors", motors).

#include "motophonique/motor.h"
#include "tesser/api.h"
#include "tesser/node.h"
#include "tessermotion/motion.h"

namespace motophonique {

// The board's motors, /motors/1 to /motors/<count>.
struct Motors {
    static constexpr int kMax = 8;
    Motor* items;
    int count;

    // Every speed to 0: each motor ramps down and lets go.
    void stopAll();
};

void describe(tesser::Object& o, Motor& motor);
void describe(tesser::Object& o, Motors& motors);

// The device's sections, in menu order, with their labels, icons and menu
// groups: /motors, then each motor's driver (/drivers/n) and the control
// loop's statistics (/motion) under "Hardware"; each speed's writer
// tracked, so a lost master's motors stop (docs/DESIGN.md section 5.3:
// call api.writerLost() when a device is lost). Before Device::begin().
void mountTree(tesser::Api& api, Motors& motors, tessermotion::Motion& motion);

}  // namespace motophonique
