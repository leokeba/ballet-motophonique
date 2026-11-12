#pragma once

// The motor controller's views (docs/DESIGN.md section 7): the page TesserUI
// shows first.

#include "motophonique/tree.h"
#include "tesserui/views.h"

namespace motophonique {

void addViews(tesserui::Views& views, const Motors& motors);

}  // namespace motophonique
