#include "motophonique/views.h"

namespace motophonique {

namespace {

// Each motor's paths, as string literals the views keep.
#define MOTOPHONIQUE_PATHS(n) \
    {"Motor " #n, "/motors/" #n "/speed", "/motors/" #n "/rpm", "/motors/" #n "/state", "/motors/" #n "/position"}
struct Paths {
    const char* label;
    const char* speed;
    const char* rpm;
    const char* state;
    const char* position;
};
const Paths kPaths[Motors::kMax] = {MOTOPHONIQUE_PATHS(1), MOTOPHONIQUE_PATHS(2), MOTOPHONIQUE_PATHS(3),
                                    MOTOPHONIQUE_PATHS(4), MOTOPHONIQUE_PATHS(5), MOTOPHONIQUE_PATHS(6),
                                    MOTOPHONIQUE_PATHS(7), MOTOPHONIQUE_PATHS(8)};
#undef MOTOPHONIQUE_PATHS

}  // namespace

void addViews(tesserui::Views& views, const Motors& motors) {
    auto& page = views.page("overview", "Motors").icon("motor");
    auto& card = page.card("Motors");
    for (int i = 0; i < motors.count && i < Motors::kMax; i++) {
        card.row()
            .widget("slider", kPaths[i].speed, kPaths[i].label)
            .widget("readout", kPaths[i].rpm, "Speed")
            .widget("readout", kPaths[i].position, "Position")
            .node(kPaths[i].state);
    }
    card.row().node("/motors/stop");
}

}  // namespace motophonique
