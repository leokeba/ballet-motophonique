// The /motors and /drivers trees (docs/DESIGN.md section 6): validation,
// access, Stop all, persistence.
#pragma once

#include <vector>

#include "rig.h"

using tesser::Status;

TEST(tree_declares_cleanly) {
    Rig r;
    CHECK_EQ(tesser::declarationErrors(), 0);
    CHECK_EQ(r.get("/motors/1/state").body, "\"stopped\"");
    CHECK_EQ(r.get("/motors/2/speed").body, "0");
    CHECK(r.get("/motors/3").status == Status::NotFound);
    CHECK(r.get("/drivers/1/status/present").status == Status::Ok);
    CHECK(r.get("/drivers/2/config/microsteps").status == Status::Ok);
    CHECK(r.get("/motion").status == Status::Ok);
}

TEST(tree_validates) {
    Rig r;
    CHECK(r.set("/motors/1/speed", "1001").status == Status::InvalidValue);
    CHECK(r.set("/motors/1/speed", "-1001").status == Status::InvalidValue);
    CHECK(r.set("/motors/1/acceleration", "-1").status == Status::InvalidValue);
    CHECK(r.set("/motors/1/state", "\"running\"").status != Status::Ok);
    CHECK(r.set("/motors/2/speed", "-12.5").status == Status::Ok);
    CHECK_EQ(r.motor[1].speed(), -12.5f);
    CHECK_EQ(r.motor[0].speed(), 0.0f);
}

TEST(tree_speed_is_users_mechanics_admins) {
    Rig r;
    CHECK(r.set("/motors/1/speed", "10", tesser::Access::User).status == Status::Ok);
    CHECK(r.set("/motors/1/acceleration", "100", tesser::Access::User).status == Status::Ok);
    CHECK(r.set("/motors/1/mechanics/ratio", "2", tesser::Access::User).status == Status::Unauthorized);
    CHECK(r.set("/motors/1/start/settle", "100", tesser::Access::User).status == Status::Unauthorized);
    CHECK(r.set("/motors/1/speed", "10", tesser::Access::Public).status == Status::Unauthorized);
}

TEST(tree_stop_all_stops_every_motor) {
    Rig r;
    CHECK(r.set("/motors/1/speed", "30").status == Status::Ok);
    CHECK(r.set("/motors/2/speed", "-30").status == Status::Ok);
    r.run(2);
    CHECK(r.motor[0].state() == motophonique::Motor::State::Running);
    CHECK(r.motor[1].state() == motophonique::Motor::State::Running);
    CHECK(r.call("/motors/stop").status == Status::Ok);
    CHECK_EQ(r.get("/motors/1/speed").body, "0");
    CHECK_EQ(r.get("/motors/2/speed").body, "0");
    r.run(1);
    CHECK(r.motor[0].state() == motophonique::Motor::State::Holding);
    CHECK(r.motor[1].state() == motophonique::Motor::State::Holding);
}

TEST(tree_persists_settings_not_the_speed) {
    std::string stored;
    {
        Rig r;
        CHECK(r.set("/motors/1", R"({"speed":-90,"acceleration":20,"limit":300})").status == Status::Ok);
        CHECK(r.set("/motors/1/mechanics", R"({"ratio":0.5,"stepsPerTurn":400})").status == Status::Ok);
        CHECK(r.set("/motors/2/start", R"({"settle":250,"coast":5000})").status == Status::Ok);
        CHECK(r.set("/drivers/2/config/runCurrent", "0.8").status == Status::Ok);
        stored = r.api.persistedState();
        CHECK(stored.find("speed") == std::string::npos);
    }
    Rig r;
    std::vector<std::string> skipped;
    CHECK(r.api.restore(stored, &skipped) == Status::Ok);
    CHECK(skipped.empty());
    CHECK_EQ(r.motor[0].speed(), 0.0f);  // a motor boots stopped
    CHECK_EQ(r.motor[0].settings.acceleration, 20.0f);
    CHECK_EQ(r.motor[0].settings.limit, 300.0f);
    CHECK_EQ(r.motor[0].settings.ratio, 0.5f);
    CHECK_EQ(r.motor[0].settings.stepsPerTurn, 400);
    CHECK_EQ(r.motor[1].settings.settleMs, 250u);
    CHECK_EQ(r.motor[1].settings.coastMs, 5000u);
    CHECK(near(r.driver[1].settings.runCurrent, 0.8, 1e-6));
}
