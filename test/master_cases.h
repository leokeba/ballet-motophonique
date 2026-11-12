// A master playing the motors (docs/DESIGN.md sections 5.2 and 5.3): Ballet
// Photophonique's controller mappings stream the speed through TesserAPI's
// RemoteNode::stream(), here from a bare master over an in-memory link, at
// the access TesserKIT's UdpApi gives peers (User); and a lost master's
// motors stopping.
#pragma once

#include <deque>
#include <string>

#include "rig.h"
#include "tesser/datagram.h"
#include "tesser/remote.h"

namespace {

tesser::PeerAddress mac(uint8_t last) {
    const uint8_t b[6] = {0x48, 0x9d, 0x31, 0xc5, 0xce, last};
    return tesser::PeerAddress::fromBytes(b, sizeof(b));
}

// A master (an Api with the node under /peers) and this controller as its
// node, on one link.
struct MasterRig {
    struct Packet {
        tesser::PeerAddress from, to;
        std::string data;
    };
    std::deque<Packet> wire;
    uint32_t clock = 0;
    tesser::PeerAddress masterMac = mac(0x01), nodeMac = mac(0x48);

    tesser::Api master;
    tesser::DatagramEndpoint masterLink;
    Rig node;
    tesser::DatagramEndpoint nodeLink;

    tesser::DatagramEndpoint::SendFn sender(tesser::PeerAddress from) {
        return [this, from](const tesser::PeerAddress& to, const std::string& m, tesser::Delivery) {
            wire.push_back({from, to, m});
            return true;
        };
    }

    MasterRig()
        : masterLink(&master, tesser::TransportKind::NowTP, sender(masterMac), [this] { return clock; }),
          nodeLink(&node.api, tesser::TransportKind::NowTP, sender(nodeMac), [this] { return clock; }) {
        nodeLink.setPeerAccess(tesser::Access::User);  // as TesserKIT's UdpApi has it
        // As the firmware wires TesserKIT's peers.onDeviceLost().
        nodeLink.onPeerLost([this](const tesser::PeerAddress&, const tesser::PeerAddress& device,
                                   tesser::PeerLoss reason) { node.api.writerLost(device, reason); });
        masterLink.mountPeers(master.object("peers"));
        masterLink.peerSeen(nodeMac, "motors-left", node.api.schemaHash());
        deliver();
    }

    // Delivers what is on the wire, and what that sends, until it settles.
    void deliver() {
        for (int i = 0; i < 20; i++) {
            masterLink.process();
            nodeLink.process();
            while (!wire.empty()) {
                Packet p = wire.front();
                wire.pop_front();
                tesser::DatagramEndpoint& to = p.to == masterMac ? masterLink : nodeLink;
                to.receive(p.from, p.data.data(), p.data.size(), true);
            }
        }
    }
    tesser::RemoteNode* remote() { return tesser::RemoteNode::byDevice(master.object("peers"), nodeMac); }
    tesser::Status stream(const char* path, const char* body) {
        tesser::Status s = remote()->stream(path, body);
        deliver();
        return s;
    }
};

}  // namespace

TEST(master_streams_a_speed_that_needs_no_renewal) {
    MasterRig r;
    CHECK(r.remote() != nullptr);
    // A centred knob at 100 of 127, -60 to 60 rpm (Photophonique's mapping).
    CHECK(r.stream("/motors/1/speed", "34.0") == tesser::Status::Ok);
    CHECK_EQ(r.node.motor[0].speed(), 34.0f);
    r.node.run(5);  // written once: five deadman times later it still turns
    CHECK(r.node.motor[0].state() == motophonique::Motor::State::Running);
    CHECK(near(r.node.motor[0].rpm(), 34, 0.01));
    // The knob back to the middle.
    CHECK(r.stream("/motors/1/speed", "0") == tesser::Status::Ok);
    r.node.run(1);
    CHECK(r.node.motor[0].state() == motophonique::Motor::State::Stopped);
    CHECK(!r.node.driver[0].enabled());
}

TEST(master_cant_reach_a_motors_mechanics) {
    MasterRig r;
    tesser::Status failed = tesser::Status::Ok;
    r.remote()->onStreamError([&failed](const std::string&, tesser::Status s, JsonVariantConst) { failed = s; });
    CHECK(r.stream("/motors/1/mechanics/ratio", "2") == tesser::Status::Ok);  // queued
    CHECK(failed == tesser::Status::Unauthorized);
    CHECK_EQ(r.node.motor[0].settings.ratio, 1.0f);
}

TEST(master_lost_stops_the_motors_it_turned) {
    MasterRig r;
    CHECK(r.stream("/motors/1/speed", "30") == tesser::Status::Ok);
    CHECK(r.node.set("/motors/2/speed", "-20").status == tesser::Status::Ok);  // the node's own page
    r.node.run(2);
    CHECK(r.node.motor[0].state() == motophonique::Motor::State::Running);
    CHECK(r.node.motor[1].state() == motophonique::Motor::State::Running);
    r.nodeLink.forgetPeer(r.masterMac);  // gone: a restart, silence, a goodbye
    r.deliver();
    CHECK_EQ(r.node.get("/motors/1/speed").body, "0");
    CHECK_EQ(r.node.motor[1].speed(), -20.0f);  // not the master's
    r.node.tick();
    CHECK(r.node.motor[0].state() == motophonique::Motor::State::Stopping);  // on its ramp
    r.node.run(1);
    CHECK(r.node.motor[0].state() == motophonique::Motor::State::Stopped);
    CHECK(r.node.motor[1].state() == motophonique::Motor::State::Running);
}

TEST(master_lost_leaves_a_speed_written_after_it) {
    MasterRig r;
    CHECK(r.stream("/motors/1/speed", "30") == tesser::Status::Ok);
    CHECK(r.node.set("/motors/1/speed", "15").status == tesser::Status::Ok);  // taken over locally
    // The master writing a motor's acceleration doesn't make its speed the
    // master's.
    CHECK(r.node.set("/motors/2/speed", "10").status == tesser::Status::Ok);
    CHECK(r.stream("/motors/2/acceleration", "120") == tesser::Status::Ok);
    CHECK_EQ(r.node.motor[1].settings.acceleration, 120.0f);
    r.nodeLink.forgetPeer(r.masterMac);
    r.deliver();
    CHECK_EQ(r.node.motor[0].speed(), 15.0f);
    CHECK_EQ(r.node.motor[1].speed(), 10.0f);
}
