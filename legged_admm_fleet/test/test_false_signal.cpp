// Experiment C gate: the two filters that stand between a lying peer and the consensus.
//
// Gate 1 (wellFormed) is syntactic and deliberately loose — with n=3, wrongly rejecting one
// good message costs a dog, so the only thing it may ever reject is what a healthy agent could
// not have produced. Gate 2 is the residual against odom, the one channel a compromised
// planning layer does not own.
//
// Both are pure functions on the wire struct, so they are tested here directly rather than
// through a second injection path in the transport: the sender-side injector exists to test the
// SYSTEM (does one lie reach both survivors and get the same verdict), not the predicate.
//
// Plain assert/main like the other gates.
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "agent_core.hpp"
#include "legged_upper_control/admm_constants.hpp"
#include "legged_upper_control/fleet_config.hpp"

using namespace admm;

namespace {

const std::vector<int> kRoster{1, 2, 3};

// A message a healthy agent would emit: walking +x at a steady speed from (x0,y0), with the
// plan's first knot one step ahead of the claimed current state (xi has no k=0).
AgentStateMsg healthy(int id, double x0, double y0, double v = 0.4) {
    AgentStateMsg m;
    m.robot_id = id;
    m.cycle_id = 100;
    m.xnow << x0, y0, v, 0.0;
    m.xibar = Eigen::VectorXd::Zero(XI_DIM);
    for (int k = 1; k <= N; ++k) {
        m.xibar[px_index(k)] = x0 + v * TS * k;
        m.xibar[py_index(k)] = y0;
        m.xibar[vx_index(k)] = v;
    }
    return m;
}

GateLimits limits() {
    GateLimits g;                       // plum: x[-2,20] y[-7,7], the launch defaults
    g.x_min = -2.0; g.x_max = 20.0; g.y_min = -7.0; g.y_max = 7.0;
    return g;
}

// Gate 2 as the node runs it: low-pass the |odom - claimed| residual and report how many slots
// it took to cross the gate. -1 if it never does.
// Must track admm_agent_node's odom_residual_gate default.
constexpr double kGate = 0.30;
// D_MIN minus the contact line: the distance a lie can be spent on before bodies touch.
constexpr double kSafetyBuffer = 1.3 - 0.867;   // 0.433 m

int slots_to_detect(double lie, double alpha = 0.2, double gate = kGate, int max_slots = 40) {
    double R = 0.0;
    for (int s = 1; s <= max_slots; ++s) {
        R = (1.0 - alpha) * R + alpha * lie;
        if (R > gate) return s;
    }
    return -1;
}

}  // namespace

int main() {
    const GateLimits g = limits();

    // 1. the honest message must pass. This is the assertion that matters most: a false
    //    positive here costs a live dog, and the whole fleet is 3.
    {
        for (double x = -1.0; x < 19.0; x += 0.5)
            for (double y : {-6.0, -1.4, 0.0, 1.4, 6.0})
                assert(wellFormed(healthy(1, x, y), kRoster, g) && "healthy message rejected");
    }
    // 2. structural garbage
    {
        AgentStateMsg m = healthy(2, 3.0, 0.0);
        m.xibar = Eigen::VectorXd::Zero(XI_DIM - 1);
        assert(!wellFormed(m, kRoster, g) && "wrong-length xibar accepted");
        AgentStateMsg s = healthy(9, 3.0, 0.0);            // not in the roster
        assert(!wellFormed(s, kRoster, g) && "off-roster id accepted");
    }
    // 3. NaN in either field. A dying agent broadcasts these for real, so this is not only an
    //    attack path — it is the crash case arriving through the same door.
    {
        const double nan = std::nan("");
        AgentStateMsg a = healthy(2, 3.0, 0.0);
        a.xibar[px_index(5)] = nan;
        assert(!wellFormed(a, kRoster, g) && "NaN in xibar accepted");
        AgentStateMsg b = healthy(2, 3.0, 0.0);
        b.xnow[1] = nan;
        assert(!wellFormed(b, kRoster, g) && "NaN in xnow accepted");
    }
    // 4. NO chain check, deliberately, and this pins that decision. xibar is LAST cycle's plan
    //    shifted one step while xnow is a fresh measurement, and a HOLD freezes the plan while
    //    the body walks off the published prefix — honest traffic was measured at 0.597, 0.623,
    //    0.668 and 1.335 m of disagreement. A bound loose enough to accept those sits above the
    //    2 m lie it would exist to catch, so no threshold works. Gate 2 anchors xnow to odom;
    //    xibar's content is self-constrained only. Documented as future work, not as coverage.
    {
        AgentStateMsg m = healthy(2, 3.0, 0.0);
        m.xnow[0] += 1.4;                                   // beyond anything measured honest
        assert(wellFormed(m, kRoster, g)
               && "Gate 1 must NOT judge xnow against xibar — that check false-positives");
    }
    // 5. off-map and teleporting plans
    {
        // Walking OFF the planning map is legitimate and supported — the farhome scenario sends
        // the fleet home to x=-5 against x_min=-2. Rejecting that made two survivors mutually
        // invisible and cost the pairwise CBF (d_0728_075637). Only absurd values may fail.
        for (double x = -5.0; x >= -8.0; x -= 1.0)
            assert(wellFormed(healthy(3, x, 0.0), kRoster, g) && "off-map plan rejected");
        AgentStateMsg far = healthy(2, 3.0, 0.0);
        far.xibar[py_index(7)] = 1e6;
        assert(!wellFormed(far, kRoster, g) && "absurd knot accepted");
        AgentStateMsg jump = healthy(2, 3.0, 0.0);
        for (int k = 6; k <= N; ++k) jump.xibar[px_index(k)] += 3.0;   // 3 m in one 0.1 s step
        assert(!wellFormed(jump, kRoster, g) && "teleporting plan accepted");
    }
    // 5b. the channels a position-only gate never looked at. Each of these passes every check
    //     above AND (except the slot one) Gate 2's odom residual, because the claimed POSITION
    //     is honest — they attack through fields that are consumed elsewhere.
    {
        // cycle_id: keys the mailbox, advances last_seen_, drives the prune window. A peer
        // claiming a far-future slot is never judged silent and erases everyone else's mail.
        AgentStateMsg future = healthy(2, 3.0, 0.0);
        future.cycle_id = 5000;
        assert(wellFormed(future, kRoster, g) && "no clock -> the slot check must be skipped");
        assert(gateCheck(future, kRoster, g, nullptr, /*now_slot=*/100) == GateReason::kSlot
               && "a far-future cycle_id was accepted");
        AgentStateMsg near = healthy(2, 3.0, 0.0);
        near.cycle_id = 104;                                  // ordinary jitter around slot 100
        assert(gateCheck(near, kRoster, g, nullptr, 100) == GateReason::kOk
               && "normal cycle_id jitter rejected");
        // acceleration: the edge CBF linearises around it, so a vacuous bound switches the
        // pairwise barrier off while both gates report clean.
        AgentStateMsg blast = healthy(2, 3.0, 0.0);
        blast.xibar[ax_index(0)] = 1e9;
        assert(gateCheck(blast, kRoster, g) == GateReason::kAccel
               && "an absurd plan acceleration was accepted");
        // claimed velocity: followSpeed brakes on it, so a large negative claim pins a peer at
        // the follow floor forever.
        AgentStateMsg crawl = healthy(2, 3.0, 0.0);
        crawl.xnow[2] = -50.0;
        assert(gateCheck(crawl, kRoster, g) == GateReason::kVel
               && "an impossible claimed velocity was accepted");
    }
    // 5c. TASK 7 ITEM 3: the shared-evidence arrays. Attacker-controlled and deserialised on the
    //     thread that services the consensus barrier -- an impossible payload here must drop the
    //     whole message, the same as every other structural check above.
    {
        // A well-formed evidence array must still pass everything else.
        AgentStateMsg ok = healthy(2, 3.0, 0.0);
        ok.ev_peer = {1, 3};
        ok.ev_pos = {1.0, 0.0, 3.0, 0.0};
        assert(wellFormed(ok, kRoster, g) && "a well-formed evidence array was rejected");

        AgentStateMsg mismatch = healthy(2, 3.0, 0.0);
        mismatch.ev_peer = {1, 3};
        mismatch.ev_pos = {1.0, 0.0};   // one entry short of 2*len(ev_peer)
        assert(gateCheck(mismatch, kRoster, g) == GateReason::kEvidence
               && "ev_pos/ev_peer length mismatch was accepted");

        // More entries than the roster could ever produce (roster is {1,2,3}, sender is 2, so at
        // most 2 OTHER peers -- but the gate itself only bounds against roster SIZE, not against
        // "peers other than the sender", so this pins the coarser, cheap-to-check invariant).
        AgentStateMsg oversized = healthy(2, 3.0, 0.0);
        oversized.ev_peer = {1, 2, 3, 4};
        oversized.ev_pos = std::vector<double>(8, 0.0);
        assert(gateCheck(oversized, kRoster, g) == GateReason::kEvidence
               && "an evidence array larger than the roster was accepted");

        AgentStateMsg off_roster = healthy(2, 3.0, 0.0);
        off_roster.ev_peer = {9};                     // not in kRoster {1,2,3}
        off_roster.ev_pos = {0.0, 0.0};
        assert(gateCheck(off_roster, kRoster, g) == GateReason::kEvidence
               && "an off-roster observed id was accepted");

        AgentStateMsg dup = healthy(2, 3.0, 0.0);
        dup.ev_peer = {1, 1};                          // same peer reported twice
        dup.ev_pos = {1.0, 0.0, 1.5, 0.0};
        assert(gateCheck(dup, kRoster, g) == GateReason::kEvidence
               && "a duplicate observed id was accepted");
    }
    // 6. AND THE POINT OF GATE 2: a consistent lie sails through Gate 1. Shifting the whole
    //    message keeps it well-formed — only an independent observation can catch it.
    {
        AgentStateMsg lie = healthy(1, 4.0, 3.0);
        lie.xnow[0] += 2.0;
        lie.xnow[1] += 2.0;
        for (int k = 1; k <= N; ++k) {
            lie.xibar[px_index(k)] += 2.0;
            lie.xibar[py_index(k)] += 2.0;
        }
        assert(wellFormed(lie, kRoster, g) && "a self-consistent lie should pass Gate 1");
        // ...and Gate 2 must catch it inside the eviction budget. The budget: D_MIN 1.3 minus
        // the 0.867 contact line is 0.433 m of slack, closed at MAX_VX 0.55 m/s = 8 slots. The
        // detector may spend at most 8 - evict_after_lying(3) = 5 of them.
        const double residual = std::sqrt(2.0 * 2.0 + 2.0 * 2.0);   // 2 m in x and y
        const int slots = slots_to_detect(residual);
        assert(slots > 0 && slots <= 5 && "2 m lie must be caught within the eviction budget");
        std::cout << "  2.83 m lie detected in " << slots << " slot(s)\n";
    }
    // 7. the detector must not be so eager that ordinary odom noise trips it. 0.1 m of steady
    //    disagreement (well past anything measured on flat ground) must never cross the gate.
    {
        assert(slots_to_detect(0.10) == -1 && "0.10 m of drift would be treated as a lie");
        assert(slots_to_detect(0.60) > 0 && "a residual over the gate must eventually fire");
        // THE INVARIANT THAT WAS MISSING. This assertion used to read
        //     slots_to_detect(0.45) == -1  // "a residual under the gate must never fire"
        // which pinned the hole in place: 0.45 > kSafetyBuffer, so it demanded silence for a lie
        // that can reach contact on its own. A gate is only sound while it sits below the buffer.
        static_assert(kGate < kSafetyBuffer,
                      "gate >= safety buffer leaves lies that are undetectable AND sufficient "
                      "to close the whole margin (measured: 0.424 m residual, 0 detections, "
                      "0.038 m body gap, 2026-07-30)");
        assert(slots_to_detect(kSafetyBuffer) > 0 && "a lie that can reach contact must fire");
    }
    // 8. THE MEMBERSHIP VOTE (T2/T3). Agents broadcast the roster they still believe in, and the
    //    coordinator uses it to stop handing formation slots to a robot its peers have evicted.
    //    The rule has to survive the attacker holding the pen: robot1 is compromised and says
    //    the fleet is just itself, which excludes both honest dogs at once.
    {
        const std::map<int, std::vector<int>> after_eviction{
            {1, {1}},        // compromised: claims a fleet of one
            {2, {2, 3}},     // honest survivors, agreeing
            {3, {2, 3}},
        };
        assert(majority_excluded(after_eviction, 1)
               && "two honest survivors must be able to evict the liar");
        // The attack this rule exists to stop: one vote must never be enough, or the liar
        // deletes the honest fleet from the coordinator's view and keeps the formation.
        assert(!majority_excluded(after_eviction, 2) && "one attacker must not evict an honest dog");
        assert(!majority_excluded(after_eviction, 3) && "one attacker must not evict an honest dog");
    }
    {
        // A tie is not a majority: with the fleet split 1-1 about robot3, nobody is evicted.
        const std::map<int, std::vector<int>> split{{1, {1, 2}}, {2, {1, 2, 3}}};
        assert(!majority_excluded(split, 3) && "a 1-1 split must not evict");
        // Pre-members senders abstain rather than voting "everyone is in" — otherwise one old
        // peer could dilute a real majority into a tie.
        const std::map<int, std::vector<int>> with_silent{{1, {2, 3}}, {2, {2, 3}}, {3, {}}};
        assert(majority_excluded(with_silent, 1) && "an empty roster must abstain, not vote");
        // Nobody has said anything yet: the healthy startup state, and it must not evict.
        const std::map<int, std::vector<int>> nothing_said{{1, {}}, {2, {}}, {3, {}}};
        assert(!majority_excluded(nothing_said, 1) && "silence is not a vote");
        assert(!majority_excluded({}, 1) && "no views at all is not a vote");
    }
    {
        // T9: a robot the fleet has already fenced must not get a vote. Reproduces the
        // measured stale-vote deadlock (coordinator-stale-vote-deadlock) -- one attacker
        // (1, correctly excluded 3-0 by 2/3/4) plus a single honest misjudgment (3
        // wrongly thinks 2 is dead) used to be enough to evict robot2 as well, because
        // 1's stale ballot against 2 still counted even after 1 itself was fenced.
        const std::map<int, std::vector<int>> attacker_plus_misjudgment{
            {1, {1, 3, 4}},  // attacker: votes against 2
            {2, {2, 4}},     // honest, but wrongly votes against 3
            {3, {3, 4}},     // honest, but wrongly votes against 2 (the misjudgment)
            {4, {2, 3, 4}},  // honest, no mistakes: everyone alive except the attacker
        };
        assert(majority_excluded(attacker_plus_misjudgment, 1)
               && "3 of 3 non-attacker voters exclude the attacker");
        assert(!majority_excluded(attacker_plus_misjudgment, 2)
               && "once the attacker's ballot is dropped, only robot3's mistaken vote "
                  "stands against robot2 out of 2 remaining voters (3,4) -- not a majority");
        assert(!majority_excluded(attacker_plus_misjudgment, 3) && "robot3 is never in majority");
        assert(!majority_excluded(attacker_plus_misjudgment, 4) && "robot4 has no votes against it");
    }
    {
        // T9: the fixed point can take more than one pass. Robot1 is excluded outright
        // (unanimous 2-0 among 2,3). Robot2 looks safe on the raw, unfiltered vote: among
        // senders {1,3}, 1 vouches for it (includes it) and only 3 votes against -- 1 of
        // 2, not a majority. Only once robot1's ballot is dropped (robot1 is itself
        // excluded) does its vouch for robot2 disappear too, leaving robot3 as the ONLY
        // remaining voter; a lone vote against is then a majority of one. The un-fixed
        // function (no ballot-dropping at all) calls robot2 safe -- wrongly.
        const std::map<int, std::vector<int>> two_pass{
            {1, {1, 2}},  // excluded on the first pass (2-0); also vouches for robot2
            {2, {2, 3}},
            {3, {3}},
        };
        assert(majority_excluded(two_pass, 1) && "unanimous 2-0, excluded on the first pass");
        assert(majority_excluded(two_pass, 2)
               && "only excluded once robot1's dropped ballot removes its vouch for robot2 "
                  "-- requires a second pass to see");
        assert(!majority_excluded(two_pass, 3) && "never in majority");
    }
    std::cout << "test_false_signal: all cases passed\n";
    return 0;
}
