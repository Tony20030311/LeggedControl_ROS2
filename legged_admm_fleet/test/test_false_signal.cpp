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
int slots_to_detect(double lie, double alpha = 0.2, double gate = 0.5, int max_slots = 40) {
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
        assert(slots_to_detect(0.45) == -1 && "a residual under the gate must never fire");
        assert(slots_to_detect(0.60) > 0 && "a residual over the gate must eventually fire");
    }
    std::cout << "test_false_signal: all cases passed\n";
    return 0;
}
