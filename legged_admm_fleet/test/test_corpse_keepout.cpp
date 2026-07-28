// Corpse keep-out geometry gate (admm::corpse_keepout).
//
// This is the only piece of the failure path whose trigger conditions cannot be produced on
// demand in Gazebo: a peer has to die in the same cycle its QP produced garbage. It ran clean
// through ten live runs without once being exercised, which is exactly why it is pinned here.
// Plain assert/main like the other gates.
#include <cassert>
#include <cmath>
#include <iostream>

#include "legged_upper_control/admm_constants.hpp"
#include "legged_upper_control/admm_node_qp.hpp"
#include "legged_upper_control/fleet_config.hpp"

using namespace admm;

namespace {
// A well-formed xibar whose ANCHOR knot (K_SEND — the last step the lower layer was actually
// given) is (x,y). The terminal knot is filled with the same value so these cases stay about
// the fallback logic rather than about which knot is read; case 1 is where that distinction
// is pinned.
Eigen::VectorXd plan_at(double x, double y) {
    Eigen::VectorXd v = Eigen::VectorXd::Zero(py_index(N) + 1);
    v[px_index(K_SEND)] = x;
    v[py_index(K_SEND)] = y;
    v[px_index(N)] = x;
    v[py_index(N)] = y;
    return v;
}
const double kMargin = 0.60;
}  // namespace

int main() {
    // 1. the plan predicts where the body coasts to after the agent dies — but at knot K_SEND,
    //    not the terminal knot. The lower layer only ever received K_SEND steps and parks when
    //    it runs out, so the terminal knot overshoots by (N-K_SEND)*TS*v. Measured over 16
    //    evictions: 0.385 m mean against a derived 0.40 m, which put the real body that much
    //    closer to the survivors than the keep-out believed.
    {
        Eigen::VectorXd v = Eigen::VectorXd::Zero(py_index(N) + 1);
        v[px_index(K_SEND)] = 3.0; v[py_index(K_SEND)] = 4.0;   // where it actually stops
        v[px_index(N)] = 5.0;      v[py_index(N)] = 6.0;        // where the plan ran on to
        const auto k = corpse_keepout(Eigen::Vector4d(1.0, 2.0, 0, 0), v, kMargin);
        assert(k && std::abs(k->pos.x() - 3.0) < 1e-12 && std::abs(k->pos.y() - 4.0) < 1e-12);
        const auto old = corpse_keepout(Eigen::Vector4d(1.0, 2.0, 0, 0), v, kMargin,
                                        /*mobile=*/false, /*anchor_knot=*/N);
        assert(old && std::abs(old->pos.x() - 5.0) < 1e-12);    // the A/B knob still reaches it
    }
    // 2. realised keep-out is exactly D_MIN — a dead dog is held like a live one, no extra margin
    {
        const auto k = corpse_keepout(Eigen::Vector4d(1, 2, 0, 0), plan_at(3, 4), kMargin);
        assert(k && std::abs((k->radius + kMargin) - D_MIN) < 1e-12);
    }
    // 3. NaN in the plan must NOT propagate: a NaN centre poisons every survivor's node QP and
    //    freezes the fleet. Fall back to the last reported state.
    {
        const double nan = std::nan("");
        const auto k = corpse_keepout(Eigen::Vector4d(1.0, 2.0, 0, 0), plan_at(nan, 4.0), kMargin);
        assert(k && std::abs(k->pos.x() - 1.0) < 1e-12 && std::abs(k->pos.y() - 2.0) < 1e-12);
        const auto k2 = corpse_keepout(Eigen::Vector4d(1.0, 2.0, 0, 0), plan_at(3.0, nan), kMargin);
        assert(k2 && std::abs(k2->pos.x() - 1.0) < 1e-12);
    }
    // 4. a short/absent plan falls back the same way (peer died before broadcasting a full one)
    {
        const auto k = corpse_keepout(Eigen::Vector4d(5.0, 6.0, 0, 0), Eigen::VectorXd(), kMargin);
        assert(k && std::abs(k->pos.x() - 5.0) < 1e-12 && std::abs(k->pos.y() - 6.0) < 1e-12);
        const auto k2 = corpse_keepout(Eigen::Vector4d(5.0, 6.0, 0, 0),
                                       Eigen::VectorXd::Zero(4), kMargin);
        assert(k2 && std::abs(k2->pos.x() - 5.0) < 1e-12);
    }
    // 5. BOTH sources garbage -> no keep-out at all. No constraint beats a poisoned one.
    {
        const double nan = std::nan("");
        assert(!corpse_keepout(Eigen::Vector4d(nan, nan, 0, 0), plan_at(nan, nan), kMargin));
        assert(!corpse_keepout(Eigen::Vector4d(nan, 2.0, 0, 0), Eigen::VectorXd(), kMargin));
    }
    // 6. radius never degenerates even if someone configures an absurd margin
    {
        const auto k = corpse_keepout(Eigen::Vector4d(0, 0, 0, 0), plan_at(1, 1), /*margin=*/9.0);
        assert(k && k->radius >= 0.10);
    }
    // 7. THE QP, not just the geometry: a keep-out that is born around a robot already inside it.
    //    Live numbers from d_0728_034832 — corpse anchor to survivor 1.137 m against an effective
    //    1.30 m. That run produced 68 consecutive non-finite solves and never moved again.
    {
        const double r = std::max(0.10, D_MIN - kMargin);  // 0.70, what corpse_keepout hands over
        const double d0 = 1.137;                           // inside: 1.137 < r + kMargin = 1.30
        Eigen::VectorXd x_now(4);
        x_now << d0, 0.0, 0.0, 0.0;
        Eigen::MatrixXd x_des(N, 2);  // reference pulls ACROSS the corpse, as a route home does
        for (int k = 0; k < N; ++k) {
            x_des(k, 0) = d0 - 0.05 * (k + 1);
            x_des(k, 1) = 0.0;
        }
        auto solve_with = [&](bool soft_k0) {
            std::vector<Obstacle> obs(1);
            obs[0].pos = Eigen::Vector2d::Zero();
            obs[0].radius = r;
            obs[0].soft_k0 = soft_k0;
            NodeSubproblem p(obs, {}, kMargin);
            return p.solve(x_now, x_des, nullptr, nullptr, nullptr);
        };
        const auto hard = solve_with(false);
        const auto soft = solve_with(true);
        // Pin the failure this flag exists to prevent, so it cannot quietly come back: with the
        // row hard, this exact geometry has NO solution and the dog is frozen for good.
        assert(!hard.finite);
        // the escape must exist and stay finite — this is the whole point
        assert(soft.finite);
        // ...and it must push OUTWARD, not merely return numbers: the first predicted step has to
        // gain ground on the keep-out it is standing in.
        assert(soft.x_pred(0, 0) > d0);
        // exactly one extra variable, and only when asked. Arena posts keep the old QP.
        assert(soft.xi.size() == hard.xi.size() + 1);
        std::cout << "  case 7: hard finite=" << hard.finite << " soft finite=" << soft.finite
                  << " first step " << d0 << " -> " << soft.x_pred(0, 0) << "\n";
    }
    std::cout << "test_corpse_keepout: all cases passed\n";
    return 0;
}
