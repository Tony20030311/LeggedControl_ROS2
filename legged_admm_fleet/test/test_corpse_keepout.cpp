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
#include "legged_upper_control/fleet_config.hpp"

using namespace admm;

namespace {
Eigen::VectorXd plan_at(double x, double y) {  // a well-formed xibar whose terminal knot is (x,y)
    Eigen::VectorXd v = Eigen::VectorXd::Zero(py_index(N) + 1);
    v[px_index(N)] = x;
    v[py_index(N)] = y;
    return v;
}
const double kMargin = 0.60;
}  // namespace

int main() {
    // 1. the plan's terminal knot wins: it leads the body, which keeps coasting after the agent dies
    {
        const auto k = corpse_keepout(Eigen::Vector4d(1.0, 2.0, 0, 0), plan_at(3.0, 4.0), kMargin);
        assert(k && std::abs(k->pos.x() - 3.0) < 1e-12 && std::abs(k->pos.y() - 4.0) < 1e-12);
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
    std::cout << "test_corpse_keepout: all cases passed\n";
    return 0;
}
