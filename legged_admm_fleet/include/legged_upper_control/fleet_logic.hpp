#pragma once
// fleet_logic — the per-dog helpers that are GENUINELY identical between the centralized
// (fleet_centralized_node) and distributed (admm_agent_node) fleet nodes, so there is one
// source of truth instead of two copies that drift.
//
// Deliberately NOT here: followSpeed / rescueTick / x0-EMA / yaw-latch. Those look similar but
// are structurally different implementations, because the two nodes have different data models
// — the distributed agent is one process per robot (scalar member state + agent_->peer_xnow()),
// the centralized node is one process for the whole fleet (per-robot std::map state). Sharing
// them would mean 10+-arg free functions (uglier than the duplication) or unifying the data
// models (a rewrite); dminEff/footprintSupport/velYaw are agent-only and not duplicated at all.
#include <Eigen/Dense>

#include <ocs2_msgs/msg/mpc_input.hpp>
#include <ocs2_msgs/msg/mpc_state.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>

namespace admm {

// Build the OCS2 MPC target (the upper->lower interface payload) from a time vector and an
// (ns x 24) state matrix; inputs are zero. Single source of truth for the 24-D target layout
// — if that contract ever changes, it changes here, for both nodes at once.
inline ocs2_msgs::msg::MpcTargetTrajectories toTargetMsg(const Eigen::VectorXd& times,
                                                         const Eigen::MatrixXd& states) {
    ocs2_msgs::msg::MpcTargetTrajectories m;
    for (int j = 0; j < times.size(); ++j) {
        m.time_trajectory.push_back(times[j]);
        ocs2_msgs::msg::MpcState st;
        st.value.resize(24);
        for (int c = 0; c < 24; ++c) st.value[c] = static_cast<float>(states(j, c));
        m.state_trajectory.push_back(st);
        ocs2_msgs::msg::MpcInput in;
        in.value.assign(24, 0.0f);
        m.input_trajectory.push_back(in);
    }
    return m;
}

}  // namespace admm
