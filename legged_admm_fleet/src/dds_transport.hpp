#pragma once
// DdsTransport — the AgentCore Transport backed by real ROS 2 topics (G4). Subscription
// callbacks (executor thread) drop messages into mailboxes; the AgentCore worker thread's
// recv_*() blocks on a condvar until the awaited message arrives OR a per-hop deadline
// elapses, in which case it throws TransportTimeout (-> AgentCore HOLD / early stop).
//
// Topic map (plan §58-60):
//   AgentState -> /robotN/admm/state          (own pub; sub each peer)
//   EdgeXi     -> /admm/edge_i_j/xi           (non-owner pub; owner sub)
//   EdgeZ      -> /admm/edge_i_j/z            (owner pub; non-owner sub)
//
// Keep the worker OFF the executor thread (a MultiThreadedExecutor or a dedicated spin
// thread must service callbacks) or recv_*() will deadlock against its own subscriptions.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

#include "admm_fleet_msgs/msg/agent_state.hpp"
#include "admm_fleet_msgs/msg/edge_xi.hpp"
#include "admm_fleet_msgs/msg/edge_z.hpp"
#include "agent_core.hpp"

namespace admm {

inline std::string edge_tag(const EdgeKey& e) {
    return "edge_" + std::to_string(e.first) + "_" + std::to_string(e.second);
}
inline Eigen::VectorXd to_vec(const std::vector<double>& v) {
    return Eigen::Map<const Eigen::VectorXd>(v.data(), static_cast<Eigen::Index>(v.size()));
}
inline std::vector<double> to_std(const Eigen::VectorXd& v) {
    return std::vector<double>(v.data(), v.data() + v.size());
}

class DdsTransport : public Transport {
public:
    DdsTransport(rclcpp::Node* node, int self_id, std::vector<int> dogs,
                 std::vector<EdgeKey> edges, std::chrono::milliseconds deadline)
        : node_(node), self_id_(self_id), dogs_(std::move(dogs)), deadline_(deadline) {
        const auto qos = rclcpp::QoS(10).reliable();
        // own AgentState pub + one sub per peer
        state_pub_ = node_->create_publisher<admm_fleet_msgs::msg::AgentState>(
            "/robot" + std::to_string(self_id_) + "/admm/state", qos);
        for (int j : dogs_)
            if (j != self_id_)
                state_subs_.push_back(node_->create_subscription<admm_fleet_msgs::msg::AgentState>(
                    "/robot" + std::to_string(j) + "/admm/state", qos,
                    [this](admm_fleet_msgs::msg::AgentState::SharedPtr m) { onState(*m); }));
        // per incident edge: owner sub xi / pub z; non-owner pub xi / sub z
        for (const EdgeKey& e : edges)
            if (e.first == self_id_ || e.second == self_id_) {
                const std::string base = "/admm/" + edge_tag(e);
                if (edge_owner(e) == self_id_) {
                    z_pubs_[e] = node_->create_publisher<admm_fleet_msgs::msg::EdgeZ>(base + "/z", qos);
                    xi_subs_.push_back(node_->create_subscription<admm_fleet_msgs::msg::EdgeXi>(
                        base + "/xi", qos,
                        [this](admm_fleet_msgs::msg::EdgeXi::SharedPtr m) { onXi(*m); }));
                } else {
                    xi_pubs_[e] = node_->create_publisher<admm_fleet_msgs::msg::EdgeXi>(base + "/xi", qos);
                    z_subs_.push_back(node_->create_subscription<admm_fleet_msgs::msg::EdgeZ>(
                        base + "/z", qos,
                        [this](admm_fleet_msgs::msg::EdgeZ::SharedPtr m) { onZ(*m); }));
                }
            }
    }

    // --- Transport interface (worker thread) ---
    void send_state(const AgentStateMsg& m) override {
        admm_fleet_msgs::msg::AgentState w;
        w.cycle_id = m.cycle_id;
        w.iter = 0;
        w.robot_id = m.robot_id;
        for (int k = 0; k < 4; ++k) w.xnow[k] = m.xnow[k];
        w.xibar = to_std(m.xibar);
        w.reset = m.reset;
        state_pub_->publish(w);
    }
    std::map<int, AgentStateMsg> recv_states(std::uint64_t c,
                                             const std::vector<int>& peers) override {
        std::unique_lock<std::mutex> l(mu_);
        const auto t0 = std::chrono::steady_clock::now();
        const bool ok = cv_.wait_for(l, deadline_, [&] {
            auto it = states_.find(c);
            if (it == states_.end()) return false;
            for (int p : peers)
                if (!it->second.count(p)) return false;
            return true;
        });
        wait_state_ns_.fetch_add(elapsed_ns(t0), std::memory_order_relaxed);
        if (!ok) { ++timeouts_; throw TransportTimeout{}; }
        std::map<int, AgentStateMsg> r;
        for (int p : peers) r[p] = states_[c][p];
        return r;
    }
    void send_xi(int, const EdgeXiMsg& m) override {
        admm_fleet_msgs::msg::EdgeXi w;
        w.cycle_id = m.cycle_id;
        w.iter = m.iter;
        w.edge_i = m.edge.first;
        w.edge_j = m.edge.second;
        w.from_robot = m.from_robot;
        w.xi = to_std(m.xi);
        w.lam = to_std(m.lam);
        xi_pubs_.at(m.edge)->publish(w);
    }
    std::map<int, EdgeXiMsg> recv_xi(const EdgeKey& e, std::uint64_t c, int it) override {
        std::unique_lock<std::mutex> l(mu_);
        auto k = key(e, c, it);
        const auto t0 = std::chrono::steady_clock::now();
        const bool ok = cv_.wait_for(l, deadline_, [&] { return xi_.count(k) && !xi_[k].empty(); });
        wait_xi_ns_.fetch_add(elapsed_ns(t0), std::memory_order_relaxed);
        if (!ok) { ++timeouts_; throw TransportTimeout{}; }
        return xi_[k];
    }
    void send_z(int, const EdgeZMsg& m) override {
        admm_fleet_msgs::msg::EdgeZ w;
        w.cycle_id = m.cycle_id;
        w.iter = m.iter;
        w.edge_i = m.edge.first;
        w.edge_j = m.edge.second;
        w.z_i = to_std(m.z_i);
        w.z_j = to_std(m.z_j);
        z_pubs_.at(m.edge)->publish(w);
    }
    EdgeZMsg recv_z(const EdgeKey& e, std::uint64_t c, int it) override {
        std::unique_lock<std::mutex> l(mu_);
        auto k = key(e, c, it);
        const auto t0 = std::chrono::steady_clock::now();
        const bool ok = cv_.wait_for(l, deadline_, [&] { return z_.count(k) > 0; });
        wait_z_ns_.fetch_add(elapsed_ns(t0), std::memory_order_relaxed);
        if (!ok) { ++timeouts_; throw TransportTimeout{}; }
        return z_[k];
    }
    int take_timeouts() override { return timeouts_.exchange(0); }

    // G5 comm-latency instrument: total per-cycle recv wait time (s) by message type, read-and-
    // reset once per cycle by admm_agent_node::publishStats. This is the effective DDS delay each
    // hop blocked for its peer's message (localhost -> ~propagation+callback; real net -> jitter +
    // retransmit). DDS-specific, not part of the Transport interface.
    struct WaitTimes { double state, xi, z; };
    WaitTimes take_wait_times() {
        return {wait_state_ns_.exchange(0) * 1e-9, wait_xi_ns_.exchange(0) * 1e-9,
                wait_z_ns_.exchange(0) * 1e-9};
    }

private:
    static long long elapsed_ns(std::chrono::steady_clock::time_point t0) {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now() - t0).count();
    }
    using Key = std::tuple<int, int, std::uint64_t, int>;
    static Key key(const EdgeKey& e, std::uint64_t c, int it) { return {e.first, e.second, c, it}; }

    // Drop mailbox entries older than kKeep slots behind `cur`. Keys embed the sim slot
    // (states_ by cycle_id; xi_/z_ as tuple<..,cycle,iter>), which advances monotonically,
    // so a slot is dead once its consensus is done + a small in-flight margin. Without this
    // the maps grow for the whole run -> OOM on long G5/soak. Call with mu_ held.
    // ponytail: fixed 8-slot window, no config knob — peers stay within ~1 slot (barrier).
    static constexpr std::uint64_t kKeep = 8;
    void pruneOld(std::uint64_t cur) {
        if (cur < kKeep) return;
        const std::uint64_t cutoff = cur - kKeep;
        states_.erase(states_.begin(), states_.lower_bound(cutoff));
        for (auto it = xi_.begin(); it != xi_.end();)
            it = (std::get<2>(it->first) < cutoff) ? xi_.erase(it) : std::next(it, 1);
        for (auto it = z_.begin(); it != z_.end();)
            it = (std::get<2>(it->first) < cutoff) ? z_.erase(it) : std::next(it, 1);
    }

    void onState(const admm_fleet_msgs::msg::AgentState& w) {
        AgentStateMsg m;
        m.cycle_id = w.cycle_id;
        m.robot_id = w.robot_id;
        for (int k = 0; k < 4; ++k) m.xnow[k] = w.xnow[k];
        m.xibar = to_vec(w.xibar);
        m.reset = w.reset;
        { std::lock_guard<std::mutex> l(mu_); states_[w.cycle_id][w.robot_id] = m; pruneOld(w.cycle_id); }
        cv_.notify_all();
    }
    void onXi(const admm_fleet_msgs::msg::EdgeXi& w) {
        EdgeXiMsg m;
        m.cycle_id = w.cycle_id;
        m.iter = w.iter;
        m.edge = EdgeKey(w.edge_i, w.edge_j);
        m.from_robot = w.from_robot;
        m.xi = to_vec(w.xi);
        m.lam = to_vec(w.lam);
        { std::lock_guard<std::mutex> l(mu_); xi_[key(m.edge, w.cycle_id, w.iter)][w.from_robot] = m; }
        cv_.notify_all();
    }
    void onZ(const admm_fleet_msgs::msg::EdgeZ& w) {
        EdgeZMsg m;
        m.cycle_id = w.cycle_id;
        m.iter = w.iter;
        m.edge = EdgeKey(w.edge_i, w.edge_j);
        m.z_i = to_vec(w.z_i);
        m.z_j = to_vec(w.z_j);
        { std::lock_guard<std::mutex> l(mu_); z_[key(m.edge, w.cycle_id, w.iter)] = m; }
        cv_.notify_all();
    }

    rclcpp::Node* node_;
    int self_id_;
    std::vector<int> dogs_;
    std::chrono::milliseconds deadline_;

    rclcpp::Publisher<admm_fleet_msgs::msg::AgentState>::SharedPtr state_pub_;
    std::vector<rclcpp::Subscription<admm_fleet_msgs::msg::AgentState>::SharedPtr> state_subs_;
    std::map<EdgeKey, rclcpp::Publisher<admm_fleet_msgs::msg::EdgeXi>::SharedPtr> xi_pubs_;
    std::map<EdgeKey, rclcpp::Publisher<admm_fleet_msgs::msg::EdgeZ>::SharedPtr> z_pubs_;
    std::vector<rclcpp::Subscription<admm_fleet_msgs::msg::EdgeXi>::SharedPtr> xi_subs_;
    std::vector<rclcpp::Subscription<admm_fleet_msgs::msg::EdgeZ>::SharedPtr> z_subs_;

    std::atomic<int> timeouts_{0};  // per-hop recv deadline misses (read-and-reset by take_timeouts)
    std::atomic<long long> wait_state_ns_{0}, wait_xi_ns_{0}, wait_z_ns_{0};  // per-cycle recv waits
    std::mutex mu_;
    std::condition_variable cv_;
    std::map<std::uint64_t, std::map<int, AgentStateMsg>> states_;
    std::map<Key, std::map<int, EdgeXiMsg>> xi_;
    std::map<Key, EdgeZMsg> z_;
};

}  // namespace admm
