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
#include <functional>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
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
        : node_(node), self_id_(self_id), dogs_(std::move(dogs)), deadline_(deadline),
          rng_(0x5eed + static_cast<unsigned>(self_id)) {
        inj_thread_ = std::thread([this] { injLoop(); });
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

    ~DdsTransport() override {
        { std::lock_guard<std::mutex> l(inj_mu_); inj_stop_ = true; }
        inj_cv_.notify_all();
        if (inj_thread_.joinable()) inj_thread_.join();
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
        w.tx_wall = wall_now_s();  // one-way latency stamp (same-host valid; real robots need NTP)
        bytes_tx_.fetch_add(wire_bytes(w), std::memory_order_relaxed);
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
        w.tx_wall = wall_now_s();
        bytes_tx_.fetch_add(wire_bytes(w), std::memory_order_relaxed);
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
        w.tx_wall = wall_now_s();
        bytes_tx_.fetch_add(wire_bytes(w), std::memory_order_relaxed);
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

    // G5 comm-volume instrument: application-payload bytes (CDR fields incl. 4-byte array
    // length prefixes; excludes RTPS/UDP framing) sent/received since the last call. tx counts
    // one publish once (DDS fans out to subscribers); rx counts delivered messages only —
    // injection-dropped messages "never arrived".
    struct Bytes { std::uint64_t tx, rx; };
    Bytes take_bytes() {
        return {bytes_tx_.exchange(0), bytes_rx_.exchange(0)};
    }

    // G5 one-way latency instrument: mean (rx_wall - tx_wall) over messages committed since the
    // last call, accumulated at COMMIT time so injected delay (the C-experiment variable) is
    // included. Same-host wall clock -> valid for localhost sweeps; real robots need NTP sync.
    double take_rx_mean() {
        const int n = rx_lat_n_.exchange(0);
        const long long ns = rx_lat_ns_.exchange(0);
        return n > 0 ? (ns * 1e-9) / n : 0.0;
    }
    // Messages that arrived for a slot older than the newest already committed (late-but-arrived).
    int take_stale() { return stale_.exchange(0); }

    // Eviction telemetry (experiment D): newest cycle_id seen per peer's AgentState.
    // The node compares against the current slot to detect a permanently-dead peer.
    std::map<int, std::uint64_t> last_seen() {
        std::lock_guard<std::mutex> l(mu_);
        return last_seen_;
    }

    // C-experiment fault injection, applied to INCOMING consensus messages only (models the
    // network; own sends are unaffected, /clock and the lower layer are untouched). drop_p:
    // discard probability; delay/jitter: hold a delivered message delay + U(0,jitter) ms before
    // it reaches the mailbox. Deterministic per-robot RNG seed -> reproducible sweeps.
    void set_inject(double drop_p, double delay_ms, double jitter_ms) {
        inj_drop_p_.store(drop_p);
        inj_delay_ms_.store(delay_ms);
        inj_jitter_ms_.store(jitter_ms);
    }

private:
    static long long elapsed_ns(std::chrono::steady_clock::time_point t0) {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now() - t0).count();
    }
    static double wall_now_s() {  // system (wall) clock for cross-node tx/rx stamps
        return std::chrono::duration<double>(
                   std::chrono::system_clock::now().time_since_epoch()).count();
    }
    void accountRx(double tx_wall) {  // one-way latency, lock-free
        if (tx_wall > 0.0) {
            rx_lat_ns_.fetch_add(static_cast<long long>((wall_now_s() - tx_wall) * 1e9),
                                 std::memory_order_relaxed);
            rx_lat_n_.fetch_add(1, std::memory_order_relaxed);
        }
    }
    void accountStale(std::uint64_t cycle_id) {  // call with mu_ held
        if (cycle_id < newest_commit_) stale_.fetch_add(1, std::memory_order_relaxed);
        else newest_commit_ = cycle_id;
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

    // CDR payload sizes (8 B per float64/uint64, 4 B per int32 + 4 B length prefix per
    // unbounded array, 1 B bool). Constant per type at fixed N; excludes RTPS/UDP framing.
    static std::size_t wire_bytes(const admm_fleet_msgs::msg::AgentState& w) {
        return 8 + 4 + 8 + 4 + 4 * 8 + (4 + 8 * w.xibar.size()) + 1;
    }
    static std::size_t wire_bytes(const admm_fleet_msgs::msg::EdgeXi& w) {
        return 8 + 4 + 8 + 4 + 4 + 4 + (4 + 8 * w.xi.size()) + (4 + 8 * w.lam.size());
    }
    static std::size_t wire_bytes(const admm_fleet_msgs::msg::EdgeZ& w) {
        return 8 + 4 + 8 + 4 + 4 + (4 + 8 * w.z_i.size()) + (4 + 8 * w.z_j.size());
    }

    // --- fault injection plumbing (executor thread -> mailbox) ---
    bool injDrop() {
        const double p = inj_drop_p_.load();
        if (p <= 0.0) return false;
        std::lock_guard<std::mutex> l(inj_mu_);
        return std::uniform_real_distribution<double>(0.0, 1.0)(rng_) < p;
    }
    // Deliver now (no delay configured) or enqueue for the delivery thread at now + delay +
    // U(0, jitter). Commit functions are the only mailbox writers either way.
    void injDeliver(std::function<void()> commit) {
        const double d = inj_delay_ms_.load(), j = inj_jitter_ms_.load();
        if (d <= 0.0 && j <= 0.0) { commit(); return; }
        std::chrono::steady_clock::time_point due;
        {
            std::lock_guard<std::mutex> l(inj_mu_);
            const double ms = d + (j > 0.0 ? std::uniform_real_distribution<double>(0.0, j)(rng_) : 0.0);
            due = std::chrono::steady_clock::now() +
                  std::chrono::microseconds(static_cast<long long>(ms * 1000.0));
            inj_q_.emplace(due, std::move(commit));
        }
        inj_cv_.notify_all();
    }
    void injLoop() {
        std::unique_lock<std::mutex> l(inj_mu_);
        while (!inj_stop_) {
            if (inj_q_.empty()) { inj_cv_.wait(l); continue; }
            inj_cv_.wait_until(l, inj_q_.begin()->first);
            const auto now = std::chrono::steady_clock::now();
            while (!inj_q_.empty() && inj_q_.begin()->first <= now) {
                auto fn = std::move(inj_q_.begin()->second);
                inj_q_.erase(inj_q_.begin());
                l.unlock();
                fn();
                l.lock();
            }
        }
    }

    void commitState(const AgentStateMsg& m, double tx_wall) {
        accountRx(tx_wall);
        {
            std::lock_guard<std::mutex> l(mu_);
            states_[m.cycle_id][m.robot_id] = m;
            auto& seen = last_seen_[m.robot_id];
            seen = std::max(seen, m.cycle_id);
            accountStale(m.cycle_id);
            pruneOld(m.cycle_id);
        }
        cv_.notify_all();
    }
    void commitXi(const EdgeXiMsg& m, double tx_wall) {
        accountRx(tx_wall);
        { std::lock_guard<std::mutex> l(mu_); accountStale(m.cycle_id);
          xi_[key(m.edge, m.cycle_id, m.iter)][m.from_robot] = m; }
        cv_.notify_all();
    }
    void commitZ(const EdgeZMsg& m, double tx_wall) {
        accountRx(tx_wall);
        { std::lock_guard<std::mutex> l(mu_); accountStale(m.cycle_id);
          z_[key(m.edge, m.cycle_id, m.iter)] = m; }
        cv_.notify_all();
    }

    void onState(const admm_fleet_msgs::msg::AgentState& w) {
        if (injDrop()) return;
        bytes_rx_.fetch_add(wire_bytes(w), std::memory_order_relaxed);
        AgentStateMsg m;
        m.cycle_id = w.cycle_id;
        m.robot_id = w.robot_id;
        for (int k = 0; k < 4; ++k) m.xnow[k] = w.xnow[k];
        m.xibar = to_vec(w.xibar);
        m.reset = w.reset;
        injDeliver([this, m, tx = w.tx_wall] { commitState(m, tx); });
    }
    void onXi(const admm_fleet_msgs::msg::EdgeXi& w) {
        if (injDrop()) return;
        bytes_rx_.fetch_add(wire_bytes(w), std::memory_order_relaxed);
        EdgeXiMsg m;
        m.cycle_id = w.cycle_id;
        m.iter = w.iter;
        m.edge = EdgeKey(w.edge_i, w.edge_j);
        m.from_robot = w.from_robot;
        m.xi = to_vec(w.xi);
        m.lam = to_vec(w.lam);
        injDeliver([this, m, tx = w.tx_wall] { commitXi(m, tx); });
    }
    void onZ(const admm_fleet_msgs::msg::EdgeZ& w) {
        if (injDrop()) return;
        bytes_rx_.fetch_add(wire_bytes(w), std::memory_order_relaxed);
        EdgeZMsg m;
        m.cycle_id = w.cycle_id;
        m.iter = w.iter;
        m.edge = EdgeKey(w.edge_i, w.edge_j);
        m.z_i = to_vec(w.z_i);
        m.z_j = to_vec(w.z_j);
        injDeliver([this, m, tx = w.tx_wall] { commitZ(m, tx); });
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
    std::atomic<std::uint64_t> bytes_tx_{0}, bytes_rx_{0};  // read-and-reset by take_bytes
    std::atomic<long long> rx_lat_ns_{0};  // sum of one-way (rx-tx) latencies since take_rx_mean
    std::atomic<int> rx_lat_n_{0}, stale_{0};
    std::uint64_t newest_commit_ = 0;  // guarded by mu_ (accountStale)
    // fault injection (C experiment): atomics set by set_inject; queue owned by inj_thread_
    std::atomic<double> inj_drop_p_{0.0}, inj_delay_ms_{0.0}, inj_jitter_ms_{0.0};
    std::mt19937 rng_;
    std::multimap<std::chrono::steady_clock::time_point, std::function<void()>> inj_q_;
    std::mutex inj_mu_;
    std::condition_variable inj_cv_;
    bool inj_stop_ = false;
    std::thread inj_thread_;
    std::mutex mu_;
    std::condition_variable cv_;
    std::map<std::uint64_t, std::map<int, AgentStateMsg>> states_;
    std::map<Key, std::map<int, EdgeXiMsg>> xi_;
    std::map<Key, EdgeZMsg> z_;
    std::map<int, std::uint64_t> last_seen_;  // per-peer newest state cycle_id (eviction)
};

}  // namespace admm
