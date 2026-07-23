// G4 robustness gate: when a peer drops out, the survivors must HOLD (barrier timeout) and
// keep returning — never deadlock. Uses a deadline-aware loopback: recv throws
// TransportTimeout if the awaited message misses the wall-clock deadline. Drops robot3 after
// a few cycles; asserts robot1/robot2 return hold=true within bounded time (no hang).
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>
#include <tuple>
#include <vector>

#include "agent_core.hpp"
#include "legged_upper_control/admm_formation.hpp"
#include "legged_upper_control/admm_node_qp.hpp"
#include "legged_upper_control/admm_reference.hpp"
#include "legged_upper_control/fleet_config.hpp"

using namespace admm;
using namespace std::chrono_literals;

class DeadlineLoopback : public Transport {
public:
    explicit DeadlineLoopback(std::chrono::milliseconds d) : deadline_(d) {}
    void send_state(const AgentStateMsg& m) override {
        { std::lock_guard<std::mutex> l(mu_); states_[m.cycle_id][m.robot_id] = m; }
        cv_.notify_all();
    }
    std::map<int, AgentStateMsg> recv_states(std::uint64_t c,
                                             const std::vector<int>& peers) override {
        std::unique_lock<std::mutex> l(mu_);
        if (!cv_.wait_for(l, deadline_, [&] {
                auto it = states_.find(c);
                if (it == states_.end()) return false;
                for (int p : peers)
                    if (!it->second.count(p)) return false;
                return true;
            }))
            throw TransportTimeout{};
        std::map<int, AgentStateMsg> r;
        for (int p : peers) r[p] = states_[c][p];
        return r;
    }
    void send_xi(int, const EdgeXiMsg& m) override {
        { std::lock_guard<std::mutex> l(mu_); xi_[key(m.edge, m.cycle_id, m.iter)][m.from_robot] = m; }
        cv_.notify_all();
    }
    std::map<int, EdgeXiMsg> recv_xi(const EdgeKey& e, std::uint64_t c, int it) override {
        std::unique_lock<std::mutex> l(mu_);
        auto k = key(e, c, it);
        if (!cv_.wait_for(l, deadline_, [&] { return !xi_[k].empty(); })) throw TransportTimeout{};
        return xi_[k];
    }
    void send_z(int, const EdgeZMsg& m) override {
        { std::lock_guard<std::mutex> l(mu_); z_[key(m.edge, m.cycle_id, m.iter)] = m; }
        cv_.notify_all();
    }
    EdgeZMsg recv_z(const EdgeKey& e, std::uint64_t c, int it) override {
        std::unique_lock<std::mutex> l(mu_);
        auto k = key(e, c, it);
        if (!cv_.wait_for(l, deadline_, [&] { return z_.count(k) > 0; })) throw TransportTimeout{};
        return z_[k];
    }

private:
    using Key = std::tuple<int, int, std::uint64_t, int>;
    static Key key(const EdgeKey& e, std::uint64_t c, int it) { return {e.first, e.second, c, it}; }
    std::chrono::milliseconds deadline_;
    std::mutex mu_;
    std::condition_variable cv_;
    std::map<std::uint64_t, std::map<int, AgentStateMsg>> states_;
    std::map<Key, std::map<int, EdgeXiMsg>> xi_;
    std::map<Key, EdgeZMsg> z_;
};

int main() {
    const std::vector<int> dogs = {1, 2, 3};
    const std::vector<EdgeKey> edges = {{1, 2}, {1, 3}, {2, 3}};
    const std::vector<Obstacle> obs;
    const std::vector<Wall> walls;
    LaplacianFormation form(formations());
    form.set_formation("V");

    std::map<int, Eigen::Vector4d> xnow4;
    xnow4[1] = Eigen::Vector4d(-1.0, 0.6, 0.0, 0.0);
    xnow4[2] = Eigen::Vector4d(-1.0, -0.6, 0.0, 0.0);
    xnow4[3] = Eigen::Vector4d(-1.7, 0.0, 0.0, 0.0);
    std::map<int, Eigen::MatrixXd> xdes;
    for (int i : dogs) {
        Eigen::MatrixX2d wp(2, 2);
        wp.row(0) = xnow4[i].head<2>().transpose();
        wp.row(1) = xnow4[i].head<2>().transpose() + Eigen::RowVector2d(3.0, 0.0);
        xdes[i] = build_reference(xnow4[i].head<2>(), wp);
    }

    DeadlineLoopback tp(200ms);
    std::map<int, std::unique_ptr<AgentCore>> ag;
    for (int i : dogs)
        ag[i] = std::make_unique<AgentCore>(i, dogs, edges, &form, 0.3, obs, walls, 1, &tp);

    const int ALIVE_CYCLES = 3, DROP_CYCLES = 3;
    std::map<int, bool> hold_after_drop{{1, false}, {2, false}};

    auto t0 = std::chrono::steady_clock::now();
    for (int c = 0; c < ALIVE_CYCLES + DROP_CYCLES; ++c) {
        const bool dropped = (c >= ALIVE_CYCLES);
        std::vector<int> live = dropped ? std::vector<int>{1, 2} : dogs;
        std::map<int, StepResult> res;
        std::mutex rm;
        std::vector<std::thread> th;
        for (int i : live)
            th.emplace_back([&, i] {
                auto r = ag[i]->step(xnow4[i], xdes[i], static_cast<std::uint64_t>(c));
                std::lock_guard<std::mutex> l(rm);
                res[i] = std::move(r);
            });
        for (auto& t : th) t.join();
        if (dropped)
            for (int i : {1, 2})
                if (res[i].hold) hold_after_drop[i] = true;
    }
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - t0)
                       .count();

    const bool no_deadlock = elapsed < 10000;  // finished at all (each drop cycle ~200ms hold)
    const bool held = hold_after_drop[1] && hold_after_drop[2];
    std::cout << "elapsed=" << elapsed << "ms  robot1 held=" << hold_after_drop[1]
              << " robot2 held=" << hold_after_drop[2] << "\n";
    if (no_deadlock && held) {
        std::cout << "G4-robustness PASS: peer drop -> survivors HOLD, no deadlock\n";
        return 0;
    }
    std::cout << "G4-robustness FAIL\n";
    return 1;
}
