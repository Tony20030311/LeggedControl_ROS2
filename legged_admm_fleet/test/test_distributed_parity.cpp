// G1 parity gate: 3x AgentCore over a synchronous in-process loopback transport must
// reproduce ADMMCoordinator::step's published xi bit-for-bit (no timeout path). xi is what
// each robot sends as its mpc_target, so bit-identical xi == the decomposition is correct.
// Plain assert/main (no gtest). Build target test_distributed_parity; run it directly.
#include <atomic>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>
#include <tuple>
#include <vector>

#include "agent_core.hpp"
#include "legged_upper_control/admm_coordinator.hpp"
#include "legged_upper_control/admm_formation.hpp"
#include "legged_upper_control/admm_node_qp.hpp"
#include "legged_upper_control/admm_reference.hpp"
#include "legged_upper_control/fleet_config.hpp"

using namespace admm;

// In-process synchronous loopback: recv blocks (condvar) until the matching message is
// posted by another agent's thread. Keyed by cycle/iter so cycles never collide.
class LoopbackTransport : public Transport {
public:
    void send_state(const AgentStateMsg& m) override {
        { std::lock_guard<std::mutex> l(mu_); states_[m.cycle_id][m.robot_id] = m; }
        cv_.notify_all();
    }
    std::map<int, AgentStateMsg> recv_states(std::uint64_t c,
                                             const std::vector<int>& peers) override {
        std::unique_lock<std::mutex> l(mu_);
        cv_.wait(l, [&] {
            auto it = states_.find(c);
            if (it == states_.end()) return false;
            for (int p : peers)
                if (!it->second.count(p)) return false;
            return true;
        });
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
        cv_.wait(l, [&] { return !xi_[k].empty(); });  // exactly one sender (the non-owner)
        return xi_[k];
    }
    void send_z(int, const EdgeZMsg& m) override {
        { std::lock_guard<std::mutex> l(mu_); z_[key(m.edge, m.cycle_id, m.iter)] = m; }
        cv_.notify_all();
    }
    EdgeZMsg recv_z(const EdgeKey& e, std::uint64_t c, int it) override {
        std::unique_lock<std::mutex> l(mu_);
        auto k = key(e, c, it);
        cv_.wait(l, [&] { return z_.count(k) > 0; });
        return z_[k];
    }

private:
    using Key = std::tuple<int, int, std::uint64_t, int>;
    static Key key(const EdgeKey& e, std::uint64_t c, int it) { return {e.first, e.second, c, it}; }
    std::mutex mu_;
    std::condition_variable cv_;
    std::map<std::uint64_t, std::map<int, AgentStateMsg>> states_;
    std::map<Key, std::map<int, EdgeXiMsg>> xi_;
    std::map<Key, EdgeZMsg> z_;
};

static bool bit_equal(const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
    return a.size() == b.size() &&
           std::memcmp(a.data(), b.data(), a.size() * sizeof(double)) == 0;
}

int main() {
    const std::vector<int> dogs = {1, 2, 3};
    const std::vector<EdgeKey> edges = {{1, 2}, {1, 3}, {2, 3}};
    const std::vector<Obstacle> obs;
    const std::vector<Wall> walls;
    const int hard_through = 1;
    const double w_form = 0.3;
    const int CYCLES = 6;

    // scenario: three dogs abreast, all commanded 3 m forward
    std::map<int, Eigen::Vector4d> xnow4;
    xnow4[1] = Eigen::Vector4d(-1.0, 0.6, 0.0, 0.0);
    xnow4[2] = Eigen::Vector4d(-1.0, -0.6, 0.0, 0.0);
    xnow4[3] = Eigen::Vector4d(-1.7, 0.0, 0.0, 0.0);
    std::map<int, Eigen::VectorXd> xnow;
    std::map<int, Eigen::MatrixXd> xdes;
    for (int i : dogs) {
        xnow[i] = xnow4[i];
        Eigen::MatrixX2d wp(2, 2);
        wp.row(0) = xnow4[i].head<2>().transpose();
        wp.row(1) = xnow4[i].head<2>().transpose() + Eigen::RowVector2d(3.0, 0.0);
        xdes[i] = build_reference(xnow4[i].head<2>(), wp);
    }

    // centralized reference (parallel=false)
    LaplacianFormation form_c(formations());
    form_c.set_formation("V");
    ADMMCoordinator coord(P_ITERS, RHO, dogs, edges, &form_c, w_form, obs, walls, hard_through);
    std::vector<std::map<int, Eigen::VectorXd>> cen(CYCLES);
    for (int c = 0; c < CYCLES; ++c) cen[c] = coord.step(xnow, xdes).first;

    // distributed: 3 AgentCore sharing one loopback transport
    LaplacianFormation form_d(formations());
    form_d.set_formation("V");
    LoopbackTransport tp;
    std::map<int, std::unique_ptr<AgentCore>> ag;
    for (int i : dogs)
        ag[i] = std::make_unique<AgentCore>(i, dogs, edges, &form_d, w_form, obs, walls,
                                            hard_through, &tp);

    int mismatches = 0;
    double worst = 0.0;
    for (int c = 0; c < CYCLES; ++c) {
        std::map<int, Eigen::VectorXd> got;
        std::mutex gm;
        std::vector<std::thread> th;
        for (int i : dogs)
            th.emplace_back([&, i] {
                auto r = ag[i]->step(xnow4[i], xdes[i], static_cast<std::uint64_t>(c));
                std::lock_guard<std::mutex> l(gm);
                got[i] = r.xi;
            });
        for (auto& t : th) t.join();
        for (int i : dogs) {
            const double d = (cen[c][i] - got[i]).cwiseAbs().maxCoeff();
            worst = std::max(worst, d);
            if (!bit_equal(cen[c][i], got[i])) {
                ++mismatches;
                std::cout << "MISMATCH cycle " << c << " dog " << i << " max|delta|=" << d << "\n";
            }
        }
    }

    std::cout << "cycles=" << CYCLES << " dogs=" << dogs.size()
              << " worst max|delta|=" << worst << " mismatches=" << mismatches << "\n";
    if (mismatches == 0) {
        std::cout << "G1 PASS: distributed AgentCore == centralized ADMMCoordinator (bit-identical)\n";
        return 0;
    }
    std::cout << "G1 FAIL\n";
    return 1;
}
