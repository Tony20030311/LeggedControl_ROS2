// Experiment-D failure-semantics gate. Three scenarios over a deadline loopback:
//   A. NaN reset broadcast: one agent's QP dies (NaN reference -> rejected OSQP update,
//      the qp_common update-guard path) -> it announces reset; the fleet cold-starts the
//      SAME slot together (announce slot k+1 all-HOLD, k+2 all-cold), then returns to
//      warm full-round cycles. No ping-pong, no deadlock.
//   B. Solo survivor: an AgentCore with no edges (post-eviction, fleet of one) steps as a
//      plain tracking MPC — finite, no barrier hang.
//   C. Post-eviction pair: a fresh 2-dog core (one edge, corpse obstacle) cold-starts and
//      reaches full rounds.
// Plain assert/main like the other gates. Build target test_agent_failover.
#include <chrono>
#include <cmath>
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

// same deadline loopback as test_agent_timeout (test-local copy, kept tiny on purpose)
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

static int failures = 0;
#define CHECK(cond, msg)                                              \
    do {                                                              \
        if (!(cond)) {                                                \
            std::cout << "FAIL: " << msg << "\n";                     \
            ++failures;                                               \
        }                                                             \
    } while (0)

static Eigen::MatrixXd straight_ref(const Eigen::Vector2d& p0) {
    Eigen::MatrixX2d wp(2, 2);
    wp.row(0) = p0.transpose();
    wp.row(1) = (p0 + Eigen::Vector2d(3.0, 0.0)).transpose();
    return build_reference(p0, wp);
}

int main() {
    const std::vector<Obstacle> no_obs;
    const std::vector<Wall> no_walls;

    // ---------- A. NaN -> fleet reset broadcast ----------
    {
        const std::vector<int> dogs = {1, 2, 3};
        const std::vector<EdgeKey> edges = {{1, 2}, {1, 3}, {2, 3}};
        // legal spawns: every pair > D_MIN=1.3 (1-2: 1.6, 1-3 = 2-3: 1.44)
        std::map<int, Eigen::Vector4d> xnow;
        xnow[1] = Eigen::Vector4d(-1.0, 0.8, 0.0, 0.0);
        xnow[2] = Eigen::Vector4d(-1.0, -0.8, 0.0, 0.0);
        xnow[3] = Eigen::Vector4d(-2.2, 0.0, 0.0, 0.0);
        std::map<int, Eigen::MatrixXd> xdes;
        for (int i : dogs) xdes[i] = straight_ref(xnow[i].head<2>());
        Eigen::MatrixXd xdes_nan = xdes[1];
        xdes_nan.setConstant(std::numeric_limits<double>::quiet_NaN());

        LaplacianFormation form(formations());
        form.set_formation("V");
        DeadlineLoopback tp(200ms);
        std::map<int, std::unique_ptr<AgentCore>> ag;
        for (int i : dogs)
            ag[i] = std::make_unique<AgentCore>(i, dogs, edges, &form, 0.3, no_obs, no_walls,
                                                1, &tp);

        auto run_cycle = [&](int c, bool poison1) {
            std::map<int, StepResult> res;
            std::mutex rm;
            std::vector<std::thread> th;
            for (int i : dogs)
                th.emplace_back([&, i] {
                    const Eigen::MatrixXd& xd = (poison1 && i == 1) ? xdes_nan : xdes[i];
                    auto r = ag[i]->step(xnow[i], xd, static_cast<std::uint64_t>(c));
                    std::lock_guard<std::mutex> l(rm);
                    res[i] = std::move(r);
                });
            for (auto& t : th) t.join();
            return res;
        };

        for (int c = 0; c < 3; ++c) {  // healthy warm-up
            auto r = run_cycle(c, false);
            for (int i : dogs) {
                CHECK(!r[i].hold, "A: healthy cycle " << "hold on dog " << i);
                CHECK(r[i].xi.allFinite(), "A: healthy cycle non-finite dog " << i);
                CHECK(r[i].achieved_rounds == P_ITERS, "A: healthy cycle short rounds dog " << i);
            }
        }
        {  // cycle 3: dog1's QP dies (NaN reference -> rejected update -> NaN xi)
            auto r = run_cycle(3, true);
            CHECK(!r[1].xi.allFinite(), "A: poisoned dog1 xi should be non-finite");
            CHECK(r[2].xi.allFinite() && r[3].xi.allFinite(),
                  "A: dogs 2/3 must stay finite on dog1's failure cycle");
        }
        {  // cycle 4: announce slot — dog1 cold announces, dogs 2/3 join the reset: all HOLD
            auto r = run_cycle(4, false);
            for (int i : dogs) CHECK(r[i].hold, "A: cycle 4 should be all-HOLD (dog " << i << ")");
        }
        {  // cycle 5: synchronized all-cold restart — full rounds again, no hold
            auto r = run_cycle(5, false);
            for (int i : dogs) {
                CHECK(!r[i].hold, "A: cycle 5 hold on dog " << i);
                CHECK(r[i].xi.allFinite(), "A: cycle 5 non-finite dog " << i);
                CHECK(r[i].achieved_rounds == P_ITERS, "A: cycle 5 short rounds dog " << i);
            }
        }
        {  // cycle 6: back to warm
            auto r = run_cycle(6, false);
            for (int i : dogs)
                CHECK(!r[i].hold && r[i].xi.allFinite() && r[i].achieved_rounds == P_ITERS,
                      "A: cycle 6 not back to normal (dog " << i << ")");
        }
        std::cout << "A: NaN -> announce -> synchronized cold restart OK\n";
    }

    // ---------- B. solo survivor (no edges) ----------
    {
        DeadlineLoopback tp(200ms);
        AgentCore solo(1, {1}, {}, nullptr, 0.3, no_obs, no_walls, 1, &tp);
        const Eigen::Vector4d x0(0.0, 0.0, 0.0, 0.0);
        const Eigen::MatrixXd xd = straight_ref(x0.head<2>());
        for (int c = 0; c < 3; ++c) {
            auto r = solo.step(x0, xd, static_cast<std::uint64_t>(c));
            CHECK(!r.hold, "B: solo hold at cycle " << c);
            CHECK(r.xi.allFinite(), "B: solo non-finite at cycle " << c);
        }
        std::cout << "B: solo survivor steps as plain MPC OK\n";
    }

    // ---------- C. post-eviction 2-dog core + corpse obstacle ----------
    {
        const std::vector<int> dogs = {1, 2};
        const std::vector<EdgeKey> edges = {{1, 2}};
        std::vector<Obstacle> obs;
        obs.push_back({Eigen::Vector2d(-2.2, 0.0), 0.7});  // dead dog3's last position
        std::map<int, Eigen::Vector4d> xnow;
        xnow[1] = Eigen::Vector4d(-1.0, 0.8, 0.0, 0.0);
        xnow[2] = Eigen::Vector4d(-1.0, -0.8, 0.0, 0.0);
        std::map<int, Eigen::MatrixXd> xdes;
        for (int i : dogs) xdes[i] = straight_ref(xnow[i].head<2>());

        DeadlineLoopback tp(200ms);
        std::map<int, std::unique_ptr<AgentCore>> ag;
        for (int i : dogs)
            ag[i] = std::make_unique<AgentCore>(i, dogs, edges, /*formation=*/nullptr, 0.3,
                                                obs, no_walls, 1, &tp);
        for (int c = 0; c < 3; ++c) {
            std::map<int, StepResult> res;
            std::mutex rm;
            std::vector<std::thread> th;
            for (int i : dogs)
                th.emplace_back([&, i] {
                    auto r = ag[i]->step(xnow[i], xdes[i], static_cast<std::uint64_t>(c));
                    std::lock_guard<std::mutex> l(rm);
                    res[i] = std::move(r);
                });
            for (auto& t : th) t.join();
            for (int i : dogs) {
                CHECK(!res[i].hold, "C: pair hold at cycle " << c << " dog " << i);
                CHECK(res[i].xi.allFinite(), "C: pair non-finite dog " << i);
                CHECK(res[i].achieved_rounds == P_ITERS, "C: pair short rounds dog " << i);
            }
        }
        std::cout << "C: 2-dog survivor consensus with corpse obstacle OK\n";
    }

    if (failures == 0) {
        std::cout << "D-failover PASS: reset broadcast + solo + survivor pair\n";
        return 0;
    }
    std::cout << "D-failover FAIL (" << failures << " checks)\n";
    return 1;
}
