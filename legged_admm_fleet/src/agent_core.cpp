// AgentCore — per-robot slice of ADMMCoordinator::step, mirrored operation-for-operation so
// 3 agents over a synchronous loopback transport reproduce the centralized result bit-for-bit
// (G1). See agent_core.hpp for the separability argument.
#include "agent_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace admm {

// Balanced ownership for the 3-dog complete graph: each robot owns exactly one edge.
// (1,2)->2, (1,3)->1, (2,3)->3. Routing only; the edge-QP is deterministic in its inputs
// so parity holds for any consistent rule. General fallback: the higher-id endpoint.
int edge_owner(const EdgeKey& e) {
    if (e == EdgeKey(1, 2)) return 2;
    if (e == EdgeKey(1, 3)) return 1;
    if (e == EdgeKey(2, 3)) return 3;
    return e.second;
}

namespace {
EdgeKey canon(int a, int b) { return a < b ? EdgeKey(a, b) : EdgeKey(b, a); }
int other(const EdgeKey& e, int self) { return e.first == self ? e.second : e.first; }
double secs_since(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}
}  // namespace

AgentCore::AgentCore(int self_id, std::vector<int> dogs, std::vector<EdgeKey> edges,
                     const LaplacianFormation* formation, double w_form,
                     std::vector<Obstacle> obstacles, std::vector<Wall> walls,
                     int hard_through, Transport* transport, double robot_margin)
    : self_id_(self_id),
      dogs_(std::move(dogs)),
      edges_(std::move(edges)),
      formation_(formation),
      w_form_(w_form),
      hard_through_(hard_through),
      transport_(transport) {
    // neighbors + my_edges in edges_ order (matches ADMMCoordinator's neighbor ordering,
    // which the consensus_target accumulation order depends on).
    for (const EdgeKey& e : edges_)
        if (e.first == self_id_ || e.second == self_id_) {
            my_edges_.push_back(e);
            neighbors_.push_back(other(e, self_id_));
        }
    node_ = std::make_unique<NodeSubproblem>(
        obstacles, walls, robot_margin, /*q_pos=*/10.0, /*q_v=*/1.0, /*r_accel=*/0.5,
        /*w_pred=*/20.0, /*n=*/N, RHO, static_cast<int>(neighbors_.size()), w_form_);
    for (const EdgeKey& e : my_edges_)
        if (edge_owner(e) == self_id_)
            owned_edge_[e] = std::make_unique<EdgeSubproblem>(N, RHO, SLACK_LAMBDA, hard_through_);
    N_ = node_->N_;
    nz_ = xi_dim(N_);
}

Eigen::VectorXd AgentCore::uncoupled_self_(const Eigen::Vector4d& xnow,
                                           const Eigen::MatrixXd& xdes) {
    return node_->solve(xnow, xdes, nullptr, nullptr, nullptr).xi.head(nz_);
}

Eigen::MatrixX2d AgentCore::formation_grad_self_(
    const std::map<int, Eigen::VectorXd>& xibar) const {
    Eigen::MatrixX2d grad = Eigen::MatrixX2d::Zero(N_, 2);
    std::size_t self_idx = 0;
    for (std::size_t k = 0; k < dogs_.size(); ++k)
        if (dogs_[k] == self_id_) self_idx = k;
    for (int k = 1; k <= N_; ++k) {
        std::vector<Eigen::Vector2d> pos_k;
        pos_k.reserve(dogs_.size());
        for (const int i : dogs_)
            pos_k.emplace_back(xibar.at(i)(px_index(k, N_)), xibar.at(i)(px_index(k, N_) + 1));
        const auto fg = formation_->compute(pos_k);  // deterministic; each agent keeps its row
        grad(k - 1, 0) = fg.second[self_idx](0);
        grad(k - 1, 1) = fg.second[self_idx](1);
    }
    return grad;
}

StepResult AgentCore::step(const Eigen::Vector4d& xnow_self,
                           const Eigen::MatrixXd& xdes_self, std::uint64_t slot) {
    // 1. operating point (self only)
    Eigen::VectorXd xibar_self =
        (cycle_ == 0) ? uncoupled_self_(xnow_self, xdes_self) : shift_xi(prev_xi_, N_);

    // 2. broadcast AgentState, barrier on peers (keyed by the global slot).
    // reset=true advertises "I am cold-starting this slot" — the fleet-wide NaN
    // recovery signal (msg doc): peers that see it drop their warm start so the
    // whole fleet cold-starts the SAME slot, mirroring the centralized all-dogs
    // reset (coordinator.cpp finite-guard) with a one-slot announce lag.
    AgentStateMsg out;
    out.cycle_id = slot;
    out.robot_id = self_id_;
    out.xnow = xnow_self;
    out.xibar = xibar_self;
    out.reset = (cycle_ == 0);
    transport_->send_state(out);
    std::vector<int> peers;
    for (const int i : dogs_)
        if (i != self_id_) peers.push_back(i);
    std::map<int, AgentStateMsg> states;
    if (!peers.empty()) try {
        states = transport_->recv_states(slot, peers);
    } catch (const TransportTimeout&) {
        // barrier miss: HOLD this cycle, do not advance warm-start state (plan §66).
        StepResult hr;
        hr.xi = has_prev_ ? prev_xi_ : xibar_self;
        hr.hold = true;
        hr.n_timeouts = transport_->take_timeouts();
        return hr;
    }
    std::map<int, Eigen::VectorXd> xibar;
    std::map<int, Eigen::Vector4d> xnow;
    xibar[self_id_] = xibar_self;
    xnow[self_id_] = xnow_self;
    for (auto& kv : states) {
        xibar[kv.first] = kv.second.xibar;
        xnow[kv.first] = kv.second.xnow;
    }
    peer_xnow_ = xnow;      // expose to admm_agent_node (followSpeed)
    peer_xibar_ = xibar;    // expose to admm_agent_node (safe_prefix)

    // 2b. fleet-reset synchronization. Rule table (stateless, no ping-pong):
    //   warm + any peer cold  -> discard this slot, drop warm start (join the reset)
    //   cold + any peer warm  -> announce-only slot (hold; wait for warm peers to join)
    //   all cold together     -> proceed with the ordinary cycle_==0 cold start below
    //     (this is also the normal first-ever cycle, so startup is unchanged)
    // Net timing vs centralized: fail at k -> k+1 all-hold -> k+2 all-cold solve, i.e.
    // the centralized post-NaN cycle with one extra announce slot.
    if (!states.empty()) {
        const bool self_cold = (cycle_ == 0);
        bool any_cold = false, all_cold = true;
        for (const auto& kv : states) { any_cold |= kv.second.reset; all_cold &= kv.second.reset; }
        if ((!self_cold && any_cold) || (self_cold && !all_cold)) {
            StepResult hr;
            hr.xi = has_prev_ ? prev_xi_ : xibar_self;
            hr.hold = true;
            hr.n_timeouts = transport_->take_timeouts();
            if (!self_cold) { has_prev_ = false; cycle_ = 0; }  // join the fleet reset
            return hr;
        }
    }

    // 2c. solo survivor (post-eviction, no edges left): no consensus to run — the node
    // QP alone IS the whole problem. Track the reference, keep warm-start bookkeeping.
    if (my_edges_.empty()) {
        Eigen::VectorXd xi_solo =
            node_->solve(xnow_self, xdes_self, (cycle_ == 0) ? nullptr : &xibar_self,
                         nullptr, nullptr).xi.head(nz_);
        StepResult sr;
        sr.xi = xi_solo;
        if (xi_solo.allFinite()) {
            prev_xi_ = xi_solo;
            has_prev_ = true;
            cycle_ += 1;
        } else {
            node_->reset_solver();
            has_prev_ = false;
            cycle_ = 0;
        }
        sr.n_timeouts = transport_->take_timeouts();
        return sr;
    }

    // 3. init self's consensus copies; warm start after cycle 0
    if (cycle_ == 0) {
        for (const EdgeKey& e : my_edges_) {
            z_self_[e] = xibar_self;
            lam_self_[e] = Eigen::VectorXd::Zero(nz_);
            if (edge_owner(e) == self_id_)  // both copies from each endpoint's xibar
                owned_z_[e] = {xibar.at(e.first), xibar.at(e.second)};
        }
    } else {
        z_self_ = prev_z_self_;
        lam_self_ = prev_lam_self_;
        owned_z_ = prev_owned_z_;
    }

    // 4. PREP once (B8): linearize the edges I own; freeze my formation-grad row (cycle>0)
    for (const EdgeKey& e : my_edges_)
        if (edge_owner(e) == self_id_) {
            const EdgeLin fr = linearize_edge(xibar.at(e.first), xibar.at(e.second),
                                              xnow.at(e.first), xnow.at(e.second), N_);
            owned_edge_[e]->set_linearization(fr.g, fr.u);
        }
    const bool use_fgrad = (formation_ != nullptr && cycle_ > 0);
    Eigen::MatrixX2d fgrad;
    if (use_fgrad) fgrad = formation_grad_self_(xibar);

    // 5. ADMM loop (fixed P_ITERS). Feasible-input parity gate: no cross-agent NaN barrier.
    // A per-hop transport timeout ends THIS cycle early (plan §68): break before the round's
    // dual update, keep the completed rounds' warm-start state. achieved counts full rounds.
    Eigen::VectorXd xi_self = xibar_self;
    std::map<EdgeKey, Eigen::VectorXd> z_prev = z_self_;
    ADMMCoordinator::Hist hist;
    int achieved = 0;
    double t_node_acc = 0.0, t_edge_acc = 0.0;  // G5 solve-time telemetry (output only)
    for (int p = 0; p < P_ITERS; ++p) {
        // node update — consensus_target over self's own copies, edges_ order preserved
        EdgeVecs zc, lc;
        for (const EdgeKey& e : my_edges_) {
            zc[e][self_id_] = z_self_[e];
            lc[e][self_id_] = lam_self_[e];
        }
        const Eigen::VectorXd ct = consensus_target(zc, lc, self_id_, neighbors_);
        const auto tn0 = std::chrono::steady_clock::now();
        xi_self = node_->solve(xnow_self, xdes_self, &xibar_self, &ct,
                               use_fgrad ? &fgrad : nullptr)
                      .xi.head(nz_);
        t_node_acc += secs_since(tn0);
        if (!xi_self.allFinite()) break;

        try {
            // edge update — send my xi/lam to each edge owner; owner solves and returns z
            for (const EdgeKey& e : my_edges_) {
                const int owner = edge_owner(e);
                if (owner != self_id_) {
                    EdgeXiMsg m;
                    m.cycle_id = slot;
                    m.iter = p;
                    m.edge = e;
                    m.from_robot = self_id_;
                    m.xi = xi_self;
                    m.lam = lam_self_[e];
                    transport_->send_xi(owner, m);
                }
            }
            for (const EdgeKey& e : my_edges_) {
                const int owner = edge_owner(e);
                if (owner == self_id_) {
                    // gather both endpoints (self local + other via transport), canonical order
                    std::map<int, EdgeXiMsg> got = transport_->recv_xi(e, slot, p);
                    const int o = other(e, self_id_);
                    std::map<int, std::pair<Eigen::VectorXd, Eigen::VectorXd>> end;  // id->(xi,lam)
                    end[self_id_] = {xi_self, lam_self_[e]};
                    end[o] = {got.at(o).xi, got.at(o).lam};
                    const auto te0 = std::chrono::steady_clock::now();
                    const EdgeSubproblem::Out so = owned_edge_[e]->solve(
                        end[e.first].first, end[e.second].first, end[e.first].second,
                        end[e.second].second);
                    t_edge_acc += secs_since(te0);
                    if (so.ok)
                        owned_z_[e] = {so.z_i, so.z_j};
                    else
                        hist.edge_fail += 1;  // dead solve: keep last owned_z_[e]
                    // ship both copies to the non-owner; keep self's own copy
                    EdgeZMsg zm;
                    zm.cycle_id = slot;
                    zm.iter = p;
                    zm.edge = e;
                    zm.z_i = owned_z_[e].first;
                    zm.z_j = owned_z_[e].second;
                    transport_->send_z(o, zm);
                    z_self_[e] = (e.first == self_id_) ? owned_z_[e].first : owned_z_[e].second;
                }
            }
            for (const EdgeKey& e : my_edges_) {
                if (edge_owner(e) != self_id_) {
                    const EdgeZMsg zm = transport_->recv_z(e, slot, p);
                    z_self_[e] = (e.first == self_id_) ? zm.z_i : zm.z_j;
                }
            }
        } catch (const TransportTimeout&) {
            break;  // no half-round dual update; xi_self holds this round's node solution
        }

        // dual update (scaled, no rho) — local, mirrors lam[e][i] += xi[i] - z[e][i]
        for (const EdgeKey& e : my_edges_)
            lam_self_[e] = lam_self_[e] + (xi_self - z_self_[e]);

        // per-edge primal residual for CycleStats (global r_prim needs a reduction; the
        // parity gate compares xi, not the residual — xi is what gets published).
        double rp2 = 0.0;
        std::vector<double> sq(nz_);
        for (const EdgeKey& e : my_edges_) {
            const Eigen::VectorXd d = xi_self - z_self_[e];
            for (int j = 0; j < nz_; ++j) sq[j] = d(j) * d(j);
            rp2 += numpy_pairwise_sum(sq.data(), sq.size());
        }
        hist.r_prim.push_back(std::sqrt(rp2));
        z_prev = z_self_;
        achieved = p + 1;
    }

    // 6. warm-start storage (finite guard, mirrors centralized)
    if (xi_self.allFinite()) {
        prev_xi_ = xi_self;
        prev_z_self_ = z_self_;
        prev_lam_self_ = lam_self_;
        prev_owned_z_ = owned_z_;
        has_prev_ = true;
        cycle_ += 1;
    } else {
        node_->reset_solver();
        for (auto& kv : owned_edge_) kv.second->reset_solver();
        has_prev_ = false;
        cycle_ = 0;
    }
    StepResult res;
    res.xi = xi_self;
    // per-edge final primal residual (output only; same pairwise-sum as the in-loop metric).
    if (xi_self.allFinite()) {
        std::vector<double> sq(nz_);
        for (const EdgeKey& e : my_edges_) {
            const Eigen::VectorXd d = xi_self - z_self_[e];
            for (int j = 0; j < nz_; ++j) sq[j] = d(j) * d(j);
            const double rp = std::sqrt(numpy_pairwise_sum(sq.data(), sq.size()));
            res.r_prim_edge.push_back(rp);
            res.r_prim = std::max(res.r_prim, rp);
        }
    }
    res.hist = std::move(hist);
    res.hold = false;
    res.achieved_rounds = achieved;
    res.n_timeouts = transport_->take_timeouts();
    res.t_node = t_node_acc;
    res.t_edge_solve = t_edge_acc;
    return res;
}

}  // namespace admm
