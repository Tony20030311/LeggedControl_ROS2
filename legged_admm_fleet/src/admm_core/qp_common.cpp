// Shared CSC + OSQP plumbing implementation.
#include "legged_upper_control/admm_qp_common.hpp"

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace admm {

// Process-wide QP health accumulators (read-and-reset, like the transport's take_*()).
// Telemetry only — nothing here feeds a solution value, so bit-parity is unaffected.
namespace {
std::atomic<unsigned long> g_qp_nonconverged{0};
std::atomic<int> g_qp_last_status{0};
}  // namespace

CscPattern build_csc_pattern(long long m, long long n, const std::vector<Trip>& t) {
    CscPattern out;
    out.m = m;
    out.n = n;
    std::vector<std::size_t> order(t.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        return t[a].c != t[b].c ? t[a].c < t[b].c : t[a].r < t[b].r;
    });
    out.i.resize(t.size());
    out.perm.resize(t.size());
    out.p.assign(static_cast<std::size_t>(n) + 1, 0);
    for (std::size_t j = 0; j < order.size(); ++j) {
        const Trip& tr = t[order[j]];
        if (j > 0) {
            const Trip& pr = t[order[j - 1]];
            if (pr.c == tr.c && pr.r == tr.r)
                throw std::runtime_error("build_csc_pattern: duplicate (row,col)");
        }
        out.i[j] = static_cast<c_int>(tr.r);
        out.perm[order[j]] = j;
        out.p[static_cast<std::size_t>(tr.c) + 1] += 1;
    }
    for (long long c = 0; c < n; ++c) out.p[c + 1] += out.p[c];
    return out;
}

void scatter_values(const CscPattern& pat, const std::vector<double>& trip_vals,
                    std::vector<double>& csc_data) {
    csc_data.resize(trip_vals.size());
    for (std::size_t t = 0; t < trip_vals.size(); ++t) csc_data[pat.perm[t]] = trip_vals[t];
}

void clip_bounds(std::vector<double>& l, std::vector<double>& u) {
    for (double& v : l) v = std::max(v, -OSQP_INFTY);
    for (double& v : u) v = std::min(v, OSQP_INFTY);
}

double numpy_pairwise_sum(const double* a, std::size_t n) {
    if (n < 8) {
        double res = 0.0;
        for (std::size_t i = 0; i < n; ++i) res += a[i];
        return res;
    }
    if (n <= 128) {
        double r[8] = {a[0], a[1], a[2], a[3], a[4], a[5], a[6], a[7]};
        std::size_t i = 8;
        for (; i < n - (n % 8); i += 8) {
            r[0] += a[i];
            r[1] += a[i + 1];
            r[2] += a[i + 2];
            r[3] += a[i + 3];
            r[4] += a[i + 4];
            r[5] += a[i + 5];
            r[6] += a[i + 6];
            r[7] += a[i + 7];
        }
        double res = ((r[0] + r[1]) + (r[2] + r[3])) + ((r[4] + r[5]) + (r[6] + r[7]));
        for (; i < n; ++i) res += a[i];
        return res;
    }
    std::size_t n2 = n / 2;
    n2 -= n2 % 8;
    return numpy_pairwise_sum(a, n2) + numpy_pairwise_sum(a + n2, n - n2);
}

OsqpProblem::~OsqpProblem() {
    if (work_) osqp_cleanup(work_);
}

void OsqpProblem::reset() {
    if (work_) {
        osqp_cleanup(work_);
        work_ = nullptr;  // is_setup() -> false, next solve() re-runs setup()
    }
    update_failed_ = false;  // fresh setup() carries fresh data
}

void OsqpProblem::setup(const CscPattern& P, const std::vector<double>& Px,
                        const std::vector<double>& q, const CscPattern& A,
                        const std::vector<double>& Ax, std::vector<double> l,
                        std::vector<double> u) {
    clip_bounds(l, u);
    std::vector<c_int> Pp(P.p), Pi(P.i), Ap(A.p), Ai(A.i);
    std::vector<double> Pxv(Px), Axv(Ax), qv(q);

    csc Pm, Am;
    Pm.nzmax = static_cast<c_int>(Pxv.size());
    Pm.m = static_cast<c_int>(P.m);
    Pm.n = static_cast<c_int>(P.n);
    Pm.p = Pp.data();
    Pm.i = Pi.data();
    Pm.x = Pxv.data();
    Pm.nz = -1;
    Am.nzmax = static_cast<c_int>(Axv.size());
    Am.m = static_cast<c_int>(A.m);
    Am.n = static_cast<c_int>(A.n);
    Am.p = Ap.data();
    Am.i = Ai.data();
    Am.x = Axv.data();
    Am.nz = -1;

    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.warm_start = 1;
    settings.eps_abs = 1e-6;
    settings.eps_rel = 1e-6;
    settings.polish = 1;
    settings.max_iter = 8000;
    // Fixed-iteration rho adaptation (multiple of check_termination=25). OSQP 0.6.x
    // default (0) adapts on WALL-CLOCK time -> solves are not run-to-run reproducible.
    settings.adaptive_rho_interval = 25;
    // parity debugging only: NOT used in production paths
    if (const char* env = std::getenv("ADMM_OSQP_POLISH"))
        settings.polish = std::atoi(env);
    if (const char* env = std::getenv("ADMM_OSQP_ADAPTIVE_INTERVAL"))
        settings.adaptive_rho_interval = std::atoi(env);

    OSQPData data;
    data.n = static_cast<c_int>(A.n);
    data.m = static_cast<c_int>(A.m);
    data.P = &Pm;
    data.A = &Am;
    data.q = qv.data();
    data.l = l.data();
    data.u = u.data();

    const c_int err = osqp_setup(&work_, &data, &settings);
    if (err != 0) throw std::runtime_error("osqp_setup failed: " + std::to_string(err));
}

void OsqpProblem::update(const std::vector<double>* q, std::vector<double> l,
                         std::vector<double> u, const std::vector<double>* Ax) {
    if (q) update_failed_ |= (osqp_update_lin_cost(work_, q->data()) != 0);
    clip_bounds(l, u);
    update_failed_ |= (osqp_update_bounds(work_, l.data(), u.data()) != 0);
    if (Ax)
        update_failed_ |=
            (osqp_update_A(work_, Ax->data(), OSQP_NULL, static_cast<c_int>(Ax->size())) != 0);
}

void OsqpProblem::update_q_only(const std::vector<double>& q) {
    update_failed_ |= (osqp_update_lin_cost(work_, q.data()) != 0);
}

void OsqpProblem::update_Alu(const std::vector<double>& Ax, std::vector<double> l,
                             std::vector<double> u) {
    clip_bounds(l, u);
    update_failed_ |= (osqp_update_bounds(work_, l.data(), u.data()) != 0);
    update_failed_ |= (osqp_update_A(work_, Ax.data(), OSQP_NULL, static_cast<c_int>(Ax.size())) != 0);
}

OsqpProblem::Result OsqpProblem::solve() {
    // A rejected update means the workspace holds stale data — a "successful" solve
    // of the wrong problem. Fail loudly as NaN; the caller's finite-guard cold-starts.
    if (update_failed_ || osqp_solve(work_) != 0) {
        update_failed_ = false;  // one-shot: the rebuild path re-setups from scratch
        Result nan_r;
        nan_r.status = "update rejected";
        nan_r.x = Eigen::VectorXd::Constant(work_->data->n,
                                            std::numeric_limits<double>::quiet_NaN());
        return nan_r;
    }
    Result r;
    r.status = work_->info->status;
    const c_int n = work_->data->n;
    r.x.resize(n);
    // Mirror osqp-python: on infeasible/non-convex statuses the wrapper returns
    // None entries which np.asarray(..., float) turns into NaN. The raw C
    // solution->x holds OSQP_NAN = (double)0x7fc00000 = 2143289344.0 — a plain
    // number, NOT a NaN — so copying it through would silently diverge.
    const c_int sv = work_->info->status_val;
    const bool x_present = !(sv == OSQP_PRIMAL_INFEASIBLE ||
                             sv == OSQP_PRIMAL_INFEASIBLE_INACCURATE ||
                             sv == OSQP_DUAL_INFEASIBLE ||
                             sv == OSQP_DUAL_INFEASIBLE_INACCURATE ||
                             sv == OSQP_NON_CVX);
    if (x_present) {
        // Instrumentation only (no control-path effect): OSQP is itself an ADMM solver,
        // so a non-SOLVED iterate is not guaranteed to satisfy the constraints — including
        // the hard k=0 CBF row. Infeasible/non-convex already become NaN above and are
        // handled; what lands here is MAX_ITER_REACHED / SOLVED_INACCURATE / TIME_LIMIT,
        // which are accepted silently today. Count them so we can find out whether that
        // ever actually happens before deciding any policy. See take_qp_health().
        if (sv != OSQP_SOLVED) {
            g_qp_nonconverged.fetch_add(1, std::memory_order_relaxed);
            g_qp_last_status.store(static_cast<int>(sv), std::memory_order_relaxed);
        }
        for (c_int j = 0; j < n; ++j) r.x(j) = work_->solution->x[j];
    } else {
        r.x.setConstant(std::numeric_limits<double>::quiet_NaN());
    }
    return r;
}

QpHealth take_qp_health() {
    QpHealth h;
    h.n_nonconverged = g_qp_nonconverged.exchange(0, std::memory_order_relaxed);
    h.last_status_val = g_qp_last_status.exchange(0, std::memory_order_relaxed);
    return h;
}

}  // namespace admm
