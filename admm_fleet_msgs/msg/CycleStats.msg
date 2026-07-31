# Per-robot per-cycle telemetry, the G5 communication-measurement instrument (plan step 6, §86).
# Recorded to rosbag2; offline notebook -> round histograms, latency CDF, bytes, r_prim curves.
uint64  cycle_id
int32   robot_id
int32   achieved_rounds  # ADMM iters completed within the TS=0.10s slot (20 ms per-hop deadline)
float64 t_node           # node-QP solve time this cycle (s)
float64 t_edge_solve     # edge-QP solve time (owner only; 0 otherwise)
float64 t_wait_state     # barrier wait for peers' AgentState (s)
float64 t_wait_xi        # wait for adopted-edge EdgeXi (s)
float64 t_wait_z         # wait for own EdgeZ (s)
float64 t_cycle_wall     # full cycle wall time (s)
float64 t_rx_mean        # mean one-way rx latency (rx_wall - tx_wall) this cycle (s)
int32   n_timeouts       # per-hop timeouts this cycle
int32   n_stale          # messages that arrived for a slot older than the newest seen (late)
# Task 10 item 5: observation-channel samples the publisher-GID pin rejected this cycle (any
# peer, admm_agent_node's peer_truth_ subscription). Nonzero means a peer's ground-truth channel
# went dark to this agent -- a late-impostor attack (inject_forged_obs) OR a benign publisher
# restart, operationally indistinguishable either way. A run with a nonzero cumulative sum here
# has a channel that silently died and MUST NOT be reported as a clean result.
int32   n_obs_dropped
uint64  bytes_tx
uint64  bytes_rx
float64[] r_prim_edge    # primal residual per adopted edge at cycle end
float64[] r_prim_hist    # primal residual per ADMM round (convergence trajectory; len = rounds run)
bool    hold             # HOLD this cycle (barrier timed out -> reused old target)
bool    reset            # NaN/cold-start cycle: solved but published no target (fleet cold-starts next slot)

# Task 11 item 1: unthrottled per-peer belief telemetry, one entry per peer beliefStep() judged
# THIS cycle -- the replacement for the 1 Hz RCLCPP_*_THROTTLE lines in admm_agent_node.cpp's
# beliefStep(), whose throttle state is per CALL SITE, not per peer: with two peers the token
# alternates between them, so ~19 of 20 slots are unsampled and, because the peer map is ordered,
# the lowest-id peer wins the token almost every time -- the attacker on the third robot is
# essentially never printed. Deriving obs_sigma/refute_ratio from that samples exactly the tail
# calibration exists to bound. Parallel arrays, same length, same indexing as tel_peer (same
# convention as AgentState's ev_peer/ev_pos). Only populated when obs_gate2=true (arm A2 and its
# variants); gate2()'s A1 EMA residual keeps its own, separately-calibrated throttled log.
int32[]   tel_peer      # peer (target) robot id this row is about
float64[] tel_resid     # |observed - claimed| this slot; NaN when tel_abstain != 0 (no evidence)
float64[] tel_l_self    # first-hand log-odds for this peer AFTER this slot's update (l_self_[j])
float64[] tel_l_total   # first-hand + capped hearsay (trust_total), computed every row for
                         # telemetry even on a blocked peer, where the live code path skips it
int32[]   tel_abstain    # why tel_resid carries no evidence this slot:
                         #   0 = evidence produced (tel_resid is real)
                         #   1 = peer_blocked    (already fenced; trust_step_self not even called)
                         #   2 = no_fresh_claim  (this agent's own AgentState barrier held this slot)
                         #   3 = no_obs_buffer   (no truth sample covers this slot -- dead/late channel)
                         #   4 = out_of_range    (admm::visible()'s range gate: farther than obs_range_)
                         #   5 = occluded        (within range, but a pillar blocks the sightline)
                         # Review round 2: split from one "not_visible" code -- admm::visible() is
                         # both tests, and a document that reads every one of these as occlusion is
                         # only correct if range never binds, which needs checking, not assuming.
# Task 11 item 4: relayed reports this agent judged geometrically impossible THIS cycle
# (evidence_plausible() false -> refuted_llr applied against the REPORTER). With no attacker
# present every one of these is a SPURIOUS refutation -- an honest reporter's sightline grazing a
# pillar, noise, or timing skew -- and refute_ratio is calibrated from this count's rate over a
# clean-flight run (see docs/superpowers/results/2026-07-31-trust-calibration.md). Same
# read-and-reset-in-publishStats pattern as n_obs_dropped above.
int32   n_refuted
# Review round 2, I2': the DENOMINATOR for n_refuted -- every relayed-report entry this agent ran
# evidence_plausible() on this cycle, refuted or not. Without this, the refute-rate calibration
# has to be reconstructed from the throttled "smear-check coverage" log line (per call site, not
# per cycle -- the same sampling problem item 1 exists to avoid), or guessed. Same
# read-and-reset-in-publishStats pattern as n_refuted above.
int32   n_evidence_checked
