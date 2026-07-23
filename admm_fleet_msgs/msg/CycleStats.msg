# Per-robot per-cycle telemetry, the G5 communication-measurement instrument (plan step 6, §86).
# Recorded to rosbag2; offline notebook -> round histograms, latency CDF, bytes, r_prim curves.
uint64  cycle_id
int32   robot_id
int32   achieved_rounds  # ADMM iters completed before the 0.08s cycle deadline
float64 t_node           # node-QP solve time this cycle (s)
float64 t_edge_solve     # edge-QP solve time (owner only; 0 otherwise)
float64 t_wait_state     # barrier wait for peers' AgentState (s)
float64 t_wait_xi        # wait for adopted-edge EdgeXi (s)
float64 t_wait_z         # wait for own EdgeZ (s)
float64 t_cycle_wall     # full cycle wall time (s)
float64 t_cycle_sim      # full cycle sim time (s)
int32   n_timeouts       # per-hop timeouts this cycle
int32   n_stale          # messages dropped for stale (cycle_id, iter)
uint64  bytes_tx
uint64  bytes_rx
float64[] r_prim_edge    # primal residual per adopted edge at cycle end
float64[] r_prim_hist    # primal residual per ADMM round (convergence trajectory; len = rounds run)
bool    hold             # HOLD this cycle (barrier timed out -> reused old target)
bool    reset            # broadcast a cold-start flag this cycle
