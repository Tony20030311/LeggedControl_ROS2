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
uint64  bytes_tx
uint64  bytes_rx
float64[] r_prim_edge    # primal residual per adopted edge at cycle end
float64[] r_prim_hist    # primal residual per ADMM round (convergence trajectory; len = rounds run)
bool    hold             # HOLD this cycle (barrier timed out -> reused old target)
bool    reset            # NaN/cold-start cycle: solved but published no target (fleet cold-starts next slot)
