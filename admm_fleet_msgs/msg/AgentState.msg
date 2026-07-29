# Per-robot ADMM state, broadcast once at the head of each control cycle.
# -> /robotN/admm/state  (plan step 2). Neighbors barrier on receiving all peers'.
uint64  cycle_id       # floor(sim_now / TS); the time-slot all agents share
int32   iter           # 0 for the cycle-head broadcast
float64 tx_wall        # sender wall-clock at publish (s); rx_wall - tx_wall = hop latency
int32   robot_id
float64[4] xnow        # world state (px, py, vx, vy)
float64[]  xibar       # operating-point plan, length 6N (N horizon steps x [p,v,a] 2D)
bool    reset          # NaN payload last cycle -> whole-fleet cold start this cycle
int32[] members        # the sender's CURRENT roster (its dogs_). A receiver missing from this
                       # list has been unilaterally dropped by the sender, which is positive
                       # evidence the sender left the consensus while still talking — the one
                       # state that deadlocks everyone else (it stays warm, never joins a reset,
                       # and every survivor holds forever). Empty = pre-members sender: treat as
                       # "everyone still in", so old peers and the loopback test transport are
                       # unaffected. Seed of the Phase 2 view_id.
