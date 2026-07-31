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

# Observation-belief-trust (spec 5.2): what the sender SAW, not what it concluded. A residual is
# unfalsifiable; a position is constrained by sensing range and by which sightlines the arena
# permits, so a receiver can refute a fabricated report without seeing either party.
# Bounded (review finding 5): these are attacker-controlled and deserialise on the thread that
# services the consensus barrier, on a host also running a 250 Hz whole-body controller. The
# bound is a statically known cap, not a tuned fleet size -- 8 comfortably covers every roster
# this project runs (N=3 primarily, N=5 tested) with room to spare -- so an oversized claim is
# rejected by the deserialiser itself instead of reaching gateCheck's own size checks at all.
int32[<=8]   ev_peer   # peer ids the sender observed (its own observation channel, spec 5.1)
                       # during the slot named by ev_slot. Empty = "saw nobody that slot", which
                       # is also what a pre-evidence sender means, so old peers and the loopback
                       # test transport are unaffected.
float64[<=16] ev_pos   # observed (x,y) per entry, length exactly 2 * len(ev_peer).
uint64    ev_slot      # the slot this evidence describes. NOT the same as cycle_id above:
                       # send_state runs at the TOP of AgentCore::step, before this cycle's
                       # evidence is even produced, so every broadcast necessarily carries the
                       # PREVIOUS cycle's observations. Without this stamp a receiver would pair
                       # them against the wrong slot's claims and every relayed term would carry
                       # a systematic bias (one slot of peer travel, ~0.055 m at MAX_VX).
