# Non-owner -> edge-owner, once per ADMM iteration (plan step 4).
# -> /admm/edge_i_j/xi.  Carries this side's node solution xi and dual lam for edge (i,j).
uint64  cycle_id
int32   iter           # ADMM round 0..P_ITERS-1
float64 tx_wall
int32   edge_i         # edge key (i<j), the canonical owner
int32   edge_j
int32   from_robot     # which endpoint sent this (i or j)
float64[] xi           # node solution for the shared block, length 6N
float64[] lam          # scaled dual for this copy, length 6N
