# Edge-owner -> non-owner, once per ADMM iteration (plan step 4).
# -> /admm/edge_i_j/z.  The owner solves the edge-QP and returns both consensus copies.
uint64  cycle_id
int32   iter
float64 tx_wall
int32   edge_i         # edge key (i<j)
int32   edge_j
float64[] z_i          # z^{ij,i} consensus copy for endpoint i, length 6N
float64[] z_j          # z^{ij,j} consensus copy for endpoint j, length 6N
