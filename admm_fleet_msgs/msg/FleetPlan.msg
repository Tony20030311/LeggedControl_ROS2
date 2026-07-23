# Fleet-logic (hosted on robot1) -> all agents, event-triggered (plan step 3, PREP).
# Slot goals, optional A* detour polylines, and yaw-latch clears. Not per-cycle.
uint64  cycle_id       # cycle this plan takes effect from
float64 tx_wall
int32[]   robot_ids    # ordering key for the parallel arrays below
float64[] goals        # flattened (x, y) per robot_id, length 2*len(robot_ids)
int32[]   path_counts  # detour polyline vertex count per robot (0 = go straight to goal)
float64[] paths        # flattened (x, y) vertices, concatenated in robot_ids order
bool[]    clear_latch  # per robot: reset yaw latch (new goal is a clean slate)
