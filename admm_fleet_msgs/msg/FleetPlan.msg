# Slot goals from the standalone fleet_coordinator_node -> all agents. Event-triggered
# (goal change / rescue reassignment) + periodic republish, latched (transient_local).
int32[]   robot_ids    # ordering key for the goals array below
float64[] goals        # flattened (x, y) per robot_id, length 2*len(robot_ids)
