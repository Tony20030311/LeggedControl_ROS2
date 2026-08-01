#!/usr/bin/env python3
"""Set parameters on several nodes SIMULTANEOUSLY, and report the skew it achieved.

    arm_attack.py /admm_agent_2:inject_fake_offset:double:0.30 \
                  /admm_agent_1:inject_odom_fake:double:0.30 \
                  /admm_agent_3:inject_odom_fake:double:0.30

Why this exists instead of three `ros2 param set` calls:

Design spec §9 protocol item 2 requires both halves of the dual-channel forgery to switch on
together. Whichever half lands first, gate2() spends the gap differencing one forged channel
against one honest one -- 0.424 m of residual for LIE=0.30 (the offset goes into x AND y)
against a 0.30 m gate -- and blocks the attacker for the gap rather than for the attack.

Three `ros2 param set` processes, even backgrounded, do NOT land together: each is a fresh
rclpy process that must discover the graph before it can call anything, and three of them
contending measured 2.78 s apart (d_0801_035833: agent3 armed at 815.057, agent1 at 817.842,
and agent1 blocked the attacker at 815.705 -- inside its own blind window). That run's A1
number measured the harness.

One process pays discovery ONCE, waits until EVERY service is up, then dispatches all the
requests back to back, so the spread is service-call latency rather than process startup.
The measured spread is printed and belongs in the results table: an unreported race is what
made the earlier number meaningless, so the fix has to be checkable rather than asserted.

Exits nonzero if any service never appears, any call fails, or any node reports the parameter
as not settable -- admm_agent_node.cpp deliberately REFUSES unknown/launch-only parameters, and
a silently ignored arming knob is exactly the failure this whole harness discipline exists to
catch. --max-skew fails the run if the achieved spread exceeds a bound.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue
from rcl_interfaces.srv import SetParameters

TYPES = {"bool": 1, "int": 2, "double": 3, "string": 4}

args = [a for a in sys.argv[1:] if not a.startswith("--")]
opts = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
TIMEOUT = float(opts.get("timeout", 30.0))
MAX_SKEW = float(opts.get("max-skew", 0.0))  # 0 = report only

specs = []
for a in args:
    node, name, ty, val = a.split(":", 3)
    if ty not in TYPES:
        sys.exit("unknown type %r in %r (want %s)" % (ty, a, "/".join(TYPES)))
    pv = ParameterValue(type=TYPES[ty])
    if ty == "bool":
        pv.bool_value = val.lower() in ("1", "true", "yes")
    elif ty == "int":
        pv.integer_value = int(val)
    elif ty == "double":
        pv.double_value = float(val)
    else:
        pv.string_value = val
    specs.append((node, name, pv))
if not specs:
    sys.exit("nothing to set")

rclpy.init()
n = Node("arm_attack")
clients = {}
for node, _name, _pv in specs:
    clients.setdefault(node, n.create_client(SetParameters, node.rstrip("/") + "/set_parameters"))

# WAIT FOR EVERY SERVICE FIRST. Dispatching as each becomes ready would reintroduce exactly the
# skew this script exists to remove -- the whole point is that no request goes out until all of
# them can go out.
deadline = time.monotonic() + TIMEOUT
for node, c in clients.items():
    while not c.service_is_ready():
        if time.monotonic() > deadline:
            sys.exit("FAIL: %s/set_parameters never appeared within %.0fs" % (node, TIMEOUT))
        rclpy.spin_once(n, timeout_sec=0.05)

# One request per node, carrying all of that node's parameters, dispatched back to back.
by_node = {}
for node, name, pv in specs:
    by_node.setdefault(node, []).append(Parameter(name=name, value=pv))

futures, stamps = {}, {}
t0 = time.monotonic()
for node, params in by_node.items():
    futures[node] = clients[node].call_async(SetParameters.Request(parameters=params))
    stamps[node] = time.monotonic()
dispatch_skew = max(stamps.values()) - min(stamps.values())

rc = 0
done = {}
deadline = time.monotonic() + TIMEOUT
while len(done) < len(futures):
    if time.monotonic() > deadline:
        missing = [k for k in futures if k not in done]
        print("FAIL: no response from %s" % ", ".join(missing))
        rc = 1
        break
    rclpy.spin_once(n, timeout_sec=0.05)
    for node, f in futures.items():
        if node not in done and f.done():
            done[node] = time.monotonic()
            for p, res in zip(by_node[node], f.result().results):
                status = "ok" if res.successful else ("REFUSED: " + res.reason)
                print("  %s %s -> %s" % (node, p.name, status))
                if not res.successful:
                    rc = 1

if done:
    ack_skew = max(done.values()) - min(done.values())
    print("dispatch skew %.4f s   ack skew %.4f s   total %.2f s"
          % (dispatch_skew, ack_skew, max(done.values()) - t0))
    if MAX_SKEW > 0 and ack_skew > MAX_SKEW:
        print("FAIL: ack skew %.4f s exceeds --max-skew=%.4f s" % (ack_skew, MAX_SKEW))
        rc = 1

n.destroy_node()
rclpy.shutdown()
sys.exit(rc)
