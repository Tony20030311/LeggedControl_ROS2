#!/usr/bin/env python3
"""End-of-run separation report over g3_dist_logger.py's dist.csv.

Splits the two questions the old single DMIN_ABORT number conflated:

  contact  -- did the base boxes actually interpenetrate? Below 0.867 m centre
              distance (2x the 0.83x0.25 box half-diagonal) they do at ANY yaw.
              That is the run scripts' abort line, and it is a real failure.
  margin   -- how long did the CBF barrier sit negative (d < D_MIN)? Transient
              h<0 on swaps and turns is expected and NOT a failure; it is a
              quality number, so it is reported here instead of aborting.

Centre distance is orientation-blind, so a sub-1.22 figure bounds the knee
envelope pessimistically -- phys_gap_logger.py is the honest instrument for
"did anything nearly touch". This only reads what the run already recorded.

Reads min_pair by COLUMN NAME, so it is correct for any fleet size (the run
scripts' live guard still hardcodes `cut -d, -f8`, i.e. exactly 3 robots).

Usage: dist_summary.py <dist.csv> [d_min=1.3] [knee=1.22] [contact=0.867]
Exit 1 if the logger produced no usable data -- a run whose collision guard
never ran must not be reported as a pass.
"""
import csv
import sys

path = sys.argv[1]
d_min = float(sys.argv[2]) if len(sys.argv) > 2 else 1.3
knee = float(sys.argv[3]) if len(sys.argv) > 3 else 1.22
contact = float(sys.argv[4]) if len(sys.argv) > 4 else 0.867

try:
    with open(path) as f:
        rows = [r for r in csv.DictReader(f) if r.get("min_pair")]
except OSError as e:
    print("dist_summary: cannot read %s (%s) -- collision guard was OFF" % (path, e))
    sys.exit(1)

if len(rows) < 10:
    print("dist_summary: only %d sample(s) in %s -- collision guard was OFF this run"
          % (len(rows), path))
    sys.exit(1)

d = [float(r["min_pair"]) for r in rows]
t = [float(r["t"]) for r in rows]
span = t[-1] - t[0]
dt = span / max(1, len(d) - 1)  # logger is fixed-rate; span/n beats trusting a nominal 20 Hz


def below(x):
    return sum(1 for v in d if v < x) * dt


t_contact = below(contact)
print("separation: min=%.3f mean=%.3f over %.1fs | "
      "below D_MIN %.2f: %.1fs (%.0f%%) | below knee %.2f: %.1fs | contact %.2f: %.1fs"
      % (min(d), sum(d) / len(d), span,
         d_min, below(d_min), 100.0 * below(d_min) / span if span else 0.0,
         knee, below(knee), contact, t_contact))

# The run scripts' live guard only samples the LAST dist.csv line every 5 s -- 1 of every 100
# rows -- so an excursion shorter than the poll interval slips past it and the run still prints
# "no contact". Observed: a hard_through=2 plum run reached 0.526 m for 4.3 s and reported PASS.
# This pass reads every row, so it is the one that must fail the run.
if t_contact > 0.0:
    print("dist_summary: %.1fs below the %.2f m contact line (min %.3f) -- NOT a clean run"
          % (t_contact, contact, min(d)))
    sys.exit(1)
