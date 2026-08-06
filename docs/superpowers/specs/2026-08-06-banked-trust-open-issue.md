# Open issue: a peer can bank trust and spend it on a lie

**Status: not fixed.** An attempted fix was reverted the same day; see the last section for
why, and for what the fix has to carry with it.

## The problem

`trust_step_self` clamps the actor ledger to `[l_evict - clamp_step, l_max]`, so a peer that
behaves well accumulates positive credit up to `l_max = +4.6`. That credit has to be spent
before any suspicion can start, and it grows with however long the peer stayed quiet.

Measured 2026-08-06, `ARM=a2`, lidar observation, `plum_dense`, `LIE=-0.30`. The
compromised robot's own ledger, in order:

    resid 0.023  l_act +0.500     honest, before the attack
    resid 0.007  l_act +0.500
    resid 0.032  l_act +0.889
    resid 0.015  l_act +0.803
    resid 0.028  l_act +1.191
    resid 0.406  l_act +1.956     <- attack armed; still positive
    resid 0.424  l_act +1.956
    resid 0.412  l_act -0.139     <- first step barely crosses zero
    resid 0.409  l_act -2.139
    resid 0.423  l_act -4.139

Every convicting step is a clean `-2.0` (`clamp_step`), and the residual it measures is
0.406-0.424 m against an injected 0.424 m -- the detection is exact. The first two of those
steps went into buying back the peer's own reputation. Conviction needs `l_evict = -9.2`,
about six steps from clean; from +1.956 it needs seven.

**The strategy this rewards is: be good, then lie.** Wait longer, get a bigger discount. The
run above never convicted, and this is one of the reasons.

## The fix that was tried

Cap the upper end at `0.0` instead of `l_max`: innocence is the ceiling. Behaving well
returns a suspected peer to clean and stops there, so recovery from a false positive still
works -- that is what the positive side was for -- but a past cannot pay for a future.

It is a one-line change and it is probably right. It was reverted because of what it takes
with it, all of which surfaced within twenty minutes:

1. **`lambda` becomes dead.** Decay is written `(L > 0.0) ? lambda * L : L`. With the
   ceiling at 0, `L` is never positive, so the decay branch never executes. The parameter
   would have to be removed or documented as inert, and `test_trust`'s once-vs-twice decay
   test becomes a statement about unreachable states.

2. **A reviewed derivation stops holding.** `test_split_end_to_end_matches_the_reviewed_derivation`
   asserts the smear arm lands at `total == +9.200`, from
   `specs/2026-08-02-actor-reporter-split-derivation.md`. That arm's whole design is "a
   smearer honest about its own position keeps a positive total and is not evicted, only
   silenced". With a ceiling of 0 the number is 0, and the derivation has to be redone, not
   edited.

3. **Every recorded trust figure moves.** The measured `-11.200` floors and `0.000` testimony
   weights in `smearer-unconvictable-inequality` were taken under the old ceiling.

## What a real fix has to include

- the ceiling change itself
- a decision on `lambda`: remove it, or state that it is inert and why
- `2026-08-02-actor-reporter-split-derivation.md` re-derived under the new ceiling, with the
  smear arm's "not convicted" property re-established some other way (it currently rests on
  the total staying positive, which will no longer be available)
- the smear arm re-measured, both fleet sizes
- G4 and the anchoring A/B re-run, since the trust layer sits in the same binary

Half a day, not an hour. Worth doing properly rather than at the end of a session.

## Related, and not the same

The same run showed the belief layer abstaining on 93% of its evaluations of the liar,
because `interp_at` required an observation bracketing an exact past instant and a real
sensor rarely has one. That has a separate fix (a bounded tolerance, `obs_tol_s`), which is
in and took the evidence rate from 6% to 52%. Even with it the run did not convict -- the
banked credit above is the remaining half of the story.
