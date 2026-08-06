#!/usr/bin/env python3
"""The anchoring A/B figure: every run plotted, because n is 3-4.

Two panels, not one with two y-axes: centre distance and body gap are different
quantities on different scales, and overlaying them would invent a comparison neither
measurement supports.

Every run is a dot. With this few samples an error bar would imply a distribution that
was never estimated, and the point of the figure is that one measure separates cleanly
while the other does not -- which you can only see by looking at the points.
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# 2026-08-06. LIE=-0.30 (the direction that puts the claimed position FURTHER away, which
# is the one anchoring can act on), LIE_DEAF=0, observation=lidar, same binary throughout,
# one parameter flipped. Raw logs in logs/.
CENTRE = {'on': [1.194, 1.1797, 1.1928], 'off': [0.979, 1.0038, 0.9970, 1.0409]}
GAP    = {'on': [0.421, 0.447, 0.389],   'off': [0.473, 0.467, 0.412, 0.543]}

BLUE, ORANGE = '#2a78d6', '#eb6834'      # validated categorical slots 1 and 2
INK, MUTED, GRID = '#1a1a19', '#6b6a63', '#e3e2dc'
D_MIN = 1.3                              # admm::D_MIN, the separation the fleet is meant to hold

fig, axes = plt.subplots(1, 2, figsize=(7.2, 3.4))
fig.patch.set_facecolor('#fcfcfb')

panels = [
    (axes[0], CENTRE, 'Centre distance to the liar', 'metres', True),
    (axes[1], GAP, 'Body gap (surface to surface)', 'metres', False),
]

for ax, data, title, unit, show_dmin in panels:
    ax.set_facecolor('#fcfcfb')
    for x, (arm, colour, label) in enumerate(
            [('on', BLUE, 'anchoring on'), ('off', ORANGE, 'anchoring off')]):
        v = data[arm]
        # Jitter only enough to stop identical values hiding each other.
        xs = [x + (i - (len(v) - 1) / 2) * 0.055 for i in range(len(v))]
        ax.plot(xs, v, 'o', color=colour, markersize=8, markeredgecolor='#fcfcfb',
                markeredgewidth=1.5, zorder=3, label=label if show_dmin else None)
        mean = sum(v) / len(v)
        ax.plot([x - 0.22, x + 0.22], [mean, mean], '-', color=colour, linewidth=2,
                zorder=2)
        ax.annotate(f'{mean:.3f}', (x + 0.26, mean), color=MUTED, fontsize=8,
                    va='center')

    if show_dmin:
        ax.axhline(D_MIN, color=MUTED, linewidth=1, linestyle=(0, (4, 3)), zorder=1)
        ax.annotate('D_MIN 1.30', (1.42, D_MIN), color=MUTED, fontsize=8,
                    va='bottom', ha='right')

    ax.set_xlim(-0.5, 1.5)
    ax.set_xticks([0, 1])
    ax.set_xticklabels(['on', 'off'], color=INK)
    ax.set_title(title, color=INK, fontsize=10, pad=8, loc='left')
    ax.set_ylabel(unit, color=MUTED, fontsize=8)
    ax.grid(axis='y', color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(GRID)
    ax.tick_params(colors=MUTED, labelsize=8)

axes[0].set_ylim(0.90, 1.36)
axes[1].set_ylim(0.33, 0.60)
axes[0].legend(frameon=False, fontsize=8, loc='lower left', labelcolor=INK)

fig.suptitle('Conservative anchoring, on vs off, same binary  (LIE = 0.42 m displacement, '
             'lidar observation)', color=INK, fontsize=10, x=0.02, ha='left', y=0.99)
fig.tight_layout(rect=(0, 0, 1, 0.94))
out = __file__.rsplit('/', 1)[0] + '/anchoring_ab.png'
fig.savefig(out, dpi=200, facecolor=fig.get_facecolor())
print('wrote', out)
