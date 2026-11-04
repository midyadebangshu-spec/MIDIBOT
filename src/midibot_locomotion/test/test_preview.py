import numpy as np

from midibot_locomotion.preview import PreviewAxis, PreviewGains


def test_constant_zmp_reached_with_zero_error():
    g = PreviewGains(0.7)
    ax = PreviewAxis(g)
    ref = np.full(600, 0.05)
    for k in range(400):
        ax.step(ref[k:k + g.n_preview + 1])
    assert abs(ax.zmp - 0.05) < 1e-4
    assert abs(ax.x[1]) < 1e-3


def test_preview_reduces_error_before_reference_step():
    """With preview the CoM starts moving before the ZMP reference jumps."""
    g = PreviewGains(0.7)
    ax = PreviewAxis(g)
    ref = np.zeros(800)
    ref[200:] = 0.1
    moved_early = False
    for k in range(400):
        ax.step(ref[k:k + g.n_preview + 1])
        if k == 190 and abs(ax.x[0]) > 1e-3:
            moved_early = True
    assert moved_early
    assert abs(ax.zmp - 0.1) < 1e-3
