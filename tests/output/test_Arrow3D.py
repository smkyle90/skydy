"""Test the output functions
"""

import pytest


@pytest.mark.output
def test_Arrow3D():
    import matplotlib.pyplot as plt

    from skydy.output import Arrow3D

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    basis = [
        (0, 2),
        (0, 3),
        (0, 4),
    ]
    arrow = Arrow3D(
        *basis,
        mutation_scale=5,
        lw=1,
        arrowstyle="-|>",
        color="r",
    )
    ax.add_artist(arrow)

    plt.show()
