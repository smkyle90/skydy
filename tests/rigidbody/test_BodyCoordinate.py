"""Test the example function
"""

import pytest


@pytest.mark.rigidbody
def test_BodyCoordinate():
    from skydy.rigidbody import BodyCoordinate

    body_name = "1"
    body_coord = BodyCoordinate(body_name)

    assert body_coord.name == body_name
    for k, v in body_coord.as_dict().items():
        assert v == 0

    # Run methods
    body_coord.symbols()
    body_coord.values()
    body_coord.as_dict()

    # Assign some values
    x_coord = 3
    y_coord = 10
    z_coord = 25

    body_coord = BodyCoordinate(body_name, x_coord, y_coord, z_coord)

    assert body_coord.values()[0] == x_coord
    assert body_coord.values()[1] == y_coord
    assert body_coord.values()[2] == z_coord

    y_coord = 0
    body_coord = BodyCoordinate(body_name, x_coord, y_coord, z_coord)

    assert body_coord.symbols()[1] == y_coord
