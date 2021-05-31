"""Test the example function
"""

import pytest


@pytest.mark.rigidbody
def test_BodyForce():
    from lib.rigidbody import BodyCoordinate, BodyForce

    body_name = "1"
    F1 = BodyForce(body_name, BodyCoordinate(body_name))

    F1.location
    F1.direction


@pytest.mark.rigidbody
def test_BodyTorque():
    from lib.rigidbody import BodyTorque

    body_name = "1"
    F1 = BodyTorque(body_name)
