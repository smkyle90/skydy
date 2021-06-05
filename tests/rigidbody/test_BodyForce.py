"""Test the example function
"""

import pytest


@pytest.mark.rigidbody
def test_BodyForce():
    from skydy.rigidbody import BodyForce

    body_name = "1"
    F1 = BodyForce(body_name)
    print(F1.symbols())
    Fx = BodyForce("1", x_dir=True)
    print(Fx.symbols())
    Fy = BodyForce("2", y_dir=True)
    print(Fy.symbols())
    Fz = BodyForce("3", z_dir=True)
    print(Fz.symbols())


@pytest.mark.rigidbody
def test_BodyTorque():
    from skydy.rigidbody import BodyTorque

    body_name = "1"
    T1 = BodyTorque(body_name)
    print(T1.symbols())
    Tx = BodyTorque("1", x_dir=True)
    print(Tx.symbols())
    Ty = BodyTorque("2", y_dir=True)
    print(Ty.symbols())
    Tz = BodyTorque("3", z_dir=True)
    print(Tz.symbols())
