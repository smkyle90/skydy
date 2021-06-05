"""Test the example function
"""

import pytest


@pytest.mark.connectors
def test_DOF():
    from skydy.connectors import DOF

    d = DOF(0)

    assert d.idx == 0
    assert d.free
    assert d.const_value == 0

    d = DOF(1, False)

    assert d.idx == 1
    assert not d.free
    assert d.const_value == 0

    d = DOF(2, False, 10)

    assert d.idx == 2
    assert not d.free
    assert d.const_value == 10


@pytest.mark.connectors
def test_SpringDamperCoeffs():
    from skydy.connectors import DOF, SpringDamperCoeffs

    s = SpringDamperCoeffs("1")


@pytest.mark.connectors
def test_Joint():
    from skydy.connectors import DOF, Joint
    from skydy.rigidbody import BodyCoordinate

    # Simple joint that moves in x-direction
    p0 = BodyCoordinate("O")
    p1 = BodyCoordinate("G/O", 0, 0, 0)
    j1 = Joint(p0, p1, [DOF(0,)])

    j1.body_in_coord
    j1.body_out_coord
    j1.dof

    j2 = Joint(p0, p1, [DOF(0,)], "test")


@pytest.mark.connectors
def test_Connection():
    from skydy.connectors import DOF, Connection, Joint
    from skydy.rigidbody import Body, BodyCoordinate

    # Simple joint that moves in x-direction
    p0 = BodyCoordinate("O")
    p1 = BodyCoordinate("G/O", 0, 0, 0)
    j1 = Joint(p0, p1, [DOF(0,)])

    b1 = Body()
    b2 = Body()

    cnx = Connection(b1, j1, b2)

    cnx.body_in
    cnx.body_out
    cnx.joint
