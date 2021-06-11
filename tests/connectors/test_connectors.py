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
def test_Joint():
    from skydy.connectors import DOF, Joint
    from skydy.rigidbody import BodyCoordinate

    # Simple joint that moves in x-direction
    p0 = BodyCoordinate("O")
    p1 = BodyCoordinate("G/O", 0, 0, 0)
    j1 = Joint(p0, p1, [DOF(0,)])

    print(j1.body_in_coord)
    print(j1.body_out_coord)
    print(j1.dof)

    for dof in j1.dof:
        if dof.idx == 0:
            assert dof.free
        else:
            assert not dof.free


@pytest.mark.connectors
def test_Connection():
    from skydy.connectors import DOF, Connection, Joint
    from skydy.rigidbody import Body, BodyCoordinate, Ground

    print("==Sliding==")
    # Simple joint that moves in x-direction
    p0 = BodyCoordinate("O")
    p1 = BodyCoordinate("G/O", 0, 0, 0)

    j1 = Joint(p0, p1, [DOF(0)])

    b1 = Ground()
    b2 = Body()

    cnx = Connection(b1, j1, b2)

    print(cnx.body_in.pos_body)
    print(cnx.body_in.rot_body)

    print(cnx.body_out.pos_body)
    print(cnx.body_out.rot_body)

    print(cnx.joint)

    # Propagate the global coordinates from the ground
    cnx.global_configuration()

    print(cnx.body_in.pos_body)
    print(cnx.body_in.rot_body)

    print(cnx.body_out.pos_body)
    print(cnx.body_out.rot_body)

    print(cnx.joint)

    # Simple joint that moves in x-direction and y-direction
    # and fixed in z-direction
    print("==Constant value==")
    p0 = BodyCoordinate("O")
    p1 = BodyCoordinate("G/O", 0, 0, 0)

    j1 = Joint(p0, p1, [DOF(0), DOF(1), DOF(2, False, 10)])

    b1 = Ground()
    b2 = Body()

    cnx = Connection(b1, j1, b2)

    print(cnx.body_in.pos_body)
    print(cnx.body_in.rot_body)

    print(cnx.body_out.pos_body)
    print(cnx.body_out.rot_body)

    print(cnx.joint)

    # Propagate the global coordinates from the ground
    cnx.global_configuration()

    print(cnx.body_in.pos_body)
    print(cnx.body_in.rot_body)

    print(cnx.body_out.pos_body)
    print(cnx.body_out.rot_body)

    print(cnx.joint)

    # Rotation
    print("==Rotation==")
    p1 = BodyCoordinate("G2/O", 2, 0, 0)

    j2 = Joint(p0, p1, [DOF(4,)])

    b2 = Body()

    cnx = Connection(b1, j2, b2)

    print(cnx.body_in.pos_body)
    print(cnx.body_in.rot_body)

    print(cnx.body_out.pos_body)
    print(cnx.body_out.rot_body)

    print(cnx.joint)

    # Propagate the global coordinates from the ground
    cnx.global_configuration()

    print(cnx.body_in.pos_body)
    print(cnx.body_in.rot_body)

    print(cnx.body_out.pos_body)
    print(cnx.body_out.rot_body)

    print(cnx.joint)
