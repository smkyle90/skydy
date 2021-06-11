"""Test the example function
"""

import pytest
import sympy as sym


@pytest.mark.rigidbody
def test_Body():
    from skydy.inertia import InertiaMatrix, MassMatrix
    from skydy.rigidbody import Body, BodyCoordinate, BodyForce, BodyTorque

    # Test empty initialiser
    b0 = Body()

    assert b0.name

    assert isinstance(b0.mass_matrix, MassMatrix)
    assert isinstance(b0.inertia_matrix, InertiaMatrix)
    assert b0.linear_forces == []
    assert b0.linear_torques == []

    # String body name
    body_name = "test"
    b2 = Body(body_name)

    assert b2.name == body_name

    # Run methods

    # Get body linear and angular velocities
    v, w = b2.body_twists()

    b2.kinetic_energy()

    g = sym.Symbol("g")
    gravity = sym.Matrix([0, 0, g])
    b2.potential_energy(gravity)

    F1 = BodyForce("1", x_dir=True)
    force_loc = BodyCoordinate("PF1", 0, 0, 0)
    b2.add_force(F1, force_loc)

    T1 = BodyTorque("1", x_dir=True)
    T2 = BodyTorque("2", y_dir=True)

    torque_loc = BodyCoordinate("PT1", 0, 0, 0)
    b2.add_torque(T1, torque_loc)
    b2.add_torque(T2, torque_loc)

    print(b2.linear_forces)
    print(b2.linear_torques)

    print(b2.dims)
    print(b2.dims.symbols())
    print(b2.dims.values())
    b2.dims.assign_values([10, 0, 0])
    print(b2.dims.symbols())
    print(b2.dims.values())


@pytest.mark.rigidbody
def test_Body_draw():
    from skydy.rigidbody import Body

    body = Body()
    ax = body.draw()
    body.apply_constraint(0)
    body.apply_constraint(1)
    body.apply_constraint(2)
    body.apply_constraint(4)
    body.apply_constraint(5)

    ax = body.draw()
    ax = body.draw(ref_body=None)
    ax = body.draw(ref_joint=None)


@pytest.mark.rigidbody
def test_Ground():
    from skydy.rigidbody import Ground

    g = Ground()

    print(g.symbols())
    print(g.values())
    print(g.as_dict())
    print(g.dims.symbols())
    print(g.dims.values())
    print(g.dims.as_dict())
    print(g.pos_body)
    print(g.rot_body)
