"""Test the example function
"""

import numpy as np
import pytest
import sympy as sym


@pytest.mark.rigidbody
def test_Body():
    from lib.inertia import InertiaMatrix, MassMatrix
    from lib.rigidbody import Body, BodyCoordinate, BodyForce, BodyTorque

    # Test empty initialiser
    b0 = Body()

    assert b0.name == "0"
    assert b0.body_id == 0
    assert Body.id_counter == 1
    assert isinstance(b0.mass_matrix, MassMatrix)
    assert isinstance(b0.inertia_matrix, InertiaMatrix)
    assert b0.linear_forces == []
    assert b0.linear_torques == []

    # Ensure it counts up on another empty initialiser
    b1 = Body()

    assert b1.name == "1"
    assert b1.body_id == 1

    # String body name
    body_name = "test"
    b2 = Body(body_name)

    assert b2.name == body_name

    # Run methods

    # Get body linear and angular velocities
    # v, w = b2.body_twists(b2.r_body, b2.R_body)

    # # b2.body_velocity(point_on_body)

    b2.kinetic_energy()

    g = sym.Symbol("g")
    gravity = sym.Matrix([0, 0, g])
    b2.potential_energy(gravity)

    p_F1 = BodyCoordinate("F1", 0, 0, 0)
    F_1 = BodyForce("1", p_F1, x_dir=True)
    T_1 = BodyTorque("2", y_dir=True)

    b2.add_force(F_1)
    b2.add_torque(T_1)

    b2.apply_constraint(0)
    b2.apply_constraint(1, 10)
    b2.apply_constraint(3, np.pi)

    b2.reset_constraints()
