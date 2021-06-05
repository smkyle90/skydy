"""Test the example function
"""

import numpy as np
import pytest


@pytest.mark.multibody
def test_MultiBody():
    from skydy.connectors import DOF, Connection, Joint
    from skydy.multibody import MultiBody
    from skydy.rigidbody import Body, BodyCoordinate, BodyForce, BodyTorque

    # Create 2 bodies. Ground, and single mass
    b_gnd = Body("gnd")

    print("\n===1DOF===")
    b_car = Body("car")

    # Dimension of half a link
    l = 2
    # Link in a pivot on the ground.
    p0 = BodyCoordinate("O")
    p1 = BodyCoordinate("G1/O", 0, 0, 0)

    # add the joint, with a sliding DOF in the x-direction
    j1 = Joint(p0, p1, [DOF(0,)])

    # Add force to cart. Force is applied at the COM
    F1 = BodyForce(1, x_dir=True)
    force_loc = BodyCoordinate("PF1", l, 0, 0)
    b_car.add_force(F1, force_loc)

    F2 = BodyForce(2, x_dir=True)
    force_loc = BodyCoordinate("PF2", -l, 0, 0)
    b_car.add_force(F2, force_loc)

    # Connect the two bodies through the joint
    cnx = Connection(b_gnd, j1, b_car)

    # Create the system
    # A rigid body is just a collection of connected Bodies
    body = MultiBody([cnx,])

    print("coordinates", body.coordinates)
    print("bodies", body.bodies)
    print("forces")
    for p, f, _ in body.forces:
        print("Pos", p)
        print("Dir", f)
    print("torques")
    for _, t, _ in body.torques:
        print(t)

    print("kinetic_energy", body.kinetic_energy)
    print("potential_energy", body.potential_energy)
    print("Symbols", body.symbols())
    LHS, RHS = body.eoms()
    print("LHS EOM", LHS)
    print("RHS EOM", RHS)
    A, B = body.system_matrices()
    print("Nonlinear A", A)
    print("Nonlinear B", B)

    A, B = body.system_matrices(True)
    print("Linear A", A)
    print("Linear B", B)

    q0, f0 = body.get_equilibria()
    print("coordinate eum", q0)
    print("force eum", f0)
    print("config", body.get_configuration())
    body.as_latex(True)

    print("\n===Pendulum===")
    # Pendulum
    b_pen = Body("pen")
    p2 = BodyCoordinate("G2/0", l, 0, 0)
    j2 = Joint(p0, p2, [DOF(4,)])

    # Add force to cart. Force is applied at the COM
    T1 = BodyTorque(1, y_dir=True)
    torque_loc = BodyCoordinate("PT1", -l, 0, 0)
    b_pen.add_torque(T1, torque_loc)

    cnx = Connection(b_gnd, j2, b_pen)

    # body.add_connection(cnx)
    body = MultiBody([cnx,])

    print("coordinates", body.coordinates)
    print("bodies", body.bodies)
    print("forces")
    for p, f, _ in body.forces:
        print("Pos", p)
        print("Dir", f)
    print("torques")
    for _, t, _ in body.torques:
        print(t)

    print("kinetic_energy", body.kinetic_energy)
    print("potential_energy", body.potential_energy)
    print("Symbols", body.symbols())
    LHS, RHS = body.eoms()
    print("LHS EOM", LHS)
    print("RHS EOM", RHS)
    A, B = body.system_matrices()
    print("Nonlinear A", A)
    print("Nonlinear B", B)

    A, B = body.system_matrices(True)
    print("Linear A", A)
    print("Linear B", B)

    q0, f0 = body.get_equilibria()
    print("coordinate eum", q0)
    print("force eum", f0)
    print("config", body.get_configuration())

    body.as_latex()
