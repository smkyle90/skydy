"""Test the example function
"""

import matplotlib.pyplot as plt
import pytest
from mpl_toolkits import mplot3d


@pytest.mark.multibody
def test_MultiBody():
    from skydy.connectors import DOF, Connection, Joint
    from skydy.multibody import MultiBody
    from skydy.rigidbody import Body, BodyCoordinate, BodyForce, BodyTorque, Ground

    # # Dimension of half a link
    l = 2

    # Create 2 bodies. Ground, and single mass
    b_gnd = Ground()
    p0 = BodyCoordinate("O")

    print("\n===1DOF===")
    b_car = Body("car")

    # Link in a pivot on the ground.
    p1 = BodyCoordinate("G1/O", 0, 0, 0)

    # add the joint, with a sliding DOF in the x-direction
    j1 = Joint(p0, p1, [DOF(0,)], name="O")

    # Add force to cart. Force is applied at the COM
    F1 = BodyForce(1, x_dir=True)
    force_loc = BodyCoordinate("PF1", 0, 0, 0)
    b_car.add_force(F1, force_loc)

    # Connect the two bodies through the joint
    cnx_car = Connection(b_gnd, j1, b_car)

    # Create the system
    # A rigid body is just a collection of connected Bodies
    body = MultiBody([cnx_car,])

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

    del body
    # Pendulum
    print("\n===Link 1===")
    b_pen_1 = Body("lk1")
    p2 = BodyCoordinate("G2/O", l, 0, 0)
    j2 = Joint(p0, p2, [DOF(4,)])

    # Add force to cart. Force is applied at the COM
    T1 = BodyTorque(1, y_dir=True)
    torque_loc = BodyCoordinate("PT1", -l, 0, 0)
    b_pen_1.add_torque(T1, torque_loc)

    # T2 = BodyTorque(2, y_dir=True)
    # torque_loc = BodyCoordinate("PT2", l, 0, 0)
    # b_pen_1.add_torque(T2, torque_loc)

    cnx_pen_1 = Connection(b_gnd, j2, b_pen_1)

    # body.add_connection(cnx_pen_1)
    body = MultiBody([cnx_pen_1,])

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

    del body
    print("\n===Cart Pendulum===")
    l = 2
    b_car = Body("c")
    b_pen = Body("p")

    b_car.dims.assign_values([4, 1, 1])
    b_pen.dims.assign_values([2 * l, 0, 0])

    p0 = BodyCoordinate("O", 0, 0, 0)
    p1 = BodyCoordinate("Gc/O", 0, 0, 0)
    p2 = BodyCoordinate("A/Gc", 0, 0, 0)
    p3 = BodyCoordinate("Gp/A", l, 0, 0)

    j1 = Joint(p0, p1, [DOF(0,)], name="O")
    j1 = Joint(p2, p3, [DOF(4,)], name="A")

    cnx_car = Connection(b_gnd, j1, b_car)
    cnx_pen = Connection(b_car, j2, b_pen)

    body = MultiBody([cnx_car, cnx_pen_1])

    body.as_latex()

    del body
    print("\n===Double Pendulum===")

    b_pen_1 = Body("p1")
    p1 = BodyCoordinate("G2/O", l, 0, 0)
    j1 = Joint(p0, p1, [DOF(4,)])

    # Add force to cart. Force is applied at the COM
    T1 = BodyTorque(1, y_dir=True)
    torque_loc = BodyCoordinate("PT1", -l, 0, 0)
    b_pen_1.add_torque(T1, torque_loc)

    cnx_pen_1 = Connection(b_gnd, j1, b_pen_1)

    # Pendulum
    b_pen_2 = Body("p2")
    p2 = BodyCoordinate("A/G2", l, 0, 0)
    p3 = BodyCoordinate("G3/A", l, 0, 0)
    j2 = Joint(p2, p3, [DOF(4,)])

    # Add force to cart. Force is applied at the COM
    T2 = BodyTorque(2, y_dir=True)
    torque_loc = BodyCoordinate("PT2", -l, 0, 0)
    b_pen_2.add_torque(T2, torque_loc)

    cnx_pen_2 = Connection(b_pen_1, j2, b_pen_2)

    body = MultiBody([cnx_pen_1, cnx_pen_2])

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


@pytest.mark.dev
def test_MultiBody_draw():
    from skydy.connectors import DOF, Connection, Joint
    from skydy.multibody import MultiBody
    from skydy.rigidbody import Body, BodyCoordinate, BodyForce, BodyTorque, Ground

    b_gnd = Ground()
    l = 2
    p0 = BodyCoordinate("O")

    print("\n===1DOF===")
    b_car = Body("car")
    b_car.dims.assign_values([1, 0.5, 0.5])

    # Link in a pivot on the ground.
    p1 = BodyCoordinate("G1/O", 0, 0, 0)

    # add the joint, with a sliding DOF in the x-direction
    j1 = Joint(p0, p1, [DOF(0,)], name="O")

    # Add force to cart. Force is applied at the COM
    F1 = BodyForce(name="1", x_dir=True)
    force_loc = BodyCoordinate("PF1", 0, 0, 0)
    b_car.add_force(F1, force_loc)

    # Connect the two bodies through the joint
    cnx_car = Connection(b_gnd, j1, b_car)

    # Create the system
    # A rigid body is just a collection of connected Bodies
    body = MultiBody([cnx_car,])

    # body.draw()
    body.as_latex()

    del body
    # Pendulum
    print("\n===Link 1===")
    b_pen_1 = Body("1")
    b_pen_1.dims.assign_values([2 * l, 0, 0])
    p2 = BodyCoordinate("G1/O", l, 0, 0)
    j2 = Joint(p0, p2, [DOF(4,)], name="O")

    # Add force to cart. Force is applied at the COM
    T1 = BodyTorque(1, y_dir=True)
    torque_loc = BodyCoordinate("PT1", -l, 0, 0)
    b_pen_1.add_torque(T1, torque_loc)

    # T2 = BodyTorque(2, y_dir=True)
    # torque_loc = BodyCoordinate("PT2", l, 0, 0)
    # b_pen_1.add_torque(T2, torque_loc)

    cnx_pen_1 = Connection(b_gnd, j2, b_pen_1)

    # body.add_connection(cnx_pen_1)
    body = MultiBody([cnx_pen_1,])
    body.as_latex()
    # body.draw()

    # print("\n===Cart Pendulum===")
    # l = 2
    # b_car = Body("c")
    # b_pen = Body("p")
    # b_car.dims.assign_values([l, l/4, l/4])
    # b_pen.dims.assign_values([2 * l, 0, 0])

    # p0 = BodyCoordinate("O", 0, 0, 0)
    # p1 = BodyCoordinate("Gc/O", 0, 0, l/8)
    # p2 = BodyCoordinate("A/Gc", 0, 0, l/8)
    # p3 = BodyCoordinate("Gp/A", l, 0, 0)

    # F1 = BodyForce(name="1", x_dir=True)
    # force_loc = BodyCoordinate("PF1", 0, 0, 0)
    # b_car.add_force(F1, force_loc)

    # T1 = BodyTorque(1, y_dir=True)
    # torque_loc = BodyCoordinate("PT1", -l, 0, 0)
    # b_pen.add_torque(T1, torque_loc)

    # j1 = Joint(p0, p1, [DOF(0,)], name="O")
    # j2 = Joint(p2, p3, [DOF(4,)], name="A")

    # cnx_car = Connection(b_gnd, j1, b_car)
    # cnx_pen = Connection(b_car, j2, b_pen)

    # body = MultiBody([cnx_car, cnx_pen])

    # body.as_latex()

    # del body
    print("\n===Double Pendulum===")

    b_pen_1 = Body("1")
    b_pen_1.dims.assign_values([4 * l, 0, 0])

    p1 = BodyCoordinate("G1/O", 2 * l, 0, 0)
    j1 = Joint(p0, p1, [DOF(4,)], name="O")

    # Add force to cart. Force is applied at the COM
    T1 = BodyTorque(1, y_dir=True)
    torque_loc = BodyCoordinate("PT1", -2 * l, 0, 0)
    b_pen_1.add_torque(T1, torque_loc)

    cnx_pen_1 = Connection(b_gnd, j1, b_pen_1)

    # Pendulum
    b_pen_2 = Body("2")
    b_pen_2.dims.assign_values([2 * l, 0, 0])
    p2 = BodyCoordinate("A/G1", l, 0, 0)
    p3 = BodyCoordinate("G2/A", l, 0, 0)
    j2 = Joint(p2, p3, [DOF(4,)], name="A")

    # Add force to cart. Force is applied at the COM
    T2 = BodyTorque(2, y_dir=True)
    torque_loc = BodyCoordinate("PT2", -l, 0, 0)
    b_pen_2.add_torque(T2, torque_loc)

    cnx_pen_2 = Connection(b_pen_1, j2, b_pen_2)

    body = MultiBody([cnx_pen_1, cnx_pen_2])

    body.as_latex()
