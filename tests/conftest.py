#!/usr/bin/python3

import pytest

from skydy.connectors import DOF, Connection, Joint
from skydy.multibody import MultiBody
from skydy.rigidbody import (
    Body,
    BodyCoordinate,
    BodyForce,
    BodyTorque,
    Ground,
    GroundCoordinate,
)


@pytest.fixture
def cart():

    l_car = 2
    w_car = 1
    h_car = 1

    car_name = "1"

    # Define the body
    b_car = Body(car_name)

    # Instantiate the car's dimensions
    b_car.dims.assign_values([l_car, w_car, h_car])

    # Add force to car in the car's x-coordinaate.
    F1 = BodyForce(name="1", x_dir=True)

    # Force is applied at the COM
    force_loc = BodyCoordinate("PF1", 0, 0, 0)

    # Add the force at the location
    b_car.add_force(F1, force_loc)

    # Instantiate the ground
    b_gnd = Ground()
    p_gnd = GroundCoordinate()

    # Location of car wrt ground
    p_car = BodyCoordinate("G1/O", 0, 0, 0)

    # Degrees of freedom
    car_dofs = [DOF(0)]

    # Ground to car joint
    j1 = Joint(p_gnd, p_car, car_dofs, name=p_gnd.name)

    # The connection of the bodies through the joint
    cnx_car = Connection(b_gnd, j1, b_car)

    # The multibody object
    oned_car = MultiBody([cnx_car,], "car")

    return oned_car


@pytest.fixture
def pendulum():
    l_pen = 2
    w_pen = 0
    h_pen = 0

    pen_name = "1"

    # Define the body
    b_pen = Body(pen_name)

    # Instantiate the pen's dimensions
    b_pen.dims.assign_values([l_pen, w_pen, h_pen])

    # Add torque to pen about the pen's y-coordinaate.
    T1 = BodyTorque(name="1", y_dir=True)

    # Torque is applied at the COM
    torque_loc = BodyCoordinate("PF1", 0, 0, 0)

    # Add the Torque at the location
    b_pen.add_torque(T1, torque_loc)

    # Instantiate the ground
    b_gnd = Ground()
    p_gnd = GroundCoordinate()

    # Location of pen wrt ground
    p_pen = BodyCoordinate("G1/O", l_pen / 2, 0, 0)

    # Degrees of freedom
    pen_dofs = [DOF(4)]

    # Ground to pen joint
    j1 = Joint(p_gnd, p_pen, pen_dofs, name=p_gnd.name)

    # The connection of the bodies through the joint
    cnx_pen = Connection(b_gnd, j1, b_pen)

    # The multibody object
    oned_pen = MultiBody([cnx_pen,], "pen")

    return oned_pen


@pytest.fixture
def cart_pendulum():

    # Cart Body
    l_car, w_car, h_car = 2, 1, 1
    l_pen, w_pen, h_pen = 2, 0, 0

    car_name = "c"
    pen_name = "p"

    # Define the body
    b_gnd = Ground()
    b_car = Body(car_name)
    b_pen = Body(pen_name)

    # Instantiate the dimensions
    b_car.dims.assign_values([l_car, w_car, h_car])
    b_pen.dims.assign_values([l_pen, w_pen, h_pen])

    # Add force to car in the car's x-coordinaate.
    F1 = BodyForce(name="1", x_dir=True)
    force_loc = BodyCoordinate("PF1", 0, 0, 0)
    b_car.add_force(F1, force_loc)

    # Add torque to pen about the pen's y-coordinaate.
    T1 = BodyTorque(name="1", y_dir=True)
    torque_loc = BodyCoordinate("PF1", -l_pen / 2, 0, 0)
    b_pen.add_torque(T1, torque_loc)

    # Geometry

    p_gnd = GroundCoordinate()
    p_car_O = BodyCoordinate("Gc/O", 0, 0, 0)
    p_car_A = BodyCoordinate("A/Gc", 0, 0, 0)
    p_A_pen = BodyCoordinate("Gp/A", l_pen / 2, 0, 0)

    # Degrees of freedom
    car_dofs = [DOF(0)]
    pen_dofs = [DOF(4)]

    # Ground to car joint
    j1 = Joint(p_gnd, p_car_O, car_dofs, name=p_gnd.name)
    j2 = Joint(p_car_A, p_A_pen, pen_dofs, name="A")

    # The connection of the bodies through the joint
    cnx_car = Connection(b_gnd, j1, b_car)
    cnx_pen = Connection(b_car, j2, b_pen)

    # The multibody object
    cart_pen = MultiBody([cnx_car, cnx_pen], "cp")

    return cart_pen


@pytest.fixture
def double_pendulum():
    l_pen = 2
    w_pen = 0
    h_pen = 0

    p1_name = "1"
    p2_name = "2"

    # Define the bodies
    b_gnd = Ground()
    b_p1 = Body(p1_name)
    b_p2 = Body(p2_name)

    # Instantiate the pen's dimensions
    b_p1.dims.assign_values([l_pen, w_pen, h_pen])
    b_p2.dims.assign_values([l_pen, w_pen, h_pen])

    # Add the torques to each body
    T1 = BodyTorque(1, y_dir=True)
    torque_loc = BodyCoordinate("PT1", -l_pen / 2, 0, 0)
    b_p1.add_torque(T1, torque_loc)

    T2 = BodyTorque(2, y_dir=True)
    torque_loc = BodyCoordinate("PT2", -l_pen / 2, 0, 0)
    b_p2.add_torque(T2, torque_loc)

    # Define the geometry
    p_gnd = GroundCoordinate()
    p_p1_O = BodyCoordinate("G1/O", l_pen / 2, 0, 0)
    p_A_p1 = BodyCoordinate("A/Gp1", l_pen / 2, 0, 0)
    p_p2_A = BodyCoordinate("Gp2/A", l_pen / 2, 0, 0)

    # Degrees of freedom
    p1_dofs = [DOF(4)]
    p2_dofs = [DOF(4)]

    # Ground to pen joint
    j1 = Joint(p_gnd, p_p1_O, p1_dofs, name=p_gnd.name)
    j2 = Joint(p_A_p1, p_p2_A, p2_dofs, name="A")

    # The connection of the bodies through the joints
    cnx_p1 = Connection(b_gnd, j1, b_p1)
    cnx_p2 = Connection(b_p1, j2, b_p2)

    # The multibody object
    double_pen = MultiBody([cnx_p1, cnx_p2], "dp")

    return double_pen


@pytest.fixture
def hovercraft():
    # Hovercraft Body
    l_hc, w_hc, h_hc = 2, 1, 1

    hc_name = "hc"

    # Define the body
    b_gnd = Ground()
    b_hc = Body(hc_name)

    # Instantiate the dimensions
    b_hc.dims.assign_values([l_hc, w_hc, h_hc])

    # Add force to hc in the hc's x-coordinaate.
    F1 = BodyForce(name="1", x_dir=True)
    force_loc = BodyCoordinate("PF1", 0, 0, 0)
    b_hc.add_force(F1, force_loc)

    # Add torque to pen about the pen's y-coordinaate.
    T1 = BodyTorque(name="1", z_dir=True)
    torque_loc = BodyCoordinate("PT1", 0, 0, 0)
    b_hc.add_torque(T1, torque_loc)

    # Geometry

    p_gnd = GroundCoordinate()
    p_hc_O = BodyCoordinate("Gc/O", 0, 0, 0)

    # Degrees of freedom
    hc_dofs = [DOF(0), DOF(1), DOF(5)]

    # Ground to hc joint
    j1 = Joint(p_gnd, p_hc_O, hc_dofs, name=p_gnd.name)

    # The connection of the bodies through the joint
    cnx_hc = Connection(b_gnd, j1, b_hc)

    # The multibody object
    hovercraft = MultiBody([cnx_hc], "hc")

    return hovercraft
