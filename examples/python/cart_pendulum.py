#!/usr/bin/python3

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

# Draw the system
cart_pen.draw(save_fig=True)

# Create the latex
cart_pen.as_latex()
