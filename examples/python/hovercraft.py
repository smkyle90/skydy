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

# Hovercraft Body
l_hc, w_hc, h_hc = 2, 1, 1

hc_name = "c"
pen_name = "p"

# Define the body
b_gnd = Ground()
b_hc = Body(hc_name)
b_pen = Body(pen_name)

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
