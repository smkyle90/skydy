#!/usr/bin/python3

from skydy.connectors import DOF, Connection, Joint
from skydy.multibody import MultiBody
from skydy.rigidbody import Body, BodyCoordinate, BodyTorque, Ground, GroundCoordinate

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
p_pen = BodyCoordinate("G1/O", 0, 0, 0)

# Degrees of freedom
pen_dofs = [DOF(4)]

# Ground to pen joint
j1 = Joint(p_gnd, p_pen, pen_dofs, name=p_gnd.name)

# The connection of the bodies through the joint
cnx_pen = Connection(b_gnd, j1, b_pen)

# The multibody object
oned_pen = MultiBody(
    [
        cnx_pen,
    ],
    "pen",
)
