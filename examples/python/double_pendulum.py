#!/usr/bin/python3

from skydy.connectors import DOF, Connection, Joint
from skydy.multibody import MultiBody
from skydy.rigidbody import Body, BodyCoordinate, BodyTorque, Ground, GroundCoordinate

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
p_p1_O = BodyCoordinate("G1/O", 0, 0, 0)
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
