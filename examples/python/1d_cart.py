#!/usr/bin/python3
"""
This is an example usage of the skydy package.

The base case is an object that can move in one-dimension,
along a line. It has one input force, co-linear with the
direction of motion.
"""
from skydy.connectors import DOF, Connection, Joint
from skydy.multibody import MultiBody
from skydy.rigidbody import Body, BodyCoordinate, BodyForce, Ground, GroundCoordinate

# For this example, the car will be able to move in the body x-coordinate.
# For simplicity, this coordinate will align the the global x-coordinate.

# Start by defining the dimension of the vehicle, in the body coordinates.
# If we do not assign a value in a dimension, it defaults
# to zero dimension in that coordinate.
# Create 2 bodies. Ground, and single mass
l_car = 2
w_car = 1
h_car = 1

# Give the car a name
car_name = "1"

# Define the body
b_car = Body(car_name)

# Instantiate the car's dimensions
b_car.dims.assign_values([l_car, w_car, h_car])

# As it stands, this is simply a rigid body with 6 degrees of freedom and some geometry.
b_car.positions()

# As it stands, the mass and moments of inertia for a Body are defined symbolically,
# but arbitrary. The mass and inertia matrices are MassMatrix and InertiaMatrix
# objects respectively. Let's see.
b_car.mass_matrix.as_mat()
b_car.inertia_matrix.as_mat()

# We can now add a force to the car.
# Add force to car in the car's x-coordinaate.
F1 = BodyForce(name="1", x_dir=True)

# Force is applied at the COM
force_loc = BodyCoordinate("PF1", 0, 0, 0)

# Add the force at the location
b_car.add_force(F1, force_loc)


# We will "reduce" the number of DOFs by adding a 'joint' between the car body and the ground.
# Every MultiBody in this package needs to have reference to the Ground, i.e., a fixed point
# that provides the global spatial coordinate frame. All geometry and kinematic are calculated
# from this fixed point. We need to instantiate the Ground body and its location. These are
# special classes.
b_gnd = Ground()
p_gnd = GroundCoordinate()

# We now have to "connect" our car to the ground, via a joint.
# A joint is not necessarily a physical object, it merelely defines
# the degrees of freedom that exists between two bodies. IN this case,
# the "joint" between the ground and the vehicle is a degree of freedom
# in the body x-coordinate of the car, applied at the centre of mass
# of the car. Thus, define the relative position of the joint in the car
# frame.
p_car = BodyCoordinate("G1/O", 0, 0, 0)

# We now ready to stipulate what Degrees of Freedom we want
# to pass through our joint. In this case, we are constrained
# to motion in the x-coordinate, as such, there is a DOF in the
# 0th index (note, indices are 0 through 5 for body coordinates x, y, z, theta_x, theta_y, theta_z)
# By default, we only need to define the coordinates that have a degree of freedom. If the
# DOF is not defined, it is assumed to be constrained with ZERO constant value. This does not have
# to be zero. Thus, we define the list of DOFs:
car_dofs = [DOF(0)]

# We are ready to define the joint that connects the ground to the car.

# Add the joint, with a sliding DOF in the x-direction. A reminder,
# the first body coordinate is the location of the joint in the first
# body's frame, and the second body coordinate is in the second body's frame.
# Name this joint whatever makes sense. Seeing it's the ground, name it that.
j1 = Joint(
    p_gnd,
    p_car,
    [
        DOF(
            0,
        )
    ],
    name=p_gnd.name,
)

# We are now ready to connect the two bodies through the joint.
# A Connection is what will allow us to perform our forward kinematics,
# i.e., get global representation of the positions and rotations of each
# connected body. It is critical the the first connection's input body is the ground.
cnx_car = Connection(b_gnd, j1, b_car)

# Create the MultiBody system - a rigid body is just a collection of connected Bodies.
# It is required that the first connection is to the ground.
oned_car = MultiBody(
    [
        cnx_car,
    ]
)

# Draw the system
oned_car.draw()

# Create the latex
oned_car.as_latex()
