#!/usr/bin/python3

import sympy as sym

from ..configuration import Configuration, DimensionSymbols
from ..inertia import InertiaMatrix, MassMatrix
from .BodyCoordinate import BodyCoordinate
from .BodyForce import BodyForce, BodyTorque

# from .BodyCoordinate import BodyCoordinate

GROUND_NAME = "0"


class Body(Configuration):
    id_counter = 1
    body_names = []

    def __init__(self, name=None):
        if (name in Body.body_names) and (name != GROUND_NAME):
            raise ValueError("Body name already exists.")
        elif name != GROUND_NAME:
            # Body accounting
            self.body_id = Body.id_counter
            Body.id_counter += 1

        if name is None:
            self.name = str(self.body_id)
        else:
            self.name = str(name)

        Body.body_names.append(self.name)

        # Initialise the Configuration object
        super().__init__(self.name)

        # Inertial Properties
        self.mass_matrix = MassMatrix(self.name)
        self.inertia_matrix = InertiaMatrix(self.name)

        self.linear_forces = []
        self.linear_torques = []

        self.dims = DimensionSymbols(self.name)

    def body_twists(self):
        # Get the rotational velocity
        omega = self.rot_body.inv() @ sym.diff(self.rot_body, sym.Symbol("t"))

        omega_body = sym.Matrix([omega[2, 1], omega[0, 2], omega[1, 0]])

        # Define the position of COM and get velocity
        v_body = sym.diff(self.pos_body, sym.Symbol("t"))

        return sym.simplify(v_body), sym.simplify(omega_body)

    def kinetic_energy(self):
        # Define the kinetic energy of the system
        v_body, w_body = self.body_twists()
        KE_tr = (1 / 2) * v_body.T @ self.mass_matrix.as_mat() @ v_body
        KE_ro = (1 / 2) * w_body.T @ self.inertia_matrix.as_mat() @ w_body
        return sym.simplify(KE_tr[0] + KE_ro[0])

    def potential_energy(self, gravity):
        g = sym.Symbol("g")
        return self.mass_matrix.as_mat()[0, 0] * g * self.pos_body.dot(gravity)

    def add_force(self, force_vector, force_location):
        assert isinstance(
            force_vector, BodyForce
        ), "Force Vector Must be a BodyForce object."
        assert isinstance(
            force_location, BodyCoordinate
        ), "Force Location Must be a BodyCoordinate object."
        linear_force = (force_vector, force_location)
        self.linear_forces.append(linear_force)

    def add_torque(self, torque_vector, torque_location):
        assert isinstance(
            torque_vector, BodyTorque
        ), "Torque Vector Must be a BodyTorque object."
        assert isinstance(
            torque_location, BodyCoordinate
        ), "Torque Location Must be a BodyCoordinate object."
        linear_torque = (torque_vector, torque_location)
        self.linear_torques.append(linear_torque)

    # def draw_body(self, ax=None):
    #     import matplotlib.pyplot as plt
    #     import numpy as np
    #     l, w, h = self.dims.values()

    #     # get all the combinations of corners
    #     corners = []
    #     for i in [-1, 1]:
    #         for j in [-1, 1]:
    #             for k in [-1, 1]:
    #                 corners.append((i*l/2, j*w/2, k*h/2))

    #     all_angles = np.pi/6

    #     r = self.pos_body.copy()
    #     R = self.rot_body.copy()


class Ground(Body):
    def __init__(self):
        super().__init__(GROUND_NAME)

        self.pos_body = sym.zeros(3, 1)
        self.rot_body = sym.eye(3)

        self.dims.assign_values([0, 0, 0])


# l = 2
# b0 = Body(0, 0)
# b1 = Body(1, l)
# b2 = Body(1, 2*l)

# # Cart on the ground
# p0 = BodyCoordinate("O")
# p1 = BodyCoordinate("G1/O", 0, 0, 0)
# j1 = Joint(p0, p1, [DOF(0,)])

# # Link on cart
# p2 = BodyCoordinate("A/G1", 0, 0, 0)
# p3 = BodyCoordinate("G2/A", l, 0, 0)
# j2 = Joint(p2, p3, [DOF(4,)])

# # Body force
# p_F1 = BodyCoordinate("F1", 0, 0, 0)
# F_1 = BodyForce("1", p_F1, x_dir=True)
# T_1 = BodyTorque("2", y_dir=True)

# # Add force to trolley
# b1.add_force(F_1)
# # Add torque to the arm
# b2.add_torque(T_1)

# # Create the system
# # A rigid body is just a collection of connected Bodies
# body = MultiBody([
#     Connection(b0, j1, b1),
#     Connection(b1, j2, b2),
# ])

# body.el_equations()
# body.calculate_forces()
