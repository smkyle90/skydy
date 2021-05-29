#!/usr/bin/python3

import numpy as np
import sympy as sym
from sympy.physics.mechanics import dynamicsymbols

from ..inertia import InertiaMatrix, MassMatrix
from .BodyCoordinate import BodyCoordinate
from .BodyForce import BodyForce, BodyTorque


class Body:
    id_counter = 0

    def __init__(self, mass, length=0, width=0, height=0, shape="rod", name=None):

        # Denote if the velocity is absolute. By default, as it is a body,
        # is a relative velocity.
        self.__abs = False

        # Body accounting
        self.body_id = Body.id_counter
        Body.id_counter += 1
        if name is None:
            self.name = str(self.body_id)
        else:
            self.name = str(name)

        self.shape = shape
        self.dims = BodyCoordinate(self.name, length, width, height)

        # Inertial Properties
        self.mass = mass
        self.mass_matrix = MassMatrix(self.name)
        self.inertia_matrix = InertiaMatrix(self.name)

        # Position and velocity variables
        self.q, self.v = self.__init_symbols()

        # Free body state and velocities
        self.__r_body_free, self.__R_body_free = self.__free_body_configuration()

        self.r_body, self.R_body = None, None

        # Constrained body state and velocities
        self.reset_constraints()

        self.linear_forces = []
        self.linear_torques = []

    def is_absolute(self):
        return self.__abs

    def __init_symbols(self):
        q = ["x_G", "y_G", "z_G", "theta_x", "theta_y", "theta_z"]
        q = ["{}_{}".format(var, self.name) for var in q]
        q = [dynamicsymbols(var) for var in q]
        v = [sym.diff(var, sym.Symbol("t")) for var in q]

        return q, v

    def __free_body_configuration(self):
        # Define the rotation matrices for each axis
        Rx = sym.rot_axis3(self.q[3]).T
        Ry = sym.rot_axis2(self.q[4]).T
        Rz = sym.rot_axis1(self.q[5]).T
        R = sym.simplify(Rz @ Ry @ Rx)

        r = sym.Matrix(self.q[:3])

        return r, R

    def body_twists(self, r, R):
        # Get the rotational velocity
        omega = sym.simplify(R.inv() @ sym.diff(R, sym.Symbol("t")))
        omega_body = sym.Matrix([omega[2, 1], omega[0, 2], omega[1, 0]])

        # Define the position of COM and get velocity
        v_body = sym.diff(r, sym.Symbol("t"))

        return v_body, omega_body

    def body_velocity(self, point_on_body):
        point_on_body = np.array(point_on_body).reshape(-1,).tolist()
        point_on_body = sym.Matrix(point_on_body)
        v_body, w_body = self.body_twists(self.r_body, self.R_body)
        return v_body + w_body.cross(point_on_body)

    def kinetic_energy(self):
        # Define the kinetic energy of the system
        v_body, w_body = self.body_twists(self.r_body, self.R_body)
        KE_tr = sym.simplify((1 / 2) * v_body.T @ self.mass_matrix.as_mat() @ v_body)
        KE_ro = sym.simplify((1 / 2) * w_body.T @ self.inertia_matrix.as_mat() @ w_body)
        return KE_tr[0] + KE_ro[0]

    def potential_energy(self, gravity):
        g = sym.Symbol("g")
        return self.mass_matrix.as_mat()[0, 0] * g * self.r_body.dot(gravity)

    def add_force(self, linear_force):
        assert isinstance(linear_force, BodyForce)
        self.linear_forces.append(linear_force)

    def add_torque(self, linear_torque):
        assert isinstance(linear_torque, BodyTorque)
        loc_torque = sym.Matrix(
            [
                self.q[idx + 3] if t_dir else 0
                for idx, t_dir in enumerate(linear_torque.direction)
            ]
        )
        linear_torque.location = loc_torque
        self.linear_torques.append(linear_torque)

    def apply_constraint(self, idx, const_value=0):
        self.__constrained_state[idx] = True

        self.r_body = sym.simplify(self.r_body.subs(self.q[idx], const_value))
        self.R_body = sym.simplify(self.R_body.subs(self.q[idx], const_value))

    def reset_constraints(self):
        self.r_body = self.__r_body_free.copy()
        self.R_body = self.__R_body_free.copy()
        self.__constrained_state = [False for val in self.q]


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
