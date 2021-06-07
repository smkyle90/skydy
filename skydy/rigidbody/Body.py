#!/usr/bin/python3

import matplotlib.pyplot as plt
import numpy as np
import sympy as sym
from mpl_toolkits import mplot3d

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

    def draw(self, ax=None, ref_body=None, sub_vals={}):
        if ax is None:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection="3d")

        if ref_body is None:
            ref_body = Ground()

        l, w, h = self.dims.values()
        # get all the combinations of body_corners
        body_corners = []
        for i in [-1, 1]:
            for j in [-1, 1]:
                for k in [-1, 1]:
                    body_corners.append([i * l / 2, j * w / 2, k * h / 2])

        this_rot = self.rot_body
        this_pos = self.pos_body
        that_orig = ref_body.pos_body

        for s, v in sub_vals.items():
            this_pos = this_pos.subs(s, v)
            this_rot = this_rot.subs(s, v)
            that_orig = that_orig.subs(s, v)

        this_pos = np.array(this_pos.tolist()).astype(float)
        this_rot = np.array(this_rot.tolist()).astype(float)
        that_orig = np.array(that_orig.tolist()).astype(float)

        body_corners = np.block(body_corners)
        rot_body_corners = (this_pos + this_rot @ body_corners.T).T

        # ax.scatter3D(body_corners[:, 0], body_corners[:, 1], body_corners[:, 2], c='r')
        ax.scatter3D(
            rot_body_corners[:, 0],
            rot_body_corners[:, 1],
            rot_body_corners[:, 2],
            c="k",
        )
        ax.scatter3D(this_pos[0, 0], this_pos[1, 0], this_pos[2, 0], c="k")

        ax.plot3D(*np.hstack((that_orig, this_pos)).tolist(), c="k", linewidth=0.5)
        ax.text(
            (this_pos + that_orig)[0, 0] / 2,
            (this_pos + that_orig)[1, 0] / 2,
            (this_pos + that_orig)[2, 0] / 2,
            symbols_to_latex(
                self.pos_body - ref_body.pos_body,
                "p^{G_{" + f"{ref_body.name}" + "}}_{G_{" + f"{self.name}" + "}}",
            ),
        )

        body_dims = np.diag([l.item(), w.item(), h.item()])

        for i in [-1, 1]:
            rot_body_dims = this_pos + i * this_rot @ body_dims / 2
            rot_body_axes = this_pos + i * this_rot @ body_dims

            # dim_pre = r + i * body_dims
            for idx, (body, axes) in enumerate(zip(rot_body_dims.T, rot_body_axes.T)):
                v_body = np.hstack((this_pos, np.array(body).reshape(-1, 1)))
                v_axes = np.hstack((this_pos, np.array(axes).reshape(-1, 1)))

                ax.plot3D(*v_axes.tolist(), c="k", linewidth=0.25, linestyle="--")
                ax.plot3D(*v_body.tolist(), c="k", linewidth=1)

                if (i == 1) and (idx + 3 in self.free_idx):
                    ax.text(*body, symbols_to_latex(self.positions()[idx + 3]))

            # for idx, col in enumerate(dim_pre.T):
            #     v = np.hstack((r, np.array(col).reshape(-1, 1)))
            #     ax.plot3D(*v.tolist(), c='k', linewidth=0.5, linestyle="--")

        return ax


class Ground(Body):
    def __init__(self):
        super().__init__(GROUND_NAME)

        self.pos_body = sym.zeros(3, 1)
        self.rot_body = sym.eye(3)
        for idx in range(6):
            self.apply_constraint(idx)

        self.dims.assign_values([0, 0, 0])


def symbols_to_latex(symbols, prefix=None):
    try:
        lat_str = sym.physics.vector.printing.vlatex(
            symbols.T.tolist()[0], mode="inline"
        )
    except Exception as e:
        lat_str = sym.physics.vector.printing.vlatex(symbols, mode="inline")

    if prefix is None:
        return lat_str
    else:
        return f"${prefix} = " + lat_str[1:]


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
