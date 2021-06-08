#!/usr/bin/python3

import copy

import matplotlib.pyplot as plt
import numpy as np
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

    def __sym_to_np(self, sym_matrix):
        return np.array(sym_matrix.tolist()).astype(float)

    def draw(self, ax=None, ref_body=None, sub_vals={}):
        if ax is None:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection="3d")

        if ref_body is None:
            ref_body = Ground()

        # Get body dimensions
        l, w, h = self.dims.values()

        # get all the combinations of body_corners
        body_corners = []
        for i in [-1, 1]:
            for j in [-1, 1]:
                for k in [-1, 1]:
                    body_corners.append(
                        [i * l.item() / 2, j * w.item() / 2, k * h.item() / 2]
                    )
        # Convert to a numpy array
        body_corners = np.block(body_corners)

        # Make copies of the position, rotation and origins
        this_rot = self.rot_body.copy()
        this_pos = self.pos_body.copy()
        that_orig = ref_body.pos_body.copy()

        # Sub in numeric values
        for s, v in sub_vals.items():
            this_pos = this_pos.subs(s, v)
            this_rot = this_rot.subs(s, v)
            that_orig = that_orig.subs(s, v)

        # Convert to numpy array
        this_pos = self.__sym_to_np(this_pos)
        this_rot = self.__sym_to_np(this_rot)
        that_orig = self.__sym_to_np(that_orig)

        # Plot the COM
        ax.scatter3D(this_pos[0, 0], this_pos[1, 0], this_pos[2, 0], c="k")

        # plot the vector from the base COM to this body's COM
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

        # Dimension plotting
        # Move translate and rotate corners into global frame
        rot_body_corners = (this_pos + this_rot @ body_corners.T).T
        # Plot the body corners
        ax.scatter3D(
            rot_body_corners[:, 0],
            rot_body_corners[:, 1],
            rot_body_corners[:, 2],
            c="k",
            s=2,
        )

        # label the dimensions of the body
        # from the 0th index in the body corners,
        # the:
        #   length is a vector from the 0th index to the 4th index,
        #   width is a vector from the 0th index to the 2th index,
        #   height is a vector from the 0th index to the 1st index.
        dim_idx = [4, 2, 1]
        for dim, symbol in zip(dim_idx, self.dims.symbols()):
            if symbol:
                v_start = rot_body_corners[0].reshape(-1, 1)
                v_end = rot_body_corners[dim].reshape(-1, 1)
                v_dim = np.hstack((v_start, v_end))
                ax.plot3D(*v_dim.tolist(), c="g", linewidth=1)
                ax.text(*v_dim.mean(axis=1), symbols_to_latex(symbol), c="g")

        # Plot the principal axes
        body_dims = np.diag([l.item(), w.item(), h.item()])
        for i in [-1, 1]:
            rot_body_axes = this_pos + i * this_rot @ body_dims

            for idx, axes in enumerate(rot_body_axes.T):
                v_axes = np.hstack((this_pos, np.array(axes).reshape(-1, 1)))
                ax.plot3D(*v_axes.tolist(), c="k", linewidth=0.25, linestyle="--")

                if (i == 1) and (idx + 3 in self.free_idx):
                    ax.text(
                        *that_orig.reshape(-1,).tolist(),
                        symbols_to_latex(self.positions()[idx + 3]),
                    )

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
