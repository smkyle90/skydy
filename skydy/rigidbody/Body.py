#!/usr/bin/python3

import copy

import matplotlib.pyplot as plt
import numpy as np
import sympy as sym

from ..configuration import Configuration, DimensionSymbols
from ..inertia import InertiaMatrix, MassMatrix
from ..output import Arrow3D
from .BodyCoordinate import BodyCoordinate
from .BodyForce import BodyForce, BodyTorque

# from .BodyCoordinate import BodyCoordinate

GROUND_NAME = "0"


class Body(Configuration):
    id_counter = 1

    def __init__(self, name=None):

        # Body accounting
        self.body_id = Body.id_counter
        Body.id_counter += 1

        if name is None:
            self.name = str(self.body_id)
        else:
            self.name = str(name)

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

    def draw(self, ax=None, ref_body=None, ref_joint=None, sub_vals={}):
        if ax is None:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection="3d")

        if ref_body is None:
            ref_body = Ground()

        if ref_joint is None:
            ref_joint = np.zeros((3, 1))

        if not sub_vals:
            sub_vals = self.as_dict()

        # Make copies of the symbols position, rotation and origins
        body_rot_mat = self.rot_body.copy()
        half_rot_mat = self.rot_body.copy()
        body_pos_mat = self.pos_body.copy()
        ref_body_origin = ref_body.pos_body.copy()

        # Sub in numeric values
        body_pos_mat = body_pos_mat.subs(sub_vals)
        body_rot_mat = body_rot_mat.subs(sub_vals)
        ref_body_origin = ref_body_origin.subs(sub_vals)

        # for the half rot, we want to only rotate THIS body's
        # values by half. This is used for plotting the angle symbols
        for s, v in sub_vals.items():
            if s in self.symbols():
                half_rot_mat = half_rot_mat.subs(s, v / 2)
            else:
                half_rot_mat = half_rot_mat.subs(s, v)

        # Convert to numpy array
        body_pos_mat = self.sym_to_np(body_pos_mat)
        body_rot_mat = self.sym_to_np(body_rot_mat)
        ref_body_origin = self.sym_to_np(ref_body_origin)
        half_rot_mat = self.sym_to_np(half_rot_mat)

        # Plot principal axes and the free rotation angles
        ax = self.__plot_principal_axes(ax, body_pos_mat, body_rot_mat)
        ax = self.__plot_free_angles(ax, ref_joint, half_rot_mat)

        # Dimension plotting
        ax = self.__plot_body_geometry(ax, body_pos_mat, body_rot_mat)

        # Plot the COM
        ax = self.__plot_COM(ax, body_pos_mat, body_rot_mat, ref_body_origin, ref_body)

        # Plot the forces
        for force, loc in self.linear_forces:
            ax = self.__plot_input(ax, force, loc, body_pos_mat, body_rot_mat, "y", "F")

        for torque, loc in self.linear_torques:
            ax = self.__plot_input(
                ax, torque, loc, body_pos_mat, body_rot_mat, "m", "\\tau"
            )

        return ax

    def __plot_COM(self, ax, pos_ref, rot_ref, ref_origin, ref_body):
        # Plot the COM
        ax.scatter3D(pos_ref[0, 0], pos_ref[1, 0], pos_ref[2, 0], c="k", s=2)
        ax.text(
            pos_ref[0, 0] - 0.2,
            pos_ref[1, 0] - 0.2,
            pos_ref[2, 0] - 0.2,
            "$G_{" + f"{self.name}" + "}$",
            c="k",
            fontsize="x-small",
        )

        ax = self.__draw_3d_line(ax, ref_origin, pos_ref, "k", 0.5)

        # plot the vector from the base COM to this body's COM
        # ax.plot3D(*np.hstack((ref_origin, pos_ref)).tolist(), c="k", linewidth=0.5)
        ax.text(
            (pos_ref + ref_origin)[0, 0] / 2,
            (pos_ref + ref_origin)[1, 0] / 2,
            (pos_ref + ref_origin)[2, 0] / 2,
            symbols_to_latex(
                sym.simplify(self.pos_body - ref_body.pos_body),
                "p^{G_{" + f"{ref_body.name}" + "}}_{G_{" + f"{self.name}" + "}}",
            ),
            fontsize="x-small",
        )

        return ax

    def __plot_principal_axes(self, ax, pos_ref, rot_ref):

        # Plot the principal axes
        body_dims = np.diag(self.dims.values().reshape(-1,))

        for i in [-1, 1]:
            rot_body_axes = pos_ref + i * (2 / 3) * rot_ref @ body_dims
            for idx, axes in enumerate(rot_body_axes.T):
                ax = self.__draw_3d_line(
                    ax, pos_ref, axes, color="k", linewidth=0.25, linestyle="--"
                )

        return ax

    def __plot_free_angles(self, ax, ref_joint, half_rot_mat):
        for idx in range(3):
            if (idx + 3) not in self.free_idx:
                continue

            base_vector = np.zeros((3, 1))
            base_vector[(idx + 2) % 3, 0] = 1
            rot_ax = ref_joint + half_rot_mat @ base_vector

            ax.text(
                *rot_ax.reshape(-1,).tolist(),
                symbols_to_latex(self.positions()[idx + 3]),
                c="m",
                fontsize="x-small",
            )

        return ax

    def __plot_body_geometry(self, ax, pos_ref, rot_ref):
        body_corners = self.__get_body_corners()

        # Move translate and rotate corners into global frame
        rot_body_corners = (pos_ref + rot_ref @ body_corners.T).T
        # Plot the body corners
        ax.scatter3D(
            rot_body_corners[:, 0],
            rot_body_corners[:, 1],
            rot_body_corners[:, 2],
            c="g",
            s=2,
        )

        # label the dimensions of the body
        # from the 0th index in the body corners,
        # the:
        #   - l is a vector from the 0th index to the 2nd index,
        #   - w is a vector from the 0th index to the 4th index,
        #   - h is a vector from the 0th index to the 1st index.
        dim_idx = [2, 4, 1]
        for dim, symbol in zip(dim_idx, self.dims.symbols()):
            if not symbol:
                continue

            v_start = rot_body_corners[0].reshape(-1, 1)
            v_end = rot_body_corners[dim].reshape(-1, 1)
            ax = self.__draw_3d_line(ax, v_start, v_end, "k", 0.5)
            v_dim = np.hstack((v_start, v_end))
            ax.text(
                *v_dim.mean(axis=1), symbols_to_latex(symbol), c="g", fontsize="x-small"
            )

        # Fill in the rest of the body
        rest_of_body = [
            (1, 3),
            (1, 5),
            (2, 3),
            (2, 6),
            (3, 7),
            (4, 6),
            (4, 5),
            (5, 7),
            (6, 7),
        ]
        for s_idx, e_idx in rest_of_body:
            ax = self.__draw_3d_line(
                ax, rot_body_corners[s_idx], rot_body_corners[e_idx], "g", 0.5
            )

        return ax

    def __get_body_corners(self):
        # Get body dimensions
        l, w, h = self.dims.values()
        # get all the combinations of body_corners as a np array
        body_corners = np.array(
            np.meshgrid(
                [-l.item() / 2, l.item() / 2],
                [-w.item() / 2, w.item() / 2],
                [-w.item() / 2, w.item() / 2],
            )
        ).T.reshape(-1, 3)

        return body_corners

    def __plot_input(
        self, ax, input_obj, input_loc, pos_ref, rot_ref, color, name_prefix
    ):
        input_val = input_obj.values()
        loc_val = input_loc.values()

        input_val = self.sym_to_np(input_val)
        input_val = rot_ref @ input_val

        loc_val = self.sym_to_np(loc_val)
        loc_val = pos_ref + rot_ref @ loc_val

        # Start and end vectors for arrow
        v_start = loc_val.reshape(-1, 1)
        v_end = (input_val + loc_val).reshape(-1, 1)
        v_vec = np.hstack((v_start, v_end))

        # Plot the arrow and add the label
        arrow = Arrow3D(
            *v_vec.tolist(), mutation_scale=5, lw=1, arrowstyle="-|>", color=color,
        )
        ax.add_artist(arrow)
        ax.text(
            *v_end.reshape(-1,),
            f"${name_prefix}_{input_obj.name}$",
            c=color,
            fontsize="x-small",
        )

        return ax

    def __draw_3d_line(self, ax, p1, p2, color="k", linewidth=1, linestyle="-"):
        """Draw a 3d line from p1 to p2
        """
        p1 = np.array(p1).reshape(-1, 1)
        p2 = np.array(p2).reshape(-1, 1)

        p_vec = np.hstack((p1, p2))

        # plot the vector from the base COM to this body's COM
        ax.plot3D(*p_vec.tolist(), c=color, linewidth=linewidth, linestyle=linestyle)

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
