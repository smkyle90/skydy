#!/usr/bin/python3
import numpy as np
import sympy as sym

from ..rigidbody import Body
from .Joint import Joint


class Connection:
    def __init__(self, body_in, joint, body_out):
        self.body_in = body_in
        self.joint = joint
        self.body_out = body_out

    @property
    def body_in(self):
        return self._body_in

    @body_in.setter
    def body_in(self, val):
        assert isinstance(val, Body)
        self._body_in = val

    @property
    def body_out(self):
        return self._body_out

    @body_out.setter
    def body_out(self, val):
        assert isinstance(val, Body)
        self._body_out = val

    @property
    def joint(self):
        return self._joint

    @joint.setter
    def joint(self, val):
        assert isinstance(val, Joint)
        self._joint = val

    def as_dict(self):
        return {
            **self.body_in.as_dict(),
            **self.body_out.as_dict(),
            **self.joint.body_in_coord.as_dict(),
            **self.joint.body_out_coord.as_dict(),
        }

    def global_configuration(self):
        for dof in self.joint.dof:
            if not dof.free:
                self.body_out.apply_constraint(dof.idx, dof.const_value)

        # Propagate rotations
        self.body_out.rot_body = sym.simplify(
            self.body_in.rot_body @ self.body_out.rot_body
        )

        # Get absolute positions
        p_in = self.body_in.pos_body  # global coordinate of input link
        p_j_in = sym.simplify(
            self.body_in.rot_body @ self.joint.body_in_coord.symbols()
        )  # global position of connection on input link
        p_out_j = sym.simplify(
            self.body_out.rot_body @ self.joint.body_out_coord.symbols()
        )  # global position of output link to joint
        add_dof = sym.simplify(
            self.body_out.rot_body @ self.body_out.pos_body
        )  # additional dofs from joint

        self.body_out.pos_body = sym.simplify(p_in + p_j_in + p_out_j + add_dof)

    def draw(self, ax=None, sub_vals={}):

        # Plot the joint and degrees of freedom from the joint
        j_loc = (
            self.body_in.pos_body
            + self.body_in.rot_body @ self.joint.body_in_coord.symbols()
        )

        for symbol, value in sub_vals.items():
            j_loc = j_loc.subs(symbol, value)

        j_loc = self.joint.body_in_coord.sym_to_np(j_loc)

        ax.text(
            *(j_loc + 0.01 * np.ones(j_loc.shape)).reshape(-1,).tolist(),
            self.joint.name,
            c="r"
        )

        # Plot the output body
        ax = self.body_out.draw(ax, self.body_in, j_loc, sub_vals)

        return ax
