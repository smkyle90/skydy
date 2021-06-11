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

        # Propagate rotations from input body to output body
        self.body_out.rot_body = sym.simplify(
            self.body_in.rot_body @ self.body_out.rot_body
        )

        # Get absolute positions
        # Global coordinate of COM of input link
        p_in = self.body_in.pos_body

        # Rotated position of joint on input link
        p_j_in = sym.simplify(
            self.body_in.rot_body @ self.joint.body_in_coord.symbols()
        )

        # Global position of COM of output link to joint
        p_out_j = sym.simplify(
            self.body_out.rot_body @ self.joint.body_out_coord.symbols()
        )

        # Additional DOFs from joint, in the input links coordinate Frame.
        add_dof = sym.simplify(self.body_in.rot_body @ self.body_out.pos_body)

        self.body_out.pos_body = sym.simplify(p_in + p_j_in + p_out_j + add_dof)

    def draw(self, ax=None, sub_vals={}):

        # Plot the joint and degrees of freedom from the joint
        joint_loc = (
            self.body_in.pos_body
            + self.body_in.rot_body @ self.joint.body_in_coord.symbols()
        )

        joint_loc = joint_loc.subs(sub_vals)
        joint_loc = self.joint.body_in_coord.sym_to_np(joint_loc)

        ax.text(
            *(joint_loc + 0.01 * np.ones(joint_loc.shape)).reshape(-1,).tolist(),
            self.joint.name,
            c="r",
            fontsize="x-small",
        )

        # Plot the output body
        ax = self.body_out.draw(ax, self.body_in, joint_loc, sub_vals)

        return ax
