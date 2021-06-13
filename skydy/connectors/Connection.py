#!/usr/bin/python3
import matplotlib.pyplot as plt
import numpy as np

from ..rigidbody import Body
from .Joint import Joint


class Connection:
    def __init__(self, body_in, joint, body_out):
        """Define the connection fo two bodies, through a joint.

        Args:
            body_in (Body): the input body
            joint (Joint): the joint, defined as a common location for the input and output bodies, and the associated DOFs. Note, it is critical here, that the joint's input coordinate is in body_in coordinate frame, and the output coordinate is in body_out coordinate frame.
            body_in (Body): the output body

        Returns:
            None

        Examples:

            >>> from skydy.connectors import DOF, Connection, Joint
            >>> from skydy.rigidbody import Body, BodyCoordinate, Ground
            >>> # Two point-masses that meet at the origin
            >>> p0 = BodyCoordinate("O")
            >>> p1 = BodyCoordinate("G/O", 0, 0, 0)
            >>> # Assume the joint can move in the x-coordinate
            >>> j1 = Joint(p0, p1, [DOF(0)])
            >>> # Define the two bodies
            >>> b1 = Body()
            >>> b2 = Body()
            >>> # Define the connection
            >>> cnx = Connection(b1, j1, b2)

        """
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
        """Return a dictionary of the coordinate and properties of the connection."""
        return {
            **self.body_in.as_dict(),
            **self.body_out.as_dict(),
            **self.joint.body_in_coord.as_dict(),
            **self.joint.body_out_coord.as_dict(),
        }

    def global_configuration(self):
        """Propagate the configuration from the input body, to the output
        body through the joint.

        Updates the attribute values in place.
        """
        for dof in self.joint.dof:
            if not dof.free:
                self.body_out.apply_constraint(dof.idx, dof.const_value)

        # TODO: draw a diagram to accompany this.

        # Propagate rotations from input body to output body
        self.body_out.rot_body = self.body_in.rot_body @ self.body_out.rot_body

        # Get absolute positions
        # Global coordinate of COM of input link
        p_in = self.body_in.pos_body

        # Rotated position of joint on input link
        p_j_in = self.body_in.rot_body @ self.joint.body_in_coord.symbols()

        # Global position of COM of output link wrt joint
        p_out_j = self.body_out.rot_body @ self.joint.body_out_coord.symbols()

        # Additional DOFs from joint, in the input links coordinate Frame.
        add_dof = self.body_in.rot_body @ self.body_out.pos_body

        # The output links position is the sum of these 4 vectors
        self.body_out.pos_body = p_in + p_j_in + p_out_j + add_dof

    def draw(self, ax=None, sub_vals=None):
        """Draw the connection

        Args:
            ax (matplotlib.axes._subplots.AxesSubplot): the axis to plot the connection on.
            sub_vals (dict): symbol-value pairs required to go from symbolic to numeric expression. It is important to note, that all symbols each body is dependent on, for example, for upstream bodies and joints, are included.

        Returns:
            ax (matplotlib.axes._subplots.AxesSubplot): updated axes, with plots.

        Example:
            >>> import matplotlib.pyplot as plt
            >>> from skydy.connectors import DOF, Connection, Joint
            >>> from skydy.rigidbody import Body, BodyCoordinate, Ground

            >>> # Two point-masses that meet at the origin
            >>> p0 = BodyCoordinate("O")
            >>> p1 = BodyCoordinate("G/O", 0, 0, 0)
            >>> # Assume the joint can move in the x-coordinate
            >>> j1 = Joint(p0, p1, [DOF(0)], "J")
            >>> # Define the two bodies
            >>> b1 = Body()
            >>> b2 = Body()
            >>> # Define the connection
            >>> cnx = Connection(b1, j1, b2)
            >>> # Define the axes
            >>> fig = plt.figure()
            >>> ax = fig.add_subplot(111, projection='3d')
            >>> ax = cnx.draw(ax)
            >>> plt.show()

        """

        if ax is None:
            fig = plt.figure()
            ax = fig.add_subplot(111, projection="3d")

        if sub_vals is None:
            sub_vals = {}

        sub_vals = {
            **sub_vals,
            **self.as_dict(),
        }

        # Plot the joint and degrees of freedom from the joint
        joint_loc = (
            self.body_in.pos_body
            + self.body_in.rot_body @ self.joint.body_in_coord.symbols()
        )

        joint_loc = joint_loc.subs(sub_vals)
        joint_loc = self.joint.body_in_coord.sym_to_np(joint_loc)

        ax.text(
            *(joint_loc + 0.01 * np.ones(joint_loc.shape))
            .reshape(
                -1,
            )
            .tolist(),
            self.joint.name,
            c="r",
            fontsize="x-small",
        )

        # Plot the output body
        ax = self.body_out.draw(ax, self.body_in, joint_loc, sub_vals)

        return ax
