#!/usr/bin/python3
from ..rigidbody import BodyCoordinate
from .DOF import DOF


class Joint:
    id_counter = 0

    def __init__(
        self,
        body_in_coord,
        body_out_coord,
        dof=None,
        name=None,
    ):
        """
        A Joint a common location for two bodies to interact, and how the bodies can
        move relative to each other, based on the DOFs or constraints a joint has.

        A Joint needs to be defined in the inputs AND output body's coordinate frames.

        By default, a joint is assumed to be free in all directions. If a user defines
        a free or non-free DOF, we account for it. Any unspecified coordintes indices are
        assumed to be constrained at a value of zero.

        Diagram:

        |  ⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀z2⠀⠀⠀y2
        |  ⠀⠀z1⠀⠀y1⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀\\⠀⠀⠀/
        |  ⠀⠀⠀|⠀⠀/⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀p_j⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀\\⠀/
        |  ⠀⠀⠀|⠀/⠀⠀⠀⠀⠀_____---->X-----_____⠀⠀⠀⠀\\/
        |  ⠀⠀⠀|/....----⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀----->O2----x2
        |  ⠀⠀⠀O1-------x1⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀^G2
        |  ⠀⠀⠀^G1

        |  Let:
        |  - Body 1 have CoM at O1, position r1, and orientation R1;
        |  - Body 2 have CoM at O2, position r2, and orientation R2;
        |  - suppose a there is a Joint at the point p_j.

        |  Then, the point p_j can be described in both Body 1 and Body 2's coordinate frames, and given by:
        |  - body_in_coord (Body 1) is the vector from O1 -> p_j in (x1, y1, z1), designated P_J/O1;
        |  - body_out_coord (Body 2), is the vector from p_j -> O2 in (x2, y2, z2), designated P_O2/J.

        The degrees of freedom are defined as motion in the input Body's coordinate frame, and is equivalent to
        the body position of r2 and orientation R2, with the joint DOFs applied to this body's coordinates.

        This means that with knowledge of the input body position and orientation, DOFs, and the two vectors O1 -> p_j and
        p_j -> O2, the global position and orientation of the output is defined by

        |  p_O2 = p_O1 + R1 * (P_J/O1 + P_J/dof + R2 * P_O2/J),
        |  p_O2 = r1 + R1 * (P_J/O1 + r2 + R2 * P_O2/J).

        Note, these are exactly the calculations that are done by the Connection object when calculating the global configuration.

        Args:
            body_in_coord (BodyCoordinate): the location of the joint in the input body's coordinate frame.
            body_out_coord (BodyCoordinate): the location of the joint in the output body's coordinate frame.
            dof (list(DOF), or None): the list of DOFs for the joint. By default, all coordinates are free.
            name (int or str): the name of the joint.

        Returns:
            None

        Example:

            >>> from skydy.connectors import DOF
            >>> from skydy.rigidbody import BodyCoordinate
            >>> # Define the location of the joint in the input coordinate frame
            >>> p_1 = BodyCoordinate(1, 10, 4, 5)
            >>> # Define the location of the joint in the output coordinate frame
            >>> p_2 = BodyCoordinate(2, -5, -3, -2)
            >>> # Define the DOFs for our joint. Say x-direction and
            >>> # theta_y directions
            >>> j_dofs = [DOF(0), DOF(4)]
            >>> # Define the joint
            >>> joint = Joint(p_1, p_2, j_dofs, "J")
            >>> # Check which DOFs are free. Expect 0 and 4.
            >>> for dof in joint.dof:
            >>>     if dof.free:
            >>>         print(dof.idx)

        """
        # Body accounting
        Joint.id_counter += 1
        self.joint_id = Joint.id_counter
        if name is None:
            self.name = str(self.joint_id)
        else:
            self.name = str(name)

        self.dof = dof

        self.body_in_coord = body_in_coord
        self.body_out_coord = body_out_coord

    @property
    def body_in_coord(self):
        return self._body_in_coord

    @body_in_coord.setter
    def body_in_coord(self, val):
        assert isinstance(val, BodyCoordinate)
        self._body_in_coord = val

    @property
    def body_out_coord(self):
        return self._body_out_coord

    @body_out_coord.setter
    def body_out_coord(self, val):
        assert isinstance(val, BodyCoordinate)
        self._body_out_coord = val

    @property
    def dof(self):
        return self._dof

    @dof.setter
    def dof(self, val):
        dof_idx = []

        # Default value means all DOFs are free
        if val is None:
            val = [DOF(0), DOF(1), DOF(2), DOF(3), DOF(4), DOF(5)]

        for v in val:
            assert isinstance(v, DOF)
            dof_idx.append(v.idx)

        all_idx = set([i for i in range(6)])
        rem_idx = all_idx - set(dof_idx)

        for idx in rem_idx:
            val.append(DOF(idx, False))
        self._dof = val
