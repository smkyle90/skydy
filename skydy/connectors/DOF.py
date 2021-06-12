#!/usr/bin/python3


class DOF:
    def __init__(self, idx, free=True, const_value=0):
        """A Degree of Freedom is nothing other than a body coordinate that is able
        to move.

        Thus, to define a DOF, we need to simply supply the free index (idx). By default,
        if it is free, there is no constant value, so we do not need to supply the second,
        or third arguments.

        Args:
            idx (int): the free coordinate index, between 0 and 5.
            free (bool): if the coordinte at index idx is free. True by default.
            const_value (int or float): If the DOF is not free, i.e., free=False on instantiation, we assign the constant value the coordinate has. By defualt this is zero.

        Returns:
            None

        Example:
            Demonstrate all combinations of the DOF construction.

                >>> from skydy.connectors import DOF
                >>> # Define a DOF in the x-coordinate
                >>> x_dof = DOF(0)
                >>> # Note the following ALSO defines a coordinate in the y-direction
                >>> y_dof = DOF(1, True)
                >>> # Define a constraint in the z-direction, at a value of 10.
                >>> z_con = DOF(1, False, 10)
                >>> # Define a constraint in the theta_z-direction, at a value of 2.
                >>> theta_z_con = DOF(5, False, 2)

        """
        self.idx = idx
        self.free = free
        self.const_value = const_value

    @property
    def idx(self):
        return self._idx

    @idx.setter
    def idx(self, val):
        if 0 <= val <= 5:
            self._idx = val
        else:
            raise ValueError("idx must be between 0 and 5.")

    @property
    def free(self):
        return self._free

    @free.setter
    def free(self, val):
        if isinstance(val, bool):
            self._free = val
        else:
            raise TypeError("free must be boolean value.")

    @property
    def const_value(self):
        return self._const_value

    @const_value.setter
    def const_value(self, val):

        if isinstance(val, int) or isinstance(val, float):
            if self.free:
                self._const_value = 0
            else:
                self._const_value = val
        else:
            raise TypeError("const_value attribute must be boolean value.")
