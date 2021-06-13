#!/usr/bin/python3

import numpy as np
import sympy as sym

from .BaseSymbols import CoordinateSymbols

NUM_COORDS = 6


class Configuration(CoordinateSymbols):
    def __init__(self, name):
        """A body's configuration is nothing other than a description of its pose
        (where it is, and how it is oriented).

        As such, it is decribed by a vector of positions, and a matrix of rotations.
        All bodies can have up to 6 DOFs, i.e., directions in which it can move.

        By applying constraints, a body can have as little as zero DOFs.

        A Configuration inherits from CoordinateSymbols, as it is solely related to a
        body's name, and the 6 CoordinateSymbols that describe it.

        Args:
            name (int or str): the name for the symbols. This will form the superscript, i.e., the body the symbols refer to.

        Returns:
            None

        Example:
            Configuration for body named "1".

                >>> from skydy.configuration import Configuration
                >>> body_name = "1"
                >>> body_config = Configuration(body_name)
                >>> # See the symbolic position and rotation of the body
                >>> print(body_config.pos_body)
                >>> print(body_config.rot_body)

        """
        super().__init__(name)

        # Get the coordinates
        q = self.positions()

        # Define the rotation matrices for each axis
        Rx = sym.rot_axis1(q[3]).T
        Ry = sym.rot_axis2(q[4]).T
        Rz = sym.rot_axis3(q[5]).T

        # Define the free, or unconstrained, configuration
        self.__pos_free = sym.Matrix(q[:3])
        self.__rot_free = sym.simplify(Rz @ Ry @ Rx)

        self.pos_body = None
        self.rot_body = None
        self.free_idx = None
        self.reset_constraints()

    def accelerations(self):
        """Returns the acceleration of the coordinates of the body."""
        return sym.Matrix([sym.diff(var, sym.Symbol("t")) for var in self.velocities()])

    def apply_constraint(self, idx, const_value=0):
        """Apply a coordinate constraint to the free configuration of the body.
        A constraint is nothing but a coordinate having constant value.

        This indices for each coordinate are:
            0 -> x
            1 -> y
            2 -> z
            3 -> theta_x
            4 -> theta_y
            5 -> theta_z

        Args:
            idx (int): the index to apply the constriaint to.
            const_value (int or float): the constant value to substitute in for the coordinate at the index. Default value is zero.

        Returns:
            None

        Example:
            Constrain some coordinate for a body named "1".

                >>> from skydy.configuration import Configuration
                >>> body_name = "1"
                >>> body_config = Configuration(body_name)
                >>> # Apply a translational constraint to the z-axis, at a height of 5 m.
                >>> body_config.apply_constraint(2, 5)
                >>> # Apply a rotational constraint about the y-axis
                >>> body_config.apply_constraint(4, 0)

        """
        q = self.positions()
        self.assign_values(const_value, idx)
        self.pos_body = self.pos_body.subs(q[idx], const_value)
        self.rot_body = self.rot_body.subs(q[idx], const_value)
        self.free_idx -= set([idx])

    def reset_constraints(self):
        """Reset the constraints, i.e., remove any restrictions on movement.

        In short, the position and rotation are the free configuration matrices
        determined on object instantiation.
        """
        self.assign_values(np.ones(self.values().shape))
        self.pos_body = self.__pos_free.copy()
        self.rot_body = self.__rot_free.copy()
        self.free_idx = set([idx for idx in range(NUM_COORDS)])

    def __free_symbols(self, symbols):
        """Helper method to return the free symbols for a sympy.matrix object.

        Args:
            symbols (sympy.matrices.dense.MutableDenseMatrix or list): a list of symbols.

        Returns:
            free_symbols (list): a list of free symbols from the input list, based on the free configuration indices.

        Example:
            See self.free_coordinates(), self.free_velocities(), self.free_accelerations() below.
        """
        return [symbols[idx] for idx in self.free_idx]

    def free_coordinates(self):
        """Return a list of free coordinates for the body."""
        return self.__free_symbols(self.positions())

    def free_velocities(self):
        """Return a list of free velocities for the body."""
        return self.__free_symbols(self.velocities())

    def free_accelerations(self):
        """Return a list of free accelerations for the body."""
        return self.__free_symbols(self.accelerations())

    @property
    def pos_body(self):
        return self._pos_body

    @pos_body.setter
    def pos_body(self, val):
        if val is None:
            self._pos_body = val
        elif (
            isinstance(val, sym.matrices.immutable.ImmutableDenseMatrix)
            or isinstance(val, sym.Matrix)
        ) and val.shape == (3, 1):
            self._pos_body = val
        else:
            raise TypeError("Body Position must be a 3 x 1 sym.Matrix.")

    @property
    def rot_body(self):
        return self._rot_body

    @rot_body.setter
    def rot_body(self, val):
        if val is None:
            self._rot_body = val
        elif (
            isinstance(val, sym.matrices.immutable.ImmutableDenseMatrix)
            or isinstance(val, sym.Matrix)
            and val.shape == (3, 3)
        ):

            self._rot_body = val
        else:
            raise TypeError("Body Rotation must be a 3 x 3 sym.Matrix.")
