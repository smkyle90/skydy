#!/usr/bin/python3

import numpy as np
import sympy as sym

from .BaseSymbols import CoordinateSymbols

NUM_COORDS = 6


class Configuration(CoordinateSymbols):
    def __init__(self, name):
        super().__init__(name)

        # Get the coordinates
        q = self.positions()
        self.__pos_free = sym.Matrix(q[:3])

        # Define the rotation matrices for each axis
        Rx = sym.rot_axis3(q[3]).T
        Ry = sym.rot_axis2(q[4]).T
        Rz = sym.rot_axis1(q[5]).T
        self.__rot_free = sym.simplify(Rz @ Ry @ Rx)

        self.pos_body = None
        self.rot_body = None
        self.free_idx = None
        self.reset_constraints()

    def accelerations(self):
        return sym.Matrix([sym.diff(var, sym.Symbol("t")) for var in self.velocities()])

    def state_vec(self):
        return sym.Matrix.vstack(self.positions(), self.velocities())

    def apply_constraint(self, idx, const_value=0):
        q = self.positions()
        self.assign_values(const_value, idx)
        self.pos_body = sym.simplify(self.pos_body.subs(q[idx], const_value))
        self.rot_body = sym.simplify(self.rot_body.subs(q[idx], const_value))
        self.free_idx -= set([idx])

    def reset_constraints(self):
        self.assign_values(np.ones(self.values().shape))
        self.pos_body = self.__pos_free.copy()
        self.rot_body = self.__rot_free.copy()
        self.free_idx = set([idx for idx in range(NUM_COORDS)])

    def __free_symbols(self, symbols):
        return [symbols[idx] for idx in self.free_idx]

    def free_coordinates(self):
        return self.__free_symbols(self.positions())

    def free_velocities(self):
        return self.__free_symbols(self.velocities())

    def free_accelerations(self):
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
            raise TypeError("Position must be a 3 x 1 sym.Matrix.")

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
            raise TypeError("Position must be a 3 x 3 sym.Matrix.")
