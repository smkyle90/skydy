#!/usr/bin/python3

import numpy as np
import sympy as sym
from sympy.physics.mechanics import dynamicsymbols


class BaseSymbols:
    def __init__(self, name, identifier, coordinates=False):
        self.name = name
        self.__coordinates = coordinates
        if coordinates:
            axes = ["x", "y", "z"]
            trans_symbols = ["{}^{}_{}".format(ax, name, identifier) for ax in axes]
            rot_symbols = ["theta^{}_{}".format(name, ax) for ax in axes]
            all_symbols = trans_symbols + rot_symbols

            self.__symbols = sym.Matrix([dynamicsymbols(var) for var in all_symbols])
        else:
            axes = ["x", "y", "z"]
            self.__symbols = sym.Matrix(
                [sym.Symbol("{}^{}_{}".format(identifier, name, ax)) for ax in axes]
            )

        self.__values = np.ones(self.__symbols.shape)

    def symbols(self):
        if self.__coordinates:
            return self.__symbols
        else:
            sym_dict = self.as_dict()
            return sym.Matrix([k if v else 0 for k, v in sym_dict.items()])

    def values(self):
        return self.__values

    def as_dict(self):
        return {s: v.item() for s, v in zip(self.__symbols, self.__values)}

    def assign_values(self, values, idx=-1):
        if hasattr(values, "__len__"):
            if len(values) == len(self.__values):
                self.__values = np.array(values).reshape(-1, 1)
            else:
                raise ValueError("Values must be same dimension as value object.")
        elif (
            isinstance(values, int)
            or isinstance(values, float)
            or isinstance(values, bool)
        ):
            if 0 <= idx < len(self.__values):
                self.__values[idx, 0] = values
            else:
                raise ValueError(
                    "Index must be in the range 0 to {}".format(len(self.__values))
                )
        else:
            raise TypeError(
                "Values must be an array like object with {} entries, or a value with an appropriate index between 0 and {}".format(
                    len(self.__values), len(self.__values) - 1
                )
            )


class CoordinateSymbols(BaseSymbols):
    def __init__(self, name):
        super().__init__(name, "G", True)

    def positions(self):
        return self.symbols()

    def velocities(self):
        return sym.Matrix([sym.diff(var, sym.Symbol("t")) for var in self.symbols()])


class DimensionSymbols(BaseSymbols):
    def __init__(self, name):
        super().__init__(name, "l", False)


class ForceSymbols(BaseSymbols):
    def __init__(self, name):
        super().__init__(name, "F", False)


class TorqueSymbols(BaseSymbols):
    def __init__(self, name):
        super().__init__(name, "T", False)


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

    def reset_constraints(self):
        self.assign_values(np.ones(self.values().shape))
        self.pos_body = self.__pos_free.copy()
        self.rot_body = self.__rot_free.copy()

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
