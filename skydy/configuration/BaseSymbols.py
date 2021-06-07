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
        super().__init__(name, "tau", False)
