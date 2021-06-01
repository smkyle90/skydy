#!/usr/bin/python3
import numpy as np
import sympy as sym


class BodyCoordinate:
    def __init__(self, name, x=0, y=0, z=0):
        self.name = name
        self.properties = {
            "x": x,
            "y": y,
            "z": z,
        }
        self.__symbols = sym.Matrix(
            [
                sym.Symbol("l_{}_{}".format(self.name, k)) if v else 0
                for k, v in self.properties.items()
            ]
        )

    def symbols(self):
        return self.__symbols

    def values(self):
        return np.array(list(self.properties.values()))
