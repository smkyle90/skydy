#!/usr/bin/python3
import numpy as np
import sympy as sym

from ..configuration import DimensionSymbols


class BodyCoordinate(DimensionSymbols):
    def __init__(self, name, x=0, y=0, z=0):
        super().__init__(name)
        self.assign_values(x, 0)
        self.assign_values(y, 1)
        self.assign_values(z, 2)


class GroundCoordinate(BodyCoordinate):
    def __init__(self):
        super().__init__("O")
