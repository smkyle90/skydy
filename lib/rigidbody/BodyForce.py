#!/usr/bin/python3

import sympy as sym


class BodyForce:
    def __init__(
        self, name, location, x_dir=False, y_dir=False, z_dir=False, prefix="F"
    ):
        self.name = str(name)
        self.location = location
        ax = ["x", "y", "z"]
        direction = [x_dir, y_dir, z_dir]
        self.direction = sym.Matrix(
            [
                sym.Symbol("{}_{}_{}".format(prefix, self.name, a)) if f else 0
                for a, f in zip(ax, direction)
            ]
        )

    @property
    def location(self):
        return self._location

    @location.setter
    def location(self, val):
        self._location = val
        # if val is None:
        #     self._location = val
        # elif isinstance(val, BodyCoordinate):
        #     self._location = val
        # else:
        #     raise TypeError("Force location must be a BodyCoordinate (force) or None (torque).")


class BodyTorque(BodyForce):
    def __init__(self, name, x_dir=False, y_dir=False, z_dir=False, prefix="T"):
        super().__init__(name, None, x_dir, y_dir, z_dir, prefix)
