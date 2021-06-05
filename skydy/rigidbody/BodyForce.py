#!/usr/bin/python3

import sympy as sym

from ..configuration import ForceSymbols, TorqueSymbols


class BodyForce(ForceSymbols):
    def __init__(self, name, x_dir=False, y_dir=False, z_dir=False):
        super().__init__(name)
        self.__x_dir = x_dir
        self.__y_dir = y_dir
        self.__z_dir = z_dir

        self.assign_values(int(x_dir), 0)
        self.assign_values(int(y_dir), 1)
        self.assign_values(int(z_dir), 2)


class BodyTorque(TorqueSymbols):
    def __init__(self, name, x_dir=False, y_dir=False, z_dir=False):
        super().__init__(name)
        self.__x_dir = x_dir
        self.__y_dir = y_dir
        self.__z_dir = z_dir

        self.assign_values(int(x_dir), 0)
        self.assign_values(int(y_dir), 1)
        self.assign_values(int(z_dir), 2)

    # @property
    # def location(self):
    #     return self._location

    # @location.setter
    # def location(self, val):
    #     self._location = val
    #     # if val is None:
    #     #     self._location = val
    #     # elif isinstance(val, BodyCoordinate):
    #     #     self._location = val
    #     # else:
    #     #     raise TypeError("Force location must be a BodyCoordinate (force) or None (torque).")


# class BodyTorque(BodyForce):
#     def __init__(self, name, x_dir=False, y_dir=False, z_dir=False, prefix="T"):
#         super().__init__(name, None, x_dir, y_dir, z_dir, prefix)
