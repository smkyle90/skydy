#!/usr/bin/python3


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
