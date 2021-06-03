#!/usr/bin/python3

import sympy as sym
from sympy.physics.mechanics import dynamicsymbols


class BaseSymbols:
    def __init__(self, name, identifier, coordinates=False):
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

    def symbols(self):
        return self.__symbols

    # def __repr__(self):
    #     self.symbols()

    # def values(self):
    #     return np.array(list(self.properties.values()))


class BodyCoordinate(BaseSymbols):
    def __init__(self, name):
        super().__init__(name, "G", coordinates=True)

    def positions(self):
        return self.symbols()

    def velocities(self):
        return sym.Matrix([sym.diff(var, sym.Symbol("t")) for var in self.symbols()])


class BodyDimension(BaseSymbols):
    def __init__(self, name):
        super().__init__(name, "l", False)


class BodyForce(BaseSymbols):
    def __init__(self, name):
        super().__init__(name, "F", False)


class BodyTorque(BaseSymbols):
    def __init__(self, name):
        super().__init__(name, "T", False)


class Configuration(BodyCoordinate):
    def __init__(self, name):
        super().__init__(name)

        # Get the coordinates
        q = self.positions()
        self.__r_free = sym.Matrix(q[:3])

        # Define the rotation matrices for each axis
        Rx = sym.rot_axis3(q[3]).T
        Ry = sym.rot_axis2(q[4]).T
        Rz = sym.rot_axis1(q[5]).T
        self.__R_free = sym.simplify(Rz @ Ry @ Rx)

    def r_body(self):
        return self.__r_free

    def R_body(self):
        return self.__R_free

    def accelerations(self):
        return sym.Matrix([sym.diff(var, sym.Symbol("t")) for var in self.velocities()])

    def state_vector(self):
        return sym.Matrix.vstack(self.positions(), self.velocities())


# class Configuration:
#     def __init__(self, name):
#         q = ["x_G", "y_G", "z_G", "theta_x", "theta_y", "theta_z"]
#         q = ["{}_{}".format(var, name) for var in q]
#         self.q = [dynamicsymbols(var) for var in q]
#         self.v = [sym.diff(var, sym.Symbol("t")) for var in q]

#         self.r = None
#         self.R = None

#         self.__free_body_configuration()

#     def get_accelerations(self):
#         return sym.Matrix([sym.diff(var, sym.Symbol("t")) for var in self.v])


#     def __free_body_configuration(self):
#         # Define the body position vector
#         self.r = sym.Matrix(self.q[:3])

#         # Define the rotation matrices for each axis
#         Rx = sym.rot_axis3(self.q[3]).T
#         Ry = sym.rot_axis2(self.q[4]).T
#         Rz = sym.rot_axis1(self.q[5]).T
#         self.R = sym.simplify(Rz @ Ry @ Rx)
