#!/usr/bin/python3

import sympy as sym


class MassMatrix:
    def __init__(self, name):
        mass_symbol = sym.Symbol("m_{}".format(name))
        self.__mat = sym.eye(3) * mass_symbol

    def as_mat(self):
        return self.__mat
