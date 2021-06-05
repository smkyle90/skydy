#!/usr/bin/python3

import sympy as sym


class InertiaMatrix:
    def __init__(self, name):
        ax = ["x", "y", "z"]
        inertia_matrix = [
            [sym.Symbol("I^{}_{}{}".format(name, a, b)) for a in ax] for b in ax
        ]
        self.__mat = sym.Matrix(inertia_matrix)

    def as_mat(self):
        return self.__mat
