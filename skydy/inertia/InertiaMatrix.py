#!/usr/bin/python3

import sympy as sym


class InertiaMatrix:
    def __init__(self, name):
        """Define a generic inertia matrix about a body coordinates frame.

        This is not a customizable oject. It is what it is, and it doesn't
        need to be any more.

        Args:
            name (str, int): the body name for the inertia matrix

        Returns:
            None

        Examples:

            >>> from skydy.inertia import InertiaMatrix
            >>> i_mat = InertiaMatrix(1)
        """
        ax = ["x", "y", "z"]
        inertia_matrix = [
            [sym.Symbol("I^{}_{}{}".format(name, a, b)) for a in ax] for b in ax
        ]
        self.__mat = sym.Matrix(inertia_matrix)

    def as_mat(self):
        """Return the inertia matrix as a sympy.matrix."""
        return self.__mat
