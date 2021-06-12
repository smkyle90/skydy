#!/usr/bin/python3

import sympy as sym


class MassMatrix:
    def __init__(self, name):
        """Define a generic mass matrix about a body coordinates frame.

        This is not a customizable oject. It is what it is, and it doesn't
        need to be any more.

        Args:
            name (str, int): the body name for the mass matrix

        Returns:
            None

        Examples:

            >>> from skydy.inertia import MassMatrix
            >>> m_mat = MassMatrix(1)
        """
        mass_symbol = sym.Symbol("m_{}".format(name))
        self.__mat = sym.eye(3) * mass_symbol

    def as_mat(self):
        """Return the mass matrix as a sympy.matrix."""
        return self.__mat
