#!/usr/bin/python3

import numpy as np
import sympy as sym
from sympy.physics.mechanics import dynamicsymbols


class BaseSymbols:
    def __init__(self, name, identifier, is_coords=False):
        """The base object for this whole package to function. It defines the sympy
        symbols that are used in the modelling process, whether it be translational
        (x,y,z), rotational (theta_x, theta_y, theta_z) coordinates, positions within
        a reference frame, or the direction and magnitude of forces and torques,
        all the symbols are generated by various combinations of this object.

        Later, we have specific object thats handle these scenarios for us, but all
        inherit from this class. s such, we never explicility use this class

        Args:
            name (int or str): the name for the symbols. This will form the superscript,
            i.e., the body the symbols refer to.
            identifier (int or str): gives meaning to the symbol set as it identifies
            what or where the symbol set refer to. For a set of coordinates, this is "G",
            the centre of mass. For a force, torque or length it is F, tau and l respectively.
            is_coords (int or str): defines if a sextuple is defines (representing the
            6 coordinates) of a body, or a triple (representing the three Cartesian coordinates).

        Example:
            A coordinate representation of body named "1", at its centre of mass, G.

                >>> from skydy.configuration import BaseSymbols
                >>> body_name = "1"
                >>> body_loc = "G"
                >>> body_symbols = BaseSymbols(body_name, body_loc, True)

            A vector representation of force named "2"

                >>> force_name = "2"
                >>> force_id = "F"
                >>> force_symbols = BaseSymbols(force_name, force_id)

            A vector representation of distance, or dimension for a body named "1"

                >>> dim_name = "1"
                >>> dim_id = "l"
                >>> dim_symbols = BaseSymbols(dim_name, dim_id)
        """
        self.name = name
        self.__is_coords = is_coords
        if is_coords:
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

        self.__values = np.ones(self.__symbols.shape)

    def symbols(self):
        """Get the sympy.matrices.dense.MutableDenseMatrix of the symbols."""
        if self.__is_coords:
            return self.__symbols
        else:
            sym_dict = self.as_dict()
            # only return the symbol if the value is non-zero.
            return sym.Matrix([k if v else 0 for k, v in sym_dict.items()])

    def values(self):
        """Return a numpy.ndarray of the values. If the values are not assigned,
        they return an array of ones by default."""
        return self.__values

    def as_dict(self):
        """Get the dictionary of symbol-value pairs."""
        return {s: v.item() for s, v in zip(self.__symbols, self.__values)}

    def assign_values(self, values, idx=-1):
        """Assign value(s) to a symbol. By defualt, the value is instantiated as 1.

        Args:
            values (array-like object, int or float): the value(s) we want to assign.
            This must be an array-like object of the len(self.__symbols), or an integer.
            If an integer is provided, a valid index must be provided.
            idx (int): for an int or float, the index to assign the value to.
            This must be a valid index in 0 to len(self.__symbols)-1.

        Returns:
            None

        Example:

            >>> from skydy.configuration import BaseSymbols
            >>> base_symbols = BaseSymbols("1", "G")
            >>> # Assign all values
            >>> base_symbols.assign_values([10, 20, 30, 40, 50, 60])
            >>> # Assign just one value
            >>> base_symbols.assign_values(24, 4)
        """

        if isinstance(values, str):
            raise TypeError("Values must be an integer, float or boolean.")
        elif hasattr(values, "__len__"):
            if len(values) == len(self.__values):
                self.__values = np.array(values).reshape(-1, 1)
            else:
                raise ValueError("Values must be same dimension as value object.")
        elif (
            isinstance(values, int)
            or isinstance(values, float)
            or isinstance(values, bool)
        ):
            if 0 <= idx < len(self.__values):
                self.__values[idx, 0] = values
            else:
                raise ValueError(
                    "Index must be in the range 0 to {}".format(len(self.__values))
                )
        else:
            raise TypeError(
                "Values must be an array like object with {} entries, or a value with an appropriate index between 0 and {}".format(
                    len(self.__values), len(self.__values) - 1
                )
            )

    def sym_to_np(self, sym_matrix):
        """Convert a numeric sympy matrix, i.e., after substituting values, to a numpy
        matrix. We pass in the sympy matrix for flexbility down the line as to what we
        can and cannot convert.

        Args:
            sym_matrix (sympy.matrices.dense.MutableDenseMatrix): a sympy matrix, with
            values substituted in.

        Returns:
            np_matrix (numpy.ndarray): a numpy matrix of same dimensions of the input matrix.

        Example:

            >>> from skydy.configuration import BaseSymbols
            >>> base_symbols = BaseSymbols("1", "G")
            >>> # Assign all values
            >>> base_symbols.assign_values([10, 20, 30, 40, 50, 60])
            >>> # Crete a matrix of symbols
            >>> sym_mat = base_symbols.symbols()
            >>> # Substitute the values in
            >>> sym_mat = sym_mat.subs(base_symbols.as_dict())
            >>> # Convert to an numpy.ndarray.
            >>> np_mat = base_symbols.sym_to_np(sym_mat)

        """
        return np.array(sym_matrix.tolist()).astype(float)


class CoordinateSymbols(BaseSymbols):
    def __init__(self, name):
        """A set of coordinate symbols, inheriting from the BaseSymbols class.
        Coordinate symbols are simply the translational (x,y,z) and rotational
        (theta_x, theta_y, theta_z) coordinates about the centre of mass of a body.

        Args:
            name (int, float or string): the name of the body or object we want to
            generate coordinates for.

        Returns:
            None

        Example:

            >>> from skydy.configuration import CoordinateSymbols
            >>> # Create a set of symbols
            >>> body_name = 1
            >>> body_coords = CoordinateSymbols(body_name)

        """
        super().__init__(name, "G", True)

    def positions(self):
        """Returns the symbols of the positions (or coordinates) of the body."""
        return self.symbols()

    def velocities(self):
        """Returns the velocities of the positions (or coordinates) of the body."""
        return sym.Matrix([sym.diff(var, sym.Symbol("t")) for var in self.symbols()])


class DimensionSymbols(BaseSymbols):
    def __init__(self, name):
        """A set of dimension symbols, inheriting from the BaseSymbols class. Dimension
        symbols simply exist in  x-, y-, z-coordinates. By default, these symbols are
        defined by l, or a length.

        Args:
            name (int, float or string): the name of the body or object we want to
            generate dimensions for.

        Returns:
            None

        Example:

            >>> from skydy.configuration import DimensionSymbols
            >>> # Create a set of symbols
            >>> body_name = 1
            >>> body_dims = DimensionSymbols(body_name)

        """
        super().__init__(name, "l", False)


class ForceSymbols(BaseSymbols):
    def __init__(self, name):
        """A set of force symbols, inheriting from the BaseSymbols class. Force symbols
        simply exist in  x-, y-, z-coordinates. By default, these symbols are defined
        by F, or a Force.

        Args:
            name (int, float or string): the name of the body or object we want to
            designate a force for.

        Returns:
            None

        Example:

            >>> from skydy.configuration import ForceSymbols
            >>> # Create a set of symbols
            >>> body_name = 1
            >>> body_forces = ForceSymbols(body_name)

        """
        super().__init__(name, "F", False)


class TorqueSymbols(BaseSymbols):
    def __init__(self, name):
        """A set of torque symbols, inheriting from the BaseSymbols class. Torque
        symbols simply exist as rotations about the x-, y-, z-coordinates. By default,
        these symbols are defined by tau, or a Torque.

        Args:
            name (int, float or string): the name of the body or object we want to
            designate a Torque for.

        Returns:
            None

        Example:

            >>> from skydy.configuration import TorqueSymbols
            >>> # Create a set of symbols
            >>> body_name = 1
            >>> body_torques = TorqueSymbols(body_name)

        """
        super().__init__(name, "tau", False)
