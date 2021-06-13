#!/usr/bin/python3

from ..configuration import DimensionSymbols


class BodyCoordinate(DimensionSymbols):
    def __init__(self, name, x=0, y=0, z=0):
        """A coordinate of a point on a body. Inherits from DimensionSymbols.


        Args:
            name (int, float or string): the name of the body or object we want to
            generate dimensions for.
            x (int or float): the x-coordinate to a point from the body's COM
            y (int or float): the y-coordinate to a point from the body's COM
            z (int or float): the z-coordinate to a point from the body's COM

        Returns:
            None

        Example:

            >>> from skydy.configuration import BodyCoordinate
            >>> # Create a set of symbols
            >>> body_name = 1
            >>> # Default constructor
            >>> body_coord = BodyCoordinate(body_name)
            >>> # Assign some distances
            >>> body_coord = BodyCoordinate(body_name, 1, 2, 3)
            >>> body_coord = BodyCoordinate(body_name, 3, -4, 4)
        """

        super().__init__(name)
        self.assign_values(x, 0)
        self.assign_values(y, 1)
        self.assign_values(z, 2)


class GroundCoordinate(BodyCoordinate):
    def __init__(self):
        """A BodyCoordinate with a default name of O, for origin,
        and (x, y, z) = (0, 0, 0)

        Example:

            >>> from skydy.rigidbody import GroundCoordinate
            >>> gnd_coord = GroundCoordinate()
        """
        super().__init__("O")
