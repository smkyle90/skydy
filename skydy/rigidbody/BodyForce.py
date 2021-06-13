#!/usr/bin/python3


from ..configuration import ForceSymbols, TorqueSymbols


class BodyForce(ForceSymbols):
    def __init__(self, name, x_dir=False, y_dir=False, z_dir=False):
        """Define a force that acts on a Body. The direction of the force
        is defined the the relevant body's coordinate frame.

        By default, a force is assgined a default value of 1, if the force is
        defined in that direction, or 0 otherwise.

        Args:
            name (int or str): the name for the force.
            x_dir (bool): if the defined force acts in the body x-direction
            y_dir (bool): if the defined force acts in the body y-direction
            z_dir (bool): if the defined force acts in the body z-direction

        Returns:
            None

        Example:

            >>> from skydy.rigidbody import BodyForce
            >>> # Instantiate a force in the body x-direction
            >>> f_1 = BodyForce("1", x_dir=True)
            >>> # Instantiate another force in the body y-direction
            >>> f_2 = BodyForce("2", y_dir=True)
            >>> # Instantiate another force in all directions
            >>> f_3 = BodyForce("3", True, True, True)

        """
        super().__init__(name)
        self.__x_dir = x_dir
        self.__y_dir = y_dir
        self.__z_dir = z_dir

        self.assign_values(int(x_dir), 0)
        self.assign_values(int(y_dir), 1)
        self.assign_values(int(z_dir), 2)


class BodyTorque(TorqueSymbols):
    def __init__(self, name, x_dir=False, y_dir=False, z_dir=False):
        """Define a torque that acts on a Body. The direction of the torque
        is defined the the relevant body's coordinate frame, and a rotation
        ABOUT that coordinate direction (as per the right hand rule).

        By default, a torque is assgined a default value of 1, if the torque is
        defined in that direction, or 0 otherwise.

        Args:
            name (int or str): the name for the torque.
            x_dir (bool): if the defined torque acts around the body x-direction
            y_dir (bool): if the defined torque acts around the body y-direction
            z_dir (bool): if the defined torque acts around the body z-direction

        Returns:
            None

        Example:

            >>> from skydy.rigidbody import BodyTorque
            >>> # Instantiate a torque about the body x-direction
            >>> t_1 = BodyTorque("1", x_dir=True)
            >>> # Instantiate another torque about the body y-direction
            >>> t_2 = BodyTorque("2", y_dir=True)
            >>> # Instantiate another torque about all directions
            >>> t_3 = BodyTorque("3", True, True, True)

        """

        super().__init__(name)
        self.__x_dir = x_dir
        self.__y_dir = y_dir
        self.__z_dir = z_dir

        self.assign_values(int(x_dir), 0)
        self.assign_values(int(y_dir), 1)
        self.assign_values(int(z_dir), 2)
