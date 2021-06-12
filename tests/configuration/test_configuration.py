"""Test the example function
"""

import numpy as np
import pytest
import sympy as sym


@pytest.mark.configuration
def test_BaseSymbols():
    from skydy.configuration import BaseSymbols

    b = BaseSymbols("1", "G", is_coords=True)

    print(b.symbols())
    print(b.values())
    print(b.as_dict())

    b.assign_values(2, 2)
    b.assign_values(3, 3)
    b.assign_values(4, 4)

    print(b.values())
    print(b.as_dict())

    b = BaseSymbols("2", "T", is_coords=False)
    print(b.symbols())
    b.assign_values([2, 4, 6])

    print(b.values())
    print(b.as_dict())

    print(b.sym_to_np(b.symbols().subs(b.as_dict())))


@pytest.mark.configuration
def test_BaseSymbols_bad_types():
    from skydy.configuration import BaseSymbols

    b = BaseSymbols(1, "G", True)

    # Incorrent number of values in list
    with pytest.raises(ValueError):
        b.assign_values([1, 2, 3])
        b.assign_values([1, 2, 3, 4, 5, 6, 7])

    # Out of bounds index
    with pytest.raises(ValueError):
        b.assign_values(2, -1)
        b.assign_values(21, 6)

    # Incorrrect value (must be int, float or bool)
    with pytest.raises(TypeError):
        b.assign_values("Bad Type", 0)


@pytest.mark.configuration
def test_CoordinateSymbols():
    from skydy.configuration import CoordinateSymbols

    body = CoordinateSymbols("1")
    print(body.symbols())
    print(body.velocities())
    print(body.positions())

    body = CoordinateSymbols("2")
    print(body.symbols())
    print(body.velocities())
    print(body.positions())

    body = CoordinateSymbols("rod")
    print(body.symbols())
    print(body.velocities())
    print(body.positions())


@pytest.mark.configuration
def test_DimensionSymbols():
    from skydy.configuration import DimensionSymbols

    dim_syms = DimensionSymbols("1")
    print(dim_syms.symbols())

    dim_syms = DimensionSymbols("2")
    print(dim_syms.symbols())

    dim_syms = DimensionSymbols("rod")
    print(dim_syms.symbols())


@pytest.mark.configuration
def test_ForceSymbols():
    from skydy.configuration import ForceSymbols

    force_syms = ForceSymbols("1")
    print(force_syms.symbols())

    force_syms = ForceSymbols("2")
    print(force_syms.symbols())

    force_syms = ForceSymbols("rod")
    print(force_syms.symbols())


@pytest.mark.configuration
def test_TorqueSymbols():
    from skydy.configuration import TorqueSymbols

    torque_syms = TorqueSymbols("1")
    print(torque_syms.symbols())

    torque_syms = TorqueSymbols("2")
    print(torque_syms.symbols())

    torque_syms = TorqueSymbols("rod")
    print(torque_syms.symbols())


@pytest.mark.configuration
def test_Configuration():
    from skydy.configuration import Configuration

    c = Configuration("1")

    print(c.positions())
    print(c.velocities())
    print(c.accelerations())

    print(c.pos_body)
    print(c.rot_body)

    c.apply_constraint(1, 0)
    print(c.pos_body)

    c.apply_constraint(0, 10)
    print(c.pos_body)

    c.apply_constraint(3, np.pi)
    c.apply_constraint(4, np.pi / 2)
    c.apply_constraint(5, 0)
    print(c.pos_body)

    print(c.as_dict())

    c.reset_constraints()
    print(c.pos_body)
    print(c.rot_body)
    print(c.as_dict())


@pytest.mark.configuration
def test_Configuration_bad_types():
    from skydy.configuration import Configuration

    c = Configuration("1")

    # Bad types
    with pytest.raises(TypeError):
        c.pos_body = "some_val"
        c.pos_body = 1
        c.pos_body = [1, 2, 3]
        c.pos_body = np.array([1, 2, 3])
        c.pos_body = sym.Matrix([1, 2])
        c.pos_body = sym.Matrix([1, 2, 3, 4])

    # Bad types
    with pytest.raises(TypeError):
        c.rot_body = "some_val"
        c.rot_body = 1
        c.rot_body = [1, 2, 3]
        c.rot_body = np.array([1, 2, 3])
        c.rot_body = np.eye(2)
        c.rot_body = np.eye(4)
        c.rot_body = sym.ones(2, 2)
        c.rot_body = sym.ones(4, 4)
        c.rot_body = sym.ones(3, 1)
