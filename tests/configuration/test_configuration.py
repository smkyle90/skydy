"""Test the example function
"""

import numpy as np
import pytest


@pytest.mark.configuration
def test_BaseSymbols():
    from skydy.configuration import BaseSymbols

    b = BaseSymbols("1", "G", coordinates=True)

    print(b.symbols())
    print(b.values())
    print(b.as_dict())

    b.assign_values(2, 2)
    b.assign_values(3, 3)
    b.assign_values(4, 4)

    print(b.values())
    print(b.as_dict())

    b = BaseSymbols("2", "T", coordinates=False)
    print(b.symbols())
    b.assign_values([2, 4, 6])

    print(b.values())
    print(b.as_dict())


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
    print(c.state_vec())

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
