"""Test the example function
"""

import pytest


@pytest.mark.configuration
def test_BaseSymbols():
    from skydy.configuration import BaseSymbols

    b = BaseSymbols("1", "G", coordinates=True)

    print(b.symbols())

    b = BaseSymbols("2", "T", coordinates=False)
    print(b.symbols())


@pytest.mark.configuration
def test_BodyCoordinate():
    from skydy.configuration import BodyCoordinate

    body = BodyCoordinate("1")
    print(body.symbols())
    print(body.velocities())
    print(body.positions())

    body = BodyCoordinate("2")
    print(body.symbols())
    print(body.velocities())
    print(body.positions())

    body = BodyCoordinate("rod")
    print(body.symbols())
    print(body.velocities())
    print(body.positions())


@pytest.mark.configuration
def test_BodyDimension():
    from skydy.configuration import BodyDimension

    dir_vec = BodyDimension("1")
    print(dir_vec.symbols())

    dir_vec = BodyDimension("2")
    print(dir_vec.symbols())

    dir_vec = BodyDimension("rod")
    print(dir_vec.symbols())


@pytest.mark.configuration
def test_BodyForce():
    from skydy.configuration import BodyForce

    body_force = BodyForce("1")
    print(body_force.symbols())

    body_force = BodyForce("2")
    print(body_force.symbols())

    body_force = BodyForce("rod")
    print(body_force.symbols())


@pytest.mark.configuration
def test_BodyTorque():
    from skydy.configuration import BodyTorque

    body_torque = BodyTorque("1")
    print(body_torque.symbols())

    body_torque = BodyTorque("2")
    print(body_torque.symbols())

    body_torque = BodyTorque("rod")
    print(body_torque.symbols())


@pytest.mark.configuration
def test_Configuration():
    from skydy.configuration import Configuration

    c = Configuration("1")

    print(c.positions())
    print(c.velocities())
    print(c.accelerations())

    print(c.r_body())
    print(c.R_body())
    print(c.state_vector())
