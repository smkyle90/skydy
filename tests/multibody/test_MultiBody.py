"""Test the example function
"""

import matplotlib.pyplot as plt
import pytest
from mpl_toolkits import mplot3d


def print_result(body):
    print(f"==={body.name.upper()}===")
    print(f"{body.name} coordinates:", body.coordinates)
    print(f"{body.name} bodies:", body.bodies)
    print(f"{body.name} forces")
    for p, f, _ in body.forces:
        print(f"{body.name} Pos:", p)
        print(f"{body.name} Dir:", f)
    print(f"{body.name} torques")
    for _, t, _ in body.torques:
        print(t)

    print(f"{body.name} kinetic_energy:", body.kinetic_energy)
    print(f"{body.name} potential_energy:", body.potential_energy)
    print(f"{body.name} Symbols:", body.symbols())
    LHS, RHS = body.eoms()
    print(f"{body.name} LHS EOM:", LHS)
    print(f"{body.name} RHS EOM:", RHS)
    A, B = body.system_matrices()
    print(f"{body.name} Nonlinear A:", A)
    print(f"{body.name} Nonlinear B:", B)

    A, B = body.system_matrices(True)
    print(f"{body.name} Linear A:", A)
    print(f"{body.name} Linear B:", B)

    q0, f0 = body.get_equilibria()
    print(f"{body.name} coordinate eum:", q0)
    print(f"{body.name} force eum:", f0)
    print(f"{body.name} config:", body.get_configuration())


@pytest.mark.multibody
def test_MultiBody(
    cart,
    pendulum,
    hovercraft,
    cart_pendulum,
    double_pendulum,
):
    from skydy.multibody import MultiBody

    # Empty multibody
    m1 = MultiBody()

    # Empty list
    m2 = MultiBody([])

    print_result(cart)
    print_result(pendulum)
    print_result(hovercraft)
    print_result(cart_pendulum)
    print_result(double_pendulum)

    cart.as_latex()
    pendulum.as_latex()
    hovercraft.as_latex()
    cart_pendulum.as_latex()
    double_pendulum.as_latex()


@pytest.mark.skip
def test_MultiBody_draw(
    cart,
    pendulum,
    hovercraft,
    cart_pendulum,
    double_pendulum,
):
    cart.draw()
    pendulum.draw()
    hovercraft.draw()
    cart_pendulum.draw()
    double_pendulum.draw()


@pytest.mark.multibody
def test_MultiBody_bad_types():
    from skydy.connectors import DOF, Connection, Joint
    from skydy.multibody import MultiBody
    from skydy.rigidbody import (
        Body,
        BodyCoordinate,
        BodyForce,
        BodyTorque,
        Ground,
        GroundCoordinate,
    )

    # Test that fact we cannot have duplicate names for MultiBody objects
    # Empty list
    mb = MultiBody([], "duplicate")

    with pytest.raises(ValueError):
        mb_bad = MultiBody([], "duplicate")

    # Test that a Body if floating in space.
    # We will have a body connected to ground,
    # and then two other bodies added to the multibody
    p0 = GroundCoordinate()
    p1 = BodyCoordinate("p1", 0, 0, 0)

    b0 = Ground()
    b1 = Body()
    j1 = Joint(p0, p1, [])

    b2 = Body()
    b3 = Body()
    j2 = Joint(p1, p1, [])

    cnx_1 = Connection(b0, j1, b1)
    cnx_2 = Connection(b2, j2, b3)

    with pytest.raises(ValueError):
        m = MultiBody([cnx_1, cnx_2])

    # The first connection must be to the ground
    with pytest.raises(TypeError):
        m = MultiBody([cnx_2])

    # incorrect type in the connectiosn list
    with pytest.raises(TypeError):
        m = MultiBody([cnx_1, "not a connection"])

    # Duplicate values
    with pytest.raises(ValueError):
        m = MultiBody([cnx_1, cnx_2, cnx_2])


@pytest.mark.multibody
def test_MultiBody_controllable(cart, cart_pendulum, hovercraft, pendulum):
    print("c linear", cart.controllable())
    print("c nonlinear", cart.controllable(False))
    print("pen linear", pendulum.controllable())
    print("pen nonlinear", pendulum.controllable(False))
    print("cp linear", cart_pendulum.controllable())
    print("cp nonlinear", cart_pendulum.controllable(False))
    print("hc linear", hovercraft.controllable())
    print("hc nonlinear", hovercraft.controllable(False))


@pytest.mark.dev
def test_MultiBody_poles(cart, cart_pendulum, hovercraft, pendulum):
    print("c linear", cart.poles())
    print("pen linear", pendulum.poles())
    print("cp linear", cart_pendulum.poles())
    print("hc linear", hovercraft.poles())
