#!/usr/bin/python3


def lie_bracket(f, g, q):
    """Take the Lie bracket of two vector fields.

    [f, g] = (d/dq)f * g - (d/dq)g * f

    Args:
        f (sympy.matrix): an N x 1 symbolic vector
        g (sympy.matrix): an N x 1 symbolic
        q (sympy.matrix or List): a length N array like object of coordinates to take partial derivates.

    Returns:
        [f, g] (sympy.matrix): the Lie bracket of f and g, an N x 1 symbolic vector

    """

    assert f.shape == g.shape, "The f and g vectors must be the same dimension."
    assert len(f) == len(q), "The vector field must represent all the coordinates."
    return f.jacobian(q) @ g - g.jacobian(q) @ f
