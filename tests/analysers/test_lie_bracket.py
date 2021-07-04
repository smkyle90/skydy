"""Test the Lie bracket function
"""

import pytest
import sympy as sym


@pytest.mark.analysers
def test_lie_bracket():
    """
    https://www.maplesoft.com/support/help/maple/view.aspx?path=DifferentialGeometry%2FLessonsAndTutorials%2FDifferentialGeometry%2FLieBracket
    """
    from skydy.analysers import lie_bracket

    # Defines some symbols
    x, y, z = sym.symbols("x y z")
    q = [x, y, z]
    # Define three vector fields, f and g
    f = sym.Matrix([x, y, 0])
    g = sym.Matrix([0, 0, x * y])

    expected_result = -sym.Matrix([0, 0, 2 * x * y])
    actual_result = lie_bracket(f, g, q)

    assert actual_result == expected_result

    # Check identities
    assert lie_bracket(f, g, q) == -lie_bracket(g, f, q)

    # Jacobi identity
    h = sym.Matrix([x, y, z])
    gh = lie_bracket(g, h, q)
    fg = lie_bracket(f, g, q)
    hf = lie_bracket(h, f, q)
    expected_result = sym.zeros(*f.shape)
    actual_result = (
        lie_bracket(f, gh, q) + lie_bracket(h, fg, q) + lie_bracket(g, hf, q)
    )
    assert actual_result == expected_result
