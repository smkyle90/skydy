"""Test the example function
"""

import numpy as np
import pytest
import sympy as sym


@pytest.mark.inertia
def test_MassMatrix():
    from lib.inertia import MassMatrix

    mass_name = "1"

    M = MassMatrix(mass_name)

    M.as_mat()
