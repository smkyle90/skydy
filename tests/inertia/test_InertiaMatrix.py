"""Test the example function
"""

import numpy as np
import pytest
import sympy as sym


@pytest.mark.inertia
def test_InertiaMatrix():
    from skydy.inertia import InertiaMatrix

    mass_name = "1"

    I = InertiaMatrix(mass_name)

    print(I.as_mat())
