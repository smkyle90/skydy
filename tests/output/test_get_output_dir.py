"""Test the output functions
"""

import os
import shutil

import pytest


@pytest.mark.skip
def test_get_output_dir():

    from skydy.output import get_output_dir
    from skydy.output.get_output_dir import DIR_NAME

    if os.path.exists(DIR_NAME):
        shutil.rmtree(DIR_NAME)

    output_dir = get_output_dir()

    output_dir = get_output_dir("./workspace")

    if os.path.exists(DIR_NAME):
        shutil.rmtree(DIR_NAME)
