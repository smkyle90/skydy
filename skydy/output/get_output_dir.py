#!/usr/bin/python3
import os

DIR_NAME = "./out"


def get_output_dir(output_dir=None):
    """Get the output directory, which is compatible
    with current directory.

    Args:
        output_dir (str or None): the desired output directory

    Returns:
        output_dir (str): the output directory, based on where function is called from.
    """
    if output_dir is None:
        if not os.path.exists(DIR_NAME):
            os.mkdir(DIR_NAME)

        output_dir = DIR_NAME

    return output_dir
