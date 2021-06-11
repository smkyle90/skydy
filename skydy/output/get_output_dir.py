#!/usr/bin/python3
import os

DIR_NAME = "./out"


def get_output_dir(output_dir=None):
    """Get the output directory, which is compatible
    with current directory.
    """
    if output_dir is None:
        if not os.path.exists(DIR_NAME):
            os.mkdir(DIR_NAME)

        output_dir = DIR_NAME

    return output_dir
