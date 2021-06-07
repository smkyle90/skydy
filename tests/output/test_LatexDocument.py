"""Test the output functions
"""

import pytest


@pytest.mark.output
def test_LatexDocument():
    from skydy.output import LatexDocument

    l = LatexDocument()

    l.add_section("Some Section", "Some Latex!")
    l.add_section("Another Section", "Another Latex!")

    l.del_section("Some Section")
    l.del_section("Bahhh")

    l.write_tex()
    l.write_pdf()
