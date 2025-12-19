"""Sample Python code with violations for testing.

This file intentionally contains code issues for testing linters.
"""

import os
import sys
import json  # noqa: F401 - unused import for testing

# Long line that exceeds 88 characters - this is a very long comment that should trigger E501
x = 1

def function_with_issues(a,b,c):
    """Function with style issues."""
    # Missing type hints, improper spacing
    result=a+b+c
    return result

def complex_function(x, y, z):
    """A function with higher complexity for testing."""
    if x > 0:
        if y > 0:
            if z > 0:
                result = x + y + z
            else:
                result = x + y - z
        else:
            if z > 0:
                result = x - y + z
            else:
                result = x - y - z
    else:
        if y > 0:
            if z > 0:
                result = -x + y + z
            else:
                result = -x + y - z
        else:
            if z > 0:
                result = -x - y + z
            else:
                result = -x - y - z
    return result


class BadClass:
    def __init__(self):
        self.x=1  # Missing spaces around operator

    def method_without_docstring(self):
        # Missing docstring
        pass
