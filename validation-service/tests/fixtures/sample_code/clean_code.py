"""Sample clean Python code for testing.

This file contains code that should pass all quality checks.
"""


def add(a: int, b: int) -> int:
    """Add two numbers.

    Args:
        a: First number
        b: Second number

    Returns:
        Sum of a and b
    """
    return a + b


def multiply(a: int, b: int) -> int:
    """Multiply two numbers.

    Args:
        a: First number
        b: Second number

    Returns:
        Product of a and b
    """
    return a * b


class Calculator:
    """A simple calculator class."""

    def __init__(self, initial: int = 0):
        """Initialize calculator with an initial value.

        Args:
            initial: Starting value (default 0)
        """
        self.value = initial

    def add(self, n: int) -> int:
        """Add n to the current value.

        Args:
            n: Number to add

        Returns:
            New value after addition
        """
        self.value += n
        return self.value

    def subtract(self, n: int) -> int:
        """Subtract n from the current value.

        Args:
            n: Number to subtract

        Returns:
            New value after subtraction
        """
        self.value -= n
        return self.value

    def reset(self) -> None:
        """Reset calculator to zero."""
        self.value = 0
