#!/usr/bin/env python3
"""A Python file that references no packages at all.

Every `pkg:` below is Python syntax or data - an annotation, a dict key, a string. None of
them is a YAML mapping key, and reading them as one produced packages named `str` and `robot`.
"""

from typing import Any


def resolve(pkg: str, timeout: float, options: Any = None) -> str:
    """`pkg: str` is a type annotation."""
    return pkg


LAUNCH_SPEC = {"launch_file": "pkg:robot.launch.yaml"}

ANOTHER = "package: not_a_reference_either"
