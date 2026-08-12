#!/usr/bin/env python3
"""A Python file that references no packages at all.

Every `pkg` and `package` below is Python syntax or data - an annotation, a dict key, a string,
an assignment. None is a YAML mapping key or a TOML entry, and each is written in the position
that only the *format* of the pattern rules out: at the start of a line, where anchoring alone
would let it through.
"""

from typing import Any

#: A module-level annotated assignment. `pkg:` starts the line, so only the fact that this file
#: is Python keeps it from reading as a YAML mapping key.
pkg: str = "hello"

#: `package = "x"` is the better_launch TOML form, and an ordinary assignment in Python.
package = "sneaky_toml_form"


def resolve(pkg: str, timeout: float, options: Any = None) -> str:
    """`pkg: str` is a type annotation."""
    return pkg


LAUNCH_SPEC = {"launch_file": "pkg:robot.launch.yaml"}

ANOTHER = "package: not_a_reference_either"
