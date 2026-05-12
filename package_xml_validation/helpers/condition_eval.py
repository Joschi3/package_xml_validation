"""REP-149 ``condition="…"`` attribute evaluation.

Thin wrapper around :func:`catkin_pkg.condition.evaluate_condition`. The
condition mini-language is small (env-var lookups, equality, boolean
ops) but parsing it correctly is non-trivial — defer to ``catkin_pkg``,
which is the canonical implementation used by the wider ROS toolchain.
"""

from __future__ import annotations

import logging
import os
from collections.abc import Mapping

from catkin_pkg.condition import evaluate_condition as _evaluate

# Per-process set of expressions we've already warned about, so a malformed
# `condition` string in a package shared across many entries doesn't spam
# the log on every iteration.
_WARNED_EXPRESSIONS: set[str] = set()


def evaluate_condition(
    condition: str | None,
    context: Mapping[str, str] | None = None,
    logger: logging.Logger | None = None,
) -> bool:
    """Evaluate a ``condition="…"`` attribute value.

    Empty/missing condition: always True (entry applies unconditionally).
    Malformed condition: True + one warning per unique expression. We
    treat unparsable as "applies" rather than "skip" to avoid silently
    dropping a dependency that the user wrote and meant to assert.

    Args:
        condition: Raw attribute value, or ``None``.
        context: Variable bindings. Defaults to ``os.environ``.
        logger: Optional logger for malformed-expression warnings.

    Returns:
        Whether the entry should be considered "active" for validation.
    """
    if condition is None or not condition.strip():
        return True
    ctx = dict(context) if context is not None else dict(os.environ)
    try:
        return bool(_evaluate(condition, ctx))
    except ValueError as exc:
        if logger is not None and condition not in _WARNED_EXPRESSIONS:
            logger.warning(
                f"Could not parse condition='{condition}': {exc}. "
                "Treating dependency as applicable."
            )
            _WARNED_EXPRESSIONS.add(condition)
        return True
