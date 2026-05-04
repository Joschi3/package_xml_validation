"""Typed boundary around the untyped ``rosdep2`` package.

All ``rosdep2`` imports live here. Other modules in this project should import
from this wrapper instead of touching ``rosdep2`` directly. That way the
``# type: ignore`` comments stay in one place, and if ``rosdep2`` ever ships
type stubs (or gets replaced) only this file changes.

Every wrapper function imports ``rosdep2`` lazily so that:
* tests which install a fake ``rosdep2`` into ``sys.modules`` do not need to
  also reload this wrapper, and
* tests which patch ``rosdep2.create_default_installer_context`` (and
  similar) keep working unchanged - the patch is resolved through
  ``sys.modules`` at call time rather than against a name captured at import.
"""

from __future__ import annotations

from typing import TYPE_CHECKING, Any


if TYPE_CHECKING:
    # Runtime-untyped objects exposed as ``Any`` aliases so call sites have
    # meaningful names at type-check time.
    InstallerContext = Any
    RosdepView = Any
    RosdepDefinition = Any
    SourcesLoader = Any
    CachedDataSourceT = Any
    ResolutionError = Exception


def __getattr__(name: str) -> Any:
    """Lazily expose select rosdep2 attributes (e.g. ``ResolutionError``)."""
    if name == "ResolutionError":
        import rosdep2  # type: ignore[import-untyped]

        return rosdep2.ResolutionError
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def get_default_view_key() -> str:
    """Return the rospkg loader's default view key."""
    from rosdep2.rospkg_loader import DEFAULT_VIEW_KEY  # type: ignore[import-untyped]

    return DEFAULT_VIEW_KEY


def get_cached_data_source_cls() -> type:
    """Return the ``CachedDataSource`` class for use in ``isinstance`` checks."""
    from rosdep2.sources_list import CachedDataSource  # type: ignore[import-untyped]

    return CachedDataSource


def create_installer_context() -> Any:
    """Return a default rosdep installer context."""
    import rosdep2  # type: ignore[import-untyped]

    return rosdep2.create_default_installer_context()


def create_lookup_from_rospkg() -> Any:
    """Return a ``RosdepLookup`` populated from rospkg."""
    import rosdep2  # type: ignore[import-untyped]

    return rosdep2.RosdepLookup.create_from_rospkg()


def create_default_sources_loader(*, verbose: bool = False) -> Any:
    """Return a ``SourcesListLoader`` rooted at the default sources cache."""
    from rosdep2.sources_list import (  # type: ignore[import-untyped]
        SourcesListLoader,
        get_sources_cache_dir,
    )

    return SourcesListLoader.create_default(
        sources_cache_dir=get_sources_cache_dir(),
        verbose=verbose,
    )
