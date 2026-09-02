# Stamped by `cz bump` (see [tool.commitizen] in pyproject.toml). Keep the
# `__version__ = "..."` line at column 0 so the version_files regex matches.
__version__ = "1.74.7"


def get_versions():
    """Compatibility shim for the former versioneer interface."""
    return {"version": __version__}
