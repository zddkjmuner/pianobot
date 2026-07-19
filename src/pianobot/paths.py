"""Project path helpers."""

from pathlib import Path

def project_root() -> Path:
    """Return the repository root (parent of ``src/``)."""
    return Path(__file__).resolve().parents[2]


def piano_sounds_dir(instrument: str = "Steinway_Grand") -> Path:
    """Return the directory for a piano-sounds instrument sample set.

    The ``piano-sounds`` folder is a git submodule and must not be modified.
    """
    return project_root() / "piano-sounds" / instrument


def default_log_dir() -> Path:
    """Default directory for simulation log ``.npy`` files."""
    return project_root() / "log"
