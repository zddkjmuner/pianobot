"""Meshcat visualizer helpers."""

from __future__ import annotations

from typing import Any, Optional

import pandas as pd

_meshcat: Any = None
_zmq_proc = None


def start_visualizer(ngrok_http_tunnel: bool = True):
    """Start the Meshcat ZMQ server and return the Meshcat instance.

    This mirrors the original notebook startup sequence. Call once before
    constructing :class:`~pianobot.project.Piano_Project` instances that need
    visualization.
    """
    global _meshcat, _zmq_proc

    from meshcat.servers.zmqserver import start_zmq_server_as_subprocess
    from manipulation.meshcat_cpp_utils import StartMeshcat

    server_args = ["--ngrok_http_tunnel"] if ngrok_http_tunnel else None
    if server_args is not None:
        _zmq_proc, zmq_url, web_url = start_zmq_server_as_subprocess(
            server_args=server_args
        )
    else:
        _zmq_proc, zmq_url, web_url = start_zmq_server_as_subprocess()

    _meshcat = StartMeshcat()
    return _meshcat


def get_meshcat():
    """Return the active Meshcat instance, starting one if needed."""
    global _meshcat
    if _meshcat is None:
        return start_visualizer()
    return _meshcat


def set_meshcat(meshcat) -> None:
    """Inject an existing Meshcat instance (e.g. from a notebook)."""
    global _meshcat
    _meshcat = meshcat


def dataframe(trajectory, times, names):
    """Convert a Drake trajectory into a pandas DataFrame."""
    assert trajectory.rows() == len(names)
    values = trajectory.vector_values(times)
    data = {"t": times}
    for i in range(len(names)):
        data[names[i]] = values[i, :]
    return pd.DataFrame(data)
