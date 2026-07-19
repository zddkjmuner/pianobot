# Robo Piano

## Project layout

```text
.
├── pyproject.toml          # Package metadata and dependencies
├── README.md
├── report.pdf              # Project report
├── piano-sounds/           # Git submodule (sample library; do not modify)
├── src/
│   └── pianobot/           # Installable Python package
│       ├── __init__.py
│       ├── music.py        # Chord libraries and scores
│       ├── models.py       # Keyboard key model
│       ├── controller.py   # Torque controller LeafSystem
│       ├── sound.py        # Pygame piano sound LeafSystem
│       ├── project.py      # Piano_Project orchestration
│       ├── visualization.py
│       ├── logging_utils.py
│       └── paths.py
└── log/                    # Optional simulation logs (not shipped; ~10GB)
```

## Dependencies

Declared in `pyproject.toml`:

| Package | Role |
| --- | --- |
| `numpy` | Numerics |
| `pandas` | Trajectory tables |
| `pygame` | Piano sample playback |
| `meshcat` | Browser visualization |
| `pydot` | Diagram / kinematic-tree SVG export |
| `ipython` | SVG / display helpers |

**Course / simulation stack** (may be outdated relative to current Drake releases; install as required by the Fall 2021 Manipulation materials):

- [Drake](https://drake.mit.edu/) (`pydrake`)
- Course `manipulation` package (`manipulation.*`, including Meshcat helpers and scenarios)
- `piano-sounds` submodule for audio samples

> **Note:** Some of these packages are pinned to the course-era toolchain. This project documents how to install and run them; it does not attempt to upgrade or patch outdated APIs.

## Installation

1. Clone the repository and initialize the sound submodule:

```bash
git clone https://github.com/seongho-yeon/playing-piano-with-a-robotic-hand.git
cd playing-piano-with-a-robotic-hand
git submodule update --init --recursive
```

2. Create a Python environment that already provides Drake and the course `manipulation` package (see the [Manipulation course](https://manipulation.csail.mit.edu/Fall2021/) setup).

3. Install this project in editable mode:

```bash
pip install -e .
```

Or install the declared PyPI dependencies only:

```bash
pip install -e . --no-deps
pip install numpy pandas pygame meshcat pydot ipython
```

## Usage

Use the `pianobot` Python package on a machine with audio output (for sound playback) and a browser (for Meshcat).

Typical workflow:

1. Start Meshcat via `start_visualizer()`.
2. Build trajectories with `Piano_Project().music_sequence_to_trajectory(...)`.
3. Run static or dynamic simulation demos.
4. Optionally reconstruct sound from saved logs under `log/`.

```python
from pianobot import (
    Piano_Project,
    generate_salute_de_amur_lowchord_5actave,
    start_visualizer,
)

start_visualizer()
project = Piano_Project()
chords = generate_salute_de_amur_lowchord_5actave()
trajectories = project.music_sequence_to_trajectory(chords)

# Optional demos (require Drake / manipulation setup):
# project.forward_kinematics_demo()
# project.inverse_kinematics_demo()
# log = project.run_simulation_demo()
```

To rebuild trajectories from a saved simulation log:

```python
from pianobot import load_log, reconstruct_logdata_to_trajectory

t = load_log("log/log_salute_damur_chords_t.npy")
x = load_log("log/log_salute_damur_chords_data.npy")
q_traj, dq_traj = reconstruct_logdata_to_trajectory(t, x)
```

## Simulation logs

Pre-computed simulation logs may be available upon request. Because of their size (~10 GB), they are not included in the repository. When present, place them under `log/` (for example `log/log_salute_damur_chords_t.npy`).

## Package overview

| Module | Contents |
| --- | --- |
| `pianobot.music` | Chord dictionaries and score generators (e.g. *Salut d'Amour*, Twinkle) |
| `pianobot.project` | `Piano_Project` — plant construction, IK, simulation |
| `pianobot.controller` | `MyControllerSystem` torque controller |
| `pianobot.sound` | `PianoOutputSystem` using samples from `piano-sounds/` |
| `pianobot.visualization` | Meshcat startup helpers |
| `pianobot.logging_utils` | Save / load / reconstruct Drake logs |

## License / attribution

Piano samples under `piano-sounds/` come from the [piano-sounds](https://github.com/seongho-yeon/piano-sounds) submodule (forked from [ledlamp/piano-sounds](https://github.com/ledlamp/piano-sounds)). Leave that directory unchanged when working on this package.
