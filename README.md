# Playing Piano with a Robotic Hand

Generate a synchronized 45-second performance of Elgar's *Salut d'Amour*: one
KUKA iiwa + Allegro hand plays the chord part, a second robot plays the melody,
and both views are rendered side by side with a stereo Steinway piano mix.

This repository is a Python package managed by
[`uv`](https://docs.astral.sh/uv/). The original notebook workflow has been
replaced by a reproducible command-line pipeline.

## Output

The default command creates these files under `build/`:

- `salute_chords_smooth.html`: interactive Meshcat recording of the chord part.
- `salute_melody_smooth.html`: interactive Meshcat recording of the melody.
- `salute_damour_complete_45s.wav`: 48 kHz stereo piano mix.
- `salute_damour_two_robots_45s.mp4`: 1920 x 540 H.264/AAC final video.
- `plan_cache.npz`: sampled trajectories used by later stages.
- `plan_cache.json`: output summary and event counts.

Intermediate PNG frames are removed after encoding unless `--keep-frames` is
passed.

## Requirements

- macOS on Apple Silicon.
- [`uv`](https://docs.astral.sh/uv/getting-started/installation/).
- Git, including submodule support.
- Approximately 3 GB of free disk space while rendering.

Python, Drake, Chromium, and ffmpeg do not need to be installed separately.
`uv` installs Python 3.11 and the locked Python dependencies. Playwright installs
Chromium, and `imageio-ffmpeg` supplies the ffmpeg executable used for encoding.

## Setup

```bash
uv sync
uv run playwright install chromium
```

The `piano-sounds/` submodule is an input asset library and must remain at the
repository root. The production mix uses the WAV samples in
`piano-sounds/Steinway_Grand/`.

## Generate The Video

Run the complete pipeline:

```bash
uv run pianobot --stage all --output-dir build --fps 15 --duration 45
```

The full run performs inverse kinematics for both parts, records two Meshcat
animations, mixes the score, renders 1,350 browser frames, and encodes the final
video. Runtime depends on CPU and disk speed.

Open the result on macOS:

```bash
open build/salute_damour_two_robots_45s.mp4
```

Use `--force` to replace existing outputs. Add `--keep-frames` when debugging
the browser rendering stage.

## Run Individual Stages

The pipeline can be resumed without repeating completed work:

```bash
uv run pianobot --stage plan  --output-dir build
uv run pianobot --stage html  --output-dir build
uv run pianobot --stage audio --output-dir build --duration 45
uv run pianobot --stage video --output-dir build --fps 15 --duration 45
```

Stage dependencies are:

1. `plan` creates `plan_cache.npz`.
2. `html` reads the plan cache and creates both Meshcat recordings.
3. `audio` is independent of Drake planning and reads the Steinway samples.
4. `video` reads both HTML files and the WAV file.
5. `all` runs the four stages in order.

## Architecture

```text
src/pianobot/
├── scores.py    # Chord dictionary and both Salut d'Amour parts
├── robot.py     # Piano model, iiwa/Allegro model, IK, and trajectory recording
├── audio.py     # Score scheduling, sample loading, and stereo WAV mixing
├── render.py    # Exact Meshcat seeking, Playwright capture, and ffmpeg encoding
├── pipeline.py  # Resumable production stages and artifact paths
└── cli.py       # `pianobot` command
```

The final animation uses planned kinematic trajectories. Full-song multi-finger
contact dynamics were intentionally removed from the production path because
contact impulses could produce visible numerical shaking. The short-contact
experiments were useful for diagnosis, but deterministic trajectory playback is
the appropriate source for a stable final render.

Audio scheduling mirrors the trajectory generator's transition, press, hold,
and release timing. The chord voice is weighted left and the melody voice right.

## Development

Run the automated checks:

```bash
uv run pytest -q
uv run ruff check src tests
uv run python -m compileall -q src tests
```

The tests verify score/event counts, score timing, Steinway filename mapping,
and stereo mixing without running the expensive IK pipeline.

## Troubleshooting

**`piano-sounds/Steinway_Grand` is missing**


**Playwright cannot find Chromium**

```bash
uv run playwright install chromium
```

