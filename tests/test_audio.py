from __future__ import annotations

from pathlib import Path

import numpy as np

from pianobot import audio
from pianobot.scores import MusicNote


def test_sample_path_for_note_uses_expected_steinway_naming(tmp_path: Path) -> None:
    project_root = tmp_path
    path = audio.sample_path_for_note("C#_2", project_root)
    assert path == project_root / "piano-sounds" / "Steinway_Grand" / "cs3.mp3.wav"


def test_mix_scheduled_score_accumulates_stereo_channels(monkeypatch) -> None:
    sequence = [MusicNote("C", key_scale=1, t_scale=0.2)]
    mix = np.zeros((1000, 2), dtype=np.float64)

    def fake_load_piano_sample(*args, **kwargs):
        return np.ones(40, dtype=np.float64)

    monkeypatch.setattr(audio, "load_piano_sample", fake_load_piano_sample)

    end_time, note_count = audio.mix_scheduled_score(
        sequence,
        audio.StereoGains(left_gain=1.0, right_gain=0.5, voice_gain=0.25),
        mix,
        sample_rate=100,
    )

    assert note_count == 1
    assert end_time > 0
    assert np.max(mix[:, 0]) > np.max(mix[:, 1])
    assert np.max(mix[:, 1]) > 0
