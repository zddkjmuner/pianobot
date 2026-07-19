from __future__ import annotations

import wave
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .scores import MusicNote, generate_note_from_chord

DEFAULT_SAMPLE_RATE = 48_000
DEFAULT_DURATION_SECONDS = 45.0


@dataclass(slots=True)
class StereoGains:
    left_gain: float
    right_gain: float
    voice_gain: float


def resolve_project_root(project_root: Path | None = None) -> Path:
    if project_root is not None:
        return project_root.resolve()
    here = Path(__file__).resolve()
    for parent in [here, *here.parents]:
        if (parent / "pyproject.toml").exists():
            return parent
    raise FileNotFoundError("Could not resolve project root (missing pyproject.toml)")


def sample_path_for_note(note_name: str, project_root: Path | None = None) -> Path:
    root = resolve_project_root(project_root)
    sample_name = note_name.replace("_", "").replace("#", "s").lower()
    octave = int(sample_name[-1]) + 1
    return root / "piano-sounds" / "Steinway_Grand" / f"{sample_name[:-1]}{octave}.mp3.wav"


def load_piano_sample(
    note_name: str,
    *,
    sample_rate: int = DEFAULT_SAMPLE_RATE,
    project_root: Path | None = None,
    cache: dict[str, np.ndarray] | None = None,
) -> np.ndarray:
    if cache is not None and note_name in cache:
        return cache[note_name]

    path = sample_path_for_note(note_name, project_root)
    with wave.open(str(path), "rb") as wav_file:
        if wav_file.getframerate() != sample_rate:
            raise ValueError(f"Unexpected sample rate for {path}: {wav_file.getframerate()}")
        if wav_file.getsampwidth() != 2:
            raise ValueError(f"Unexpected sample width for {path}: {wav_file.getsampwidth()}")
        channels = wav_file.getnchannels()
        samples = np.frombuffer(wav_file.readframes(wav_file.getnframes()), dtype="<i2")
    waveform = samples.reshape(-1, channels).astype(np.float64) / 32768.0
    mono = waveform.mean(axis=1)

    if cache is not None:
        cache[note_name] = mono
    return mono


def score_onsets(sequence: list[MusicNote]) -> tuple[list[tuple[float, MusicNote]], float]:
    # Preserve notebook timing semantics used by music_sequence_to_trajectory.
    current_time = 0.1
    transition_time = 3.0
    scheduled: list[tuple[float, MusicNote]] = []
    for event in sequence:
        if event.name == "none":
            transition_time += event.t_push + event.t_hold + event.t_release + event.t_transition
            continue
        current_time += transition_time
        current_time += event.t_push
        scheduled.append((current_time, event))
        current_time += event.t_hold + event.t_release
        transition_time = event.t_transition
    return scheduled, current_time + 1.0


def _apply_envelope(note_audio: np.ndarray, sample_rate: int) -> None:
    attack = min(int(0.008 * sample_rate), note_audio.size)
    release = min(int(0.35 * sample_rate), note_audio.size)
    if attack:
        note_audio[:attack] *= np.linspace(0.0, 1.0, attack, endpoint=True)
    if release:
        note_audio[-release:] *= np.linspace(1.0, 0.0, release, endpoint=True)


def mix_scheduled_score(
    sequence: list[MusicNote],
    gains: StereoGains,
    audio_mix: np.ndarray,
    *,
    sample_rate: int = DEFAULT_SAMPLE_RATE,
    project_root: Path | None = None,
    cache: dict[str, np.ndarray] | None = None,
) -> tuple[float, int]:
    scheduled, score_end = score_onsets(sequence)
    note_count = 0
    max_frames = audio_mix.shape[0]
    for onset, event in scheduled:
        for note_name in generate_note_from_chord(event.name, event.key_scale):
            sample = load_piano_sample(
                note_name,
                sample_rate=sample_rate,
                project_root=project_root,
                cache=cache,
            )
            start = int(round(onset * sample_rate))
            stop = min(max_frames, start + sample.size)
            if stop <= start:
                continue
            note_audio = sample[: stop - start].copy()
            _apply_envelope(note_audio, sample_rate)
            audio_mix[start:stop, 0] += note_audio * gains.voice_gain * gains.left_gain
            audio_mix[start:stop, 1] += note_audio * gains.voice_gain * gains.right_gain
            note_count += 1
    return score_end, note_count


def build_salut_damour_audio(
    low_score: list[MusicNote],
    high_score: list[MusicNote],
    output_wav: Path,
    *,
    duration_seconds: float = DEFAULT_DURATION_SECONDS,
    sample_rate: int = DEFAULT_SAMPLE_RATE,
    project_root: Path | None = None,
) -> tuple[Path, dict[str, float]]:
    output_wav = output_wav.resolve()
    output_wav.parent.mkdir(parents=True, exist_ok=True)

    audio_frames = int(sample_rate * duration_seconds)
    audio_mix = np.zeros((audio_frames, 2), dtype=np.float64)
    cache: dict[str, np.ndarray] = {}

    chord_score_end, chord_note_count = mix_scheduled_score(
        low_score,
        StereoGains(left_gain=1.0, right_gain=0.38, voice_gain=0.24),
        audio_mix,
        sample_rate=sample_rate,
        project_root=project_root,
        cache=cache,
    )
    melody_score_end, melody_note_count = mix_scheduled_score(
        high_score,
        StereoGains(left_gain=0.38, right_gain=1.0, voice_gain=0.30),
        audio_mix,
        sample_rate=sample_rate,
        project_root=project_root,
        cache=cache,
    )

    audio_mix = np.tanh(audio_mix * 1.15)
    peak = float(np.max(np.abs(audio_mix)))
    if peak > 0:
        audio_mix *= 0.95 / peak

    audio_pcm = np.round(audio_mix * 32767.0).astype("<i2")
    with wave.open(str(output_wav), "wb") as output_handle:
        output_handle.setnchannels(2)
        output_handle.setsampwidth(2)
        output_handle.setframerate(sample_rate)
        output_handle.writeframes(audio_pcm.tobytes())

    metrics = {
        "duration_seconds": duration_seconds,
        "chord_score_end": chord_score_end,
        "melody_score_end": melody_score_end,
        "chord_note_count": float(chord_note_count),
        "melody_note_count": float(melody_note_count),
    }
    return output_wav, metrics
