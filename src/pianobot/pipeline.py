from __future__ import annotations

import json
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from pydrake.all import PiecewisePolynomial

from .audio import build_salut_damour_audio, resolve_project_root, score_onsets
from .render import (
    CAMERA_LEFT,
    CAMERA_RIGHT,
    compose_side_by_side_video,
    render_meshcat_html_to_frames,
)
from .robot import PianoRobotPlanner
from .scores import (
    generate_salut_damour_high_note_score,
    generate_salut_damour_low_chord_score,
)


@dataclass(slots=True)
class PlanData:
    low_q_all: PiecewisePolynomial
    high_q_all: PiecewisePolynomial


@dataclass(slots=True)
class PipelineArtifacts:
    output_dir: Path
    low_html: Path
    high_html: Path
    audio_wav: Path
    video_mp4: Path
    cache_json: Path


def default_artifacts(output_dir: Path) -> PipelineArtifacts:
    return PipelineArtifacts(
        output_dir=output_dir,
        low_html=output_dir / "salute_chords_smooth.html",
        high_html=output_dir / "salute_melody_smooth.html",
        audio_wav=output_dir / "salute_damour_complete_45s.wav",
        video_mp4=output_dir / "salute_damour_two_robots_45s.mp4",
        cache_json=output_dir / "plan_cache.json",
    )


def _save_plan_cache(plan: PlanData, output_dir: Path, *, fps: int) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    cache_npz = output_dir / "plan_cache.npz"
    low_times = np.arange(0.0, plan.low_q_all.end_time() + 1e-9, 1.0 / float(fps))
    high_times = np.arange(0.0, plan.high_q_all.end_time() + 1e-9, 1.0 / float(fps))
    np.savez_compressed(
        cache_npz,
        low_q=plan.low_q_all.vector_values(low_times),
        high_q=plan.high_q_all.vector_values(high_times),
        low_t=low_times,
        high_t=high_times,
        low_end=np.array([plan.low_q_all.end_time()], dtype=float),
        high_end=np.array([plan.high_q_all.end_time()], dtype=float),
    )
    return cache_npz


def _load_plan_cache(output_dir: Path) -> PlanData:
    cache_npz = output_dir / "plan_cache.npz"
    if not cache_npz.exists():
        raise FileNotFoundError(f"Missing plan cache: {cache_npz}")
    data = np.load(cache_npz)
    low_q_all = PiecewisePolynomial.FirstOrderHold(data["low_t"], data["low_q"])
    high_q_all = PiecewisePolynomial.FirstOrderHold(data["high_t"], data["high_q"])
    return PlanData(low_q_all=low_q_all, high_q_all=high_q_all)


def stage_plan(output_dir: Path, *, fps: int = 15) -> PlanData:
    low_score = generate_salut_damour_low_chord_score()
    high_score = generate_salut_damour_high_note_score()

    planner_low = PianoRobotPlanner()
    _, _, low_q_all, _, _, _, _, _ = planner_low.music_sequence_to_trajectory(low_score)

    planner_high = PianoRobotPlanner()
    _, _, high_q_all, _, _, _, _, _ = planner_high.music_sequence_to_trajectory(high_score)

    plan = PlanData(low_q_all=low_q_all, high_q_all=high_q_all)
    _save_plan_cache(plan, output_dir, fps=fps)
    return plan


def stage_html(
    output_dir: Path,
    *,
    duration: float = 45.0,
    force: bool = False,
    plan: PlanData | None = None,
) -> tuple[Path, Path]:
    artifacts = default_artifacts(output_dir)
    if not force and artifacts.low_html.exists() and artifacts.high_html.exists():
        return artifacts.low_html, artifacts.high_html

    plan_data = plan if plan is not None else _load_plan_cache(output_dir)

    low_planner = PianoRobotPlanner()
    low_html = low_planner.record_static_html(plan_data.low_q_all, artifacts.low_html, duration=duration)

    high_planner = PianoRobotPlanner()
    high_html = high_planner.record_static_html(plan_data.high_q_all, artifacts.high_html, duration=duration)
    return low_html, high_html


def stage_audio(
    output_dir: Path,
    *,
    duration: float = 45.0,
    project_root: Path | None = None,
    force: bool = False,
) -> tuple[Path, dict[str, float]]:
    artifacts = default_artifacts(output_dir)
    if not force and artifacts.audio_wav.exists():
        return artifacts.audio_wav, {}

    root = resolve_project_root(project_root)
    low_score = generate_salut_damour_low_chord_score()
    high_score = generate_salut_damour_high_note_score()
    output_wav, metrics = build_salut_damour_audio(
        low_score,
        high_score,
        artifacts.audio_wav,
        duration_seconds=duration,
        project_root=root,
    )
    return output_wav, metrics


def stage_video(
    output_dir: Path,
    *,
    fps: int = 15,
    duration: float = 45.0,
    force: bool = False,
    keep_frames: bool = False,
) -> Path:
    artifacts = default_artifacts(output_dir)
    if not force and artifacts.video_mp4.exists():
        return artifacts.video_mp4

    frames_dir = output_dir / "frames"
    low_frames = frames_dir / "chords"
    high_frames = frames_dir / "melody"

    render_meshcat_html_to_frames(
        artifacts.low_html,
        low_frames,
        fps=fps,
        duration=duration,
        camera=CAMERA_LEFT,
    )
    render_meshcat_html_to_frames(
        artifacts.high_html,
        high_frames,
        fps=fps,
        duration=duration,
        camera=CAMERA_RIGHT,
    )

    output_path = compose_side_by_side_video(
        low_frames,
        high_frames,
        artifacts.audio_wav,
        artifacts.video_mp4,
        fps=fps,
        duration=duration,
    )
    if not keep_frames:
        shutil.rmtree(frames_dir)
    return output_path


def stage_all(
    output_dir: Path,
    *,
    fps: int = 15,
    duration: float = 45.0,
    project_root: Path | None = None,
    force: bool = False,
    keep_frames: bool = False,
) -> dict[str, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    artifacts = default_artifacts(output_dir)

    plan = stage_plan(output_dir, fps=fps)
    low_html, high_html = stage_html(output_dir, duration=duration, force=force, plan=plan)
    audio_wav, audio_metrics = stage_audio(
        output_dir,
        duration=duration,
        project_root=project_root,
        force=force,
    )
    video_mp4 = stage_video(
        output_dir,
        fps=fps,
        duration=duration,
        force=force,
        keep_frames=keep_frames,
    )

    low_events = len(score_onsets(generate_salut_damour_low_chord_score())[0])
    high_events = len(score_onsets(generate_salut_damour_high_note_score())[0])

    summary = {
        "low_html": str(low_html),
        "high_html": str(high_html),
        "audio_wav": str(audio_wav),
        "video_mp4": str(video_mp4),
        "low_events": low_events,
        "high_events": high_events,
        **audio_metrics,
    }
    artifacts.cache_json.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary
