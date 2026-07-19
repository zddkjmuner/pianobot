from __future__ import annotations

import argparse
from pathlib import Path

from .pipeline import (
    stage_all,
    stage_audio,
    stage_html,
    stage_plan,
    stage_video,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pianobot production pipeline")
    parser.add_argument(
        "--stage",
        choices=["plan", "html", "audio", "video", "all"],
        default="all",
        help="Pipeline stage to run",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("build"),
        help="Output directory for generated artifacts",
    )
    parser.add_argument("--fps", type=int, default=15, help="Frame rate for rendered video")
    parser.add_argument("--duration", type=float, default=45.0, help="Requested output duration in seconds")
    parser.add_argument("--force", action="store_true", help="Regenerate artifacts even if they already exist")
    parser.add_argument(
        "--keep-frames",
        action="store_true",
        help="Keep intermediate PNG frames after video encoding",
    )
    return parser


def main() -> None:
    args = build_parser().parse_args()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    if args.stage == "plan":
        print("[plan] generating trajectories...")
        stage_plan(output_dir, fps=args.fps)
        print("[plan] completed")
        return

    if args.stage == "html":
        print("[html] generating static Meshcat HTML recordings...")
        low_html, high_html = stage_html(output_dir, duration=args.duration, force=args.force)
        print(f"[html] saved low view: {low_html}")
        print(f"[html] saved high view: {high_html}")
        return

    if args.stage == "audio":
        print(f"[audio] synthesizing {args.duration:g}-second stereo Steinway WAV...")
        audio_wav, metrics = stage_audio(output_dir, duration=args.duration, force=args.force)
        print(f"[audio] saved: {audio_wav}")
        if metrics:
            print(
                "[audio]",
                f"chord_notes={int(metrics['chord_note_count'])}",
                f"melody_notes={int(metrics['melody_note_count'])}",
            )
        return

    if args.stage == "video":
        print("[video] rendering frame sequences and composing MP4...")
        video_mp4 = stage_video(
            output_dir,
            fps=args.fps,
            duration=args.duration,
            force=args.force,
            keep_frames=args.keep_frames,
        )
        print(f"[video] saved: {video_mp4}")
        return

    print("[all] running full pipeline...")
    summary = stage_all(
        output_dir,
        fps=args.fps,
        duration=args.duration,
        force=args.force,
        keep_frames=args.keep_frames,
    )
    print("[all] completed")
    for key, value in summary.items():
        print(f"  - {key}: {value}")


if __name__ == "__main__":
    main()
