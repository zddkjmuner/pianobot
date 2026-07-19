from __future__ import annotations

import subprocess
from dataclasses import dataclass
from pathlib import Path

import imageio_ffmpeg
from playwright.sync_api import sync_playwright


@dataclass(frozen=True, slots=True)
class CameraPreset:
    position: tuple[float, float, float]
    target: tuple[float, float, float]


# Mirrored close views validated against the generated Meshcat scene.
CAMERA_LEFT = CameraPreset(position=(-1.1, 0.72, 0.55), target=(-0.2, 0.22, -0.65))
CAMERA_RIGHT = CameraPreset(position=(0.7, 0.72, 0.55), target=(-0.2, 0.22, -0.65))


def _prepare_meshcat(page, preset: CameraPreset) -> float:
        return page.evaluate(
        """
        ({ position, target }) => {
          const v = globalThis.viewer;
                    if (!v || !v.camera || !v.controls || !v.animator) {
                        throw new Error("Meshcat viewer did not initialize");
                    }
                    document.documentElement.style.cssText =
                        "width:960px;height:540px;overflow:hidden";
                    document.body.style.cssText =
                        "margin:0;width:960px;height:540px;overflow:hidden";
                    const pane = document.querySelector("#meshcat-pane");
                    pane.style.cssText =
                        "position:absolute;left:0;top:0;width:960px;height:540px;overflow:hidden";
                    for (const selector of [".dg.main", "#status-message", "#stats-plot"]) {
                        document.querySelectorAll(selector).forEach((element) => {
                            element.style.display = "none";
                        });
                    }
                    v.renderer.setSize(960, 540, false);
                    v.camera.aspect = 16 / 9;
                    v.camera.updateProjectionMatrix();
          v.camera.position.set(position[0], position[1], position[2]);
          v.controls.target.set(target[0], target[1], target[2]);
          v.controls.update();
                    v.animator.pause();
                    v.set_dirty();
                    v.render();
                    return v.animator.duration;
        }
        """,
        {"position": list(preset.position), "target": list(preset.target)},
    )


def render_meshcat_html_to_frames(
    html_path: Path,
    output_dir: Path,
    *,
    fps: int,
    duration: float,
    camera: CameraPreset,
    viewport: tuple[int, int] = (960, 540),
) -> list[Path]:
    html_path = html_path.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    for stale_frame in output_dir.glob("frame_*.png"):
        stale_frame.unlink()
    total_frames = int(round(fps * duration))
    rendered: list[Path] = []

    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(headless=True)
        page = browser.new_page(
            viewport={"width": viewport[0], "height": viewport[1]},
            device_scale_factor=1,
        )
        page.goto(html_path.as_uri(), wait_until="networkidle")
        page.wait_for_timeout(800)
        recording_duration = _prepare_meshcat(page, camera)
        page.wait_for_timeout(200)

        for index in range(total_frames):
            timestamp = min(index / float(fps), recording_duration)
            page.evaluate(
                """
                (timestamp) => {
                  const v = globalThis.viewer;
                  v.animator.seek(timestamp);
                  v.set_dirty();
                  v.render();
                }
                """,
                timestamp,
            )
            output_path = output_dir / f"frame_{index:04d}.png"
            page.screenshot(
                path=str(output_path),
                clip={"x": 0, "y": 0, "width": viewport[0], "height": viewport[1]},
            )
            rendered.append(output_path)

        browser.close()

    return rendered


def compose_side_by_side_video(
    left_frames_dir: Path,
    right_frames_dir: Path,
    audio_path: Path,
    output_path: Path,
    *,
    fps: int,
    duration: float,
) -> Path:
    output_path = output_path.resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)

    ffmpeg_exe = imageio_ffmpeg.get_ffmpeg_exe()
    command = [
        ffmpeg_exe,
        "-y",
        "-framerate",
        str(fps),
        "-start_number",
        "0",
        "-i",
        str((left_frames_dir / "frame_%04d.png").resolve()),
        "-framerate",
        str(fps),
        "-start_number",
        "0",
        "-i",
        str((right_frames_dir / "frame_%04d.png").resolve()),
        "-i",
        str(audio_path.resolve()),
        "-filter_complex",
        "[0:v]scale=960:540:flags=lanczos[left];"
        "[1:v]scale=960:540:flags=lanczos[right];"
        "[left][right]hstack=inputs=2[v]",
        "-map",
        "[v]",
        "-map",
        "2:a",
        "-c:v",
        "libx264",
        "-preset",
        "medium",
        "-crf",
        "18",
        "-pix_fmt",
        "yuv420p",
        "-c:a",
        "aac",
        "-b:a",
        "256k",
        "-ar",
        "48000",
        "-t",
        str(duration),
        "-movflags",
        "+faststart",
        str(output_path),
    ]
    subprocess.run(command, check=True)
    return output_path
