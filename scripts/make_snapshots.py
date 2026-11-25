#!/usr/bin/env python3
r"""
make_snapshots.py  ->  figures/snapshots/scene<N>_snap_<FFFF>.png

Clean, reproducible snapshots of a SYCL scene for the paper: runs

    boidSimSYCL -scene N -snapshot F1 F2 ... -window W H -camera ex ey ez cx cy cz
                -sh MODE -kappa K -seed S -snapshot-dir OUT

which renders the scene without the overlay text and writes one PNG per frame
(frame 0 = initial state, frame F = after F fixed-dt steps of 1/60 s), then quits.
The seed and the fixed time step make every run of the same command produce the
same images.

With no --scene, it reproduces the paper figures of the scaling and the accuracy
scenes:
  scene 4 (two groups) and scene 5 (four groups), long-range avoidance with SH:
    frames 0 (start), 1700 (groups passing each other at a distance) and 2400
    (after); near top-down 1200x1000 view; Dirac projection, goal seeking with
    weight 2 (-goal 2) so the groups keep their course (without it they mill in
    place). With vMF the head-on groups do not avoid each other (they overlap at
    the centre), so these figures use Dirac.
The accuracy scenes are driven by the exact reference, as in their evaluation:
  scene 9 (plate lanes): frames 0 (start), 900 (groups at the plates) and 1500
    (groups deflected along the plates); portrait 1080x1350 view from above and in
    front of the lane stack; k = 2.95.
  scene 8 (four groups composition): frames 0 (start), 1200 (red veering toward
    orange's direction of travel) and 2000 (the other groups meeting at the
    centre); near top-down 1200x1000 view of the plane of motion; factor kernel
    k = 1.5, as in its evaluation.
For a scene without defaults, the scene's own camera preset is used unless --camera
is given, and --frames is required.

Needs: the built SYCL binary (bsh/build/boidSimSYCL), a display (the app opens a GL
window) and the oneAPI runtime (sourced from /opt/intel/oneapi/setvars.sh if
ONEAPI_ROOT is not set).

Run:  python3 make_snapshots.py                                   # all figures (scenes 4, 5, 8, 9)
      python3 make_snapshots.py --scene 9 --frames 5 15 200
      python3 make_snapshots.py --scene 7 --frames 0 600 1200 --sh vmf
      python3 make_snapshots.py --camera 120 1380 -120 590 700 550 --window 1920 1080
"""
import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SETVARS = "/opt/intel/oneapi/setvars.sh"

# paper figure settings per scene; camera = eye x y z, look-at x y z (world units)
TOP_DOWN = [600, 1250, 430, 600, 600, 590]   # near top-down on the plane of motion (1200 world)
SCENES = {
    4: dict(frames=[0, 1700, 2400], camera=TOP_DOWN, window=[1200, 1000], kappa=2.95, sh="dirac", goal=2.0),
    5: dict(frames=[0, 1700, 2400], camera=TOP_DOWN, window=[1200, 1000], kappa=2.95, sh="dirac", goal=2.0),
    9: dict(frames=[0, 900, 1500], camera=[120, 1380, -120, 590, 700, 550],   # above / in front of the lanes
            window=[1080, 1350], kappa=2.95, sh="exact"),
    8: dict(frames=[0, 1200, 2000], camera=TOP_DOWN, window=[1200, 1000], kappa=1.5, sh="exact"),
}


def run_scene(binary, build_dir, out_dir, scene, frames, camera, window, sh, kappa, seed, goal):
    """One snapshot run of a scene; checks that every frame was written."""
    cmd = [binary, "-scene", str(scene), "-snapshot"] + [str(f) for f in frames]
    cmd += ["-window", str(window[0]), str(window[1]),
            "-sh", sh, "-kappa", f"{kappa:g}", "-seed", str(seed), "-snapshot-dir", out_dir]
    if camera:
        cmd += ["-camera"] + [f"{c:g}" for c in camera]
    if goal > 0:
        cmd += ["-goal", f"{goal:g}"]
    print("running:", " ".join(cmd), flush=True)
    if "ONEAPI_ROOT" not in os.environ and os.path.exists(SETVARS):
        cmd = ["bash", "-c", f'source "{SETVARS}" >/dev/null 2>&1; exec "$@"', "bash"] + cmd

    res = subprocess.run(cmd, cwd=build_dir, capture_output=True, text=True)
    written = [l.split(": ", 1)[1] for l in res.stdout.splitlines() if l.startswith("snapshot: ")]
    for w in written:
        print("  wrote", w)
    if res.returncode != 0 or len(written) != len(set(frames)):
        sys.exit(f"snapshot run failed (exit {res.returncode}, {len(written)} of {len(set(frames))} images):\n"
                 f"{res.stdout}\n{res.stderr}")


def main():
    ap = argparse.ArgumentParser(description="Clean, reproducible snapshots of a SYCL scene.")
    ap.add_argument("--scene", type=int, help="scene number (keys 1-9); default: all paper figures")
    ap.add_argument("--frames", type=int, nargs="+", help="frames to write (default per scene)")
    ap.add_argument("--sh", choices=["dirac", "vmf", "exact"],
                    help="SH projection driving the scene (default per scene; 'exact' = reference)")
    ap.add_argument("--kappa", type=float, help="vMF concentration k (default per scene)")
    ap.add_argument("--seed", type=int, default=1, help="seed of the initial placement")
    ap.add_argument("--goal", type=float, help="goal-seeking weight, scenes 4 and 5 (default per scene)")
    ap.add_argument("--window", type=int, nargs=2, metavar=("W", "H"),
                    help="image size in pixels (default per scene)")
    ap.add_argument("--camera", type=float, nargs=6, metavar=("EX", "EY", "EZ", "CX", "CY", "CZ"),
                    help="eye position and look-at point (default per scene, else the scene preset)")
    ap.add_argument("--build-dir", default=os.path.join(HERE, "..", "bsh", "build"),
                    help="directory holding boidSimSYCL (it must run from there)")
    ap.add_argument("--out-dir", default=os.path.join(HERE, "figures", "snapshots"),
                    help="where the PNGs are written")
    a = ap.parse_args()

    build_dir = os.path.abspath(a.build_dir)
    out_dir = os.path.abspath(a.out_dir)
    binary = os.path.join(build_dir, "boidSimSYCL")
    if not os.access(binary, os.X_OK):
        sys.exit(f"{binary} not found; build it first (cmake --build build --target boidSimSYCL)")
    os.makedirs(out_dir, exist_ok=True)

    for scene in ([a.scene] if a.scene is not None else sorted(SCENES)):
        d = SCENES.get(scene, {})
        frames = a.frames or d.get("frames")
        if not frames:
            sys.exit(f"no default frames for scene {scene}: pass --frames")
        run_scene(binary, build_dir, out_dir, scene, frames,
                  a.camera or d.get("camera"), a.window or d.get("window", [1080, 1350]),
                  a.sh or d.get("sh", "exact"), a.kappa if a.kappa is not None else d.get("kappa", 2.95),
                  a.seed, a.goal if a.goal is not None else d.get("goal", 0.0))


if __name__ == "__main__":
    main()
