#!/usr/bin/env python3
"""
compare_bev_calibrations.py
Visualize multiple BEV calibrations side-by-side to diagnose waypoint spread.

Usage:
    python compare_bev_calibrations.py \\
        --calib may21:/path/to/bev_may21.npz:/path/to/frame_may21.png \\
        --calib may26:/path/to/bev_may26.npz:/path/to/frame_may26.png \\
        --calib jun05:/path/to/bev_jun05.npz:/path/to/frame_jun05.png \\
        --save comparison.png

Each --calib is LABEL:NPZ_PATH:IMAGE_PATH.
Legacy .npy files (matrix only, no y_7_5 / y_10) are also accepted.
"""
import argparse
import cv2
import numpy as np
import matplotlib.pyplot as plt

# ZED2i intrinsics, copied from utils_path.load_camera_calib(sim=False)
MTX = np.array([[532.655, 0.0,     621.155],
                [0.0,     532.595, 349.6815],
                [0.0,     0.0,     1.0]])
DIST = np.array([-0.0435025, 0.0158395, 0.0012899, -0.00129192, -0.00655099])

# Fallbacks for legacy .npy that don't carry y values
DEFAULT_Y_7_5 = 411
DEFAULT_Y_10  = 222

# Must match utils_path.computing_mid_pointS
TH_Y    = 300
N_POINT = 6   # → N_POINT - 1 = 5 sample rows between TH_Y and y_7_5


def load_calib(path):
    """Return (matrix, y_7_5, y_10). Falls back to defaults for legacy .npy."""
    if path.endswith(".npz"):
        data = np.load(path)
        matrix = data["matrix"]
        y_7_5 = int(data["y_7_5"]) if "y_7_5" in data.files else DEFAULT_Y_7_5
        y_10  = int(data["y_10"])  if "y_10"  in data.files else DEFAULT_Y_10
        if "y_7_5" not in data.files:
            print(f"[warn] {path}: missing y_7_5/y_10, using defaults")
    else:
        matrix = np.load(path)
        y_7_5, y_10 = DEFAULT_Y_7_5, DEFAULT_Y_10
        print(f"[warn] {path}: legacy .npy, using defaults "
              f"y_7_5={y_7_5}, y_10={y_10}")
    return matrix, y_7_5, y_10


def warp(img, matrix):
    undist = cv2.undistort(img, MTX, DIST, None, MTX)
    h, w = undist.shape[:2]
    return cv2.warpPerspective(undist, matrix, (w, h), flags=cv2.INTER_CUBIC)


def annotate(bev, y_7_5, y_10):
    """Draw th_y, y_10, y_7_5, and the 5 sample rows on a copy of bev."""
    out = bev.copy()
    if out.ndim == 2:
        out = cv2.cvtColor(out, cv2.COLOR_GRAY2BGR)
    h, w = out.shape[:2]
    cx = w // 2

    # Reference horizontal lines (BGR)
    cv2.line(out, (0, TH_Y),  (w, TH_Y),  (255, 255, 0), 1)   # cyan
    cv2.line(out, (0, y_10),  (w, y_10),  (0, 255, 255), 1)   # yellow
    cv2.line(out, (0, y_7_5), (w, y_7_5), (0, 255, 0),   2)   # green

    # Sample rows — same formula as computing_mid_pointS
    sample_ys = [int(v) for v in np.linspace(TH_Y, y_7_5, N_POINT)[:-1]]
    for ys in sample_ys:
        cv2.circle(out, (cx, ys), 6, (0, 0, 255), -1)         # red

    cv2.putText(out, f"th_y={TH_Y}",     (10, TH_Y - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
    cv2.putText(out, f"10 m  y={y_10}",  (10, y_10 - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    cv2.putText(out, f"7.5 m y={y_7_5}", (10, y_7_5 - 6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return out


def parse_calib(spec):
    parts = spec.split(":", 2)
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(
            f"--calib must be LABEL:NPZ:IMG, got {spec!r}")
    return tuple(parts)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calib", action="append", required=True, type=parse_calib,
                    help="LABEL:NPZ_PATH:IMAGE_PATH (repeat per date)")
    ap.add_argument("--save", default=None,
                    help="Optional path to save the comparison figure")
    args = ap.parse_args()

    n = len(args.calib)
    fig, axes = plt.subplots(1, n, figsize=(6 * n, 6))
    if n == 1:
        axes = [axes]

    print(f"\n{'label':<10} {'y_7_5':>6} {'y_10':>6} "
          f"{'span':>6} {'step(px)':>10}")
    print("-" * 44)

    for ax, (label, npz_path, img_path) in zip(axes, args.calib):
        matrix, y_7_5, y_10 = load_calib(npz_path)
        img = cv2.imread(img_path)
        if img is None:
            raise FileNotFoundError(f"Could not read image: {img_path}")

        bev = warp(img, matrix)
        annotated = annotate(bev, y_7_5, y_10)

        span = y_7_5 - TH_Y
        step = span / (N_POINT - 1)
        print(f"{label:<10} {y_7_5:>6} {y_10:>6} {span:>6} {step:>10.1f}")

        ax.imshow(cv2.cvtColor(annotated, cv2.COLOR_BGR2RGB))
        ax.set_title(f"{label}\ny_7.5={y_7_5}  span={span}px  step={step:.1f}px")
        ax.axis("off")

    plt.tight_layout()
    if args.save:
        plt.savefig(args.save, dpi=120, bbox_inches="tight")
        print(f"\nSaved figure to: {args.save}")
    plt.show()


if __name__ == "__main__":
    main()
