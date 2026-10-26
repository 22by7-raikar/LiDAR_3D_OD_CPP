#!/usr/bin/env python3
"""Renders the stage-by-stage figures used in docs/ from the CSV dumps of
lidar_od_stages (one frame, every algorithm) and lidar_od_run (whole sequence).

    ./build/lidar_od_stages --frame 100 --out output/stages_100
    ./build/lidar_od_run --out output/run
    python3 scripts/render_figures.py --stages output/stages_100 --run output/run \
        --data data/sequence_02 --out docs/images

Requires numpy, matplotlib and pillow.
"""

import argparse
import os
import struct

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from matplotlib.patches import Polygon, Wedge  # noqa: E402

# ---------------------------------------------------------------- style ----
INK = "#1f2328"
MUTED = "#8c959f"
GROUND = "#c9ced6"
OBSTACLE = "#d1495b"
ACCENT = "#2f6fdf"
EGO = "#111111"
QUAL = ["#2f6fdf", "#e07a1f", "#2a9d8f", "#d1495b", "#7b5ea7", "#8a6d3b", "#c2457a",
        "#4c8c2b", "#3a8fb7", "#b8860b", "#6c757d", "#9b2226"]
BEV_X, BEV_Y = (-20, 40), (-12, 12)

plt.rcParams.update({
    "figure.facecolor": "white", "axes.facecolor": "white", "axes.edgecolor": MUTED,
    "axes.labelcolor": INK, "xtick.color": MUTED, "ytick.color": MUTED, "text.color": INK,
    "axes.titleweight": "bold", "axes.titlesize": 12, "font.size": 10,
    "axes.spines.top": False, "axes.spines.right": False,
})


def colour(i):
    return QUAL[int(i) % len(QUAL)]


def read_csv(path):
    """CSV with a header row -> numpy record array (numeric columns as float)."""
    with open(path) as f:
        names = f.readline().strip().split(",")
        rows = [line.rstrip("\n").split(",") for line in f if line.strip()]
    columns = []
    for k, values in enumerate(zip(*rows) if rows else [[] for _ in names]):
        try:
            columns.append(np.array(values, dtype=float))
        except ValueError:
            columns.append(np.array(values, dtype=str))
    return np.rec.fromarrays(columns, names=names)


def read_pcd(path):
    """Minimal reader for binary x y z intensity PCD files."""
    with open(path, "rb") as f:
        header = {}
        while True:
            key, *vals = f.readline().decode().split()
            header[key] = vals
            if key == "DATA":
                break
        n = int(header["POINTS"][0])
        if header["DATA"][0] != "binary" or header["FIELDS"] != ["x", "y", "z", "intensity"]:
            raise ValueError(f"{path}: expected binary x y z intensity")
        return np.frombuffer(f.read(16 * n), dtype=np.float32).reshape(n, 4)


def stat(stats, stage, method):
    row = stats[(stats["stage"] == stage) & (stats["method"] == method)]
    return float(row["value"][0])


def box_corners(cx, cy, length, width, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    ex, ey = np.array([c, s]) * length / 2, np.array([-s, c]) * width / 2
    ctr = np.array([cx, cy])
    return np.array([ctr + ex + ey, ctr - ex + ey, ctr - ex - ey, ctr + ex - ey])


def draw_box(ax, b, col, lw=1.4, ls="-", label=None):
    ax.add_patch(Polygon(box_corners(b["cx"], b["cy"], b["length"], b["width"], b["yaw"]),
                         closed=True, fill=False, ec=col, lw=lw, ls=ls, label=label))


def bev(ax, title=None):
    ax.set_xlim(*BEV_X)
    ax.set_ylim(*BEV_Y)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m] (forward)")
    ax.set_ylabel("y [m] (left)")
    ax.add_patch(Polygon([[-1.5, -0.9], [2.6, -0.9], [2.6, 0.9], [-1.5, 0.9]], closed=True,
                         fc=EGO, ec="none", zorder=5))
    if title:
        ax.set_title(title, loc="left")


# ------------------------------------------------------- oblique camera ----
class Camera:
    """Pinhole camera for 3-D style renders without a 3-D plotting backend."""

    def __init__(self, eye, target, fov_deg=55.0):
        eye, target = np.asarray(eye, float), np.asarray(target, float)
        fwd = target - eye
        fwd /= np.linalg.norm(fwd)
        right = np.cross(fwd, [0, 0, 1.0])
        right /= np.linalg.norm(right)
        up = np.cross(right, fwd)
        self.R = np.stack([right, up, fwd])
        self.eye = eye
        self.f = 1.0 / np.tan(np.radians(fov_deg) / 2)

    def project(self, xyz):
        p = (np.asarray(xyz, float) - self.eye) @ self.R.T
        depth = p[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            uv = self.f * p[:, :2] / depth[:, None]
        return uv, depth

    def scatter(self, ax, xyz, **kw):
        uv, d = self.project(xyz)
        ok = d > 0.5
        order = np.argsort(-d[ok])  # far first
        c = kw.pop("c", None)
        if c is not None and np.ndim(c) > 0 and len(c) == len(xyz):
            c = np.asarray(c)[ok][order]
        ax.scatter(uv[ok][order, 0], uv[ok][order, 1], c=c, **kw)

    def box(self, ax, b, col, lw=1.3, alpha=1.0):
        cs = box_corners(b["cx"], b["cy"], b["length"], b["width"], b["yaw"])
        z0, z1 = b["cz"] - b["height"] / 2, b["cz"] + b["height"] / 2
        pts = np.vstack([np.c_[cs, np.full(4, z0)], np.c_[cs, np.full(4, z1)]])
        uv, d = self.project(pts)
        if np.any(d < 0.5):
            return
        for a, bb in [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4),
                      (0, 4), (1, 5), (2, 6), (3, 7)]:
            ax.plot(uv[[a, bb], 0], uv[[a, bb], 1], color=col, lw=lw, alpha=alpha)

    @staticmethod
    def frame(ax, title=None):
        ax.set_xlim(-1.25, 1.25)
        ax.set_ylim(-0.62, 0.55)
        ax.set_aspect("equal")
        ax.axis("off")
        if title:
            ax.set_title(title, loc="left")


CAM = Camera(eye=(-17.0, -11.0, 12.0), target=(14.0, 0.0, -2.0))


def save(fig, out, name):
    path = os.path.join(out, name)
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print("wrote", path)


# --------------------------------------------------------------- figures ----
def fig_raw(P, out):
    xyz = np.c_[P["x"], P["y"], P["z"]]
    near = np.hypot(P["x"], P["y"]) < 45
    fig, axs = plt.subplots(1, 2, figsize=(16, 5.4), gridspec_kw={"width_ratios": [1.25, 1]})
    CAM.scatter(axs[0], xyz[near], s=0.15, c=np.clip(P["z"][near], -2.2, 1.5), cmap="viridis",
                linewidths=0)
    roi_lo, roi_hi = (-20, -12, -2.5), (40, 12, 1.0)
    CAM.box(axs[0], {"cx": 10, "cy": 0, "cz": (roi_lo[2] + roi_hi[2]) / 2, "length": 60, "width": 24,
                     "height": roi_hi[2] - roi_lo[2], "yaw": 0.0}, ACCENT, lw=1.0, alpha=0.8)
    Camera.frame(axs[0], f"Raw scan: {len(P):,} points, coloured by height (blue box = region of interest)")
    ax = axs[1]
    ax.scatter(P["x"], P["y"], s=0.05, c=MUTED, linewidths=0)
    roi = P["roi"] == 1
    ax.scatter(P["x"][roi], P["y"][roi], s=0.08, c=INK, linewidths=0)
    bev(ax, f"Bird's-eye view: {roi.sum():,} points kept after ROI crop")
    ax.set_xlim(-45, 60)
    ax.set_ylim(-30, 30)
    ax.add_patch(Polygon([[-20, -12], [40, -12], [40, 12], [-20, 12]], closed=True, fill=False,
                         ec=ACCENT, lw=1.5))
    save(fig, out, "01_raw_and_roi.png")


def fig_range_image(P, out):
    rows, cols = int(P["row"].max()) + 1, int(P["col"].max()) + 1
    rng = np.full((rows, cols), np.nan)
    lab = np.full((rows, cols), -1.0)
    r, c = P["row"].astype(int), P["col"].astype(int)
    ok = r >= 0
    dist = np.sqrt(P["x"] ** 2 + P["y"] ** 2 + P["z"] ** 2)
    rng[r[ok], c[ok]] = dist[ok]
    cl = P["cluster_range_image"]
    m = ok & (cl >= 0) & (P["roi"] == 1)
    lab[r[m], c[m]] = cl[m]
    # Show the forward-facing half (azimuth -90..+90 degrees), centred on +x.
    shift = np.roll(np.arange(cols), cols // 4)[: cols // 2][::-1]
    fig, axs = plt.subplots(2, 1, figsize=(16, 5.2))
    im = axs[0].imshow(rng[:, shift], aspect="auto", cmap="magma_r", vmin=2, vmax=45,
                       interpolation="nearest")
    axs[0].set_title("Range image (rows = the 64 lasers recovered from scan order, columns = azimuth); "
                     "colour = range [m]", loc="left")
    fig.colorbar(im, ax=axs[0], pad=0.01, fraction=0.03)
    rgb = np.ones((rows, cols // 2, 3))
    sub = lab[:, shift]
    gnd = np.zeros_like(sub, bool)
    g = ok & (P["ground_region"] == 1)
    gmask = np.zeros((rows, cols), bool)
    gmask[r[g], c[g]] = True
    gnd = gmask[:, shift]
    rgb[gnd] = matplotlib.colors.to_rgb(GROUND)
    for k in np.unique(sub[sub >= 0]):
        rgb[sub == k] = matplotlib.colors.to_rgb(colour(k))
    axs[1].imshow(rgb, aspect="auto", interpolation="nearest")
    axs[1].set_title("Same image after ground removal and range-image clustering (one colour per object)",
                     loc="left")
    for ax in axs:
        ax.set_yticks([0, 31, 63])
        ax.set_yticklabels(["ring 0 (top)", "31", "63 (bottom)"])
        ax.set_xticks(np.linspace(0, cols // 2 - 1, 5))
        ax.set_xticklabels(["+90° (left)", "+45°", "0° (ahead)", "−45°", "−90° (right)"])
    fig.tight_layout()
    save(fig, out, "02_range_image.png")


def fig_ground(P, stats, out):
    roi = P["roi"] == 1
    fig, axs = plt.subplots(2, 2, figsize=(16, 7.4))
    names = {"ransac": "RANSAC plane (adaptive, refined)", "region": "Region-wise planes (concentric zones)",
             "ray": "Ray-based slope walk", "grid": "Grid minimum height"}
    for ax, m in zip(axs.flat, ["ransac", "region", "ray", "grid"]):
        g = P["ground_" + m]
        ax.scatter(P["x"][roi & (g == 1)], P["y"][roi & (g == 1)], s=0.06, c=GROUND, linewidths=0)
        ax.scatter(P["x"][roi & (g == 0)], P["y"][roi & (g == 0)], s=0.12, c=OBSTACLE, linewidths=0)
        bev(ax, f"{names[m]}   {stat(stats, 'ground', m):.1f} ms, "
                f"{int(stat(stats, 'ground_points', m)):,} ground pts")
    fig.tight_layout()
    save(fig, out, "03_ground_comparison.png")


def fig_zones(bins, P, out):
    edges = [2.7, 12.0, 22.0, 40.0, 80.0]
    rings, sectors = [2, 4, 4, 4], [16, 32, 54, 32]
    fig, ax = plt.subplots(figsize=(9, 9))
    for b in bins:
        z, rr, s = int(b["zone"]), int(b["ring"]), int(b["sector"])
        w = (edges[z + 1] - edges[z]) / rings[z]
        r0 = edges[z] + rr * w
        a0, a1 = 360.0 * s / sectors[z], 360.0 * (s + 1) / sectors[z]
        fc = "#f2f3f5" if b["points"] == 0 else ("#cfe3d6" if b["accepted"] else "#f3c7cd")
        ax.add_patch(Wedge((0, 0), r0 + w, a0, a1, width=w, fc=fc, ec="white", lw=0.6))
    roi = P["roi"] == 1
    g = P["ground_region"]
    ax.scatter(P["x"][roi & (g == 0)], P["y"][roi & (g == 0)], s=0.05, c=OBSTACLE, linewidths=0)
    ax.set_xlim(-42, 42)
    ax.set_ylim(-42, 42)
    ax.set_aspect("equal")
    ax.set_title("Concentric zone model: green = bin accepted as ground, pink = rejected "
                 "(plane too steep or too high), grey = empty", loc="left", fontsize=10)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    save(fig, out, "04_region_zones.png")


def fig_clusters(P, stats, out):
    roi = P["roi"] == 1
    fig, axs = plt.subplots(1, 2, figsize=(16, 3.9))
    for ax, m, title in [(axs[0], "range_image", "Range-image clustering"),
                         (axs[1], "euclidean", "Euclidean clustering (k-d tree)")]:
        ax.scatter(P["x"][roi], P["y"][roi], s=0.04, c="#e6e8eb", linewidths=0)
        cl = P["cluster_" + m]
        sel = roi & (cl >= 0)
        ax.scatter(P["x"][sel], P["y"][sel], s=0.25, c=[colour(k * 7) for k in cl[sel]], linewidths=0)
        bev(ax, f"{title}: {int(stat(stats, 'clusters', m))} clusters in {stat(stats, 'cluster', m):.1f} ms")
    fig.tight_layout()
    save(fig, out, "05_clustering.png")


def fig_boxes(P, B, out, focus):
    fig, axs = plt.subplots(1, len(focus), figsize=(5.2 * len(focus), 4.6))
    styles = {"aabb": (MUTED, ":", "axis-aligned"), "pca": ("#e07a1f", "--", "PCA"),
              "lshape": (ACCENT, "-", "L-shape")}
    for ax, (det, name) in zip(np.atleast_1d(axs), focus):
        sel = P["detection"] == det
        ax.scatter(P["x"][sel], P["y"][sel], s=4, c=INK, linewidths=0)
        for m, (col, ls, label) in styles.items():
            b = B[(B["method"] == m) & (B["detection"] == det)][0]
            draw_box(ax, b, col, lw=2.0, ls=ls, label=label)
        lb = B[(B["method"] == "lshape") & (B["detection"] == det)][0]
        ax.set_xlim(lb["cx"] - 3.6, lb["cx"] + 3.6)
        ax.set_ylim(lb["cy"] - 3.0, lb["cy"] + 3.0)
        ax.set_aspect("equal")
        ax.set_title(name, loc="left")
        ax.set_xlabel("x [m]")
    np.atleast_1d(axs)[0].legend(loc="lower left", frameon=False)
    fig.tight_layout()
    save(fig, out, "06_box_fitting.png")


def fig_scores(S, B, out, det):
    s = S[S["detection"] == det]
    fig, axs = plt.subplots(1, 3, figsize=(15, 3.4))
    best = np.degrees(B[(B["method"] == "lshape") & (B["detection"] == det)]["yaw"][0]) % 90
    for ax, key, title in [(axs[0], "area", "Area (smaller rectangle = better)"),
                           (axs[1], "closeness", "Closeness (points hug the edges)"),
                           (axs[2], "variance", "Variance (crisp edges), used by default")]:
        v = s[key]
        ax.plot(s["theta_deg"], (v - v.min()) / (np.ptp(v) + 1e-12), color=ACCENT, lw=2)
        ax.axvline(best, color=OBSTACLE, lw=1, ls="--")
        ax.set_title(title, loc="left")
        ax.set_xlabel("candidate heading θ [deg]")
        ax.set_yticks([])
    axs[0].set_ylabel("normalised score")
    fig.tight_layout()
    save(fig, out, "07_lshape_criteria.png")


def fig_detections(P, B, out):
    xyz = np.c_[P["x"], P["y"], P["z"]]
    roi = P["roi"] == 1
    det = P["detection"]
    rej = P["rejected"] >= 0
    fig, ax = plt.subplots(figsize=(15, 7))
    g = roi & (P["ground_region"] == 1)
    CAM.scatter(ax, xyz[g], s=0.08, c=GROUND, linewidths=0)
    CAM.scatter(ax, xyz[roi & rej], s=0.15, c=MUTED, linewidths=0)
    for k in np.unique(det[det >= 0]):
        CAM.scatter(ax, xyz[det == k], s=0.5, c=colour(k), linewidths=0)
    for b in B[B["method"] == "lshape"]:
        CAM.box(ax, b, colour(b["detection"]))
    Camera.frame(ax, f"Detections: {len(np.unique(det[det >= 0]))} objects with L-shape boxes "
                     "(grey = clusters rejected as walls, vegetation or noise)")
    save(fig, out, "08_detections.png")


def fig_tracking(T, poses, out):
    fig, ax = plt.subplots(figsize=(15, 6.5))
    conf = T[T["confirmed"] == 1]
    ax.plot(poses["x"], poses["y"], color=EGO, lw=2.5, label="ego vehicle (LiDAR odometry)")
    static = []
    for tid in np.unique(conf["track"]):
        t = conf[conf["track"] == tid]
        speed = np.median(np.hypot(t["vx"], t["vy"]))
        if len(t) >= 15 and speed > 2.0:
            col = colour(tid)
            ax.plot(t["wx"], t["wy"], color=col, lw=2)
            ax.annotate(f"#{int(tid)}  {speed * 3.6:.0f} km/h", (t["wx"][-1], t["wy"][-1]), color=col,
                        fontsize=9, fontweight="bold", xytext=(4, 4), textcoords="offset points")
        elif len(t) >= 15:
            static.append((np.mean(t["wx"]), np.mean(t["wy"])))
    if static:
        s = np.array(static)
        ax.scatter(s[:, 0], s[:, 1], s=6, c=MUTED, label="static objects (median speed < 2 m/s)")
    ax.set_aspect("equal")
    ax.set_xlabel("world x [m]")
    ax.set_ylabel("world y [m]")
    ax.legend(loc="upper left", frameon=False)
    ax.set_title("Whole sequence in the world frame: moving tracks with their median speed", loc="left")
    save(fig, out, "09_tracking_world.png")


def fig_timing(tim, out):
    keys = ["preprocess_ms", "ground_ms", "cluster_ms", "boxes_ms", "ego_motion_ms", "tracking_ms"]
    labels = ["preprocess + scan layout", "ground (region-wise)", "clustering (range image)",
              "boxes (L-shape)", "ego-motion (NDT)", "tracking"]
    cols = ["#8c959f", "#2a9d8f", "#2f6fdf", "#7b5ea7", "#e07a1f", "#d1495b"]
    fig, ax = plt.subplots(figsize=(14, 3.8))
    bottom = np.zeros(len(tim))
    for k, l, c in zip(keys, labels, cols):
        ax.bar(tim["frame"], tim[k], bottom=bottom, color=c, width=1.0, label=f"{l}  ({np.mean(tim[k]):.1f} ms)")
        bottom += tim[k]
    ax.axhline(100, color=INK, lw=1, ls="--")
    ax.text(len(tim) - 1, 101, "10 Hz budget", ha="right", va="bottom", fontsize=9)
    ax.set_xlabel("frame")
    ax.set_ylabel("ms")
    ax.set_xlim(-0.5, len(tim) - 0.5)
    ax.legend(loc="upper left", ncol=3, frameon=False, fontsize=9)
    ax.set_title(f"Per-frame runtime (single thread except RANSAC scoring): mean "
                 f"{np.mean(tim['total_ms']):.0f} ms", loc="left")
    save(fig, out, "10_timing.png")


def gif_tracking(T, data, out, first, last):
    files = sorted(f for f in os.listdir(data) if f.endswith(".pcd"))
    fig, ax = plt.subplots(figsize=(9, 3.9))
    conf = T[T["confirmed"] == 1]

    def draw(f):
        ax.clear()
        pts = read_pcd(os.path.join(data, files[f]))
        sel = (pts[:, 0] > BEV_X[0]) & (pts[:, 0] < BEV_X[1]) & (np.abs(pts[:, 1]) < BEV_Y[1])
        p = pts[sel][::2]
        high = p[:, 2] > -1.45
        ax.scatter(p[~high, 0], p[~high, 1], s=0.05, c=GROUND, linewidths=0)
        ax.scatter(p[high, 0], p[high, 1], s=0.1, c=MUTED, linewidths=0)
        for t in conf[conf["frame"] == f]:
            speed = np.hypot(t["vx"], t["vy"])
            moving = speed > 1.0
            col = colour(t["track"]) if moving else "#9aa1a9"
            b = {"cx": t["sx"], "cy": t["sy"], "length": t["length"], "width": t["width"], "yaw": t["syaw"]}
            draw_box(ax, b, col, lw=1.6 if moving else 0.7)
            if moving:
                ax.text(t["sx"], t["sy"] + t["width"] / 2 + 0.4, f"#{int(t['track'])} {speed * 3.6:.0f} km/h",
                        color=col, fontsize=7, ha="center", fontweight="bold")
        bev(ax, f"frame {f}: confirmed tracks (coloured = moving)")
        ax.set_xlabel("")
        ax.set_ylabel("")

    anim = FuncAnimation(fig, draw, frames=range(first, last + 1))
    path = os.path.join(out, "tracking.gif")
    anim.save(path, writer=PillowWriter(fps=10), dpi=80)
    plt.close(fig)
    print("wrote", path)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stages", required=True)
    ap.add_argument("--run", required=True)
    ap.add_argument("--data", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frame", type=int, default=100, help="frame the stages were dumped for")
    ap.add_argument("--gif", default="40:153", help="frame range of the animation, first:last")
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    P = read_csv(os.path.join(args.stages, "points.csv"))
    B = read_csv(os.path.join(args.stages, "boxes.csv"))
    S = read_csv(os.path.join(args.stages, "lshape_scores.csv"))
    stats = read_csv(os.path.join(args.stages, "stats.csv"))
    bins = read_csv(os.path.join(args.stages, "region_bins.csv"))
    T = read_csv(os.path.join(args.run, "tracks.csv"))
    poses = read_csv(os.path.join(args.run, "poses.csv"))
    tim = read_csv(os.path.join(args.run, "timing.csv"))

    # Box-fitting close-ups: the largest car-sized detections plus the cyclist.
    L = B[B["method"] == "lshape"]
    cars = [b for b in L if 3.0 < b["length"] < 6.0 and 1.3 < b["width"] < 2.4]
    cars.sort(key=lambda b: -np.sum(P["detection"] == b["detection"]))
    focus = [(int(b["detection"]), f"car, detection {int(b['detection'])}") for b in cars[:3]]
    moving = T[(T["frame"] == args.frame) & (T["confirmed"] == 1) & (T["detection"] >= 0)]
    moving = moving[(np.hypot(moving["vx"], moving["vy"]) > 3.0) & (moving["length"] < 2.2)]
    if len(moving):
        focus.append((int(moving["detection"][0]), f"cyclist, detection {int(moving['detection'][0])}"))

    fig_raw(P, args.out)
    fig_range_image(P, args.out)
    fig_ground(P, stats, args.out)
    fig_zones(bins, P, args.out)
    fig_clusters(P, stats, args.out)
    fig_boxes(P, B, args.out, focus)
    fig_scores(S, B, args.out, focus[0][0])
    fig_detections(P, B, args.out)
    fig_tracking(T, poses, args.out)
    fig_timing(tim, args.out)
    first, last = (int(v) for v in args.gif.split(":"))
    gif_tracking(T, args.data, args.out, first, last)


if __name__ == "__main__":
    main()
