#!/usr/bin/env python3
"""Renders the figures in docs/images from the CSV output of lidar_od_stages
(one frame, every algorithm) and lidar_od_run (whole sequence).

    ./build/lidar_od_stages --frame 100 --out output/stages_100
    OMP_NUM_THREADS=1 ./build/lidar_od_run --out output/run
    python3 scripts/render_figures.py --stages output/stages_100 --run output/run \
        --data data/sequence_02 --out docs/images

Requires numpy, matplotlib and pillow.
"""

import argparse
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.animation import FuncAnimation, PillowWriter  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch, Polygon, Rectangle, Wedge  # noqa: E402

SENSOR_HEIGHT = 1.73
INK = "#1f2328"
MUTED = "#8c959f"
LIGHT = "#d6dae0"
FAINT = "#eceef1"
RED = "#d1495b"
BLUE = "#2f6fdf"
GREEN = "#2a9d8f"
ORANGE = "#e07a1f"
# Ten well separated colours; clusters are coloured in azimuth order so that
# neighbouring objects never share a colour.
PALETTE = ["#2f6fdf", "#e07a1f", "#2a9d8f", "#d1495b", "#7b5ea7",
           "#b8860b", "#c2457a", "#4c8c2b", "#3a8fb7", "#8a6d3b"]
ROI_X, ROI_Y = (-20, 40), (-12, 12)

plt.rcParams.update({
    "figure.facecolor": "white", "axes.facecolor": "white", "axes.edgecolor": MUTED,
    "axes.labelcolor": INK, "xtick.color": MUTED, "ytick.color": MUTED, "text.color": INK,
    "axes.titleweight": "bold", "axes.titlesize": 12, "axes.titlelocation": "left",
    "font.size": 10, "axes.spines.top": False, "axes.spines.right": False,
    "legend.frameon": False,
})


# ------------------------------------------------------------------ input ----
def read_csv(path):
    """CSV with a header row -> numpy record array (numeric columns as float)."""
    with open(path) as f:
        names = f.readline().strip().split(",")
        rows = [line.rstrip("\n").split(",") for line in f if line.strip()]
    columns = []
    for values in (zip(*rows) if rows else [[] for _ in names]):
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


# ---------------------------------------------------------------- helpers ----
def box_corners(cx, cy, length, width, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    ex, ey = np.array([c, s]) * length / 2, np.array([-s, c]) * width / 2
    ctr = np.array([cx, cy])
    return np.array([ctr + ex + ey, ctr - ex + ey, ctr - ex - ey, ctr + ex - ey])


def draw_box(ax, cx, cy, length, width, yaw, colour, lw=1.6, ls="-", fill=False, label=None, z=4):
    ax.add_patch(Polygon(box_corners(cx, cy, length, width, yaw), closed=True,
                         fill=fill, fc=colour if fill else "none", alpha=None,
                         ec=colour, lw=lw, ls=ls, label=label, zorder=z))


def ego(ax, label=True):
    ax.add_patch(Rectangle((-1.5, -0.9), 4.1, 1.8, fc=INK, ec="none", zorder=6))
    if label:
        ax.annotate("ego vehicle", (1.0, 0.9), xytext=(1.0, 2.6), ha="center", fontsize=9,
                    arrowprops=dict(arrowstyle="-", color=INK, lw=0.8), zorder=7)


def bev(ax, xlim=ROI_X, ylim=ROI_Y, title=None, ego_label=False):
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m], forward")
    ax.set_ylabel("y [m], left")
    if xlim[0] < 0 < xlim[1] and ylim[0] < 0 < ylim[1]:
        ego(ax, ego_label)
    if title:
        ax.set_title(title)


def cluster_colours(P, labels):
    """Colour per cluster id, assigned in order of the cluster's azimuth."""
    ids = np.unique(labels[labels >= 0])
    az = [np.arctan2(P["y"][labels == k].mean(), P["x"][labels == k].mean()) for k in ids]
    order = ids[np.argsort(az)]
    return {int(k): PALETTE[i % len(PALETTE)] for i, k in enumerate(order)}


def save(fig, out, name):
    path = os.path.join(out, name)
    fig.savefig(path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    print("wrote", path)


# ---------------------------------------------------------------- figures ----
def fig_input(P, out):
    fig, ax = plt.subplots(figsize=(13, 6.2))
    sel = (np.abs(P["x"]) < 60) & (np.abs(P["y"]) < 32)
    h = np.clip(P["z"][sel] + SENSOR_HEIGHT, -0.3, 3.0)
    order = np.argsort(h)
    sc = ax.scatter(P["x"][sel][order], P["y"][sel][order], c=h[order], s=0.4, cmap="viridis",
                    linewidths=0, rasterized=True)
    fig.colorbar(sc, ax=ax, pad=0.01, fraction=0.025, label="height above road [m]")
    ax.text(0.5, 3.4, "ego vehicle", ha="center", fontsize=9,
            bbox=dict(fc="white", ec="none", alpha=0.85, pad=1.5), zorder=8)
    ax.add_patch(Rectangle((ROI_X[0], ROI_Y[0]), ROI_X[1] - ROI_X[0], ROI_Y[1] - ROI_Y[0],
                           fill=False, ec=RED, lw=1.8, zorder=5))
    ax.text(ROI_X[1] - 0.5, ROI_Y[1] + 0.8, "region of interest", color=RED, ha="right", fontsize=10)
    roi = int((P["roi"] == 1).sum())
    bev(ax, (-60, 60), (-32, 32), f"One scan seen from above: {len(P):,} points, {roi:,} inside the "
                                  "region of interest")
    save(fig, out, "01_input.png")


def fig_ground(P, stats, out):
    roi = P["roi"] == 1
    g = P["ground_region"]
    fig, ax = plt.subplots(figsize=(13, 5.6))
    ax.scatter(P["x"][roi & (g == 1)], P["y"][roi & (g == 1)], s=0.3, c=LIGHT, linewidths=0, rasterized=True)
    ax.scatter(P["x"][roi & (g == 0)], P["y"][roi & (g == 0)], s=0.8, c=RED, linewidths=0, rasterized=True)
    bev(ax, title=f"Ground segmentation (region-wise, {stat(stats, 'ground', 'region'):.0f} ms): "
                  "grey = ground, red = everything else", ego_label=True)
    save(fig, out, "02_ground.png")


def fig_ground_differences(P, stats, out):
    roi = P["roi"] == 1
    methods = ["ransac", "region", "ray", "grid"]
    names = {"ransac": "RANSAC plane", "region": "Region-wise planes", "ray": "Ray slope walk",
             "grid": "Grid minimum"}
    votes = sum((P["ground_" + m] == 0).astype(int) for m in methods)
    majority_obstacle = votes >= 3
    majority_ground = votes <= 1
    fig, axs = plt.subplots(2, 2, figsize=(15, 7.4))
    for ax, m in zip(axs.flat, methods):
        obstacle = P["ground_" + m] == 0
        extra = roi & obstacle & majority_ground      # obstacle where most say ground
        missed = roi & ~obstacle & majority_obstacle  # ground where most say obstacle
        ax.scatter(P["x"][roi], P["y"][roi], s=0.15, c=FAINT, linewidths=0, rasterized=True)
        ax.scatter(P["x"][extra], P["y"][extra], s=2.5, c=RED, linewidths=0, rasterized=True)
        ax.scatter(P["x"][missed], P["y"][missed], s=2.5, c=BLUE, linewidths=0, rasterized=True)
        bev(ax, title=f"{names[m]}: {stat(stats, 'ground', m):.1f} ms, "
                      f"{extra.sum():,} extra / {missed.sum():,} missed obstacle points")
    handles = [Line2D([], [], marker="o", ls="", color=RED, label="called obstacle, most others say ground"),
               Line2D([], [], marker="o", ls="", color=BLUE, label="called ground, most others say obstacle")]
    fig.legend(handles=handles, loc="lower center", ncol=2, bbox_to_anchor=(0.5, -0.02), fontsize=11)
    fig.tight_layout(rect=(0, 0.04, 1, 1))
    save(fig, out, "03_ground_differences.png")


def fig_zones(bins, P, out):
    edges = [2.7, 12.0, 22.0, 40.0, 80.0]
    rings, sectors = [2, 4, 4, 4], [16, 32, 54, 32]
    fig, ax = plt.subplots(figsize=(8.5, 8.5))
    colours = {"empty": "#f4f5f7", "ok": "#cfe8dc", "rejected": "#f6cbd1"}
    for b in bins:
        z, rr, s = int(b["zone"]), int(b["ring"]), int(b["sector"])
        w = (edges[z + 1] - edges[z]) / rings[z]
        r0 = edges[z] + rr * w
        state = "empty" if b["points"] == 0 else ("ok" if b["accepted"] else "rejected")
        ax.add_patch(Wedge((0, 0), r0 + w, 360.0 * s / sectors[z], 360.0 * (s + 1) / sectors[z],
                           width=w, fc=colours[state], ec="white", lw=0.7))
    for e in edges[:4]:
        ax.add_patch(plt.Circle((0, 0), e, fill=False, ec=MUTED, lw=1.0))
        ax.text(e * 0.707 + 0.4, -e * 0.707 - 0.4, f"{e:g} m", fontsize=8, color=MUTED)
    roi = P["roi"] == 1
    obstacle = roi & (P["ground_region"] == 0)
    ax.scatter(P["x"][obstacle], P["y"][obstacle], s=0.3, c=RED, linewidths=0, rasterized=True)
    ego(ax, False)
    ax.set_xlim(-42, 42)
    ax.set_ylim(-42, 42)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Concentric zone model of the region-wise ground fit")
    ax.legend(handles=[Patch(fc=colours["ok"], label="plane accepted as ground"),
                       Patch(fc=colours["rejected"], label="plane rejected (too steep or too high)"),
                       Patch(fc=colours["empty"], label="no points (outside region of interest)"),
                       Line2D([], [], marker="o", ls="", color=RED, label="non-ground points")],
              loc="upper left", fontsize=9, frameon=True, facecolor="white", framealpha=0.9)
    save(fig, out, "04_ground_zones.png")


def fig_range_image(P, out, highlight):
    rows, cols = int(P["row"].max()) + 1, int(P["col"].max()) + 1
    r, c = P["row"].astype(int), P["col"].astype(int)
    ok = r >= 0
    dist = np.sqrt(P["x"] ** 2 + P["y"] ** 2 + P["z"] ** 2)
    rng = np.full((rows, cols), np.nan)
    rng[r[ok], c[ok]] = dist[ok]
    # Forward-facing 120 degrees, left on the left.
    span = cols // 3
    centre = 0
    take = (centre + np.arange(span // 2, -span // 2, -1)) % cols

    fig, axs = plt.subplots(2, 1, figsize=(15, 6.4))
    im = axs[0].imshow(rng[:, take], aspect="auto", cmap="magma_r", vmin=2, vmax=40, interpolation="nearest")
    fig.colorbar(im, ax=axs[0], pad=0.01, fraction=0.025, label="range [m]")
    axs[0].set_title("Range image: one row per laser (64), one column per 0.18° of azimuth")

    labels = P["cluster_range_image"]
    m = ok & (labels >= 0) & (P["roi"] == 1)
    colours = cluster_colours(P, labels)
    rgb = np.ones((rows, cols, 3))
    ground = ok & (P["ground_region"] == 1)
    rgb[r[ground], c[ground]] = matplotlib.colors.to_rgb(LIGHT)
    for k, col in colours.items():
        s = m & (labels == k)
        rgb[r[s], c[s]] = matplotlib.colors.to_rgb(col)
    axs[1].imshow(rgb[:, take], aspect="auto", interpolation="nearest")
    axs[1].set_title("After ground removal: each colour is one object found by range-image clustering "
                     "(grey = ground)")
    for ax in axs:
        ax.set_yticks([0, 32, 63])
        ax.set_yticklabels(["top laser", "32", "bottom laser"])
        ax.set_xticks(np.linspace(0, span - 1, 5))
        ax.set_xticklabels(["60° left", "30° left", "straight ahead", "30° right", "60° right"])
    pos = {int(v): i for i, v in enumerate(take)}
    for name, det in highlight:
        s = ok & (P["detection"] == det)
        cs = [pos[v] for v in c[s] if int(v) in pos]
        if not cs:
            continue
        x0, y0 = np.median(cs), np.min(r[s])
        for ax in axs[1:]:
            ax.annotate(name, (x0, y0), xytext=(x0, y0 - 9), ha="center", fontsize=10, fontweight="bold",
                        color=INK, arrowprops=dict(arrowstyle="->", color=INK, lw=1.2), annotation_clip=False)
    fig.tight_layout()
    save(fig, out, "05_range_image.png")


def fig_clustering(P, stats, out, zoom):
    fig, axs = plt.subplots(1, 2, figsize=(14, 7.6))
    roi = P["roi"] == 1
    inside = roi & (P["x"] > zoom[0][0]) & (P["x"] < zoom[0][1]) & (P["y"] > zoom[1][0]) & (P["y"] < zoom[1][1])
    for ax, m, title in [(axs[0], "range_image", "Range-image clustering"),
                         (axs[1], "euclidean", "Euclidean clustering")]:
        labels = P["cluster_" + m]
        colours = cluster_colours(P, labels)
        ax.scatter(P["x"][inside], P["y"][inside], s=1.2, c=LIGHT, linewidths=0, rasterized=True)
        sel = inside & (labels >= 0)
        ax.scatter(P["x"][sel], P["y"][sel], s=5, c=[colours[int(k)] for k in labels[sel]], linewidths=0,
                   rasterized=True)
        n_zoom = len(np.unique(labels[sel]))
        bev(ax, zoom[0], zoom[1],
            f"{title}\n{n_zoom} clusters in this window, {int(stat(stats, 'clusters', m))} in the frame, "
            f"{stat(stats, 'cluster', m):.1f} ms")
    fig.tight_layout()
    save(fig, out, "06_clustering.png")


def fig_boxes(P, B, out, focus):
    fig, axs = plt.subplots(1, len(focus), figsize=(4.4 * len(focus), 4.9))
    styles = [("aabb", MUTED, ":", "axis-aligned"), ("pca", ORANGE, "--", "PCA"), ("lshape", BLUE, "-", "L-shape")]
    for ax, (det, name) in zip(np.atleast_1d(axs), focus):
        sel = P["detection"] == det
        ax.scatter(P["x"][sel], P["y"][sel], s=6, c=INK, linewidths=0)
        lb = B[(B["method"] == "lshape") & (B["detection"] == det)][0]
        for m, col, ls, label in styles:
            b = B[(B["method"] == m) & (B["detection"] == det)][0]
            draw_box(ax, b["cx"], b["cy"], b["length"], b["width"], b["yaw"], col, lw=2.2, ls=ls, label=label)
        # Small arrow towards the sensor (the visible sides face it), placed in
        # the panel corner farthest from the points.
        d = -np.array([lb["cx"], lb["cy"]])
        d /= np.linalg.norm(d)
        pts = np.c_[P["x"][sel], P["y"][sel]]
        corners = [np.array([lb["cx"] + sx * 2.6, lb["cy"] + sy * 2.6]) for sx in (-1, 1) for sy in (-1, 1)]
        start = max(corners, key=lambda q: np.min(np.linalg.norm(pts - q, axis=1)))
        ax.annotate("", start + 0.6 * d, xytext=start - 0.3 * d,
                    arrowprops=dict(arrowstyle="->", color=MUTED, lw=1.4))
        ax.text(start[0], start[1] - 0.45, "sensor", fontsize=9, color=MUTED, ha="center", va="top")
        ax.set_xlim(lb["cx"] - 3.4, lb["cx"] + 3.4)
        ax.set_ylim(lb["cy"] - 3.4, lb["cy"] + 3.4)
        ax.set_aspect("equal")
        ax.set_title(name)
        ax.set_xlabel("x [m]")
    np.atleast_1d(axs)[0].set_ylabel("y [m]")
    handles, labels = np.atleast_1d(axs)[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="lower center", ncol=3, bbox_to_anchor=(0.5, -0.04), fontsize=11)
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    save(fig, out, "07_box_fitting.png")


def fig_lshape(P, S, out, det):
    s = S[S["detection"] == det]
    sel = P["detection"] == det
    xy = np.c_[P["x"][sel], P["y"][sel]]
    crits = [("area", ORANGE, "area"), ("closeness", GREEN, "closeness"), ("variance", BLUE, "variance (default)")]
    fig, (ax0, ax1) = plt.subplots(1, 2, figsize=(13, 4.8), gridspec_kw={"width_ratios": [1, 1.6]})
    ax0.scatter(xy[:, 0], xy[:, 1], s=6, c=INK, linewidths=0)
    for key, col, label in crits:
        theta = np.radians(s["theta_deg"][np.argmax(s[key])])
        e1, e2 = np.array([np.cos(theta), np.sin(theta)]), np.array([-np.sin(theta), np.cos(theta)])
        a, b = xy @ e1, xy @ e2
        ctr = 0.5 * (a.min() + a.max()) * e1 + 0.5 * (b.min() + b.max()) * e2
        draw_box(ax0, ctr[0], ctr[1], a.max() - a.min(), b.max() - b.min(), theta, col, lw=2, label=label)
        v = s[key]
        ax1.plot(s["theta_deg"], (v - v.min()) / (np.ptp(v) + 1e-12), color=col, lw=2.2, label=label)
        ax1.axvline(np.degrees(theta), color=col, lw=1, ls="--")
    c = xy.mean(axis=0)
    ax0.set_xlim(c[0] - 3.2, c[0] + 3.2)
    ax0.set_ylim(c[1] - 3.2, c[1] + 3.2)
    ax0.set_aspect("equal")
    ax0.set_title("Best rectangle under each criterion")
    ax0.set_xlabel("x [m]")
    ax0.set_ylabel("y [m]")
    ax1.set_title("Score of every candidate heading (dashed = best)")
    ax1.set_xlabel("candidate heading θ [deg]")
    ax1.set_ylabel("score, scaled to [0, 1]")
    ax1.legend(loc="upper center", bbox_to_anchor=(0.5, -0.16), ncol=3)
    fig.tight_layout()
    save(fig, out, "08_lshape_criteria.png")


def fig_detections(P, B, out, highlight):
    roi = P["roi"] == 1
    det = P["detection"]
    rej = P["rejected"] >= 0
    colours = cluster_colours(P, det)
    fig, ax = plt.subplots(figsize=(14, 6.2))
    ax.scatter(P["x"][roi & (P["ground_region"] == 1)], P["y"][roi & (P["ground_region"] == 1)], s=0.2,
               c=FAINT, linewidths=0, rasterized=True)
    ax.scatter(P["x"][roi & rej], P["y"][roi & rej], s=0.8, c=MUTED, linewidths=0, rasterized=True)
    sel = det >= 0
    ax.scatter(P["x"][sel], P["y"][sel], s=1.2, c=[colours[int(k)] for k in det[sel]], linewidths=0,
               rasterized=True)
    for b in B[B["method"] == "lshape"]:
        draw_box(ax, b["cx"], b["cy"], b["length"], b["width"], b["yaw"], colours[int(b["detection"])], lw=1.4)
    for name, d in highlight:
        b = B[(B["method"] == "lshape") & (B["detection"] == d)][0]
        ax.annotate(name, (b["cx"], b["cy"] + b["width"] / 2), xytext=(b["cx"] + 3, b["cy"] + 4), fontsize=11,
                    fontweight="bold", arrowprops=dict(arrowstyle="->", color=INK, lw=1.2), zorder=8)
    n = len(B[B["method"] == "lshape"])
    bev(ax, title=f"Detections: {n} objects with L-shape boxes", ego_label=True)
    ax.legend(handles=[Line2D([], [], color=BLUE, lw=1.5, label="detected object (one colour each)"),
                       Line2D([], [], marker="o", ls="", color=MUTED,
                              label="cluster rejected by the size filter (walls, vegetation)")],
              loc="lower right", fontsize=9, frameon=True, facecolor="white", framealpha=0.9)
    save(fig, out, "09_detections.png")


def moving_tracks(T, min_frames=15, min_speed=2.0):
    conf = T[T["confirmed"] == 1]
    out = []
    for tid in np.unique(conf["track"]):
        t = conf[conf["track"] == tid]
        speed = np.median(np.hypot(t["vx"], t["vy"]))
        if len(t) >= min_frames and speed > min_speed:
            out.append((int(tid), t, speed))
    out.sort(key=lambda e: -len(e[1]))
    return out


def fig_tracks(T, poses, out):
    fig, ax = plt.subplots(figsize=(14, 7))
    conf = T[T["confirmed"] == 1]
    statics = []
    for tid in np.unique(conf["track"]):
        t = conf[conf["track"] == tid]
        if len(t) >= 15 and np.median(np.hypot(t["vx"], t["vy"])) <= 2.0:
            statics.append((np.mean(t["wx"]), np.mean(t["wy"])))
    s = np.array(statics)
    ax.scatter(s[:, 0], s[:, 1], s=7, c=LIGHT, label="stationary objects (median speed < 7 km/h)")
    ax.plot(poses["x"], poses["y"], color=INK, lw=3, label="ego vehicle")
    ax.annotate("", (poses["x"][-1], poses["y"][-1]), xytext=(poses["x"][-4], poses["y"][-4]),
                arrowprops=dict(arrowstyle="-|>", color=INK, lw=2, mutation_scale=18))
    movers = moving_tracks(T)
    for i, (tid, t, speed) in enumerate(movers[:8]):
        col = PALETTE[i % len(PALETTE)]
        size = f"{np.median(t['length']):.1f}×{np.median(t['width']):.1f}×{np.median(t['height']):.1f} m"
        ax.plot(t["wx"], t["wy"], color=col, lw=2.2,
                label=f"track #{tid}: {len(t)} frames, {speed * 3.6:.0f} km/h, {size}")
        ax.annotate("", (t["wx"][-1], t["wy"][-1]), xytext=(t["wx"][-3], t["wy"][-3]),
                    arrowprops=dict(arrowstyle="-|>", color=col, lw=1.5, mutation_scale=12))
    ax.set_aspect("equal")
    ax.set_xlabel("world x [m]")
    ax.set_ylabel("world y [m]")
    ax.set_title("Whole sequence in the world frame: the 8 longest moving tracks (arrow = direction of travel)")
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=9)
    save(fig, out, "10_tracks.png")


def fig_timing(tim, out):
    keys = ["preprocess_ms", "ground_ms", "cluster_ms", "boxes_ms", "ego_motion_ms", "tracking_ms"]
    labels = ["preprocess", "ground", "clustering", "boxes", "ego-motion", "tracking"]
    cols = [MUTED, GREEN, BLUE, "#7b5ea7", ORANGE, RED]
    fig, ax = plt.subplots(figsize=(14, 4))
    bottom = np.zeros(len(tim))
    for k, l, c in zip(keys, labels, cols):
        ax.bar(tim["frame"], tim[k], bottom=bottom, color=c, width=1.0, label=f"{l} ({np.mean(tim[k]):.0f} ms)")
        bottom += tim[k]
    ax.axhline(100, color=INK, lw=1, ls="--")
    ax.text(len(tim) - 1, 103, "100 ms = one frame at 10 Hz", ha="right", va="bottom", fontsize=9)
    top = max(130, np.percentile(bottom, 99) * 1.1)
    ax.set_ylim(0, top)
    for f in np.where(bottom > top)[0]:  # label bars that run off the chart
        ax.annotate(f"{bottom[f]:.0f} ms", (f, top), xytext=(f + 3, top * 0.93), fontsize=9,
                    arrowprops=dict(arrowstyle="-", color=INK, lw=0.8))
    ax.set_xlabel("frame")
    ax.set_ylabel("time [ms]")
    ax.set_xlim(-0.5, len(tim) - 0.5)
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=9, title="mean per stage")
    ax.set_title(f"Runtime per frame, one CPU core: mean {np.mean(tim['total_ms']):.0f} ms")
    save(fig, out, "11_timing.png")


def gif_tracking(T, data, out, first, last):
    files = sorted(f for f in os.listdir(data) if f.endswith(".pcd"))
    xlim, ylim = (-12, 28), (-11, 11)
    fig, ax = plt.subplots(figsize=(9, 5.4))
    fig.subplots_adjust(left=0.07, right=0.98, bottom=0.08, top=0.88)
    conf = T[T["confirmed"] == 1]
    # Same definition and colours as the track figure.
    colour_of = {tid: PALETTE[i % len(PALETTE)] for i, (tid, _, _) in enumerate(moving_tracks(T))}

    def draw(f):
        ax.clear()
        pts = read_pcd(os.path.join(data, files[f]))
        sel = (pts[:, 0] > xlim[0]) & (pts[:, 0] < xlim[1]) & (pts[:, 1] > ylim[0]) & (pts[:, 1] < ylim[1])
        p = pts[sel]
        high = p[:, 2] > -SENSOR_HEIGHT + 0.25
        ax.scatter(p[~high, 0], p[~high, 1], s=0.15, c=FAINT, linewidths=0)
        ax.scatter(p[high, 0], p[high, 1], s=0.5, c=MUTED, linewidths=0)
        for t in conf[conf["frame"] == f]:
            tid = int(t["track"])
            if tid in colour_of:
                col = colour_of[tid]
                speed = np.hypot(t["vx"], t["vy"])
                draw_box(ax, t["sx"], t["sy"], t["length"], t["width"], t["syaw"], col, lw=2.2)
                ax.text(t["sx"], t["sy"] - max(t["width"], 1.0) / 2 - 0.4, f"#{tid}  {speed * 3.6:.0f} km/h",
                        color=col, fontsize=10, ha="center", va="top", fontweight="bold", clip_on=True)
            else:
                draw_box(ax, t["sx"], t["sy"], t["length"], t["width"], t["syaw"], "#b0b6be", lw=0.8)
        bev(ax, xlim, ylim)
        ax.set_title(f"frame {f}   coloured = moving tracks (as in the track figure), grey = other tracks",
                     fontsize=11, pad=10)
        ax.set_xlabel("")
        ax.set_ylabel("")

    anim = FuncAnimation(fig, draw, frames=range(first, last + 1))
    path = os.path.join(out, "tracking.gif")
    anim.save(path, writer=PillowWriter(fps=10), dpi=72)
    plt.close(fig)
    print("wrote", path)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--stages", required=True)
    ap.add_argument("--run", required=True)
    ap.add_argument("--data", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--frame", type=int, default=100, help="frame the stages were dumped for")
    ap.add_argument("--zoom", default="3:17:-9:8", help="clustering close-up window xmin:xmax:ymin:ymax")
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

    # The longest moving track that is visible in this frame is called out by its id.
    here = T[(T["frame"] == args.frame) & (T["confirmed"] == 1) & (T["detection"] >= 0)]
    long_movers = [tid for tid, _, _ in moving_tracks(T)]
    highlight = []
    for tid in long_movers:
        row = here[here["track"] == tid]
        if len(row) and row["sx"][0] > 0:
            highlight.append((f"track #{tid}", int(row["detection"][0])))
            break

    L = B[B["method"] == "lshape"]
    cars = [b for b in L if 3.0 < b["length"] < 6.0 and 1.3 < b["width"] < 2.4]
    cars.sort(key=lambda b: -np.sum(P["detection"] == b["detection"]))
    focus = [(int(b["detection"]), f"car-sized object {k + 1}") for k, b in enumerate(cars[:3])]
    focus += [(d, name) for name, d in highlight]

    zoom = [float(v) for v in args.zoom.split(":")]
    fig_input(P, args.out)
    fig_ground(P, stats, args.out)
    fig_ground_differences(P, stats, args.out)
    fig_zones(bins, P, args.out)
    fig_range_image(P, args.out, highlight)
    fig_clustering(P, stats, args.out, ((zoom[0], zoom[1]), (zoom[2], zoom[3])))
    fig_boxes(P, B, args.out, focus)
    fig_lshape(P, S, args.out, focus[0][0])
    fig_detections(P, B, args.out, highlight)
    fig_tracks(T, poses, args.out)
    fig_timing(tim, args.out)
    first, last = (int(v) for v in args.gif.split(":"))
    gif_tracking(T, args.data, args.out, first, last)


if __name__ == "__main__":
    main()
