from __future__ import print_function

import csv
import math
import os
import traceback

from odbAccess import openOdb


ODB_PATH = r"E:\abaqus\3D50mm\RollPress_3D_SelfSupport_DiffSpeed_DPC_S00008B5_50mm_Roll2mm_RigidRoller_Stable.odb"
OUT_DIR = r"E:\abaqus\3D50mm"
TARGET_THICKNESS = 0.135
X_MIN_SEL = -2.0
X_MAX_SEL = 0.0


def stats(values):
    values = list(values)
    if not values:
        return None
    values.sort()
    n = len(values)

    def pct(q):
        if n == 1:
            return values[0]
        pos = q * (n - 1)
        lo = int(math.floor(pos))
        hi = int(math.ceil(pos))
        if lo == hi:
            return values[lo]
        return values[lo] * (hi - pos) + values[hi] * (pos - lo)

    return {
        "count": n,
        "min": values[0],
        "p05": pct(0.05),
        "mean": sum(values) / n,
        "p50": pct(0.50),
        "p95": pct(0.95),
        "max": values[-1],
    }


def fmt_stats(s):
    if s is None:
        return "count=0"
    return (
        "count={count}, min={min:.9f}, p05={p05:.9f}, mean={mean:.9f}, "
        "p50={p50:.9f}, p95={p95:.9f}, max={max:.9f}"
    ).format(**s)


def group_stats(name, subset, init_thickness, frame_count):
    final_ts = [p["final_t"] for p in subset]
    min_ts = [p["min_t"] for p in subset]
    rebound = [p["rebound_final_minus_min"] for p in subset]
    red_final = [(init_thickness - p["final_t"]) / init_thickness * 100.0 for p in subset]
    red_min = [(init_thickness - p["min_t"]) / init_thickness * 100.0 for p in subset]
    return {
        "name": name,
        "count": len(subset),
        "final_thickness": stats(final_ts),
        "min_thickness": stats(min_ts),
        "final_reduction_pct": stats(red_final),
        "min_reduction_pct": stats(red_min),
        "rebound_mm": stats(rebound),
        "columns_final_le_target": sum(1 for p in subset if p["final_t"] <= TARGET_THICKNESS),
        "columns_final_gt_target": sum(1 for p in subset if p["final_t"] > TARGET_THICKNESS),
        "columns_rebound_gt_0p0001": sum(1 for p in subset if p["rebound_final_minus_min"] > 0.0001),
        "columns_rebound_gt_0p001": sum(1 for p in subset if p["rebound_final_minus_min"] > 0.001),
        "columns_min_at_final": sum(1 for p in subset if p["min_frame"] == frame_count - 1),
    }


def write_svg(svg_path, station_lines, init_thickness):
    width, height = 900, 520
    margin_l, margin_r, margin_t, margin_b = 80, 40, 50, 80
    xs = [row["x0"] for row in station_lines]
    ys_final = [row["final_mean"] for row in station_lines]
    ys_min = [row["min_over_time_mean"] for row in station_lines]
    x_min = min(xs)
    x_max = max(xs)
    y_min_plot = min(min(ys_final), min(ys_min), TARGET_THICKNESS) - 0.002
    y_max_plot = max(max(ys_final), max(ys_min), init_thickness) + 0.002

    def sx(x):
        den = x_max - x_min if x_max != x_min else 1.0
        return margin_l + (x - x_min) / den * (width - margin_l - margin_r)

    def sy(y):
        den = y_max_plot - y_min_plot if y_max_plot != y_min_plot else 1.0
        return height - margin_b - (y - y_min_plot) / den * (height - margin_t - margin_b)

    final_pts = " ".join("%.2f,%.2f" % (sx(x), sy(y)) for x, y in zip(xs, ys_final))
    min_pts = " ".join("%.2f,%.2f" % (sx(x), sy(y)) for x, y in zip(xs, ys_min))
    target_y = sy(TARGET_THICKNESS)
    init_y = sy(init_thickness)

    lines = []
    lines.append('<?xml version="1.0" encoding="UTF-8"?>')
    lines.append('<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d">' % (width, height, width, height))
    lines.append('<rect width="100%" height="100%" fill="white"/>')
    lines.append('<text x="%d" y="28" font-family="Arial" font-size="18" fill="#111">2 mm rolled segment: thickness and rebound</text>' % margin_l)
    lines.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="#333"/>' % (margin_l, height - margin_b, width - margin_r, height - margin_b))
    lines.append('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="#333"/>' % (margin_l, margin_t, margin_l, height - margin_b))
    for val in [0.126, 0.130, 0.135, 0.140, 0.145, 0.150]:
        if y_min_plot <= val <= y_max_plot:
            y = sy(val)
            lines.append('<line x1="%d" y1="%.2f" x2="%d" y2="%.2f" stroke="#ddd"/>' % (margin_l, y, width - margin_r, y))
            lines.append('<text x="18" y="%.2f" font-family="Arial" font-size="12" fill="#333">%.3f</text>' % (y + 4, val))
    for val in [-2.0, -1.5, -1.0, -0.5, 0.0]:
        x = sx(val)
        lines.append('<line x1="%.2f" y1="%d" x2="%.2f" y2="%d" stroke="#ddd"/>' % (x, margin_t, x, height - margin_b))
        lines.append('<text x="%.2f" y="%d" font-family="Arial" font-size="12" fill="#333" text-anchor="middle">%.1f</text>' % (x, height - margin_b + 22, val))
    lines.append('<line x1="%d" y1="%.2f" x2="%d" y2="%.2f" stroke="#2ca02c" stroke-dasharray="6 4"/>' % (margin_l, target_y, width - margin_r, target_y))
    lines.append('<line x1="%d" y1="%.2f" x2="%d" y2="%.2f" stroke="#999" stroke-dasharray="3 4"/>' % (margin_l, init_y, width - margin_r, init_y))
    lines.append('<polyline fill="none" stroke="#1f77b4" stroke-width="2.5" points="%s"/>' % final_pts)
    lines.append('<polyline fill="none" stroke="#d62728" stroke-width="2.0" points="%s"/>' % min_pts)
    lines.append('<text x="%d" y="%d" font-family="Arial" font-size="13" fill="#1f77b4">final mean thickness</text>' % (width - 245, margin_t + 10))
    lines.append('<text x="%d" y="%d" font-family="Arial" font-size="13" fill="#d62728">minimum during rolling</text>' % (width - 245, margin_t + 30))
    lines.append('<text x="%d" y="%d" font-family="Arial" font-size="13" fill="#2ca02c">target 0.135 mm</text>' % (width - 245, margin_t + 50))
    lines.append('<text x="%d" y="%d" font-family="Arial" font-size="14" fill="#111" text-anchor="middle">initial x0 in rolled 2 mm segment (mm)</text>' % ((width + margin_l - margin_r) // 2, height - 25))
    lines.append('<text x="20" y="%d" font-family="Arial" font-size="14" fill="#111" transform="rotate(-90 20,%d)" text-anchor="middle">thickness (mm)</text>' % (height // 2, height // 2))
    lines.append("</svg>")
    with open(svg_path, "wb") as f:
        f.write(("\n".join(lines)).encode("utf-8"))


def main():
    odb = openOdb(ODB_PATH, readOnly=True)
    try:
        root = odb.rootAssembly
        inst = None
        inst_name = None
        for name, candidate in root.instances.items():
            if name.upper() == "ACTIVELAYER-1":
                inst = candidate
                inst_name = name
                break
        if inst is None:
            for name, candidate in root.instances.items():
                if "ROLLER" not in name.upper():
                    inst = candidate
                    inst_name = name
                    break
        if inst is None:
            raise RuntimeError("Could not find the active layer instance")

        nodes = list(inst.nodes)
        ys = [n.coordinates[1] for n in nodes]
        y_min = min(ys)
        y_max = max(ys)
        init_thickness = y_max - y_min
        tol = max(1.0e-8, init_thickness * 1.0e-4)

        top_by_key = {}
        bot_by_key = {}
        for n in nodes:
            x0, y0, z0 = n.coordinates
            key = (round(x0, 8), round(z0, 8))
            if abs(y0 - y_max) <= tol:
                top_by_key[key] = n
            elif abs(y0 - y_min) <= tol:
                bot_by_key[key] = n

        pairs = []
        for key, top in top_by_key.items():
            bot = bot_by_key.get(key)
            if bot is None:
                continue
            x0, z0 = key
            if X_MIN_SEL - 1.0e-8 <= x0 <= X_MAX_SEL + 1.0e-8:
                pairs.append(
                    {
                        "x0": x0,
                        "z0": z0,
                        "top": top.label,
                        "bottom": bot.label,
                        "top_y0": top.coordinates[1],
                        "bot_y0": bot.coordinates[1],
                        "top_x0": top.coordinates[0],
                        "bot_x0": bot.coordinates[0],
                        "final_t": None,
                        "final_x_mid": None,
                        "min_t": None,
                        "min_time": None,
                        "min_frame": None,
                    }
                )
        if not pairs:
            raise RuntimeError("No paired top/bottom surface nodes in x0=[-2,0]")

        step = odb.steps["Rolling"]
        frames = list(step.frames)
        selected_labels = set()
        for p in pairs:
            selected_labels.add(p["top"])
            selected_labels.add(p["bottom"])

        history_by_col = [[] for _ in pairs]
        station_series = []
        for frame_index, frame in enumerate(frames):
            print("Processing Rolling frame %d/%d, t=%.9f" % (frame_index + 1, len(frames), frame.frameValue))
            u_values = frame.fieldOutputs["U"].getSubset(region=inst).values
            u = {}
            for v in u_values:
                if v.nodeLabel in selected_labels:
                    u[v.nodeLabel] = v.data

            frame_thicknesses = []
            for idx, p in enumerate(pairs):
                ut = u.get(p["top"], (0.0, 0.0, 0.0))
                ub = u.get(p["bottom"], (0.0, 0.0, 0.0))
                thickness = (p["top_y0"] + ut[1]) - (p["bot_y0"] + ub[1])
                history_by_col[idx].append(thickness)
                frame_thicknesses.append(thickness)
                if p["min_t"] is None or thickness < p["min_t"]:
                    p["min_t"] = thickness
                    p["min_time"] = frame.frameValue
                    p["min_frame"] = frame_index
                if frame_index == len(frames) - 1:
                    p["final_t"] = thickness
                    p["final_x_mid"] = 0.5 * (p["top_x0"] + ut[0] + p["bot_x0"] + ub[0])

            station_series.append(
                (
                    frame.frameValue,
                    min(frame_thicknesses),
                    sum(frame_thicknesses) / len(frame_thicknesses),
                    max(frame_thicknesses),
                )
            )

        for idx, p in enumerate(pairs):
            p["rebound_final_minus_min"] = p["final_t"] - p["min_t"]
            compression_at_min = init_thickness - p["min_t"]
            if compression_at_min > 1.0e-12:
                p["rebound_pct_of_comp"] = p["rebound_final_minus_min"] / compression_at_min * 100.0
            else:
                p["rebound_pct_of_comp"] = 0.0

        subset_all = pairs
        subset_exit = [p for p in pairs if p["final_x_mid"] is not None and p["final_x_mid"] > 0.0]
        subset_targetish = [p for p in pairs if 0.133 <= p["final_t"] <= 0.137]
        summaries = [
            group_stats("nominal_rolled_segment_initial_x_-2_to_0", subset_all, init_thickness, len(frames)),
            group_stats("outlet_side_at_final_xdef_gt_0_within_initial_x_-2_to_0", subset_exit, init_thickness, len(frames)),
            group_stats("near_target_final_0p133_to_0p137_within_initial_x_-2_to_0", subset_targetish, init_thickness, len(frames)),
        ]

        by_x = {}
        for p in pairs:
            by_x.setdefault(p["x0"], []).append(p)
        station_lines = []
        for x0 in sorted(by_x):
            group = by_x[x0]
            station_lines.append(
                {
                    "x0": x0,
                    "count": len(group),
                    "final_mean": sum(p["final_t"] for p in group) / len(group),
                    "final_min": min(p["final_t"] for p in group),
                    "final_max": max(p["final_t"] for p in group),
                    "min_over_time_mean": sum(p["min_t"] for p in group) / len(group),
                    "rebound_mean": sum(p["rebound_final_minus_min"] for p in group) / len(group),
                    "final_x_mean": sum(p["final_x_mid"] for p in group) / len(group),
                    "n_le_target": sum(1 for p in group if p["final_t"] <= TARGET_THICKNESS),
                }
            )

        summary_path = os.path.join(OUT_DIR, "rolled_2mm_thickness_rebound_summary.txt")
        csv_path = os.path.join(OUT_DIR, "rolled_2mm_thickness_rebound_by_column.csv")
        station_csv_path = os.path.join(OUT_DIR, "rolled_2mm_thickness_rebound_by_initial_x.csv")
        series_csv_path = os.path.join(OUT_DIR, "rolled_2mm_thickness_time_series.csv")
        svg_path = os.path.join(OUT_DIR, "rolled_2mm_thickness_rebound_by_initial_x.svg")

        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(
                [
                    "x0_mm",
                    "z0_mm",
                    "x_final_mid_mm",
                    "final_thickness_mm",
                    "min_thickness_during_rolling_mm",
                    "min_time_in_rolling_s",
                    "rebound_final_minus_min_mm",
                    "rebound_pct_of_min_compression",
                    "final_reduction_pct",
                    "min_reduction_pct",
                    "final_le_target_0p135",
                    "top_node",
                    "bottom_node",
                ]
            )
            for p in sorted(pairs, key=lambda q: (q["x0"], q["z0"])):
                w.writerow(
                    [
                        "%.8f" % p["x0"],
                        "%.8f" % p["z0"],
                        "%.8f" % p["final_x_mid"],
                        "%.9f" % p["final_t"],
                        "%.9f" % p["min_t"],
                        "%.9f" % p["min_time"],
                        "%.9f" % p["rebound_final_minus_min"],
                        "%.6f" % p["rebound_pct_of_comp"],
                        "%.6f" % ((init_thickness - p["final_t"]) / init_thickness * 100.0),
                        "%.6f" % ((init_thickness - p["min_t"]) / init_thickness * 100.0),
                        int(p["final_t"] <= TARGET_THICKNESS),
                        p["top"],
                        p["bottom"],
                    ]
                )

        with open(station_csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(
                [
                    "x0_mm",
                    "columns",
                    "final_x_mean_mm",
                    "final_t_mean_mm",
                    "final_t_min_mm",
                    "final_t_max_mm",
                    "min_over_time_mean_mm",
                    "rebound_mean_mm",
                    "columns_le_target",
                ]
            )
            for row in station_lines:
                w.writerow(
                    [
                        "%.8f" % row["x0"],
                        row["count"],
                        "%.8f" % row["final_x_mean"],
                        "%.9f" % row["final_mean"],
                        "%.9f" % row["final_min"],
                        "%.9f" % row["final_max"],
                        "%.9f" % row["min_over_time_mean"],
                        "%.9f" % row["rebound_mean"],
                        row["n_le_target"],
                    ]
                )

        with open(series_csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["rolling_time_s", "thickness_min_mm", "thickness_mean_mm", "thickness_max_mm"])
            for row in station_series:
                w.writerow(["%.9f" % row[0], "%.9f" % row[1], "%.9f" % row[2], "%.9f" % row[3]])

        with open(summary_path, "wb") as f:
            def write(line=""):
                f.write((line + "\n").encode("utf-8"))

            write("2 mm rolled segment thickness and rebound check")
            write("ODB: %s" % ODB_PATH)
            write("Instance: %s" % inst_name)
            write("Step: Rolling, frames=%d, final frame time=%.9f s" % (len(frames), frames[-1].frameValue))
            write("Selection: initial x0 in [%.3f, %.3f] mm, paired top/bottom surface columns by initial (x,z)." % (X_MIN_SEL, X_MAX_SEL))
            write("Initial thickness from surface nodes: %.9f mm" % init_thickness)
            write("Target thickness: %.9f mm" % TARGET_THICKNESS)
            write("")
            for gs in summaries:
                write("[%s]" % gs["name"])
                write("columns: %d" % gs["count"])
                write("final_thickness_mm: %s" % fmt_stats(gs["final_thickness"]))
                write("min_thickness_during_rolling_mm: %s" % fmt_stats(gs["min_thickness"]))
                write("final_reduction_pct: %s" % fmt_stats(gs["final_reduction_pct"]))
                write("min_reduction_pct: %s" % fmt_stats(gs["min_reduction_pct"]))
                write("rebound_final_minus_min_mm: %s" % fmt_stats(gs["rebound_mm"]))
                write("columns_final_le_target: %d" % gs["columns_final_le_target"])
                write("columns_final_gt_target: %d" % gs["columns_final_gt_target"])
                write("columns_rebound_gt_0.0001mm: %d" % gs["columns_rebound_gt_0p0001"])
                write("columns_rebound_gt_0.001mm: %d" % gs["columns_rebound_gt_0p001"])
                write("columns_minimum_occurs_at_final_frame: %d" % gs["columns_min_at_final"])
                write("")
            write("[station_by_initial_x]")
            for row in station_lines:
                write(
                    "x0={x0:.8f}, n={count}, x_final_mean={final_x_mean:.8f}, final_mean={final_mean:.9f}, "
                    "final_min={final_min:.9f}, final_max={final_max:.9f}, min_time_mean={min_over_time_mean:.9f}, "
                    "rebound_mean={rebound_mean:.9f}, n_le_target={n_le_target}".format(**row)
                )
            write("")
            write("[time_series_selected_segment]")
            idxs = sorted(
                set(
                    [
                        0,
                        1,
                        2,
                        len(station_series) // 4,
                        len(station_series) // 2,
                        3 * len(station_series) // 4,
                        len(station_series) - 3,
                        len(station_series) - 2,
                        len(station_series) - 1,
                    ]
                )
            )
            for idx in idxs:
                t, mn, av, mx = station_series[idx]
                write("frame=%d, rolling_time=%.9f, min=%.9f, mean=%.9f, max=%.9f" % (idx, t, mn, av, mx))

        write_svg(svg_path, station_lines, init_thickness)

        print("DONE")
        print("summary_path=%s" % summary_path)
        print("csv_path=%s" % csv_path)
        print("station_csv_path=%s" % station_csv_path)
        print("series_csv_path=%s" % series_csv_path)
        print("svg_path=%s" % svg_path)
        for gs in summaries:
            final_mean = "NA" if gs["final_thickness"] is None else "%.9f" % gs["final_thickness"]["mean"]
            rebound_mean = "NA" if gs["rebound_mm"] is None else "%.9f" % gs["rebound_mm"]["mean"]
            print("%s: columns=%d final_mean=%s rebound_mean=%s" % (gs["name"], gs["count"], final_mean, rebound_mean))
    finally:
        odb.close()


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("ERROR: %s" % exc)
        print(traceback.format_exc())
        raise
