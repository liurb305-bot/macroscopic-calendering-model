# -*- coding: utf-8 -*-
# 检查弹性辊是否发生轴向弯曲：
# 1) 沿辊子轴向 z 分段，计算每个截面的平均 U2；
# 2) 对比中部和两端的 U2 差值，判断弯曲量；
# 3) 额外检查接触侧表面附近的 U2 变化。

from __future__ import print_function

import os
import sys
import csv

from odbAccess import openOdb


WORKDIR = r"E:\abaqus\3Dnihe2.0\3Dnihe2.0"
JOB = "RP3D_Self_DPC_S030_50mm_Roll10mm_dt5e6_CoarseRoller_V2p2"
ODB_PATH = os.path.join(WORKDIR, JOB + ".odb")

ROLLER_INSTANCE_NAMES = ["UPPERROLLER-1", "LOWERROLLER-1"]
NBINS = 21


def mean(values):
    if not values:
        return None
    return sum(values) / float(len(values))


def get_final_frame(odb):
    # 优先取 Rolling 步最后一帧；若不存在，取最后一个分析步最后一帧。
    if "Rolling" in odb.steps:
        return "Rolling", odb.steps["Rolling"].frames[-1]
    step_name = list(odb.steps.keys())[-1]
    return step_name, odb.steps[step_name].frames[-1]


def bin_index(value, vmin, vmax, nbins):
    if vmax <= vmin:
        return 0
    idx = int((value - vmin) / (vmax - vmin) * nbins)
    if idx < 0:
        idx = 0
    if idx >= nbins:
        idx = nbins - 1
    return idx


def analyze_instance(odb, frame, inst_name):
    assembly = odb.rootAssembly
    if inst_name not in assembly.instances:
        print("Instance not found: %s" % inst_name)
        print("Available instances: %s" % ", ".join(assembly.instances.keys()))
        return None

    inst = assembly.instances[inst_name]
    ufield = frame.fieldOutputs["U"].getSubset(region=inst)
    u_by_label = {}
    for v in ufield.values:
        u_by_label[v.nodeLabel] = v.data

    nodes = list(inst.nodes)
    zs = [n.coordinates[2] for n in nodes]
    xs = [n.coordinates[0] for n in nodes]
    ys = [n.coordinates[1] for n in nodes]
    zmin, zmax = min(zs), max(zs)
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)

    bins = []
    for _ in range(NBINS):
        bins.append({"z": [], "u2": [], "current_y": [], "surface_u2": [], "surface_y": []})

    # 接触侧表面：上辊取原始 y 最小侧；下辊取原始 y 最大侧。
    # 只用靠近极片接触线的外表面节点，用于粗略判断辊面沿 z 的形状。
    y_tol = max((ymax - ymin) * 0.02, 0.5)
    x_center = 0.5 * (xmin + xmax)
    x_tol = max((xmax - xmin) * 0.15, 5.0)
    upper = inst_name.upper().startswith("UPPER")

    for n in nodes:
        u = u_by_label.get(n.label)
        if u is None:
            continue
        x, y, z = n.coordinates
        idx = bin_index(z, zmin, zmax, NBINS)
        u2 = float(u[1])
        cy = float(y + u[1])
        bins[idx]["z"].append(z)
        bins[idx]["u2"].append(u2)
        bins[idx]["current_y"].append(cy)

        if upper:
            is_contact_side = (y <= ymin + y_tol)
        else:
            is_contact_side = (y >= ymax - y_tol)
        if is_contact_side and abs(x - x_center) <= x_tol:
            bins[idx]["surface_u2"].append(u2)
            bins[idx]["surface_y"].append(cy)

    rows = []
    section_u2 = []
    section_z = []
    surface_u2 = []
    surface_y = []
    for i, b in enumerate(bins):
        zavg = mean(b["z"])
        u2avg = mean(b["u2"])
        cyavg = mean(b["current_y"])
        su2avg = mean(b["surface_u2"])
        syavg = mean(b["surface_y"])
        rows.append({
            "bin": i,
            "z_avg_mm": zavg,
            "node_count": len(b["u2"]),
            "section_mean_u2_mm": u2avg,
            "section_mean_current_y_mm": cyavg,
            "surface_node_count": len(b["surface_u2"]),
            "surface_mean_u2_mm": su2avg,
            "surface_mean_current_y_mm": syavg,
        })
        if u2avg is not None:
            section_z.append(zavg)
            section_u2.append(u2avg)
        if su2avg is not None:
            surface_u2.append(su2avg)
            surface_y.append(syavg)

    # 用两端几个截面的平均值作为“端部基准”，与中部截面对比。
    nedge = max(1, NBINS // 5)
    left_vals = [r["section_mean_u2_mm"] for r in rows[:nedge] if r["section_mean_u2_mm"] is not None]
    right_vals = [r["section_mean_u2_mm"] for r in rows[-nedge:] if r["section_mean_u2_mm"] is not None]
    edge_vals = left_vals + right_vals
    center = rows[NBINS // 2]["section_mean_u2_mm"]
    edge_mean = mean(edge_vals)

    section_range = None
    center_minus_edge = None
    if section_u2:
        section_range = max(section_u2) - min(section_u2)
    if center is not None and edge_mean is not None:
        center_minus_edge = center - edge_mean

    surface_range = None
    surface_y_range = None
    if surface_u2:
        surface_range = max(surface_u2) - min(surface_u2)
    if surface_y:
        surface_y_range = max(surface_y) - min(surface_y)

    return {
        "instance": inst_name,
        "bbox": (xmin, xmax, ymin, ymax, zmin, zmax),
        "rows": rows,
        "section_u2_range_mm": section_range,
        "section_center_minus_edge_mm": center_minus_edge,
        "surface_u2_range_mm": surface_range,
        "surface_y_range_mm": surface_y_range,
    }


def write_rows(path, rows):
    fieldnames = [
        "instance", "bin", "z_avg_mm", "node_count",
        "section_mean_u2_mm", "section_mean_current_y_mm",
        "surface_node_count", "surface_mean_u2_mm",
        "surface_mean_current_y_mm",
    ]
    with open(path, "w") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        for r in rows:
            writer.writerow(r)


def main():
    odb = openOdb(ODB_PATH, readOnly=True)
    step_name, frame = get_final_frame(odb)
    all_rows = []
    summaries = []

    for inst_name in ROLLER_INSTANCE_NAMES:
        result = analyze_instance(odb, frame, inst_name)
        if result is None:
            continue
        for r in result["rows"]:
            rr = dict(r)
            rr["instance"] = inst_name
            all_rows.append(rr)
        summaries.append(result)

    csv_path = os.path.join(WORKDIR, JOB + "_roller_bending_by_z.csv")
    write_rows(csv_path, all_rows)

    md_path = os.path.join(WORKDIR, JOB + "_roller_bending_summary.md")
    with open(md_path, "w") as f:
        f.write("# Roller bending check\n\n")
        f.write("- Job: `%s`\n" % JOB)
        f.write("- Step/frame: `%s`, frame time = %.8g s\n\n" % (step_name, frame.frameValue))
        for s in summaries:
            f.write("## %s\n\n" % s["instance"])
            f.write("- Bounding box x/y/z: `%s`\n" % (s["bbox"],))
            f.write("- Section mean U2 range along z: %.9g mm = %.6f um\n" %
                    (s["section_u2_range_mm"], s["section_u2_range_mm"] * 1000.0))
            f.write("- Center section U2 minus end-section mean U2: %.9g mm = %.6f um\n" %
                    (s["section_center_minus_edge_mm"], s["section_center_minus_edge_mm"] * 1000.0))
            if s["surface_u2_range_mm"] is not None:
                f.write("- Contact-side surface U2 range along z: %.9g mm = %.6f um\n" %
                        (s["surface_u2_range_mm"], s["surface_u2_range_mm"] * 1000.0))
            if s["surface_y_range_mm"] is not None:
                f.write("- Contact-side surface current-y range along z: %.9g mm = %.6f um\n" %
                        (s["surface_y_range_mm"], s["surface_y_range_mm"] * 1000.0))
            f.write("\n")

    odb.close()
    print("Wrote %s" % csv_path)
    print("Wrote %s" % md_path)


if __name__ == "__main__":
    main()
