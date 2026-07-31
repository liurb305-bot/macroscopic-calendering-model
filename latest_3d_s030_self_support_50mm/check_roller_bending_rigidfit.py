# -*- coding: utf-8 -*-
# 用最小二乘刚体配准扣除辊子的整体平移和转动，再检查残余 U2。
# 这样可以避免把辊子的整体转动误判为轴向弯曲。

from __future__ import print_function

import os
import csv
import numpy as np

from odbAccess import openOdb


WORKDIR = r"E:\abaqus\3Dnihe2.0\3Dnihe2.0"
JOB = "RP3D_Self_DPC_S030_50mm_Roll10mm_dt5e6_CoarseRoller_V2p2"
ODB_PATH = os.path.join(WORKDIR, JOB + ".odb")
ROLLER_INSTANCE_NAMES = ["UPPERROLLER-1", "LOWERROLLER-1"]
NBINS = 21


def mean(vals):
    vals = list(vals)
    if not vals:
        return None
    return sum(vals) / float(len(vals))


def get_final_frame(odb):
    if "Rolling" in odb.steps:
        return "Rolling", odb.steps["Rolling"].frames[-1]
    step_name = list(odb.steps.keys())[-1]
    return step_name, odb.steps[step_name].frames[-1]


def fit_rigid(P, Q):
    # P/Q: N x 3. Return fitted Q and residual Q-Qfit.
    cP = P.mean(axis=0)
    cQ = Q.mean(axis=0)
    Pc = P - cP
    Qc = Q - cQ
    H = np.dot(Pc.T, Qc)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1.0
        R = np.dot(Vt.T, U.T)
    Qfit = np.dot(Pc, R.T) + cQ
    return Qfit, Q - Qfit, R, cP, cQ


def bin_index(value, vmin, vmax):
    if vmax <= vmin:
        return 0
    idx = int((value - vmin) / (vmax - vmin) * NBINS)
    if idx < 0:
        idx = 0
    if idx >= NBINS:
        idx = NBINS - 1
    return idx


def analyze(odb, frame, inst_name):
    inst = odb.rootAssembly.instances[inst_name]
    uvals = frame.fieldOutputs["U"].getSubset(region=inst).values
    u_by_label = dict((v.nodeLabel, v.data) for v in uvals)

    labels = []
    P_list = []
    Q_list = []
    for n in inst.nodes:
        u = u_by_label.get(n.label)
        if u is None:
            continue
        labels.append(n.label)
        p = np.array(n.coordinates, dtype=float)
        q = p + np.array(u, dtype=float)
        P_list.append(p)
        Q_list.append(q)

    P = np.vstack(P_list)
    Q = np.vstack(Q_list)
    Qfit, residual, R, cP, cQ = fit_rigid(P, Q)

    zmin = float(P[:, 2].min())
    zmax = float(P[:, 2].max())
    ymin = float(P[:, 1].min())
    ymax = float(P[:, 1].max())
    xmin = float(P[:, 0].min())
    xmax = float(P[:, 0].max())

    upper = inst_name.upper().startswith("UPPER")
    y_tol = max((ymax - ymin) * 0.02, 0.5)
    x_center = 0.5 * (xmin + xmax)
    x_tol = max((xmax - xmin) * 0.15, 5.0)

    bins = []
    for i in range(NBINS):
        bins.append({"z": [], "res_y": [], "surface_res_y": []})

    for i in range(P.shape[0]):
        x, y, z = P[i, :]
        idx = bin_index(z, zmin, zmax)
        ry = float(residual[i, 1])
        bins[idx]["z"].append(float(z))
        bins[idx]["res_y"].append(ry)

        if upper:
            contact_side = y <= ymin + y_tol
        else:
            contact_side = y >= ymax - y_tol
        if contact_side and abs(x - x_center) <= x_tol:
            bins[idx]["surface_res_y"].append(ry)

    rows = []
    section_means = []
    surface_means = []
    for i, b in enumerate(bins):
        r = {
            "instance": inst_name,
            "bin": i,
            "z_avg_mm": mean(b["z"]),
            "node_count": len(b["res_y"]),
            "mean_residual_u2_mm": mean(b["res_y"]),
            "surface_node_count": len(b["surface_res_y"]),
            "surface_mean_residual_u2_mm": mean(b["surface_res_y"]),
        }
        rows.append(r)
        if r["mean_residual_u2_mm"] is not None:
            section_means.append(r["mean_residual_u2_mm"])
        if r["surface_mean_residual_u2_mm"] is not None:
            surface_means.append(r["surface_mean_residual_u2_mm"])

    nedge = max(1, NBINS // 5)
    edge_vals = []
    for r in rows[:nedge] + rows[-nedge:]:
        if r["mean_residual_u2_mm"] is not None:
            edge_vals.append(r["mean_residual_u2_mm"])
    center = rows[NBINS // 2]["mean_residual_u2_mm"]
    edge_mean = mean(edge_vals)
    center_minus_edge = None
    if center is not None and edge_mean is not None:
        center_minus_edge = center - edge_mean

    summary = {
        "instance": inst_name,
        "section_residual_u2_range_mm": max(section_means) - min(section_means),
        "section_center_minus_edge_residual_u2_mm": center_minus_edge,
        "surface_residual_u2_range_mm": (max(surface_means) - min(surface_means)) if surface_means else None,
        "rms_residual_mm": float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1)))),
        "max_abs_residual_mm": float(np.max(np.sqrt(np.sum(residual ** 2, axis=1)))),
    }
    return rows, summary


def main():
    odb = openOdb(ODB_PATH, readOnly=True)
    step_name, frame = get_final_frame(odb)
    frame_value = frame.frameValue
    all_rows = []
    summaries = []
    for inst_name in ROLLER_INSTANCE_NAMES:
        rows, summary = analyze(odb, frame, inst_name)
        all_rows.extend(rows)
        summaries.append(summary)
    odb.close()

    csv_path = os.path.join(WORKDIR, JOB + "_roller_bending_rigidfit_by_z.csv")
    with open(csv_path, "w") as f:
        fieldnames = [
            "instance", "bin", "z_avg_mm", "node_count",
            "mean_residual_u2_mm", "surface_node_count",
            "surface_mean_residual_u2_mm",
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        for row in all_rows:
            writer.writerow(row)

    md_path = os.path.join(WORKDIR, JOB + "_roller_bending_rigidfit_summary.md")
    with open(md_path, "w") as f:
        f.write("# Roller bending check after rigid-motion removal\n\n")
        f.write("- Job: `%s`\n" % JOB)
        f.write("- Step/frame: `%s`, frame time = %.8g s\n\n" % (step_name, frame_value))
        for s in summaries:
            f.write("## %s\n\n" % s["instance"])
            f.write("- Section residual U2 range along z: %.9g mm = %.6f um\n" %
                    (s["section_residual_u2_range_mm"], s["section_residual_u2_range_mm"] * 1000.0))
            f.write("- Center residual U2 minus end residual U2: %.9g mm = %.6f um\n" %
                    (s["section_center_minus_edge_residual_u2_mm"],
                     s["section_center_minus_edge_residual_u2_mm"] * 1000.0))
            if s["surface_residual_u2_range_mm"] is not None:
                f.write("- Contact-side surface residual U2 range along z: %.9g mm = %.6f um\n" %
                        (s["surface_residual_u2_range_mm"], s["surface_residual_u2_range_mm"] * 1000.0))
            f.write("- RMS residual deformation after rigid fit: %.9g mm = %.6f um\n" %
                    (s["rms_residual_mm"], s["rms_residual_mm"] * 1000.0))
            f.write("- Max residual deformation after rigid fit: %.9g mm = %.6f um\n\n" %
                    (s["max_abs_residual_mm"], s["max_abs_residual_mm"] * 1000.0))

    print("Wrote %s" % csv_path)
    print("Wrote %s" % md_path)


if __name__ == "__main__":
    main()
