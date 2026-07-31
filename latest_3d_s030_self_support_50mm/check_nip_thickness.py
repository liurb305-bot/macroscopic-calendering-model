# -*- coding: utf-8 -*-
# 逐帧检查极片在辊缝附近以及接触区内的实际厚度。
# 目的：判断极片是否曾在辊缝中被压薄到目标厚度，还是本来就没有被压下去。

from __future__ import print_function

import os
import math
import csv

from odbAccess import openOdb


WORKDIR = r"E:\abaqus\3Dnihe2.0\3Dnihe2.0"
JOB = "RP3D_Self_DPC_S030_50mm_Roll10mm_dt5e6_CoarseRoller_V2p2"
ODB_PATH = os.path.join(WORKDIR, JOB + ".odb")

INITIAL_THICKNESS_UM = 150.0
TARGET_THICKNESS_UM = 135.0
NIP_WINDOWS_MM = [0.5, 1.0, 2.0]
CPRESS_CONTACT_TOL = 1.0e-8


def scalar_data(data):
    try:
        return float(data)
    except Exception:
        try:
            return float(data[0])
        except Exception:
            return 0.0


def vec_add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def dist(a, b):
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def pair_key(node):
    return (round(node.coordinates[0], 6), round(node.coordinates[2], 6))


def find_active_instance(odb):
    for name in odb.rootAssembly.instances.keys():
        if name.upper().startswith("ACTIVELAYER"):
            return odb.rootAssembly.instances[name]
    raise RuntimeError("Cannot find ACTIVELAYER instance.")


def top_bottom_pairs(inst):
    ys = [node.coordinates[1] for node in inst.nodes]
    ymin = min(ys)
    ymax = max(ys)
    tol = max(1.0e-8, (ymax - ymin) * 1.0e-5)
    top = {}
    bottom = {}
    for node in inst.nodes:
        y = node.coordinates[1]
        if abs(y - ymax) <= tol:
            top[pair_key(node)] = node
        elif abs(y - ymin) <= tol:
            bottom[pair_key(node)] = node
    keys = sorted([key for key in top.keys() if key in bottom])
    return top, bottom, keys


def displacement_map(frame, instance_name):
    if "U" not in frame.fieldOutputs:
        return {}
    out = {}
    for value in frame.fieldOutputs["U"].values:
        if value.instance is not None and value.instance.name == instance_name:
            out[value.nodeLabel] = value.data
    return out


def active_cpress_map(frame, instance_name):
    out = {}
    global_max = 0.0
    for name, field in frame.fieldOutputs.items():
        if not name.strip().startswith("CPRESS"):
            continue
        for value in field.values:
            val = abs(scalar_data(value.data))
            global_max = max(global_max, val)
            if value.instance is not None and value.instance.name == instance_name:
                label = getattr(value, "nodeLabel", None)
                if label is not None:
                    out[label] = max(out.get(label, 0.0), val)
    return out, global_max


def pair_state(top_node, bottom_node, u):
    tu = u.get(top_node.label, (0.0, 0.0, 0.0))
    bu = u.get(bottom_node.label, (0.0, 0.0, 0.0))
    tp = vec_add(top_node.coordinates, tu)
    bp = vec_add(bottom_node.coordinates, bu)
    return {
        "actual_um": dist(tp, bp) * 1000.0,
        "vertical_um": (tp[1] - bp[1]) * 1000.0,
        "mid_x": 0.5 * (tp[0] + bp[0]),
        "mid_z": 0.5 * (tp[2] + bp[2]),
    }


def stats(values):
    if not values:
        return None, None, None
    return sum(values) / float(len(values)), min(values), max(values)


def update_best(best, candidate, key):
    val = candidate.get(key)
    if val is None:
        return best
    if best is None or val < best.get(key, 1.0e99):
        return dict(candidate)
    return best


def main():
    odb = openOdb(ODB_PATH, readOnly=True)
    inst = find_active_instance(odb)
    inst_name = inst.name
    top, bottom, keys = top_bottom_pairs(inst)

    rows = []
    best_global_min = None
    best_nip = dict((w, None) for w in NIP_WINDOWS_MM)
    best_contact = None
    best_cpress_frame = None

    cumulative_offset = 0.0
    for step_name, step in odb.steps.items():
        for frame in step.frames:
            u = displacement_map(frame, inst_name)
            cpress, global_cp = active_cpress_map(frame, inst_name)

            all_actual = []
            all_vertical = []
            nip_actual = dict((w, []) for w in NIP_WINDOWS_MM)
            nip_vertical = dict((w, []) for w in NIP_WINDOWS_MM)
            contact_actual = []
            contact_vertical = []
            contact_cpress = []

            for key in keys:
                st = pair_state(top[key], bottom[key], u)
                cp = max(cpress.get(top[key].label, 0.0),
                         cpress.get(bottom[key].label, 0.0))
                all_actual.append(st["actual_um"])
                all_vertical.append(st["vertical_um"])
                for w in NIP_WINDOWS_MM:
                    if abs(st["mid_x"]) <= w:
                        nip_actual[w].append(st["actual_um"])
                        nip_vertical[w].append(st["vertical_um"])
                if cp > CPRESS_CONTACT_TOL:
                    contact_actual.append(st["actual_um"])
                    contact_vertical.append(st["vertical_um"])
                    contact_cpress.append(cp)

            all_avg, all_min, all_max = stats(all_actual)
            all_vavg, all_vmin, all_vmax = stats(all_vertical)
            row = {
                "step": step_name,
                "frame": frame.frameId,
                "step_time_s": frame.frameValue,
                "total_time_s": cumulative_offset + frame.frameValue,
                "global_cpress_max_mpa": global_cp,
                "all_actual_avg_um": all_avg,
                "all_actual_min_um": all_min,
                "all_actual_max_um": all_max,
                "all_vertical_avg_um": all_vavg,
                "all_vertical_min_um": all_vmin,
                "all_vertical_max_um": all_vmax,
                "contact_pair_count": len(contact_actual),
                "contact_actual_avg_um": stats(contact_actual)[0],
                "contact_actual_min_um": stats(contact_actual)[1],
                "contact_vertical_avg_um": stats(contact_vertical)[0],
                "contact_vertical_min_um": stats(contact_vertical)[1],
                "contact_pair_cpress_max_mpa": max(contact_cpress) if contact_cpress else 0.0,
            }
            for w in NIP_WINDOWS_MM:
                aavg, amin, amax = stats(nip_actual[w])
                vavg, vmin, vmax = stats(nip_vertical[w])
                tag = str(w).replace(".", "p")
                row["nip%s_pair_count" % tag] = len(nip_actual[w])
                row["nip%s_actual_avg_um" % tag] = aavg
                row["nip%s_actual_min_um" % tag] = amin
                row["nip%s_vertical_avg_um" % tag] = vavg
                row["nip%s_vertical_min_um" % tag] = vmin
                best_nip[w] = update_best(best_nip[w], row, "nip%s_actual_min_um" % tag)

            rows.append(row)
            best_global_min = update_best(best_global_min, row, "all_actual_min_um")
            best_contact = update_best(best_contact, row, "contact_actual_min_um")
            if best_cpress_frame is None or global_cp > best_cpress_frame["global_cpress_max_mpa"]:
                best_cpress_frame = dict(row)

        if step.frames:
            cumulative_offset += step.frames[-1].frameValue

    odb.close()

    csv_path = os.path.join(WORKDIR, JOB + "_nip_thickness_history.csv")
    fieldnames = list(rows[0].keys())
    with open(csv_path, "w") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    md_path = os.path.join(WORKDIR, JOB + "_nip_thickness_summary.md")
    with open(md_path, "w") as f:
        f.write("# Nip thickness check\n\n")
        f.write("- Job: `%s`\n" % JOB)
        f.write("- Active instance: `%s`\n" % inst_name)
        f.write("- Paired top/bottom nodes: %d\n" % len(keys))
        f.write("- Initial thickness: %.3f um\n" % INITIAL_THICKNESS_UM)
        f.write("- Target thickness: %.3f um\n\n" % TARGET_THICKNESS_UM)

        f.write("## Key result\n\n")
        f.write("- Minimum actual thickness anywhere during run: %.6f um at %s t=%.6g s\n" %
                (best_global_min["all_actual_min_um"], best_global_min["step"],
                 best_global_min["step_time_s"]))
        for w in NIP_WINDOWS_MM:
            tag = str(w).replace(".", "p")
            b = best_nip[w]
            f.write("- Minimum actual thickness in |x|<=%.3f mm nip window: %.6f um "
                    "(pairs=%s, CPRESS max=%.6e MPa, step=%s, t=%.6g s)\n" %
                    (w, b["nip%s_actual_min_um" % tag],
                     b["nip%s_pair_count" % tag],
                     b["global_cpress_max_mpa"], b["step"], b["step_time_s"]))
        if best_contact and best_contact.get("contact_actual_min_um") is not None:
            f.write("- Minimum actual thickness among CPRESS>%.1e contact pairs: %.6f um "
                    "(contact pairs=%s, pair CPRESS max=%.6e MPa, step=%s, t=%.6g s)\n" %
                    (CPRESS_CONTACT_TOL, best_contact["contact_actual_min_um"],
                     best_contact["contact_pair_count"],
                     best_contact["contact_pair_cpress_max_mpa"],
                     best_contact["step"], best_contact["step_time_s"]))
        else:
            f.write("- No active-film node pair had CPRESS above %.1e.\n" % CPRESS_CONTACT_TOL)

        f.write("\n## Maximum CPRESS frame\n\n")
        f.write("- Global CPRESS max: %.6e MPa at %s t=%.6g s\n" %
                (best_cpress_frame["global_cpress_max_mpa"],
                 best_cpress_frame["step"], best_cpress_frame["step_time_s"]))
        f.write("- At that frame, all-pair actual min/avg/max: %.6f / %.6f / %.6f um\n" %
                (best_cpress_frame["all_actual_min_um"],
                 best_cpress_frame["all_actual_avg_um"],
                 best_cpress_frame["all_actual_max_um"]))
        for w in NIP_WINDOWS_MM:
            tag = str(w).replace(".", "p")
            f.write("- At that frame, |x|<=%.3f mm actual min/avg: %.6f / %.6f um, pairs=%s\n" %
                    (w, best_cpress_frame["nip%s_actual_min_um" % tag],
                     best_cpress_frame["nip%s_actual_avg_um" % tag],
                     best_cpress_frame["nip%s_pair_count" % tag]))
        f.write("- At that frame, contact-pair actual min/avg: %s / %s um, pairs=%s\n" %
                (best_cpress_frame["contact_actual_min_um"],
                 best_cpress_frame["contact_actual_avg_um"],
                 best_cpress_frame["contact_pair_count"]))

        f.write("\n## Interpretation\n\n")
        min_any = best_global_min["all_actual_min_um"]
        if min_any > TARGET_THICKNESS_UM + 5.0:
            f.write("- The film was not compressed close to the 135 um target even inside the nip/contact frames.\n")
            f.write("- Therefore the final 150 um thickness is not mainly caused by severe rebound; it was almost never pressed down.\n")
        else:
            f.write("- The film did reach near-target thickness during contact; downstream thickness should be interpreted as rebound/unloading.\n")

    print("Wrote %s" % csv_path)
    print("Wrote %s" % md_path)
    print("Minimum actual thickness anywhere: %.6f um" % best_global_min["all_actual_min_um"])
    for w in NIP_WINDOWS_MM:
        tag = str(w).replace(".", "p")
        print("Minimum in |x|<=%.3f mm: %.6f um" %
              (w, best_nip[w]["nip%s_actual_min_um" % tag]))
    if best_contact and best_contact.get("contact_actual_min_um") is not None:
        print("Minimum among contact pairs: %.6f um" %
              best_contact["contact_actual_min_um"])
    print("Max global CPRESS: %.6e MPa" % best_cpress_frame["global_cpress_max_mpa"])


if __name__ == "__main__":
    main()
