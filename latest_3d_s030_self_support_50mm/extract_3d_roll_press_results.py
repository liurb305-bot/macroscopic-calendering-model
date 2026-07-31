# -*- coding: ascii -*-
"""
Post-process a short 3D steady-through roll pressing verification ODB.

The script measures sheet thickness from paired material nodes on the top and
bottom surfaces.  The reported downstream thickness is measured only from
paired nodes that have moved past the nip exit and whose final active-surface
CPRESS is approximately zero.
"""

from odbAccess import openOdb
import csv
import math
import os
import re
import sys


WORKDIR = r"E:\\abaqus\\3Dnihe2.0\\3Dnihe2.0"
JOB = (sys.argv[1] if len(sys.argv) > 1
       else "RP3D_Self_DPC_S030_50mm_Roll10mm_dt5e6_CoarseRoller_V2p2")

ODB_PATH = os.path.join(WORKDIR, JOB + ".odb")
STA_PATH = os.path.join(WORKDIR, JOB + ".sta")
DAT_PATH = os.path.join(WORKDIR, JOB + ".dat")
MSG_PATH = os.path.join(WORKDIR, JOB + ".msg")
CSV_PATH = os.path.join(WORKDIR, JOB + "_downstream_thickness_history.csv")
PAIR_CSV_PATH = os.path.join(WORKDIR, JOB + "_final_downstream_pairs.csv")
PROFILE_CSV_PATH = os.path.join(WORKDIR, JOB + "_final_all_pairs_profile.csv")
TIME_PNG_PATH = os.path.join(WORKDIR, JOB + "_thickness_time.png")
PROFILE_PNG_PATH = os.path.join(WORKDIR, JOB + "_final_thickness_x.png")
SUMMARY_PATH = os.path.join(WORKDIR, JOB + "_verified_result_summary.md")

INITIAL_THICKNESS_MM = 0.150
TARGET_THICKNESS_MM = 0.135
ROLLER_RADIUS_MM = 50.0
NOMINAL_REDUCTION_MM = INITIAL_THICKNESS_MM - TARGET_THICKNESS_MM
NIP_EXIT_X_MM = math.sqrt(2.0 * ROLLER_RADIUS_MM * NOMINAL_REDUCTION_MM)
CPRESS_ZERO_TOL = 1.0e-4


def scalar_data(data):
    if hasattr(data, "__len__"):
        if len(data) == 0:
            return 0.0
        return float(data[0])
    return float(data)


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
    raise RuntimeError("Cannot find ACTIVELAYER instance in ODB.")


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
    return ymin, ymax, top, bottom, keys


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


def stress_pressure_stats(frame, instance_name):
    max_comp_press = None
    if "S" in frame.fieldOutputs:
        for value in frame.fieldOutputs["S"].values:
            if value.instance is None or value.instance.name != instance_name:
                continue
            data = value.data
            if len(data) >= 3:
                press = -(float(data[0]) + float(data[1]) + float(data[2])) / 3.0
                if max_comp_press is None or press > max_comp_press:
                    max_comp_press = press
    min_pevol = None
    max_pevol = None
    max_abs_pevol = None
    if "PE" in frame.fieldOutputs:
        for value in frame.fieldOutputs["PE"].values:
            if value.instance is None or value.instance.name != instance_name:
                continue
            data = value.data
            if len(data) >= 3:
                pevol = float(data[0]) + float(data[1]) + float(data[2])
                min_pevol = pevol if min_pevol is None else min(min_pevol, pevol)
                max_pevol = pevol if max_pevol is None else max(max_pevol, pevol)
                ape = abs(pevol)
                max_abs_pevol = ape if max_abs_pevol is None else max(max_abs_pevol, ape)
    return max_comp_press, min_pevol, max_pevol, max_abs_pevol


def history_values(odb, var_name):
    vals = []
    for step in odb.steps.values():
        for region_name, region in step.historyRegions.items():
            if var_name in region.historyOutputs:
                for t, v in region.historyOutputs[var_name].data:
                    vals.append((region_name, t, float(v)))
    return vals


def history_final_max_abs(odb, var_name):
    vals = history_values(odb, var_name)
    if not vals:
        return None, None
    max_abs = max(abs(v[2]) for v in vals)
    final = vals[-1][2]
    return final, max_abs


def history_region_report(odb, var_names):
    lines = []
    for step in odb.steps.values():
        for region_name, region in step.historyRegions.items():
            for var in var_names:
                if var in region.historyOutputs:
                    data = region.historyOutputs[var].data
                    if data:
                        vals = [float(v[1]) for v in data]
                        lines.append((region_name, var, vals[-1],
                                      max(abs(v) for v in vals)))
    return lines


def parse_mass_increase():
    if not os.path.exists(STA_PATH):
        return None, None
    vals = []
    sci = re.compile(r"[-+]?\d+\.\d+E[-+]\d+")
    with open(STA_PATH, "r") as f:
        for line in f:
            parts = sci.findall(line)
            # Explicit status increment lines end with "PERCENT CHNG MASS".
            # Some compact status formats contain six scientific-notation
            # values rather than seven because the increment and critical
            # element columns are integers.
            if len(parts) >= 6:
                try:
                    value = float(parts[-1])
                except Exception:
                    continue
                if abs(value) > 1.0:
                    vals.append(value)
    if not vals:
        return None, None
    return vals[-1], max(vals)


def text_flags():
    flags = {
        "distortion": False,
        "negative_or_zero_volume": False,
        "zero_time_increment": False,
        "deformation_wave_ratio": False,
        "errors": False,
    }
    paths = [STA_PATH, DAT_PATH, MSG_PATH]
    for path in paths:
        if not os.path.exists(path):
            continue
        with open(path, "r") as f:
            text = f.read().lower()
        if "distort" in text:
            flags["distortion"] = True
        if "negative" in text and "volume" in text:
            flags["negative_or_zero_volume"] = True
        if "zero" in text and "time increment" in text:
            flags["zero_time_increment"] = True
        if "deformation speed to wave speed" in text:
            flags["deformation_wave_ratio"] = True
        if "***error" in text or "error in job" in text:
            flags["errors"] = True
    return flags


def compute_pair_state(top_node, bottom_node, u):
    tu = u.get(top_node.label, (0.0, 0.0, 0.0))
    bu = u.get(bottom_node.label, (0.0, 0.0, 0.0))
    tp = vec_add(top_node.coordinates, tu)
    bp = vec_add(bottom_node.coordinates, bu)
    return {
        "top": tp,
        "bottom": bp,
        "actual_t": dist(tp, bp),
        "vertical_t": tp[1] - bp[1],
        "mid_x": 0.5 * (tp[0] + bp[0]),
        "mid_z": 0.5 * (tp[2] + bp[2]),
    }


def choose_downstream_keys(final_states, final_pair_cpress):
    thresholds = [NIP_EXIT_X_MM + 0.05, NIP_EXIT_X_MM, 1.0, 0.8, 0.6, 0.4, 0.2]
    for threshold in thresholds:
        candidates = []
        for key, state in final_states.items():
            cp = final_pair_cpress.get(key, 0.0)
            if state["mid_x"] >= threshold and cp <= CPRESS_ZERO_TOL:
                candidates.append(key)
        if candidates:
            return candidates, threshold, True
    # If contact labels were not available or all downstream pairs still have a
    # tiny reported value, fall back to geometry only and record this in report.
    for threshold in thresholds:
        candidates = [key for key, state in final_states.items()
                      if state["mid_x"] >= threshold]
        if candidates:
            return candidates, threshold, False
    return list(final_states.keys()), -1.0, False


def stats(values):
    if not values:
        return None, None, None
    return sum(values) / len(values), min(values), max(values)


def try_write_plots(rows, profile_rows):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception:
        return False

    times = [row["time_s"] for row in rows]
    thickness = [row["selected_actual_avg_um"] for row in rows]
    plt.figure(figsize=(8.0, 4.8))
    plt.plot(times, thickness, "r-o", markersize=2.5, linewidth=1.0,
             label="simulation")
    plt.axhline(INITIAL_THICKNESS_MM * 1000.0, color="0.5",
                linestyle="--", linewidth=1.0, label="initial 150 um")
    plt.axhline(TARGET_THICKNESS_MM * 1000.0, color="0.2",
                linestyle="-.", linewidth=1.0, label="target 135 um")
    plt.xlabel("Frame time / s")
    plt.ylabel("Selected downstream thickness / um")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(TIME_PNG_PATH, dpi=180)
    plt.close()

    xs = [row[2] for row in profile_rows]
    ts = [row[3] for row in profile_rows]
    plt.figure(figsize=(8.0, 4.8))
    plt.scatter(xs, ts, s=8, c="tab:blue", alpha=0.75,
                label="final material-node pairs")
    plt.axhline(INITIAL_THICKNESS_MM * 1000.0, color="0.5",
                linestyle="--", linewidth=1.0, label="initial 150 um")
    plt.axhline(TARGET_THICKNESS_MM * 1000.0, color="0.2",
                linestyle="-.", linewidth=1.0, label="target 135 um")
    plt.xlabel("Final pair mid x / mm")
    plt.ylabel("Actual thickness / um")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(PROFILE_PNG_PATH, dpi=180)
    plt.close()
    return True


def main():
    odb = openOdb(ODB_PATH, readOnly=True)
    inst = find_active_instance(odb)
    inst_name = inst.name
    ymin, ymax, top, bottom, keys = top_bottom_pairs(inst)

    frame_records = []
    final_states = {}
    final_pair_cpress = {}
    max_global_cpress = 0.0
    max_pressure = None
    min_pevol = None
    max_pevol = None
    max_abs_pevol = None

    for step_name, step in odb.steps.items():
        for frame in step.frames:
            u = displacement_map(frame, inst.name)
            cpress_map, global_cp = active_cpress_map(frame, inst.name)
            max_global_cpress = max(max_global_cpress, global_cp)
            comp_press, frame_min_pevol, frame_max_pevol, frame_abs_pevol = (
                stress_pressure_stats(frame, inst.name))
            if comp_press is not None:
                max_pressure = comp_press if max_pressure is None else max(max_pressure, comp_press)
            if frame_min_pevol is not None:
                min_pevol = frame_min_pevol if min_pevol is None else min(min_pevol, frame_min_pevol)
                max_pevol = frame_max_pevol if max_pevol is None else max(max_pevol, frame_max_pevol)
                max_abs_pevol = frame_abs_pevol if max_abs_pevol is None else max(max_abs_pevol, frame_abs_pevol)

            states = {}
            pair_cp = {}
            for key in keys:
                state = compute_pair_state(top[key], bottom[key], u)
                states[key] = state
                pair_cp[key] = max(cpress_map.get(top[key].label, 0.0),
                                   cpress_map.get(bottom[key].label, 0.0))
            frame_records.append({
                "step": step_name,
                "frame": frame.frameId,
                "time": frame.frameValue,
                "states": states,
                "pair_cpress": pair_cp,
                "global_cpress": global_cp,
            })
            final_states = states
            final_pair_cpress = pair_cp

    selected_keys, threshold, contact_filtered = choose_downstream_keys(
        final_states, final_pair_cpress)

    rows = []
    selected_min_over_time = None
    selected_last5 = []
    for rec in frame_records:
        all_actual = [st["actual_t"] for st in rec["states"].values()]
        all_vertical = [st["vertical_t"] for st in rec["states"].values()]
        sel_actual = [rec["states"][key]["actual_t"] for key in selected_keys]
        sel_vertical = [rec["states"][key]["vertical_t"] for key in selected_keys]
        sel_cp = [rec["pair_cpress"].get(key, 0.0) for key in selected_keys]
        sel_midx = [rec["states"][key]["mid_x"] for key in selected_keys]
        aavg, amin, amax = stats(all_actual)
        vavg, vmin, vmax = stats(all_vertical)
        savg, smin, smax = stats(sel_actual)
        svavg, svmin, svmax = stats(sel_vertical)
        selected_min_over_time = smin if selected_min_over_time is None else min(selected_min_over_time, smin)
        rows.append({
            "step": rec["step"],
            "frame": rec["frame"],
            "time_s": rec["time"],
            "global_actual_avg_um": aavg * 1000.0,
            "global_actual_min_um": amin * 1000.0,
            "global_actual_max_um": amax * 1000.0,
            "global_vertical_avg_um": vavg * 1000.0,
            "selected_actual_avg_um": savg * 1000.0,
            "selected_actual_min_um": smin * 1000.0,
            "selected_actual_max_um": smax * 1000.0,
            "selected_vertical_avg_um": svavg * 1000.0,
            "selected_vertical_min_um": svmin * 1000.0,
            "selected_vertical_max_um": svmax * 1000.0,
            "selected_avg_mid_x_mm": sum(sel_midx) / len(sel_midx),
            "selected_max_cpress_mpa": max(sel_cp) if sel_cp else 0.0,
            "global_max_cpress_mpa": rec["global_cpress"],
        })
    selected_last5 = rows[-5:] if len(rows) >= 5 else rows

    final = rows[-1]
    final_selected_cp = final["selected_max_cpress_mpa"]
    final_avg = final["selected_actual_avg_um"]
    final_min = final["selected_actual_min_um"]
    final_max = final["selected_actual_max_um"]
    last5_vals = [row["selected_actual_avg_um"] for row in selected_last5]
    last5_mean = sum(last5_vals) / len(last5_vals)
    last5_span_pct = ((max(last5_vals) - min(last5_vals)) / last5_mean * 100.0
                      if last5_mean else None)

    allie_final, allie_max = history_final_max_abs(odb, "ALLIE")
    allke_final, allke_max = history_final_max_abs(odb, "ALLKE")
    allae_final, allae_max = history_final_max_abs(odb, "ALLAE")
    allmw_final, allmw_max = history_final_max_abs(odb, "ALLMW")
    allpd_final, allpd_max = history_final_max_abs(odb, "ALLPD")
    rf_report = history_region_report(odb, ("RF2", "U2", "UR3"))
    odb.close()

    mass_last, mass_max = parse_mass_increase()
    flags = text_flags()

    with open(CSV_PATH, "w", newline="") as f:
        fieldnames = [
            "step", "frame", "time_s",
            "global_actual_avg_um", "global_actual_min_um", "global_actual_max_um",
            "global_vertical_avg_um",
            "selected_actual_avg_um", "selected_actual_min_um", "selected_actual_max_um",
            "selected_vertical_avg_um", "selected_vertical_min_um", "selected_vertical_max_um",
            "selected_avg_mid_x_mm", "selected_max_cpress_mpa", "global_max_cpress_mpa"]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    with open(PAIR_CSV_PATH, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["initial_x_mm", "initial_z_mm", "final_mid_x_mm",
                         "final_actual_thickness_um", "final_vertical_thickness_um",
                         "final_pair_cpress_mpa"])
        for key in selected_keys:
            st = final_states[key]
            writer.writerow([key[0], key[1], st["mid_x"],
                             st["actual_t"] * 1000.0,
                             st["vertical_t"] * 1000.0,
                             final_pair_cpress.get(key, 0.0)])

    profile_rows = []
    with open(PROFILE_CSV_PATH, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["initial_x_mm", "initial_z_mm", "final_mid_x_mm",
                         "final_actual_thickness_um", "final_vertical_thickness_um",
                         "final_pair_cpress_mpa"])
        for key in keys:
            st = final_states[key]
            row = [key[0], key[1], st["mid_x"], st["actual_t"] * 1000.0,
                   st["vertical_t"] * 1000.0, final_pair_cpress.get(key, 0.0)]
            profile_rows.append(row)
            writer.writerow(row)

    plots_written = try_write_plots(rows, profile_rows)

    ke_ie_final = None
    ke_ie_max = None
    if allie_final not in (None, 0.0) and allke_final is not None:
        ke_ie_final = allke_final / allie_final
    if allie_max not in (None, 0.0) and allke_max is not None:
        ke_ie_max = allke_max / allie_max

    status = "not_target"
    if (136.0 <= final_avg <= 142.0 and final_selected_cp <= CPRESS_ZERO_TOL and
            last5_span_pct is not None and last5_span_pct <= 1.0):
        status = "candidate"
    if final_avg < 135.0 or flags["negative_or_zero_volume"] or flags["zero_time_increment"]:
        status = "too_soft_or_unstable"

    with open(SUMMARY_PATH, "w") as f:
        f.write("# 3D short steady-through DPC parameter verification\n\n")
        f.write("- Job: `%s`\n" % JOB)
        f.write("- Active instance: `%s`\n" % inst_name)
        f.write("- Paired top/bottom material points: %d\n" % len(keys))
        f.write("- Selected downstream pairs: %d\n" % len(selected_keys))
        f.write("- Downstream x threshold: %.6f mm\n" % threshold)
        f.write("- CPRESS filtered selection: %s\n" % str(contact_filtered))
        f.write("- Initial thickness: %.3f um\n" % (INITIAL_THICKNESS_MM * 1000.0))
        f.write("- Target thickness: %.3f um\n" % (TARGET_THICKNESS_MM * 1000.0))
        f.write("- Final downstream actual thickness avg/min/max: %.6f / %.6f / %.6f um\n" %
                (final_avg, final_min, final_max))
        f.write("- Final downstream vertical thickness avg: %.6f um\n" %
                final["selected_vertical_avg_um"])
        f.write("- Selected downstream minimum thickness during run: %.6f um\n" %
                (selected_min_over_time * 1000.0))
        f.write("- Last 5 output-frame downstream avg thickness span: %.6f %%\n" %
                last5_span_pct)
        f.write("- Final downstream CPRESS max: %.6e MPa\n" % final_selected_cp)
        f.write("- Final global CPRESS max: %.6e MPa\n" % final["global_max_cpress_mpa"])
        f.write("- Max global CPRESS during run: %.6e MPa\n" % max_global_cpress)
        f.write("- Max compressive hydrostatic pressure from S: %s MPa\n" %
                str(max_pressure))
        f.write("- Plastic volumetric strain PE11+PE22+PE33 min/max/maxabs: %s / %s / %s\n" %
                (str(min_pevol), str(max_pevol), str(max_abs_pevol)))
        f.write("- ALLIE final/maxabs: %s / %s\n" % (str(allie_final), str(allie_max)))
        f.write("- ALLKE final/maxabs: %s / %s\n" % (str(allke_final), str(allke_max)))
        f.write("- ALLKE/ALLIE final/maxabs-based: %s / %s\n" %
                (str(ke_ie_final), str(ke_ie_max)))
        f.write("- ALLAE final/maxabs: %s / %s\n" % (str(allae_final), str(allae_max)))
        f.write("- ALLMW final/maxabs: %s / %s\n" % (str(allmw_final), str(allmw_max)))
        f.write("- ALLPD final/maxabs: %s / %s\n" % (str(allpd_final), str(allpd_max)))
        f.write("- Percent mass increase last/max from STA: %s / %s\n" %
                (str(mass_last), str(mass_max)))
        f.write("- Text flags: %s\n" % str(flags))
        f.write("- Plots written: %s\n" % str(plots_written))
        f.write("- Auto status: `%s`\n\n" % status)
        f.write("## Roller and plate history outputs\n\n")
        for region, var, final_val, max_abs in rf_report:
            f.write("- %s, %s: final=%s, maxabs=%s\n" %
                    (region, var, str(final_val), str(max_abs)))

    print("Wrote %s" % CSV_PATH)
    print("Wrote %s" % PAIR_CSV_PATH)
    print("Wrote %s" % PROFILE_CSV_PATH)
    print("Wrote %s" % SUMMARY_PATH)
    if plots_written:
        print("Wrote %s" % TIME_PNG_PATH)
        print("Wrote %s" % PROFILE_PNG_PATH)
    print("Final downstream actual thickness avg/min/max: %.6f / %.6f / %.6f um" %
          (final_avg, final_min, final_max))
    print("Final downstream CPRESS max: %.6e MPa" % final_selected_cp)
    print("Last5 thickness span pct: %.6f" % last5_span_pct)
    print("Auto status: %s" % status)


if __name__ == "__main__":
    main()
