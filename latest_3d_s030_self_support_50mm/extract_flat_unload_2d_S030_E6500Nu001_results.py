# -*- coding: utf-8 -*-
# 提取 2D 平板压缩-卸载测试结果：
# 厚度历史、压缩阶段最小厚度、卸载零接触后的残余厚度、塑性体积应变与能量。

from odbAccess import openOdb
import csv
import os


WORKDIR = r"E:\abaqus\3Dnihe2.0\3Dnihe2.0"
JOB_NAME = "FlatUnload_2D_DPC_S030_E6500Nu001_FreeUnload"
ODB_NAME = JOB_NAME + ".odb"
INSTANCE_NAME = "ACTIVELAYER-1"
CSV_NAME = "flat_unload_2d_S030_E6500Nu001_free_unload_thickness_history.csv"
REPORT_NAME = "flat_unload_2d_S030_E6500Nu001_free_unload_report.md"

INITIAL_THICKNESS_UM = 150.0
TARGET_THICKNESS_UM = 135.0
Y_TOL = 1.0e-7
ZERO_TOL = 1.0e-4


def scalar(data):
    try:
        return float(data)
    except Exception:
        try:
            return float(data[0])
        except Exception:
            return 0.0


def history_series(odb, region_fragment, variable):
    out = []
    for step_name, step in odb.steps.items():
        for region_name, region in step.historyRegions.items():
            if region_fragment.upper() in region_name.upper():
                if variable in region.historyOutputs:
                    out.extend((step_name, t, float(v))
                               for t, v in region.historyOutputs[variable].data)
    return out


def build_lookup(series):
    lookup = {}
    for step_name, t, v in series:
        lookup.setdefault(step_name, []).append((t, v))
    return lookup


def nearest(lookup, step_name, t):
    vals = lookup.get(step_name, [])
    if not vals:
        return None
    return min(vals, key=lambda item: abs(item[0] - t))[1]


def all_history_latest(odb):
    final = {}
    maxabs = {}
    for step in odb.steps.values():
        for region in step.historyRegions.values():
            for key, output in region.historyOutputs.items():
                if output.data:
                    vals = [float(v[1]) for v in output.data]
                    final[key] = vals[-1]
                    maxabs[key] = max(abs(v) for v in vals)
    return final, maxabs


def max_cpress(frame):
    best = 0.0
    for name, field in frame.fieldOutputs.items():
        if name.strip().startswith("CPRESS") or name.strip().startswith("CSTRESS"):
            for value in field.values:
                best = max(best, abs(scalar(value.data)))
    return best


def pevol_stats(frame, inst):
    if "PE" not in frame.fieldOutputs:
        return None, None, None
    vals = []
    for value in frame.fieldOutputs["PE"].getSubset(region=inst).values:
        data = value.data
        if len(data) >= 3:
            vals.append(float(data[0]) + float(data[1]) + float(data[2]))
    if not vals:
        return None, None, None
    return min(vals), max(vals), max(abs(v) for v in vals)


def pressure_stats(frame, inst):
    if "S" not in frame.fieldOutputs:
        return None
    best = None
    for value in frame.fieldOutputs["S"].getSubset(region=inst).values:
        data = value.data
        if len(data) >= 3:
            press = -(float(data[0]) + float(data[1]) + float(data[2])) / 3.0
            best = press if best is None else max(best, press)
    return best


def thickness_for_frame(inst, frame):
    disp = {}
    if "U" in frame.fieldOutputs:
        for value in frame.fieldOutputs["U"].getSubset(region=inst).values:
            disp[value.nodeLabel] = value.data
    nodes = list(inst.nodes)
    y_min = min(node.coordinates[1] for node in nodes)
    y_max = max(node.coordinates[1] for node in nodes)
    top = {}
    bottom = {}
    for node in nodes:
        x0, y0 = node.coordinates[0], node.coordinates[1]
        u = disp.get(node.label, (0.0, 0.0))
        uy = float(u[1])
        key = round(x0, 7)
        if abs(y0 - y_min) <= Y_TOL:
            bottom[key] = y0 + uy
        elif abs(y0 - y_max) <= Y_TOL:
            top[key] = y0 + uy
    vals = []
    for key in sorted(set(top).intersection(bottom)):
        vals.append((top[key] - bottom[key]) * 1000.0)
    if not vals:
        raise RuntimeError("No paired top/bottom nodes found")
    return sum(vals) / len(vals), min(vals), max(vals), len(vals)


def main():
    odb = openOdb(os.path.join(WORKDIR, ODB_NAME), readOnly=True)
    try:
        inst = odb.rootAssembly.instances[INSTANCE_NAME]
        rf2_lookup = build_lookup(history_series(odb, "TOPPLATE", "RF2"))
        u2_lookup = build_lookup(history_series(odb, "TOPPLATE", "U2"))
        rows = []
        zero_unload_rows = []
        min_thickness_row = None
        max_press = None
        pevol_min = None
        pevol_max = None
        pevol_abs = None

        total_offset = 0.0
        for step_name, step in odb.steps.items():
            for idx, frame in enumerate(step.frames):
                avg_t, min_t, max_t, ncols = thickness_for_frame(inst, frame)
                rf2 = nearest(rf2_lookup, step_name, frame.frameValue)
                u2 = nearest(u2_lookup, step_name, frame.frameValue)
                cp = max_cpress(frame)
                press = pressure_stats(frame, inst)
                pmin, pmax, pabs = pevol_stats(frame, inst)
                if press is not None:
                    max_press = press if max_press is None else max(max_press, press)
                if pmin is not None:
                    pevol_min = pmin if pevol_min is None else min(pevol_min, pmin)
                    pevol_max = pmax if pevol_max is None else max(pevol_max, pmax)
                    pevol_abs = pabs if pevol_abs is None else max(pevol_abs, pabs)
                row = {
                    "step": step_name,
                    "frame_index": idx,
                    "step_time_s": frame.frameValue,
                    "total_time_s": total_offset + frame.frameValue,
                    "top_u2_mm": u2,
                    "top_rf2_N": rf2,
                    "max_cpress_MPa": cp,
                    "max_compressive_PRESS_MPa": press,
                    "pevol_min": pmin,
                    "pevol_max": pmax,
                    "pevol_max_abs": pabs,
                    "avg_thickness_um": avg_t,
                    "min_thickness_um": min_t,
                    "max_thickness_um": max_t,
                    "column_count": ncols,
                }
                row["unloaded_zero_contact"] = (
                    step_name in ("Unload", "Free_Settle") and
                    (rf2 is None or abs(rf2) <= 1.0e-3) and
                    cp <= ZERO_TOL)
                rows.append(row)
                if row["unloaded_zero_contact"]:
                    zero_unload_rows.append(row)
                if min_thickness_row is None or min_t < min_thickness_row["min_thickness_um"]:
                    min_thickness_row = row
            if step.frames:
                total_offset += step.frames[-1].frameValue

        residual = zero_unload_rows[-1] if zero_unload_rows else rows[-1]
        residual_basis = ("last unloaded frame with near-zero RF2 and CPRESS"
                          if zero_unload_rows else
                          "no strict zero-contact frame; final frame used")

        final_hist, max_hist = all_history_latest(odb)
        ke_ie = None
        if abs(final_hist.get("ALLIE", 0.0)) > 0.0:
            ke_ie = abs(final_hist.get("ALLKE", 0.0) / final_hist["ALLIE"])

        csv_path = os.path.join(WORKDIR, CSV_NAME)
        with open(csv_path, "w") as f:
            fieldnames = [
                "step", "frame_index", "step_time_s", "total_time_s",
                "top_u2_mm", "top_rf2_N", "max_cpress_MPa",
                "max_compressive_PRESS_MPa", "pevol_min", "pevol_max",
                "pevol_max_abs", "avg_thickness_um", "min_thickness_um",
                "max_thickness_um", "column_count", "unloaded_zero_contact"]
            writer = csv.DictWriter(f, fieldnames=fieldnames, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)

        status = "not_ok"
        if 135.0 <= residual["avg_thickness_um"] <= 142.0:
            status = "target_with_small_rebound"
        elif residual["avg_thickness_um"] < 135.0:
            status = "over_compressed_or_too_soft"
        elif residual["avg_thickness_um"] > 145.0:
            status = "mostly_elastic_rebound_or_no_plastic_compaction"

        report_path = os.path.join(WORKDIR, REPORT_NAME)
        with open(report_path, "w") as f:
            f.write("# Flat compression-unload check: DPC S=0.3, E=6500 MPa, nu=0.01\n\n")
            f.write("- ODB: `%s`\n" % ODB_NAME)
            f.write("- Initial thickness: %.6f um\n" % INITIAL_THICKNESS_UM)
            f.write("- Compressed target thickness: %.6f um\n" % TARGET_THICKNESS_UM)
            f.write("- Residual basis: %s\n\n" % residual_basis)
            f.write("## Thickness result\n\n")
            f.write("- Minimum thickness during compression: %.6f um at %s t=%.6g s\n" %
                    (min_thickness_row["min_thickness_um"],
                     min_thickness_row["step"],
                     min_thickness_row["step_time_s"]))
            f.write("- Residual average thickness: %.6f um\n" %
                    residual["avg_thickness_um"])
            f.write("- Residual min/max thickness: %.6f / %.6f um\n" %
                    (residual["min_thickness_um"], residual["max_thickness_um"]))
            f.write("- Top plate U2 at residual: %s mm\n" % residual["top_u2_mm"])
            f.write("- Top plate RF2 at residual: %s N\n" % residual["top_rf2_N"])
            f.write("- Max CPRESS at residual: %.6e MPa\n" %
                    residual["max_cpress_MPa"])
            f.write("- Auto status: `%s`\n\n" % status)
            f.write("## Plasticity and energy\n\n")
            f.write("- Max compressive hydrostatic PRESS from S: %s MPa\n" %
                    str(max_press))
            f.write("- PE11+PE22+PE33 min/max/maxabs: %s / %s / %s\n" %
                    (str(pevol_min), str(pevol_max), str(pevol_abs)))
            f.write("- ALLPD final/maxabs: %.8g / %.8g\n" %
                    (final_hist.get("ALLPD", 0.0), max_hist.get("ALLPD", 0.0)))
            f.write("- ALLIE final/maxabs: %.8g / %.8g\n" %
                    (final_hist.get("ALLIE", 0.0), max_hist.get("ALLIE", 0.0)))
            f.write("- ALLKE final/maxabs: %.8g / %.8g\n" %
                    (final_hist.get("ALLKE", 0.0), max_hist.get("ALLKE", 0.0)))
            if ke_ie is not None:
                f.write("- Final ALLKE/ALLIE: %.8g\n" % ke_ie)

        print("Wrote %s" % csv_path)
        print("Wrote %s" % report_path)
        print("Minimum thickness during compression: %.6f um" %
              min_thickness_row["min_thickness_um"])
        print("Residual average thickness: %.6f um" %
              residual["avg_thickness_um"])
        print("Status: %s" % status)
    finally:
        odb.close()


if __name__ == "__main__":
    main()
