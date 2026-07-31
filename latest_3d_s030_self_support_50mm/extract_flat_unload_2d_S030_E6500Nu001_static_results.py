# -*- coding: utf-8 -*-
# 提取 Standard 静力平板压缩-卸载测试结果。

from odbAccess import openOdb
import csv
import os


WORKDIR = r"E:\abaqus\3Dnihe2.0\3Dnihe2.0"
JOB_NAME = "FlatUnload_2D_DPC_S030_E6500Nu001_StaticContactOff"
INSTANCE_NAME = "ACTIVELAYER-1"
CSV_NAME = "flat_unload_2d_S030_E6500Nu001_static_contact_off_thickness_history.csv"
REPORT_NAME = "flat_unload_2d_S030_E6500Nu001_static_contact_off_report.md"

INITIAL_THICKNESS_UM = 150.0
TARGET_THICKNESS_UM = 135.0
Y_TOL = 1.0e-7
CPRESS_ZERO_TOL = 1.0e-4
RF_ZERO_TOL = 1.0e-3


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
        key = round(x0, 7)
        if abs(y0 - y_min) <= Y_TOL:
            bottom[key] = y0 + float(u[1])
        elif abs(y0 - y_max) <= Y_TOL:
            top[key] = y0 + float(u[1])
    vals = []
    for key in sorted(set(top).intersection(bottom)):
        vals.append((top[key] - bottom[key]) * 1000.0)
    if not vals:
        raise RuntimeError("No paired top/bottom nodes found")
    return sum(vals) / len(vals), min(vals), max(vals), len(vals)


def max_cpress(frame):
    best = 0.0
    for name, field in frame.fieldOutputs.items():
        if name.strip().upper().startswith("CPRESS"):
            for value in field.values:
                best = max(best, abs(scalar(value.data)))
        elif name.strip().upper().startswith("CSTRESS"):
            for value in field.values:
                best = max(best, abs(scalar(value.data)))
    return best


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


def main():
    odb_path = os.path.join(WORKDIR, JOB_NAME + ".odb")
    odb = openOdb(odb_path, readOnly=True)
    try:
        inst = odb.rootAssembly.instances[INSTANCE_NAME]
        rf2_lookup = build_lookup(history_series(odb, "TOPPLATE", "RF2"))
        u2_lookup = build_lookup(history_series(odb, "TOPPLATE", "U2"))

        rows = []
        zero_unload_rows = []
        min_row = None
        max_press = None
        pe_min = None
        pe_max = None
        pe_abs = None

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
                    pe_min = pmin if pe_min is None else min(pe_min, pmin)
                    pe_max = pmax if pe_max is None else max(pe_max, pmax)
                    pe_abs = pabs if pe_abs is None else max(pe_abs, pabs)
                row = {
                    "step": step_name,
                    "frame_index": idx,
                    "step_time": frame.frameValue,
                    "total_time": total_offset + frame.frameValue,
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
                    "UNLOAD" in step_name.upper() and
                    (rf2 is None or abs(rf2) <= RF_ZERO_TOL) and
                    cp <= CPRESS_ZERO_TOL)
                rows.append(row)
                if row["unloaded_zero_contact"]:
                    zero_unload_rows.append(row)
                if min_row is None or min_t < min_row["min_thickness_um"]:
                    min_row = row
            if step.frames:
                total_offset += step.frames[-1].frameValue

        residual = zero_unload_rows[-1] if zero_unload_rows else rows[-1]
        residual_basis = ("last unloaded frame with near-zero RF2 and CPRESS"
                          if zero_unload_rows else
                          "no strict zero-contact unloaded frame; last converged frame used")

        csv_path = os.path.join(WORKDIR, CSV_NAME)
        with open(csv_path, "w") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()),
                                    lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)

        status = "not_ok"
        if zero_unload_rows and 135.0 <= residual["avg_thickness_um"] <= 142.0:
            status = "target_with_small_rebound"
        elif zero_unload_rows and residual["avg_thickness_um"] < 135.0:
            status = "over_compressed_or_too_soft"
        elif residual["avg_thickness_um"] > 145.0:
            status = "mostly_elastic_rebound_or_no_plastic_compaction"

        report_path = os.path.join(WORKDIR, REPORT_NAME)
        with open(report_path, "w") as f:
            f.write("# Static flat compression-unload check: DPC S=0.3, E=6500 MPa, nu=0.01\n\n")
            f.write("- ODB: `%s.odb`\n" % JOB_NAME)
            f.write("- Initial thickness: %.6f um\n" % INITIAL_THICKNESS_UM)
            f.write("- Target compressed thickness: %.6f um\n" % TARGET_THICKNESS_UM)
            f.write("- Residual basis: %s\n\n" % residual_basis)
            f.write("## Thickness result\n\n")
            f.write("- Minimum thickness during compression: %.6f um at %s t=%.6g\n" %
                    (min_row["min_thickness_um"], min_row["step"], min_row["step_time"]))
            f.write("- Residual/last average thickness: %.6f um\n" %
                    residual["avg_thickness_um"])
            f.write("- Residual/last min/max thickness: %.6f / %.6f um\n" %
                    (residual["min_thickness_um"], residual["max_thickness_um"]))
            f.write("- Top plate U2 at residual/last: %s mm\n" %
                    str(residual["top_u2_mm"]))
            f.write("- Top plate RF2 at residual/last: %s N\n" %
                    str(residual["top_rf2_N"]))
            f.write("- Max CPRESS at residual/last: %.6e MPa\n" %
                    residual["max_cpress_MPa"])
            f.write("- Auto status: `%s`\n\n" % status)
            f.write("## Plasticity\n\n")
            f.write("- Max compressive hydrostatic PRESS from S: %s MPa\n" %
                    str(max_press))
            f.write("- PE11+PE22+PE33 min/max/maxabs: %s / %s / %s\n" %
                    (str(pe_min), str(pe_max), str(pe_abs)))

        print("Wrote %s" % csv_path)
        print("Wrote %s" % report_path)
        print("Minimum thickness during compression: %.6f um" %
              min_row["min_thickness_um"])
        print("Residual/last average thickness: %.6f um" %
              residual["avg_thickness_um"])
        print("Residual basis: %s" % residual_basis)
        print("Status: %s" % status)
    finally:
        odb.close()


if __name__ == "__main__":
    main()
