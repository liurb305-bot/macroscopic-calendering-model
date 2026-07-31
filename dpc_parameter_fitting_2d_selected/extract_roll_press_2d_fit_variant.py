# -*- coding: ascii -*-
"""
Extract valid post-nip thickness from a 2D roll-press fitting variant.

Usage:
    abaqus python extract_roll_press_2d_fit_variant.py JOB RESULT_PREFIX
"""

from odbAccess import openOdb
import csv
import os
import sys


WORKDIR = r"E:\abaqus\2D3.1"
INSTANCE_NAME = "ACTIVELAYER-1"
INITIAL_THICKNESS_UM = 150.0
DESIRED_FINAL_THICKNESS_UM = 135.0
NOMINAL_ROLL_GAP_UM = 135.0
ROLLER_CONTACT_ARC_HALF_WIDTH = 4.0
EXIT_MARGIN = 0.25
VALID_INITIAL_X_MAX = -2.35
VALID_INITIAL_X_MIN = -2.975
Y_TOL = 1.0e-7


def last_frame_with_data(odb):
    for step_name in reversed(list(odb.steps.keys())):
        step = odb.steps[step_name]
        if step.frames:
            return step_name, step.frames[-1]
    raise RuntimeError("No frames found")


def collect_history(odb):
    final = {}
    max_abs = {}
    for step in odb.steps.values():
        for region in step.historyRegions.values():
            for key, output in region.historyOutputs.items():
                if output.data:
                    vals = [float(v[1]) for v in output.data]
                    final[key] = vals[-1]
                    max_abs[key] = max(abs(v) for v in vals)
    return final, max_abs


def main():
    if len(sys.argv) < 3:
        raise RuntimeError("Expected arguments: JOB RESULT_PREFIX")
    job = sys.argv[1]
    prefix = sys.argv[2]
    odb_name = job + ".odb"

    odb = openOdb(os.path.join(WORKDIR, odb_name), readOnly=True)
    try:
        inst = odb.rootAssembly.instances[INSTANCE_NAME]
        step_name, frame = last_frame_with_data(odb)
        u_field = frame.fieldOutputs["U"].getSubset(region=inst)
        disp = {v.nodeLabel: v.data for v in u_field.values}

        nodes = list(inst.nodes)
        y_min = min(n.coordinates[1] for n in nodes)
        y_max = max(n.coordinates[1] for n in nodes)
        top = {}
        bottom = {}
        for node in nodes:
            x0, y0 = node.coordinates[0], node.coordinates[1]
            ux, uy = disp.get(node.label, (0.0, 0.0))[:2]
            key = round(x0, 7)
            if abs(y0 - y_min) <= Y_TOL:
                bottom[key] = (x0 + ux, y0 + uy)
            elif abs(y0 - y_max) <= Y_TOL:
                top[key] = (x0 + ux, y0 + uy)

        rows = []
        valid = []
        for x0 in sorted(set(top).intersection(bottom)):
            xt, yt = top[x0]
            xb, yb = bottom[x0]
            final_mid_x = 0.5 * (xt + xb)
            t_um = (yt - yb) * 1000.0
            exited = final_mid_x > ROLLER_CONTACT_ARC_HALF_WIDTH + EXIT_MARGIN
            entered_after_target_gap = x0 <= VALID_INITIAL_X_MAX
            away_from_back_edge = x0 >= VALID_INITIAL_X_MIN
            is_valid = exited and entered_after_target_gap and away_from_back_edge
            row = (x0, final_mid_x, INITIAL_THICKNESS_UM,
                   NOMINAL_ROLL_GAP_UM, DESIRED_FINAL_THICKNESS_UM,
                   t_um, int(entered_after_target_gap), int(exited),
                   int(away_from_back_edge), int(is_valid))
            rows.append(row)
            if is_valid:
                valid.append(t_um)

        if not valid:
            raise RuntimeError("No valid post-nip material columns found")

        avg_v = sum(valid) / len(valid)
        min_v = min(valid)
        max_v = max(valid)
        span_v = max_v - min_v

        hist, hist_max_abs = collect_history(odb)
        ke_ie = ""
        if abs(hist.get("ALLIE", 0.0)) > 0.0:
            ke_ie = abs(hist.get("ALLKE", 0.0) / hist["ALLIE"])

        csv_path = os.path.join(WORKDIR, prefix + "_valid_thickness.csv")
        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["initial_x_mm", "final_mid_x_mm",
                             "initial_thickness_um", "nominal_roll_gap_um",
                             "desired_final_thickness_um",
                             "simulated_final_thickness_um",
                             "entered_after_target_gap",
                             "exited_nip_region",
                             "away_from_back_edge",
                             "valid_post_nip_column"])
            writer.writerows(rows)

        summary_path = os.path.join(WORKDIR, prefix + "_valid_summary.txt")
        with open(summary_path, "w") as f:
            f.write("2D DPC roll fitting valid post-nip thickness result\n")
            f.write("ODB: %s\n" % odb_name)
            f.write("Final frame: step=%s frameValue=%.8g\n" %
                    (step_name, frame.frameValue))
            f.write("Initial thickness: %.6f um\n" % INITIAL_THICKNESS_UM)
            f.write("Nominal roll gap: %.6f um\n" % NOMINAL_ROLL_GAP_UM)
            f.write("Desired final thickness: %.6f um\n" %
                    DESIRED_FINAL_THICKNESS_UM)
            f.write("Valid post-nip column count: %d\n" % len(valid))
            f.write("Valid post-nip average thickness: %.6f um\n" % avg_v)
            f.write("Valid post-nip min thickness: %.6f um\n" % min_v)
            f.write("Valid post-nip max thickness: %.6f um\n" % max_v)
            f.write("Valid post-nip thickness span: %.6f um\n" % span_v)
            f.write("Reached desired final thickness: %s\n" %
                    ("YES" if avg_v <= DESIRED_FINAL_THICKNESS_UM + 0.5
                     else "NO"))
            if ke_ie != "":
                f.write("Final ALLKE/ALLIE: %.8g\n" % ke_ie)
            for key in sorted(hist):
                if key.startswith("ALL"):
                    f.write("%s: %.8g\n" % (key, hist[key]))
            for key in sorted(hist_max_abs):
                if key.startswith("ALL"):
                    f.write("%s_max_abs: %.8g\n" %
                            (key, hist_max_abs[key]))
        print(summary_path)
        print("avg=%.6f min=%.6f max=%.6f span=%.6f count=%d" %
              (avg_v, min_v, max_v, span_v, len(valid)))
    finally:
        odb.close()


if __name__ == "__main__":
    main()
