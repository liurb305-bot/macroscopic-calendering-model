"""Postprocess the wide_roll_1_8 Abaqus ODB.

Outputs:
  - wide_roll_1_8_thickness.csv
  - wide_roll_1_8_summary.txt
  - optional PNG contour plots when run with Abaqus/CAE noGUI
"""

from __future__ import print_function

import csv
import os

from abaqusConstants import CONTOURS_ON_DEF, FEATURE, INTEGRATION_POINT, INVARIANT, PNG
from odbAccess import openOdb


JOB_NAME = "wide_roll_1_8"
ODB_PATH = JOB_NAME + ".odb"
THICKNESS_CSV = JOB_NAME + "_thickness.csv"
SUMMARY_TXT = JOB_NAME + "_summary.txt"
CONTACT_CSV = JOB_NAME + "_contact_pressure.csv"


def flatten_nodes(node_container):
    out = []
    for item in node_container:
        try:
            for node in item:
                out.append(node)
        except TypeError:
            out.append(item)
    return out


def field_map_by_label(field_values):
    data = {}
    for value in field_values:
        data[value.nodeLabel] = value.data
    return data


def field_by_prefix(frame, prefix):
    for key in frame.fieldOutputs.keys():
        if key.strip().startswith(prefix):
            return frame.fieldOutputs[key]
    return None


def write_thickness_and_summary(odb):
    assembly = odb.rootAssembly
    step = odb.steps["ROLL_DOWN_20PCT"]
    frame = step.frames[-1]

    thickness_set = assembly.nodeSets["THICKNESS_LINE"]
    thickness_nodes = flatten_nodes(thickness_set.nodes)
    disp = frame.fieldOutputs["U"].getSubset(region=thickness_set)
    disp_by_label = field_map_by_label(disp.values)

    rows = []
    for node in thickness_nodes:
        x, y, z = node.coordinates
        u2 = disp_by_label.get(node.label, (0.0, 0.0, 0.0))[1]
        half_y = y + u2
        thickness_mm = 2.0 * half_y
        rows.append((x, z, y, u2, thickness_mm, thickness_mm * 1000.0))
    rows.sort(key=lambda row: row[0])

    with open(THICKNESS_CSV, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "x_mm",
                "z_mm",
                "initial_top_y_mm",
                "u2_mm",
                "full_thickness_mm",
                "full_thickness_um",
            ]
        )
        writer.writerows(rows)

    roll_set = assembly.nodeSets["ROLL_TOP"]
    rf2_total = None
    if "RF" in frame.fieldOutputs:
        rf = frame.fieldOutputs["RF"].getSubset(region=roll_set)
        rf2_total = 0.0
        for value in rf.values:
            rf2_total += value.data[1]

    cpress_max = None
    cpress_count = 0
    cpress_field = field_by_prefix(frame, "CPRESS")
    if cpress_field is not None:
        cpress = cpress_field
        vals = [value.data for value in cpress.values if value.data is not None]
        vals = [float(v) if not hasattr(v, "__len__") else float(v[0]) for v in vals]
        if vals:
            cpress_max = max(vals)
            cpress_count = len(vals)
    else:
        cstress = field_by_prefix(frame, "CSTRESS")
        if cstress is not None:
            vals = []
            for value in cstress.values:
                data = value.data
                if data is None:
                    continue
                if hasattr(data, "__len__"):
                    if len(data):
                        vals.append(float(data[0]))
                else:
                    vals.append(float(data))
            if vals:
                cpress_max = max(vals)
                cpress_count = len(vals)

    if cpress_field is not None:
        instances = {}
        for inst_name, inst in assembly.instances.items():
            coord_by_label = {}
            for node in inst.nodes:
                coord_by_label[node.label] = node.coordinates
            instances[inst_name] = coord_by_label
        with open(CONTACT_CSV, "w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["node_label", "x_mm", "y_mm", "z_mm", "cpress_mpa"])
            for value in cpress_field.values:
                inst_name = value.instance.name
                coords = instances.get(inst_name, {}).get(value.nodeLabel)
                if coords is None:
                    continue
                data = value.data
                if hasattr(data, "__len__"):
                    if not len(data):
                        continue
                    pressure = float(data[0])
                else:
                    pressure = float(data)
                writer.writerow([value.nodeLabel, coords[0], coords[1], coords[2], pressure])
    elif field_by_prefix(frame, "CSTRESS") is not None:
        cstress = field_by_prefix(frame, "CSTRESS")
        vals = []
        for value in cstress.values:
            data = value.data
            if data is None:
                continue
            if hasattr(data, "__len__"):
                if len(data):
                    vals.append(float(data[0]))
            else:
                vals.append(float(data))
        if vals:
            cpress_max = max(vals)
            cpress_count = len(vals)

    thickness_values = [row[5] for row in rows]
    summary_lines = []
    summary_lines.append("ODB: %s" % os.path.abspath(ODB_PATH))
    summary_lines.append("Step frames: %d" % len(step.frames))
    summary_lines.append("Final step time: %.8g" % frame.frameValue)
    summary_lines.append("Thickness samples: %d" % len(rows))
    if thickness_values:
        summary_lines.append("Thickness min/max/mean (um): %.6f / %.6f / %.6f" % (
            min(thickness_values),
            max(thickness_values),
            sum(thickness_values) / len(thickness_values),
        ))
    if rf2_total is not None:
        summary_lines.append("Total RF2 on ROLL_TOP (N): %.6f" % rf2_total)
    if cpress_max is not None:
        summary_lines.append("CPRESS max (MPa): %.6f from %d values" % (cpress_max, cpress_count))
    else:
        summary_lines.append("CPRESS max (MPa): unavailable")

    with open(SUMMARY_TXT, "w") as handle:
        handle.write("\n".join(summary_lines) + "\n")

    print("\n".join(summary_lines))


def make_png_plots():
    try:
        from abaqus import session
        import visualization  # noqa: F401
    except Exception as exc:
        print("PNG plots skipped: session unavailable: %s" % exc)
        return

    odb_path = os.path.abspath(ODB_PATH)
    try:
        try:
            odb = session.odbs[odb_path]
        except Exception:
            odb = session.openOdb(name=odb_path)

        viewport = session.Viewport(name="wide_roll_1_8_post", origin=(0, 0), width=170, height=115)
        viewport.setValues(displayedObject=odb)
        viewport.odbDisplay.display.setValues(plotState=(CONTOURS_ON_DEF,))
        viewport.odbDisplay.commonOptions.setValues(visibleEdges=FEATURE)
        viewport.view.fitView()
    except Exception as exc:
        print("PNG plots skipped: viewport/ODB display failed: %s" % exc)
        return

    plot_specs = [
        ("S", INTEGRATION_POINT, (INVARIANT, "Mises"), JOB_NAME + "_mises"),
        ("PEEQ", INTEGRATION_POINT, None, JOB_NAME + "_peeq"),
    ]
    for variable, position, refinement, out_name in plot_specs:
        try:
            if refinement:
                viewport.odbDisplay.setPrimaryVariable(
                    variableLabel=variable,
                    outputPosition=position,
                    refinement=refinement,
                )
            else:
                viewport.odbDisplay.setPrimaryVariable(variableLabel=variable, outputPosition=position)
            viewport.view.fitView()
            session.printToFile(fileName=out_name, format=PNG, canvasObjects=(viewport,))
            print("Wrote %s.png" % out_name)
        except Exception as exc:
            print("Plot %s skipped: %s" % (out_name, exc))

    try:
        viewport.odbDisplay.setPrimaryVariable(variableLabel="CPRESS")
        viewport.view.fitView()
        session.printToFile(fileName=JOB_NAME + "_cpress", format=PNG, canvasObjects=(viewport,))
        print("Wrote %s_cpress.png" % JOB_NAME)
    except Exception as exc:
        print("CPRESS plot skipped: %s" % exc)


def main():
    odb_path = os.path.abspath(ODB_PATH)
    if not os.path.exists(odb_path):
        raise RuntimeError("ODB not found: %s" % odb_path)
    odb = openOdb(path=odb_path, readOnly=True)
    try:
        write_thickness_and_summary(odb)
    finally:
        odb.close()
    make_png_plots()


if __name__ == "__main__":
    main()
