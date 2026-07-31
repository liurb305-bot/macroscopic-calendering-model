# -*- coding: utf-8 -*-
"""提取两层极片平板压缩—卸载的逐帧厚度曲线。"""

from __future__ import print_function

import csv
import os

from odbAccess import openOdb

import postprocess_two_layer_electrode_compression as common


WORK_DIR = r"E:\abaqus\3Dyang"
ODB_PATH = os.path.join(WORK_DIR, "Yang_Macro_TwoLayerElectrode_Compression.odb")
CSV_PATH = os.path.join(WORK_DIR, "Yang_Macro_TwoLayerElectrode_Compression_thickness_curve.csv")


def interpolate_history(data, target_time):
    points = [(float(time), float(value)) for time, value in data]
    if target_time <= points[0][0]:
        return points[0][1]
    if target_time >= points[-1][0]:
        return points[-1][1]
    for index in range(1, len(points)):
        t0, v0 = points[index - 1]
        t1, v1 = points[index]
        if t0 <= target_time <= t1:
            fraction = (target_time - t0) / (t1 - t0)
            return v0 + fraction * (v1 - v0)
    return points[-1][1]


def main():
    odb = openOdb(path=ODB_PATH, readOnly=True)
    try:
        active = odb.rootAssembly.instances[common.ACTIVE_INSTANCE_NAME]
        aluminum = odb.rootAssembly.instances[common.AL_INSTANCE_NAME]
        sets = {
            "active_top": active.nodeSets[common.ACTIVE_TOP_NODE_SET],
            "active_interface": active.nodeSets[common.ACTIVE_INTERFACE_NODE_SET],
            "al_interface": aluminum.nodeSets[common.AL_INTERFACE_NODE_SET],
            "al_bottom": aluminum.nodeSets[common.AL_BOTTOM_NODE_SET],
        }
        initial = {
            name: common.initial_y_map(node_set) for name, node_set in sets.items()
        }
        rows = []
        offset = 0.0
        for step_index, step_name in enumerate((
            common.STEP_COMPRESSION, common.STEP_UNLOAD,
        )):
            step = odb.steps[step_name]
            region = common.find_history_region(step, ("U2", "RF2"), "上平板RP")
            u2_history = region.historyOutputs["U2"].data
            for frame_index, frame in enumerate(step.frames):
                current_y = {
                    name: common.average_current_y(frame, sets[name], initial[name])
                    for name in sets
                }
                total = current_y["active_top"] - current_y["al_bottom"]
                active_thickness = (
                    current_y["active_top"] - current_y["active_interface"]
                )
                collector_thickness = (
                    current_y["al_interface"] - current_y["al_bottom"]
                )
                rows.append({
                    "step_name": step_name,
                    "stage": "COMPRESSION" if step_index == 0 else "UNLOAD",
                    "frame_index": frame_index,
                    "step_time": float(frame.frameValue),
                    "process_coordinate": offset + float(frame.frameValue),
                    "top_plate_u2_mm": interpolate_history(
                        u2_history, float(frame.frameValue)
                    ),
                    "total_thickness_mm": total,
                    "active_layer_thickness_mm": active_thickness,
                    "collector_thickness_mm": collector_thickness,
                })
            offset += float(step.timePeriod)
        columns = (
            "step_name", "stage", "frame_index", "step_time",
            "process_coordinate", "top_plate_u2_mm", "total_thickness_mm",
            "active_layer_thickness_mm", "collector_thickness_mm",
        )
        with open(CSV_PATH, "w", encoding="utf-8-sig", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=columns)
            writer.writeheader()
            writer.writerows(rows)
        compression = [row for row in rows if row["stage"] == "COMPRESSION"]
        unload = [row for row in rows if row["stage"] == "UNLOAD"]
        print("initial={:.9g} mm".format(rows[0]["total_thickness_mm"]))
        print("minimum={:.9g} mm".format(min(row["total_thickness_mm"] for row in compression)))
        print("residual={:.9g} mm".format(unload[-1]["total_thickness_mm"]))
        print(CSV_PATH)
    finally:
        odb.close()


if __name__ == "__main__":
    main()
