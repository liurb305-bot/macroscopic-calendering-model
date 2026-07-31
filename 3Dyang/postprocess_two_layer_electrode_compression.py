# -*- coding: utf-8 -*-
"""后处理两层极片压缩ODB并导出CSV。"""

from __future__ import print_function

import csv
import os
import sys

from odbAccess import openOdb


WORK_DIR = r"E:\abaqus\3Dyang"
JOB_NAME = "Yang_Macro_TwoLayerElectrode_Compression"
ODB_PATH = os.path.join(WORK_DIR, JOB_NAME + ".odb")
STA_PATH = os.path.join(WORK_DIR, JOB_NAME + ".sta")
CSV_PATH = os.path.join(WORK_DIR, JOB_NAME + "_results.csv")

X_LENGTH = 1.0
Z_WIDTH = 1.0
TOTAL_THICKNESS = 0.150
COMPRESSION_AREA = X_LENGTH * Z_WIDTH
STEP_COMPRESSION = "Step-1 Compression"
STEP_UNLOAD = "Step-2 Unload"
ACTIVE_INSTANCE_NAME = "ACTIVE_LAYER-1"
AL_INSTANCE_NAME = "AL_COLLECTOR-1"
ACTIVE_ELEMENT_SET = "ACTIVE_LAYER_ALL"
AL_ELEMENT_SET = "AL_COLLECTOR_ALL"
ACTIVE_TOP_NODE_SET = "ACTIVE_TOP_NODES"
ACTIVE_INTERFACE_NODE_SET = "ACTIVE_INTERFACE_NODES"
AL_INTERFACE_NODE_SET = "AL_INTERFACE_NODES"
AL_BOTTOM_NODE_SET = "AL_BOTTOM_NODES"

THICKNESS_TOL = 1.0e-6
PEEQ_TOL = 1.0e-10
ALLPD_TOL = 1.0e-12
ALLIE_TOL = 1.0e-20
STABILIZATION_RATIO_LIMIT = 0.01

CSV_COLUMNS = (
    "record_type", "step_name", "step_time", "total_time",
    "u2_mm", "rf2_N", "stress_avg_MPa", "strain_eng",
    "metric", "value", "unit", "criterion_met", "note",
)


def fail(message):
    raise RuntimeError(message)


def require_successful_analysis():
    if not os.path.isfile(STA_PATH):
        fail("未找到状态文件：{}".format(STA_PATH))
    with open(STA_PATH, "r", encoding="utf-8", errors="ignore") as sta_file:
        sta_text = sta_file.read().upper()
    if "THE ANALYSIS HAS COMPLETED SUCCESSFULLY" not in sta_text:
        fail("分析未成功完成，拒绝处理不完整ODB。")


def find_history_region(step, required_outputs, description):
    required = set(required_outputs)
    matches = []
    for key, region in step.historyRegions.items():
        if required.issubset(set(region.historyOutputs.keys())):
            matches.append((key, region))
    if not matches:
        fail("步骤'{}'缺少{}历史输出：{}。".format(
            step.name, description, ", ".join(required_outputs)
        ))
    if len(matches) > 1:
        preferred = [
            item for item in matches
            if any(token in item[0].upper() for token in ("TOP_RP", "ASSEMBLY", "NODE"))
        ]
        if len(preferred) == 1:
            return preferred[0][1]
        fail("步骤'{}'中的{}历史区域不唯一。".format(step.name, description))
    return matches[0][1]


def history_pairs(step):
    region = find_history_region(step, ("U2", "RF2"), "上平板RP")
    u2_data = dict(region.historyOutputs["U2"].data)
    rf2_data = dict(region.historyOutputs["RF2"].data)
    common_times = sorted(set(u2_data).intersection(rf2_data))
    if not common_times:
        fail("步骤'{}'的U2与RF2没有共同时间点。".format(step.name))
    return [
        (time_value, float(u2_data[time_value]), float(rf2_data[time_value]))
        for time_value in common_times
    ]


def final_energy(step, variable):
    region = find_history_region(step, (variable,), "全模型能量")
    data = region.historyOutputs[variable].data
    if not data:
        fail("步骤'{}'的{}历史为空。".format(step.name, variable))
    return float(data[-1][1])


def flatten_nodes(node_set):
    raw_nodes = node_set.nodes
    nodes = []
    if len(raw_nodes) > 0 and hasattr(raw_nodes[0], "label"):
        nodes.extend(raw_nodes)
    else:
        for node_array in raw_nodes:
            nodes.extend(node_array)
    if not nodes:
        fail("节点集'{}'为空。".format(node_set.name))
    return nodes


def initial_y_map(node_set):
    return {
        node.label: float(node.coordinates[1])
        for node in flatten_nodes(node_set)
    }


def average_current_y(frame, node_set, initial_y_by_label):
    """严格按当前Y=初始Y+U2计算节点面的平均当前Y。"""
    if "U" not in frame.fieldOutputs:
        fail("场输出缺少U，无法计算当前厚度。")
    displacement_values = frame.fieldOutputs["U"].getSubset(region=node_set).values
    u2_by_label = {
        value.nodeLabel: float(value.data[1]) for value in displacement_values
    }
    missing = set(initial_y_by_label).difference(u2_by_label)
    if missing:
        fail("位移场缺少节点集中的{}个节点。".format(len(missing)))
    current_y = [
        initial_y + u2_by_label[label]
        for label, initial_y in initial_y_by_label.items()
    ]
    return sum(current_y) / float(len(current_y))


def maximum_scalar_field(odb, field_name, element_set):
    maximum = None
    for step_name in (STEP_COMPRESSION, STEP_UNLOAD):
        for frame in odb.steps[step_name].frames:
            if field_name not in frame.fieldOutputs:
                continue
            values = frame.fieldOutputs[field_name].getSubset(region=element_set).values
            for value in values:
                scalar = float(value.data)
                maximum = scalar if maximum is None else max(maximum, scalar)
    if maximum is None:
        fail("未找到{}场输出。".format(field_name))
    return maximum


def maximum_mises(odb, element_set, description):
    maximum = None
    for step_name in (STEP_COMPRESSION, STEP_UNLOAD):
        for frame in odb.steps[step_name].frames:
            if "S" not in frame.fieldOutputs:
                continue
            values = frame.fieldOutputs["S"].getSubset(region=element_set).values
            for value in values:
                mises = float(value.mises)
                maximum = mises if maximum is None else max(maximum, mises)
    if maximum is None:
        fail("未找到{}的Mises应力。".format(description))
    return maximum


def summary_row(metric, value, unit="", criterion_met="", note=""):
    row = {column: "" for column in CSV_COLUMNS}
    row.update({
        "record_type": "SUMMARY", "metric": metric, "value": value,
        "unit": unit, "criterion_met": criterion_met, "note": note,
    })
    return row


def process_odb():
    require_successful_analysis()
    if not os.path.isfile(ODB_PATH):
        fail("未找到ODB：{}".format(ODB_PATH))

    odb = None
    try:
        odb = openOdb(path=ODB_PATH, readOnly=True)
        for step_name in (STEP_COMPRESSION, STEP_UNLOAD):
            if step_name not in odb.steps:
                fail("ODB缺少步骤'{}'。".format(step_name))

        instances = odb.rootAssembly.instances
        active_key = ACTIVE_INSTANCE_NAME.upper()
        al_key = AL_INSTANCE_NAME.upper()
        if active_key not in instances or al_key not in instances:
            fail("ODB缺少活性层或集流体实例。")
        active_instance = instances[active_key]
        al_instance = instances[al_key]

        active_top_set = active_instance.nodeSets[ACTIVE_TOP_NODE_SET]
        active_interface_set = active_instance.nodeSets[ACTIVE_INTERFACE_NODE_SET]
        al_interface_set = al_instance.nodeSets[AL_INTERFACE_NODE_SET]
        al_bottom_set = al_instance.nodeSets[AL_BOTTOM_NODE_SET]
        active_element_set = active_instance.elementSets[ACTIVE_ELEMENT_SET]
        al_element_set = al_instance.elementSets[AL_ELEMENT_SET]

        node_sets = {
            "active_top": active_top_set,
            "active_interface": active_interface_set,
            "al_interface": al_interface_set,
            "al_bottom": al_bottom_set,
        }
        initial_maps = {
            name: initial_y_map(node_set) for name, node_set in node_sets.items()
        }

        rows = []
        history_records = []
        total_time_offset = 0.0
        for step_name in (STEP_COMPRESSION, STEP_UNLOAD):
            step = odb.steps[step_name]
            for step_time, u2, rf2 in history_pairs(step):
                record = {
                    "record_type": "HISTORY", "step_name": step_name,
                    "step_time": step_time,
                    "total_time": total_time_offset + step_time,
                    "u2_mm": u2, "rf2_N": rf2,
                    "stress_avg_MPa": abs(rf2) / COMPRESSION_AREA,
                    "strain_eng": abs(u2) / TOTAL_THICKNESS,
                    "metric": "", "value": "", "unit": "",
                    "criterion_met": "", "note": "",
                }
                rows.append(record)
                history_records.append(record)
            total_time_offset += float(step.timePeriod)
        if not history_records:
            fail("没有提取到U2-RF2历史。")

        compression_thicknesses = []
        for frame in odb.steps[STEP_COMPRESSION].frames:
            top_y = average_current_y(frame, active_top_set, initial_maps["active_top"])
            bottom_y = average_current_y(frame, al_bottom_set, initial_maps["al_bottom"])
            compression_thicknesses.append(top_y - bottom_y)
        if not compression_thicknesses:
            fail("压缩步没有场输出帧。")
        minimum_total_thickness = min(compression_thicknesses)

        unload_frames = odb.steps[STEP_UNLOAD].frames
        if not unload_frames:
            fail("卸载步没有场输出帧。")
        final_frame = unload_frames[-1]
        final_y = {
            name: average_current_y(final_frame, node_sets[name], initial_maps[name])
            for name in node_sets
        }
        residual_total = final_y["active_top"] - final_y["al_bottom"]
        residual_active = final_y["active_top"] - final_y["active_interface"]
        residual_al = final_y["al_interface"] - final_y["al_bottom"]
        interface_average_y_difference = (
            final_y["active_interface"] - final_y["al_interface"]
        )
        layer_sum_difference = (residual_active + residual_al) - residual_total

        max_active_peeq = maximum_scalar_field(odb, "PEEQ", active_element_set)
        max_active_mises = maximum_mises(odb, active_element_set, "活性层")
        max_al_mises = maximum_mises(odb, al_element_set, "集流体")
        allpd_final = final_energy(odb.steps[STEP_UNLOAD], "ALLPD")
        allie_final = final_energy(odb.steps[STEP_UNLOAD], "ALLIE")
        allsd_final = final_energy(odb.steps[STEP_UNLOAD], "ALLSD")

        if abs(allie_final) <= ALLIE_TOL:
            stabilization_ratio = None
            stabilization_criterion = False
        else:
            stabilization_ratio = abs(allsd_final) / abs(allie_final)
            stabilization_criterion = stabilization_ratio < STABILIZATION_RATIO_LIMIT

        max_force_record = max(
            history_records, key=lambda item: abs(float(item["rf2_N"]))
        )
        max_abs_rf2 = abs(float(max_force_record["rf2_N"]))
        max_stress = max_abs_rf2 / COMPRESSION_AREA
        max_strain = max(float(item["strain_eng"]) for item in history_records)

        thickness_criterion = residual_total < TOTAL_THICKNESS - THICKNESS_TOL
        peeq_criterion = max_active_peeq > PEEQ_TOL
        allpd_criterion = allpd_final > ALLPD_TOL
        model_valid = (
            thickness_criterion and peeq_criterion
            and allpd_criterion and stabilization_criterion
        )
        judgement = (
            "VALID_FOR_2D_MACRO_ROLLING"
            if model_valid else "NOT_VALID_FOR_2D_MACRO_ROLLING"
        )

        rows.extend([
            summary_row("minimum_total_thickness_compression", minimum_total_thickness, "mm"),
            summary_row("residual_total_thickness_unloaded", residual_total, "mm"),
            summary_row("residual_active_layer_thickness", residual_active, "mm"),
            summary_row("residual_al_collector_thickness", residual_al, "mm"),
            summary_row("tie_interface_average_y_difference", interface_average_y_difference, "mm", note="活性层Tie面平均Y减集流体Tie面平均Y。"),
            summary_row("layer_sum_minus_total_thickness", layer_sum_difference, "mm"),
            summary_row("maximum_abs_RF2", max_abs_rf2, "N"),
            summary_row("maximum_average_compressive_stress", max_stress, "MPa"),
            summary_row("maximum_engineering_compressive_strain", max_strain, "1"),
            summary_row("maximum_active_layer_PEEQ", max_active_peeq, "1"),
            summary_row("maximum_active_layer_Mises", max_active_mises, "MPa"),
            summary_row("maximum_al_collector_Mises", max_al_mises, "MPa"),
            summary_row("final_ALLPD", allpd_final, "N*mm"),
            summary_row("final_ALLIE", allie_final, "N*mm"),
            summary_row("final_ALLSD", allsd_final, "N*mm"),
            summary_row(
                "final_ALLSD_over_ALLIE",
                "UNDEFINED" if stabilization_ratio is None else stabilization_ratio,
                "1", str(stabilization_criterion),
                "判据：abs(ALLSD)/abs(ALLIE)<0.01",
            ),
            summary_row("criterion_residual_total_thickness", residual_total, "mm", str(thickness_criterion)),
            summary_row("criterion_active_layer_PEEQ", max_active_peeq, "1", str(peeq_criterion)),
            summary_row("criterion_ALLPD", allpd_final, "N*mm", str(allpd_criterion)),
            summary_row(
                "model_validity", judgement, "", str(model_valid),
                "仅按用户规定的四项判据判断。",
            ),
        ])

        with open(CSV_PATH, "w", newline="", encoding="utf-8-sig") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_COLUMNS)
            writer.writeheader()
            writer.writerows(rows)

        print("后处理完成：{}".format(CSV_PATH))
        print("最大|RF2|={:.9g} N".format(max_abs_rf2))
        print("卸载后总厚度={:.9g} mm".format(residual_total))
        print("活性层最大PEEQ={:.9g}".format(max_active_peeq))
        if stabilization_ratio is not None:
            print("ALLSD/ALLIE={:.6%}".format(stabilization_ratio))
        print("模型判断：{}".format(judgement))
    finally:
        if odb is not None:
            odb.close()


if __name__ == "__main__":
    try:
        process_odb()
    except Exception as error:
        print("后处理失败：{}".format(error), file=sys.stderr)
        sys.exit(1)

