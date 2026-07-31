# -*- coding: utf-8 -*-
"""读取三层极片压缩 ODB，输出载荷曲线、分层厚度和有效性判断。"""

from __future__ import print_function

import csv
import os
import sys

from odbAccess import openOdb


# =============================================================================
# 与建模脚本一致的固定参数和名称
# =============================================================================
WORK_DIR = r"E:\abaqus\3Dyang"
JOB_NAME = "Yang_Macro_LayeredElectrode_Compression"
ODB_PATH = os.path.join(WORK_DIR, JOB_NAME + ".odb")
STA_PATH = os.path.join(WORK_DIR, JOB_NAME + ".sta")
CSV_PATH = os.path.join(WORK_DIR, JOB_NAME + "_results.csv")

X_LENGTH = 1.0
Z_WIDTH = 1.0
TOTAL_THICKNESS = 0.165
COMPRESSION_AREA = X_LENGTH * Z_WIDTH
STEP_COMPRESSION = "Step-1 Compression"
STEP_UNLOAD = "Step-2 Unload"
INSTANCE_NAME = "LAYERED_ELECTRODE-1"

LOWER_ACTIVE_SET = "LOWER_ACTIVE"
AL_SET = "AL_COLLECTOR"
UPPER_ACTIVE_SET = "UPPER_ACTIVE"
BOTTOM_NODE_SET = "BOTTOM_SURFACE_NODES"
LOWER_INTERFACE_NODE_SET = "LOWER_ACTIVE_AL_INTERFACE_NODES"
UPPER_INTERFACE_NODE_SET = "AL_UPPER_ACTIVE_INTERFACE_NODES"
TOP_NODE_SET = "TOP_SURFACE_NODES"

THICKNESS_TOL = 1.0e-6
PEEQ_TOL = 1.0e-10
ALLPD_TOL = 1.0e-12
ALLIE_TOL = 1.0e-20
STABILIZATION_RATIO_LIMIT = 0.01
THICKNESS_CLOSURE_TOL = 1.0e-8

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
        fail("分析没有成功完成，拒绝对不完整 ODB 生成验证结论。")


def find_history_region(step, required_outputs, description):
    required = set(required_outputs)
    matches = []
    for key, region in step.historyRegions.items():
        if required.issubset(set(region.historyOutputs.keys())):
            matches.append((key, region))
    if not matches:
        fail("步骤 '{}' 中缺少{}历史输出：{}。".format(
            step.name, description, ", ".join(required_outputs)
        ))
    if len(matches) > 1:
        preferred = [
            item for item in matches
            if any(token in item[0].upper() for token in ("TOP_RP", "ASSEMBLY", "NODE"))
        ]
        if len(preferred) == 1:
            return preferred[0][1]
        fail("步骤 '{}' 中{}历史区域不唯一。".format(step.name, description))
    return matches[0][1]


def history_pairs(step):
    region = find_history_region(step, ("U2", "RF2"), "上平板RP")
    u2_data = dict(region.historyOutputs["U2"].data)
    rf2_data = dict(region.historyOutputs["RF2"].data)
    common_times = sorted(set(u2_data).intersection(rf2_data))
    if not common_times:
        fail("步骤 '{}' 中 U2 与 RF2 没有共同时间点。".format(step.name))
    return [
        (time_value, float(u2_data[time_value]), float(rf2_data[time_value]))
        for time_value in common_times
    ]


def final_energy(step, variable):
    region = find_history_region(step, (variable,), "全模型能量")
    data = region.historyOutputs[variable].data
    if not data:
        fail("步骤 '{}' 的 {} 历史为空。".format(step.name, variable))
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
        fail("节点集 '{}' 为空。".format(node_set.name))
    return nodes


def initial_y_map(node_set):
    return {
        node.label: float(node.coordinates[1])
        for node in flatten_nodes(node_set)
    }


def average_current_y(frame, node_set, initial_y_by_label):
    if "U" not in frame.fieldOutputs:
        fail("场输出中缺少 U，无法按当前坐标计算厚度。")
    displacement_values = frame.fieldOutputs["U"].getSubset(region=node_set).values
    u2_by_label = {
        value.nodeLabel: float(value.data[1]) for value in displacement_values
    }
    missing = set(initial_y_by_label).difference(u2_by_label)
    if missing:
        fail("位移场缺少节点集中的 {} 个节点。".format(len(missing)))
    current_y_values = [
        initial_y + u2_by_label[label]
        for label, initial_y in initial_y_by_label.items()
    ]
    return sum(current_y_values) / float(len(current_y_values))


def maximum_scalar_field(odb, field_name, element_sets):
    maximum = None
    for step_name in (STEP_COMPRESSION, STEP_UNLOAD):
        for frame in odb.steps[step_name].frames:
            if field_name not in frame.fieldOutputs:
                continue
            field = frame.fieldOutputs[field_name]
            for element_set in element_sets:
                for value in field.getSubset(region=element_set).values:
                    scalar = float(value.data)
                    maximum = scalar if maximum is None else max(maximum, scalar)
    if maximum is None:
        fail("未找到 {} 场输出。".format(field_name))
    return maximum


def maximum_mises(odb, element_sets, material_description):
    maximum = None
    for step_name in (STEP_COMPRESSION, STEP_UNLOAD):
        for frame in odb.steps[step_name].frames:
            if "S" not in frame.fieldOutputs:
                continue
            stress_field = frame.fieldOutputs["S"]
            for element_set in element_sets:
                for value in stress_field.getSubset(region=element_set).values:
                    mises = float(value.mises)
                    maximum = mises if maximum is None else max(maximum, mises)
    if maximum is None:
        fail("未找到{}的 Mises 应力。".format(material_description))
    return maximum


def summary_row(metric, value, unit="", criterion_met="", note=""):
    row = {column: "" for column in CSV_COLUMNS}
    row.update({
        "record_type": "SUMMARY",
        "metric": metric,
        "value": value,
        "unit": unit,
        "criterion_met": criterion_met,
        "note": note,
    })
    return row


def process_odb():
    require_successful_analysis()
    if not os.path.isfile(ODB_PATH):
        fail("未找到 ODB：{}".format(ODB_PATH))

    odb = None
    try:
        odb = openOdb(path=ODB_PATH, readOnly=True)
        for step_name in (STEP_COMPRESSION, STEP_UNLOAD):
            if step_name not in odb.steps:
                fail("ODB 中缺少分析步 '{}'。".format(step_name))

        instance_key = INSTANCE_NAME.upper()
        if instance_key not in odb.rootAssembly.instances:
            fail("ODB 中缺少实例 '{}'。".format(instance_key))
        instance = odb.rootAssembly.instances[instance_key]

        node_set_names = (
            BOTTOM_NODE_SET,
            LOWER_INTERFACE_NODE_SET,
            UPPER_INTERFACE_NODE_SET,
            TOP_NODE_SET,
        )
        node_sets = {}
        initial_y_maps = {}
        for name in node_set_names:
            key = name.upper()
            if key not in instance.nodeSets:
                fail("ODB 中缺少节点集 '{}'。".format(key))
            node_sets[name] = instance.nodeSets[key]
            initial_y_maps[name] = initial_y_map(instance.nodeSets[key])

        element_sets = {}
        for name in (LOWER_ACTIVE_SET, AL_SET, UPPER_ACTIVE_SET):
            key = name.upper()
            if key not in instance.elementSets:
                fail("ODB 中缺少单元集 '{}'。".format(key))
            element_sets[name] = instance.elementSets[key]

        rows = []
        history_records = []
        total_time_offset = 0.0
        for step_name in (STEP_COMPRESSION, STEP_UNLOAD):
            step = odb.steps[step_name]
            for step_time, u2, rf2 in history_pairs(step):
                record = {
                    "record_type": "HISTORY",
                    "step_name": step_name,
                    "step_time": step_time,
                    "total_time": total_time_offset + step_time,
                    "u2_mm": u2,
                    "rf2_N": rf2,
                    "stress_avg_MPa": abs(rf2) / COMPRESSION_AREA,
                    "strain_eng": abs(u2) / TOTAL_THICKNESS,
                    "metric": "", "value": "", "unit": "",
                    "criterion_met": "", "note": "",
                }
                rows.append(record)
                history_records.append(record)
            total_time_offset += float(step.timePeriod)
        if not history_records:
            fail("没有提取到 U2-RF2 历史曲线。")

        # 每个压缩帧均以四个节点面的平均当前Y计算总厚度。
        compression_total_thicknesses = []
        for frame in odb.steps[STEP_COMPRESSION].frames:
            current_bottom = average_current_y(
                frame, node_sets[BOTTOM_NODE_SET], initial_y_maps[BOTTOM_NODE_SET]
            )
            current_top = average_current_y(
                frame, node_sets[TOP_NODE_SET], initial_y_maps[TOP_NODE_SET]
            )
            compression_total_thicknesses.append(current_top - current_bottom)
        if not compression_total_thicknesses:
            fail("压缩步没有场输出帧。")
        minimum_total_thickness = min(compression_total_thicknesses)

        unload_frames = odb.steps[STEP_UNLOAD].frames
        if not unload_frames:
            fail("卸载步没有场输出帧。")
        final_frame = unload_frames[-1]
        final_y = {}
        for name in node_set_names:
            final_y[name] = average_current_y(
                final_frame, node_sets[name], initial_y_maps[name]
            )
        residual_total_thickness = final_y[TOP_NODE_SET] - final_y[BOTTOM_NODE_SET]
        residual_lower_active = (
            final_y[LOWER_INTERFACE_NODE_SET] - final_y[BOTTOM_NODE_SET]
        )
        residual_al = (
            final_y[UPPER_INTERFACE_NODE_SET] - final_y[LOWER_INTERFACE_NODE_SET]
        )
        residual_upper_active = (
            final_y[TOP_NODE_SET] - final_y[UPPER_INTERFACE_NODE_SET]
        )
        layer_sum = residual_lower_active + residual_al + residual_upper_active
        thickness_closure_error = abs(layer_sum - residual_total_thickness)
        closure_criterion = thickness_closure_error <= THICKNESS_CLOSURE_TOL

        active_sets = (
            element_sets[LOWER_ACTIVE_SET], element_sets[UPPER_ACTIVE_SET]
        )
        max_active_peeq = maximum_scalar_field(odb, "PEEQ", active_sets)
        max_active_mises = maximum_mises(odb, active_sets, "活性层")
        max_al_mises = maximum_mises(
            odb, (element_sets[AL_SET],), "铝集流体"
        )

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
        max_engineering_strain = max(
            float(item["strain_eng"]) for item in history_records
        )

        thickness_criterion = residual_total_thickness < (
            TOTAL_THICKNESS - THICKNESS_TOL
        )
        peeq_criterion = max_active_peeq > PEEQ_TOL
        allpd_criterion = allpd_final > ALLPD_TOL
        model_valid = (
            thickness_criterion and peeq_criterion and allpd_criterion
            and stabilization_criterion and closure_criterion
        )
        judgement = (
            "VALID_FOR_2D_MACRO_ROLLING"
            if model_valid else "NOT_VALID_FOR_2D_MACRO_ROLLING"
        )

        rows.extend([
            summary_row("minimum_total_thickness_compression", minimum_total_thickness, "mm"),
            summary_row("residual_total_thickness_unloaded", residual_total_thickness, "mm"),
            summary_row("residual_lower_active_thickness", residual_lower_active, "mm"),
            summary_row("residual_al_collector_thickness", residual_al, "mm"),
            summary_row("residual_upper_active_thickness", residual_upper_active, "mm"),
            summary_row("layer_thickness_closure_error", thickness_closure_error, "mm", str(closure_criterion)),
            summary_row("maximum_abs_RF2", max_abs_rf2, "N"),
            summary_row("maximum_average_compressive_stress", max_stress, "MPa"),
            summary_row("maximum_engineering_compressive_strain", max_engineering_strain, "1"),
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
                "判据：abs(ALLSD)/abs(ALLIE) < 0.01",
            ),
            summary_row(
                "criterion_residual_total_thickness",
                residual_total_thickness, "mm", str(thickness_criterion),
                "判据：残余总厚度 < {:.9g} mm".format(TOTAL_THICKNESS - THICKNESS_TOL),
            ),
            summary_row(
                "criterion_active_layer_PEEQ", max_active_peeq, "1",
                str(peeq_criterion), "判据：PEEQ > {:.3g}".format(PEEQ_TOL),
            ),
            summary_row(
                "criterion_ALLPD", allpd_final, "N*mm", str(allpd_criterion),
                "判据：ALLPD > {:.3g} N*mm".format(ALLPD_TOL),
            ),
            summary_row(
                "model_validity", judgement, "", str(model_valid),
                "所有塑性、稳定能量和厚度闭合判据必须同时满足。",
            ),
        ])

        with open(CSV_PATH, "w", newline="", encoding="utf-8-sig") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_COLUMNS)
            writer.writeheader()
            writer.writerows(rows)

        print("后处理完成：{}".format(CSV_PATH))
        print("最大|RF2| = {:.9g} N".format(max_abs_rf2))
        print("卸载后总厚度 = {:.9g} mm".format(residual_total_thickness))
        print("活性层最大PEEQ = {:.9g}".format(max_active_peeq))
        if stabilization_ratio is not None:
            print("ALLSD/ALLIE = {:.6%}".format(stabilization_ratio))
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

