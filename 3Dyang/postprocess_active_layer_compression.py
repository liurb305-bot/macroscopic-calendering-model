# -*- coding: utf-8 -*-
"""读取活性层压缩 ODB，导出载荷曲线、厚度及塑性判断结果。

运行方式：
    abaqus python postprocess_active_layer_compression.py
    abaqus python postprocess_active_layer_compression.py --job <其他Job名>
"""

from __future__ import print_function

import csv
import os
import sys

from odbAccess import openOdb


# =============================================================================
# 关键参数区：必须与建模脚本保持一致
# =============================================================================
WORK_DIR = r"E:\abaqus\3Dyang"
DEFAULT_JOB_NAME = "Yang_Macro_ActiveLayer_Compression"


def command_line_job_name():
    """默认处理规定的 Job；--job 仅用于后续对照模型。"""
    if "--job" not in sys.argv:
        return DEFAULT_JOB_NAME
    index = sys.argv.index("--job")
    if index + 1 >= len(sys.argv):
        raise ValueError("--job 后必须提供 Job 名称。")
    return sys.argv[index + 1]


JOB_NAME = command_line_job_name()
ODB_PATH = os.path.join(WORK_DIR, JOB_NAME + ".odb")
CSV_PATH = os.path.join(WORK_DIR, JOB_NAME + "_results.csv")

X_LENGTH = 1.0
Y_THICKNESS = 0.150
Z_WIDTH = 1.0
COMPRESSION_AREA = X_LENGTH * Z_WIDTH

STEP_COMPRESSION = "Step-1 Compression"
STEP_UNLOAD = "Step-2 Unload"
ACTIVE_INSTANCE_NAME = "ACTIVE_LAYER-1"
TOP_NODE_SET_NAME = "TOP_SURFACE_NODES"
BOTTOM_NODE_SET_NAME = "BOTTOM_SURFACE_NODES"

# 判断容差：用于排除浮点舍入造成的伪非零值
THICKNESS_TOL = 1.0e-6       # mm
ALLPD_TOL = 1.0e-12          # N*mm
PEEQ_TOL = 1.0e-10

CSV_COLUMNS = (
    "record_type",
    "step_name",
    "step_time",
    "total_time",
    "u2_mm",
    "rf2_N",
    "stress_avg_MPa",
    "strain_eng",
    "metric",
    "value",
    "unit",
    "criterion_met",
    "note",
)


def fail(message):
    raise RuntimeError(message)


def find_history_region(step, required_outputs, description):
    """按所含变量定位历史区域，避免依赖 Abaqus 自动生成的区域键名。"""
    matches = []
    required = set(required_outputs)
    for key, region in step.historyRegions.items():
        if required.issubset(set(region.historyOutputs.keys())):
            matches.append((key, region))
    if not matches:
        fail("步骤 '{}' 中未找到包含 {} 的{}历史区域。".format(
            step.name, ", ".join(required_outputs), description
        ))
    if len(matches) > 1:
        # 对 RP 历史优先选择描述中含 Assembly/Node/RP 的区域；仍不唯一则明确报错。
        preferred = [
            item for item in matches
            if any(token in item[0].upper() for token in ("TOP_RP", "ASSEMBLY", "NODE"))
        ]
        if len(preferred) == 1:
            return preferred[0][1]
        fail("步骤 '{}' 中找到多个可能的{}历史区域：{}。".format(
            step.name, description, ", ".join(item[0] for item in matches)
        ))
    return matches[0][1]


def history_pairs(step):
    """返回当前步骤中时间一致的 U2、RF2 数据。"""
    region = find_history_region(step, ("U2", "RF2"), "上平板 RP")
    u2_data = dict(region.historyOutputs["U2"].data)
    rf2_data = dict(region.historyOutputs["RF2"].data)
    common_times = sorted(set(u2_data).intersection(rf2_data))
    if not common_times:
        fail("步骤 '{}' 的 U2 与 RF2 历史没有共同时间点。".format(step.name))
    return [(time_value, u2_data[time_value], rf2_data[time_value])
            for time_value in common_times]


def final_energy(step, variable):
    """取得指定步骤末端的全模型能量值。"""
    region = find_history_region(step, (variable,), "全模型能量")
    data = region.historyOutputs[variable].data
    if not data:
        fail("步骤 '{}' 的 {} 历史为空。".format(step.name, variable))
    return float(data[-1][1])


def instance_node_map(node_set):
    """将实例级 ODB 节点集转换为 label -> 初始 Y 坐标。"""
    raw_nodes = node_set.nodes
    nodes = []
    if len(raw_nodes) > 0 and hasattr(raw_nodes[0], "label"):
        # 实例级节点集通常直接返回一个 OdbMeshNodeArray。
        nodes.extend(raw_nodes)
    else:
        # 装配级节点集可能返回由多个 OdbMeshNodeArray 组成的序列。
        for node_array in raw_nodes:
            nodes.extend(node_array)
    if not nodes:
        fail("节点集 '{}' 为空。".format(node_set.name))
    return {node.label: float(node.coordinates[1]) for node in nodes}


def average_current_y(frame, node_set, initial_y_by_label):
    """严格按 当前Y=初始Y+U2 计算一个表面节点集的平均当前 Y。"""
    if "U" not in frame.fieldOutputs:
        fail("步骤帧中缺少位移场 U，无法计算当前厚度。")
    values = frame.fieldOutputs["U"].getSubset(region=node_set).values
    u2_by_label = {value.nodeLabel: float(value.data[1]) for value in values}
    missing = set(initial_y_by_label).difference(u2_by_label)
    if missing:
        fail("位移场 U 缺少节点集中的 {} 个节点。".format(len(missing)))
    current_y = [
        initial_y + u2_by_label[label]
        for label, initial_y in initial_y_by_label.items()
    ]
    return sum(current_y) / float(len(current_y))


def frame_thickness(frame, top_set, bottom_set, top_initial_y, bottom_initial_y):
    top_average_y = average_current_y(frame, top_set, top_initial_y)
    bottom_average_y = average_current_y(frame, bottom_set, bottom_initial_y)
    return top_average_y - bottom_average_y


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
    if not os.path.isfile(ODB_PATH):
        fail("未找到 ODB：{}。请先另行提交并完成分析。".format(ODB_PATH))

    odb = None
    try:
        odb = openOdb(path=ODB_PATH, readOnly=True)
        for step_name in (STEP_COMPRESSION, STEP_UNLOAD):
            if step_name not in odb.steps:
                fail("ODB 中缺少分析步 '{}'。".format(step_name))

        root_assembly = odb.rootAssembly
        instance_key = ACTIVE_INSTANCE_NAME.upper()
        if instance_key not in root_assembly.instances:
            fail("ODB 中缺少活性层实例 '{}'。".format(instance_key))
        active_instance = root_assembly.instances[instance_key]

        top_key = TOP_NODE_SET_NAME.upper()
        bottom_key = BOTTOM_NODE_SET_NAME.upper()
        if top_key not in active_instance.nodeSets:
            fail("活性层实例中缺少节点集 '{}'。".format(top_key))
        if bottom_key not in active_instance.nodeSets:
            fail("活性层实例中缺少节点集 '{}'。".format(bottom_key))
        if "ACTIVE_LAYER_ALL" not in active_instance.elementSets:
            fail("活性层实例中缺少单元集 'ACTIVE_LAYER_ALL'。")
        top_set = active_instance.nodeSets[top_key]
        bottom_set = active_instance.nodeSets[bottom_key]
        active_element_set = active_instance.elementSets["ACTIVE_LAYER_ALL"]
        top_initial_y = instance_node_map(top_set)
        bottom_initial_y = instance_node_map(bottom_set)

        rows = []
        history_records = []
        total_time_offset = 0.0
        for step_name in (STEP_COMPRESSION, STEP_UNLOAD):
            step = odb.steps[step_name]
            for step_time, u2, rf2 in history_pairs(step):
                stress = abs(float(rf2)) / COMPRESSION_AREA
                strain = abs(float(u2)) / Y_THICKNESS
                record = {
                    "record_type": "HISTORY",
                    "step_name": step_name,
                    "step_time": step_time,
                    "total_time": total_time_offset + step_time,
                    "u2_mm": u2,
                    "rf2_N": rf2,
                    "stress_avg_MPa": stress,
                    "strain_eng": strain,
                    "metric": "",
                    "value": "",
                    "unit": "",
                    "criterion_met": "",
                    "note": "",
                }
                rows.append(record)
                history_records.append(record)
            total_time_offset += float(step.timePeriod)

        if not history_records:
            fail("未提取到任何 U2-RF2 历史数据。")

        # 对每个场输出帧计算平均厚度；压缩最小值与卸载末值分别汇总。
        compression_thicknesses = []
        for frame in odb.steps[STEP_COMPRESSION].frames:
            compression_thicknesses.append(
                frame_thickness(
                    frame, top_set, bottom_set, top_initial_y, bottom_initial_y
                )
            )
        if not compression_thicknesses:
            fail("压缩步骤没有场输出帧。")
        minimum_compression_thickness = min(compression_thicknesses)

        unload_frames = odb.steps[STEP_UNLOAD].frames
        if not unload_frames:
            fail("卸载步骤没有场输出帧。")
        residual_thickness = frame_thickness(
            unload_frames[-1], top_set, bottom_set, top_initial_y, bottom_initial_y
        )

        # PEEQ 取两个步骤所有帧、所有活性层积分点的最大值。
        max_peeq = 0.0
        peeq_found = False
        for step_name in (STEP_COMPRESSION, STEP_UNLOAD):
            for frame in odb.steps[step_name].frames:
                if "PEEQ" not in frame.fieldOutputs:
                    continue
                values = frame.fieldOutputs["PEEQ"].getSubset(
                    region=active_element_set
                ).values
                if values:
                    peeq_found = True
                    max_peeq = max(max_peeq, max(float(value.data) for value in values))
        if not peeq_found:
            fail("ODB 中未找到活性层的 PEEQ 场输出。")

        allpd_final = final_energy(odb.steps[STEP_UNLOAD], "ALLPD")
        allie_final = final_energy(odb.steps[STEP_UNLOAD], "ALLIE")
        try:
            allsd_final = final_energy(odb.steps[STEP_UNLOAD], "ALLSD")
        except RuntimeError:
            # 原始无稳定模型未要求 ALLSD；稳定化对照模型会输出此变量。
            allsd_final = None

        max_force_record = max(history_records, key=lambda item: abs(float(item["rf2_N"])))
        max_abs_rf2 = abs(float(max_force_record["rf2_N"]))
        max_average_stress = max_abs_rf2 / COMPRESSION_AREA
        max_engineering_strain = max(float(item["strain_eng"]) for item in history_records)

        thickness_criterion = residual_thickness < (Y_THICKNESS - THICKNESS_TOL)
        allpd_criterion = allpd_final > ALLPD_TOL
        peeq_criterion = max_peeq > PEEQ_TOL
        plastic_compression = thickness_criterion and allpd_criterion and peeq_criterion
        basically_elastic = (
            abs(residual_thickness - Y_THICKNESS) <= THICKNESS_TOL
            and not allpd_criterion
            and not peeq_criterion
        )
        if plastic_compression:
            judgement = "PLASTIC_COMPRESSION"
            judgement_note = "三个判据均满足，材料发生塑性压缩。"
        elif basically_elastic:
            judgement = "BASICALLY_ELASTIC"
            judgement_note = "厚度恢复且 ALLPD/PEEQ 近零，材料基本为弹性压缩。"
        else:
            judgement = "MIXED_OR_INCONCLUSIVE"
            judgement_note = "三个判据未同时满足，请结合曲线和收敛状态检查。"

        rows.extend([
            summary_row("minimum_average_thickness_compression", minimum_compression_thickness, "mm"),
            summary_row("residual_average_thickness_unloaded", residual_thickness, "mm"),
            summary_row("maximum_abs_RF2", max_abs_rf2, "N"),
            summary_row("maximum_average_compressive_stress", max_average_stress, "MPa"),
            summary_row("maximum_engineering_compressive_strain", max_engineering_strain, "1"),
            summary_row("maximum_PEEQ", max_peeq, "1"),
            summary_row("final_ALLPD", allpd_final, "N*mm"),
            summary_row("final_ALLIE", allie_final, "N*mm"),
            summary_row(
                "criterion_residual_thickness_below_initial",
                residual_thickness,
                "mm",
                str(thickness_criterion),
                "判据：residual thickness < {:.9g} mm".format(Y_THICKNESS - THICKNESS_TOL),
            ),
            summary_row(
                "criterion_ALLPD_positive",
                allpd_final,
                "N*mm",
                str(allpd_criterion),
                "判据：ALLPD > {:.3g} N*mm".format(ALLPD_TOL),
            ),
            summary_row(
                "criterion_PEEQ_nonzero",
                max_peeq,
                "1",
                str(peeq_criterion),
                "判据：PEEQ > {:.3g}".format(PEEQ_TOL),
            ),
            summary_row(
                "plasticity_judgement",
                judgement,
                "",
                str(plastic_compression),
                judgement_note,
            ),
        ])
        if allsd_final is not None:
            stabilization_ratio = (
                abs(allsd_final) / abs(allie_final) if abs(allie_final) > 0.0 else ""
            )
            rows.extend([
                summary_row("final_ALLSD", allsd_final, "N*mm"),
                summary_row(
                    "final_ALLSD_over_ALLIE",
                    stabilization_ratio,
                    "1",
                    note="稳定化对照模型的人工耗散能比例。",
                ),
            ])

        # utf-8-sig 便于 Windows Excel 直接识别中文注释。
        with open(CSV_PATH, "w", newline="", encoding="utf-8-sig") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_COLUMNS)
            writer.writeheader()
            writer.writerows(rows)

        print("后处理完成：{}".format(CSV_PATH))
        print("最大 |RF2| = {:.9g} N".format(max_abs_rf2))
        print("压缩阶段最小平均厚度 = {:.9g} mm".format(minimum_compression_thickness))
        print("卸载后残余平均厚度 = {:.9g} mm".format(residual_thickness))
        print("塑性判断：{}（{}）".format(judgement, judgement_note))
    finally:
        if odb is not None:
            odb.close()


if __name__ == "__main__":
    try:
        process_odb()
    except Exception as error:
        print("后处理失败：{}".format(error), file=sys.stderr)
        sys.exit(1)
