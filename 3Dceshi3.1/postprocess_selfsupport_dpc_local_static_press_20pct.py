# -*- coding: utf-8 -*-
from __future__ import print_function

"""
读取 SelfSupport_DPC_LocalStaticPress_Ch3_1.odb 并输出CSV后处理结果。

运行方式示例：
    abaqus python postprocess_selfsupport_dpc_local_static_press.py
"""

import csv
import os
import sys

from odbAccess import openOdb
from abaqusConstants import NODAL


WORK_DIR = r"E:\abaqus\3Dceshi3.1"
MODEL_NAME = "SelfSupport_DPC_LocalStaticPress_20pct_Ch3_1"
ODB_PATH = os.path.join(WORK_DIR, MODEL_NAME + ".odb")
CSV_PATH = os.path.join(WORK_DIR, MODEL_NAME + "_results.csv")

INITIAL_THICKNESS = 0.150
FILM_WIDTH_Z = 20.0
STABILIZATION_RATIO_LIMIT = 0.05


def flatten_nodes(node_set):
    nodes = []
    for item in node_set.nodes:
        try:
            for node in item:
                nodes.append(node)
        except TypeError:
            nodes.append(item)
    return nodes


def find_instance(odb, keyword):
    keyword = keyword.upper()
    for name, inst in odb.rootAssembly.instances.items():
        if keyword in name.upper():
            return inst
    raise RuntimeError("未找到包含关键字 %s 的ODB实例。" % keyword)


def find_node_set(odb, instance, set_name):
    if set_name in instance.nodeSets:
        return instance.nodeSets[set_name]
    if set_name in odb.rootAssembly.nodeSets:
        return odb.rootAssembly.nodeSets[set_name]
    raise RuntimeError("未找到节点集：%s" % set_name)


def scalar_value(value):
    data = value.data
    try:
        return float(data)
    except TypeError:
        if hasattr(value, "magnitude"):
            return float(value.magnitude)
        return max([abs(float(x)) for x in data])


def max_field_scalar(frame, var_name):
    if var_name not in frame.fieldOutputs:
        return None
    vals = []
    for value in frame.fieldOutputs[var_name].values:
        try:
            vals.append(scalar_value(value))
        except Exception:
            pass
    if not vals:
        return None
    return max(vals)


def max_press_from_s(frame):
    if "S" not in frame.fieldOutputs:
        return None
    vals = []
    for value in frame.fieldOutputs["S"].values:
        data = value.data
        if len(data) >= 3:
            # Abaqus压力常按压缩为正，可由 -trace(S)/3 得到。
            vals.append(-(float(data[0]) + float(data[1]) + float(data[2])) / 3.0)
    if not vals:
        return None
    return max(vals)


def extreme_signed_plastic_vol_strain(frame):
    if "PE" not in frame.fieldOutputs:
        return None
    extreme = None
    for value in frame.fieldOutputs["PE"].values:
        data = value.data
        if len(data) >= 3:
            epv = float(data[0]) + float(data[1]) + float(data[2])
            if extreme is None or abs(epv) > abs(extreme):
                extreme = epv
    return extreme


def nodal_u2_map(frame, node_set):
    if "U" not in frame.fieldOutputs:
        raise RuntimeError("ODB帧中缺少U位移输出，无法按 初始Y+U2 计算厚度。")
    subset = frame.fieldOutputs["U"].getSubset(region=node_set, position=NODAL)
    result = {}
    for value in subset.values:
        result[value.nodeLabel] = float(value.data[1])
    return result


def average_current_y(frame, node_set):
    nodes = flatten_nodes(node_set)
    u2 = nodal_u2_map(frame, node_set)
    vals = []
    for node in nodes:
        if node.label in u2:
            # 当前Y坐标必须等于初始Y坐标 + 当前帧U2，不能直接使用初始坐标。
            vals.append(float(node.coordinates[1]) + u2[node.label])
    if not vals:
        raise RuntimeError("节点集没有可用U2数据，无法计算平均当前Y。")
    return sum(vals) / float(len(vals))


def average_thickness(frame, top_set, bottom_set):
    return average_current_y(frame, top_set) - average_current_y(frame, bottom_set)


def collect_history(odb):
    data = {}
    for step_name, step in odb.steps.items():
        for region_name, region in step.historyRegions.items():
            for key, output in region.historyOutputs.items():
                data.setdefault(key, [])
                for time_value, value in output.data:
                    data[key].append((step_name, float(time_value), float(value), region_name))
    return data


def max_abs_history(history, key):
    if key not in history or not history[key]:
        return None
    return max([abs(row[2]) for row in history[key]])


def final_history_value(history, key):
    if key not in history or not history[key]:
        return None
    return history[key][-1][2]


def max_energy_ratio(history):
    if "ALLSD" not in history or "ALLIE" not in history:
        return None
    ratios = []
    n = min(len(history["ALLSD"]), len(history["ALLIE"]))
    for i in range(n):
        allsd = history["ALLSD"][i][2]
        allie = history["ALLIE"][i][2]
        if abs(allie) > 1.0e-30:
            ratios.append(abs(allsd) / abs(allie))
    if not ratios:
        return None
    return max(ratios)


def compaction_judgement(allpd_max, plastic_vol, residual_thickness):
    allpd_ok = allpd_max is not None and allpd_max > 1.0e-10
    plastic_ok = plastic_vol is not None and abs(plastic_vol) > 1.0e-8
    residual_ok = residual_thickness is not None and residual_thickness < INITIAL_THICKNESS - 1.0e-5
    if allpd_ok and plastic_ok and residual_ok:
        return "发生有效DPC帽盖塑性压实"
    return "基本为弹性压缩或塑性压实不明显"


def write_csv(rows):
    if sys.version_info[0] >= 3:
        f = open(CSV_PATH, "w", newline="", encoding="utf-8-sig")
    else:
        f = open(CSV_PATH, "wb")
    try:
        writer = csv.writer(f)
        writer.writerow(["item", "value"])
        for key, value in rows:
            writer.writerow([key, value])
    finally:
        f.close()


def main():
    if not os.path.isfile(ODB_PATH):
        raise RuntimeError("未找到ODB：%s。请先手动提交Job并生成ODB。" % ODB_PATH)

    odb = openOdb(ODB_PATH, readOnly=True)
    try:
        film = find_instance(odb, "FILM")
        top_set = find_node_set(odb, film, "FILM_TOP_NODES")
        bottom_set = find_node_set(odb, film, "FILM_BOTTOM_NODES")

        min_thickness_down = None
        residual_thickness = None
        max_cpress = None
        max_press = None
        extreme_pe_vol = None

        for step_name, step in odb.steps.items():
            for frame in step.frames:
                if step_name == "Clamp_Down":
                    th = average_thickness(frame, top_set, bottom_set)
                    if min_thickness_down is None or th < min_thickness_down:
                        min_thickness_down = th
                if step_name == "Unload" and frame == step.frames[-1]:
                    residual_thickness = average_thickness(frame, top_set, bottom_set)

                cpress = max_field_scalar(frame, "CPRESS")
                if cpress is not None and (max_cpress is None or cpress > max_cpress):
                    max_cpress = cpress

                press = max_field_scalar(frame, "PRESS")
                if press is None:
                    press = max_press_from_s(frame)
                if press is not None and (max_press is None or press > max_press):
                    max_press = press

                pe_vol = extreme_signed_plastic_vol_strain(frame)
                if pe_vol is not None and (extreme_pe_vol is None or abs(pe_vol) > abs(extreme_pe_vol)):
                    extreme_pe_vol = pe_vol

        history = collect_history(odb)
        max_rf2_abs = max_abs_history(history, "RF2")
        max_unit_width_force = None if max_rf2_abs is None else max_rf2_abs / FILM_WIDTH_Z
        allpd_max = max_abs_history(history, "ALLPD")
        allpd_positive = allpd_max is not None and allpd_max > 1.0e-10
        ratio = max_energy_ratio(history)
        if ratio is None:
            stabilization_note = "未输出ALLSD或ALLIE，无法计算"
        elif ratio > STABILIZATION_RATIO_LIMIT:
            stabilization_note = "稳定耗散偏大，结果只能作趋势参考"
        else:
            stabilization_note = "稳定耗散比例不大"

        rows = [
            ("压下阶段的最小平均厚度_mm", min_thickness_down),
            ("卸载后的残余平均厚度_mm", residual_thickness),
            ("上辊最大反力_abs_RF2_N", max_rf2_abs),
            ("单位宽度辊压力_abs_RF2_over_width_N_per_mm", max_unit_width_force),
            ("最大CPRESS_MPa", max_cpress),
            ("极片内部最大PRESS_MPa", max_press),
            ("最大塑性体积应变_PE11_plus_PE22_plus_PE33_signed", extreme_pe_vol),
            ("ALLPD是否大于0", allpd_positive),
            ("最大ALLPD", allpd_max),
            ("最大ALLSD_over_ALLIE", ratio),
            ("稳定耗散判断", stabilization_note),
            ("DPC帽盖塑性压实判断", compaction_judgement(allpd_max, extreme_pe_vol, residual_thickness)),
        ]
        write_csv(rows)
        print("后处理完成，CSV已输出：%s" % CSV_PATH)
    finally:
        odb.close()


if __name__ == "__main__":
    main()
