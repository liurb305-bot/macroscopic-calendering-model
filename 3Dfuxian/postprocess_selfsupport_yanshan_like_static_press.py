# -*- coding: utf-8 -*-
"""
读取单层自支撑膜三维局部静态辊压模型 ODB，并导出 CSV。

厚度计算严格采用：当前 Y 坐标 = 初始 Y 坐标 + U2。
不使用初始节点坐标的最大最小值直接代表变形后厚度。
"""

from odbAccess import openOdb
from abaqusConstants import *
import os
import csv
import math


# =============================================================================
# 关键参数区
# =============================================================================

MODEL_NAME = 'SelfSupport_YanshanParam_LocalStaticPress'
WORK_DIR = r'E:\abaqus\3Dfuxian'
ODB_PATH = os.path.join(WORK_DIR, MODEL_NAME + '.odb')
SUMMARY_CSV = os.path.join(WORK_DIR, MODEL_NAME + '_summary.csv')
RP_HISTORY_CSV = os.path.join(WORK_DIR, MODEL_NAME + '_upper_rp_history.csv')
THICKNESS_PROFILE_CSV = os.path.join(
    WORK_DIR, MODEL_NAME + '_thickness_profile_Z.csv')

FULL_Z_WIDTH = 100.0
INITIAL_THICKNESS = 0.150
Y_TOP0 = INITIAL_THICKNESS / 2.0
Y_BOTTOM0 = -INITIAL_THICKNESS / 2.0
SYMMETRY_FACTOR_XZ = 4.0

CONTACT_HALF_X = 2.6
CENTER_HALF_X = 0.5
COORD_TOL = 1.0e-5
ZERO_TOL = 1.0e-8
THICKNESS_TOL = 1.0e-5


def as_float_or_na(value):
    if value is None:
        return 'NA'
    if isinstance(value, str):
        return value
    return '%.10g' % value


def find_instance(odb, preferred_name):
    """按名称查找实例；ODB 中名称通常会被转换为大写。"""
    root = odb.rootAssembly
    if preferred_name in root.instances:
        return root.instances[preferred_name]
    target = preferred_name.upper()
    for name, inst in root.instances.items():
        if name.upper() == target:
            return inst
    for name, inst in root.instances.items():
        if 'FILM' in name.upper():
            return inst
    raise KeyError('未找到膜实例：%s' % preferred_name)


def final_frame(odb, step_name=None):
    if step_name and step_name in odb.steps:
        step = odb.steps[step_name]
        return step.frames[-1]
    last_step = odb.steps[odb.steps.keys()[-1]]
    return last_step.frames[-1]


def field_values_for_instance(frame, field_name, instance=None):
    if field_name not in frame.fieldOutputs:
        return []
    field = frame.fieldOutputs[field_name]
    if instance is None:
        return list(field.values)
    try:
        return list(field.getSubset(region=instance).values)
    except Exception:
        return [v for v in field.values if getattr(v, 'instance', None) == instance]


def displacement_map(frame, instance):
    vals = field_values_for_instance(frame, 'U', instance)
    return dict((v.nodeLabel, v.data) for v in vals)


def pair_surface_nodes(instance):
    """按初始 X/Z 坐标配对膜上下表面节点。"""
    top = {}
    bottom = {}
    for node in instance.nodes:
        x, y, z = node.coordinates
        key = (round(x, 6), round(z, 6))
        if abs(y - Y_TOP0) <= COORD_TOL:
            top[key] = node
        elif abs(y - Y_BOTTOM0) <= COORD_TOL:
            bottom[key] = node

    pairs = []
    for key, top_node in top.items():
        bot_node = bottom.get(key)
        if bot_node is not None:
            pairs.append((key, top_node, bot_node))
    if not pairs:
        raise RuntimeError('未能按初始 X/Z 坐标配对膜上下表面节点。')
    return pairs


def average_thickness(frame, instance, pairs, x_limit=None):
    """使用当前 Y=初始 Y+U2 计算平均厚度。"""
    u_map = displacement_map(frame, instance)
    values = []
    for key, top_node, bot_node in pairs:
        x = key[0]
        if x_limit is not None and x > x_limit + COORD_TOL:
            continue
        top_u2 = u_map.get(top_node.label, (0.0, 0.0, 0.0))[1]
        bot_u2 = u_map.get(bot_node.label, (0.0, 0.0, 0.0))[1]
        top_y = top_node.coordinates[1] + top_u2
        bot_y = bot_node.coordinates[1] + bot_u2
        values.append(top_y - bot_y)
    if not values:
        return None
    return sum(values) / float(len(values))


def thickness_profile_along_z(frame, instance, pairs, x_target=0.0):
    """在指定 X 截面按当前坐标计算厚度，并镜像为完整 Z 宽度。"""
    u_map = displacement_map(frame, instance)
    half_profile = []
    for key, top_node, bot_node in pairs:
        x, z = key
        if abs(x - x_target) > COORD_TOL:
            continue
        top_u2 = u_map.get(top_node.label, (0.0, 0.0, 0.0))[1]
        bot_u2 = u_map.get(bot_node.label, (0.0, 0.0, 0.0))[1]
        top_y = top_node.coordinates[1] + top_u2
        bot_y = bot_node.coordinates[1] + bot_u2
        half_profile.append((z, top_y - bot_y))

    if not half_profile:
        raise RuntimeError('未找到 X=%.6g 截面的膜厚度节点对。' % x_target)

    full_profile = []
    for z, thickness in half_profile:
        if abs(z) > COORD_TOL:
            full_profile.append((-z, thickness))
        full_profile.append((z, thickness))
    return sorted(full_profile, key=lambda item: item[0])


def write_thickness_profile(hold_profile, unload_profile):
    hold_map = dict((round(z, 6), thickness) for z, thickness in hold_profile)
    unload_map = dict((round(z, 6), thickness) for z, thickness in unload_profile)
    z_values = sorted(set(hold_map.keys()) | set(unload_map.keys()))

    with open(THICKNESS_PROFILE_CSV, 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.writer(f)
        writer.writerow([
            'Z_distance_from_center_mm',
            'initial_thickness_um',
            'hold_thickness_um',
            'unload_thickness_um',
            'hold_change_from_initial_um',
            'unload_change_from_initial_um',
        ])
        initial_um = INITIAL_THICKNESS * 1000.0
        for z in z_values:
            hold_um = None if z not in hold_map else hold_map[z] * 1000.0
            unload_um = None if z not in unload_map else unload_map[z] * 1000.0
            writer.writerow([
                as_float_or_na(z),
                as_float_or_na(initial_um),
                as_float_or_na(hold_um),
                as_float_or_na(unload_um),
                as_float_or_na(None if hold_um is None else hold_um - initial_um),
                as_float_or_na(None if unload_um is None else unload_um - initial_um),
            ])


def pressure_from_s(frame, instance):
    vals = field_values_for_instance(frame, 'S', instance)
    if not vals:
        return None, None
    pressures = []
    for val in vals:
        data = val.data
        pressures.append(-(data[0] + data[1] + data[2]) / 3.0)
    return max(pressures), min(pressures)


def pevol_stats(frame, instance):
    vals = field_values_for_instance(frame, 'PE', instance)
    if not vals:
        return None, None, None
    pevol = []
    for val in vals:
        data = val.data
        pevol.append(data[0] + data[1] + data[2])
    return max(pevol), min(pevol), max(abs(v) for v in pevol)


def max_contact_pressure(frame):
    vals = field_values_for_instance(frame, 'CPRESS', None)
    if vals:
        data = []
        for val in vals:
            try:
                data.append(float(val.data))
            except TypeError:
                data.append(float(val.data[0]))
        if data:
            return max(data)

    # 如果 ODB 没有单独 CPRESS 字段，尝试从 CSTRESS 组件中提取。
    if 'CSTRESS' not in frame.fieldOutputs:
        return None
    field = frame.fieldOutputs['CSTRESS']
    labels = [label.upper() for label in getattr(field, 'componentLabels', ())]
    if 'CPRESS' not in labels:
        return None
    idx = labels.index('CPRESS')
    data = []
    for val in field.values:
        try:
            data.append(float(val.data[idx]))
        except Exception:
            pass
    if not data:
        return None
    return max(data)


def collect_history(odb, var_name):
    """收集所有 history region 中指定变量的数据。"""
    rows = []
    time_offset = 0.0
    for step_name in odb.steps.keys():
        step = odb.steps[step_name]
        step_rows = []
        for region_name, region in step.historyRegions.items():
            if var_name in region.historyOutputs:
                for t, value in region.historyOutputs[var_name].data:
                    step_rows.append((region_name, t, value))
        for region_name, t, value in step_rows:
            rows.append((step_name, time_offset + t, region_name, value))
        if step.frames:
            time_offset += step.frames[-1].frameValue
    return rows


def upper_rp_history(odb):
    """读取上辊 RP 的 U2/RF2 历史，按 step/time 合并。"""
    combined = {}
    time_offset = 0.0
    for step_name in odb.steps.keys():
        step = odb.steps[step_name]
        for region_name, region in step.historyRegions.items():
            has_u2 = 'U2' in region.historyOutputs
            has_rf2 = 'RF2' in region.historyOutputs
            if not (has_u2 or has_rf2):
                continue
            if 'UPPER' not in region_name.upper() and 'RP' not in region_name.upper():
                # 若 history region 名称不含 UPPER，也保留带 RF2/U2 的点，后面由数据筛选。
                pass
            if has_u2:
                for t, value in region.historyOutputs['U2'].data:
                    key = (step_name, time_offset + t, region_name)
                    combined.setdefault(key, {})['U2'] = value
            if has_rf2:
                for t, value in region.historyOutputs['RF2'].data:
                    key = (step_name, time_offset + t, region_name)
                    combined.setdefault(key, {})['RF2'] = value
        if step.frames:
            time_offset += step.frames[-1].frameValue

    rows = []
    for key in sorted(combined.keys(), key=lambda k: (k[1], k[0], k[2])):
        data = combined[key]
        if 'U2' in data or 'RF2' in data:
            rows.append((key[0], key[1], key[2], data.get('U2'), data.get('RF2')))
    return rows


def final_history_value(odb, var_name):
    rows = collect_history(odb, var_name)
    if not rows:
        return None
    return rows[-1][3]


def max_abs_rf2_model(rp_rows, allowed_steps=None):
    vals = [row[4] for row in rp_rows
            if row[4] is not None and
            (allowed_steps is None or row[0] in allowed_steps)]
    if not vals:
        return None
    return max(vals, key=lambda v: abs(v))


def write_rp_history(rows):
    with open(RP_HISTORY_CSV, 'w', newline='', encoding='utf-8-sig') as f:
        writer = csv.writer(f)
        writer.writerow(['step', 'total_time', 'history_region', 'U2_model_mm',
                         'RF2_model_N', 'RF2_full_equiv_N'])
        for step, total_time, region, u2, rf2 in rows:
            full_rf2 = None if rf2 is None else rf2 * SYMMETRY_FACTOR_XZ
            writer.writerow([step, as_float_or_na(total_time), region,
                             as_float_or_na(u2), as_float_or_na(rf2),
                             as_float_or_na(full_rf2)])


def judge_result(allpd, pevol_abs, residual_contact, residual_center, allsd, allie):
    allsd_ratio = None
    if allsd is not None and allie not in (None, 0.0):
        allsd_ratio = allsd / allie

    has_plastic_energy = allpd is not None and allpd > ZERO_TOL
    has_pevol = pevol_abs is not None and pevol_abs > ZERO_TOL
    has_residual = (
        (residual_contact is not None and residual_contact < INITIAL_THICKNESS - THICKNESS_TOL) or
        (residual_center is not None and residual_center < INITIAL_THICKNESS - THICKNESS_TOL)
    )
    stable_ok = allsd_ratio is None or allsd_ratio < 0.05

    if has_plastic_energy and has_pevol and has_residual and stable_ok:
        return '较可信的塑性压实'

    elastic_like = (
        (allpd is None or abs(allpd) <= ZERO_TOL) and
        (pevol_abs is None or abs(pevol_abs) <= ZERO_TOL) and
        (residual_contact is None or abs(residual_contact - INITIAL_THICKNESS) <= THICKNESS_TOL) and
        (residual_center is None or abs(residual_center - INITIAL_THICKNESS) <= THICKNESS_TOL)
    )
    if elastic_like:
        return '材料基本仍为弹性压缩'

    if allsd_ratio is not None and allsd_ratio >= 0.05:
        return '自动稳定能量偏大，未满足可信塑性压实判据，结果仅作趋势参考'
    return '结果需结合收敛和场变量进一步判断'


def main():
    if not os.path.isfile(ODB_PATH):
        raise RuntimeError('未找到 ODB：%s。请先手动提交 Job 并完成计算。' % ODB_PATH)

    odb = openOdb(path=ODB_PATH, readOnly=True)
    try:
        film = find_instance(odb, 'Film-1')
        pairs = pair_surface_nodes(film)

        hold_frame = final_frame(odb, 'Hold')
        unload_frame = final_frame(odb, 'Unload')

        rp_rows = upper_rp_history(odb)
        write_rp_history(rp_rows)

        # 辊压力峰值只取压下/保载阶段，避免卸载稳定产生的回位伪反力污染结果。
        rf2_model_at_max = max_abs_rf2_model(
            rp_rows, allowed_steps=('Clamp_Down', 'Hold'))
        rf2_model_all_steps = max_abs_rf2_model(rp_rows)
        max_rf2_full = None if rf2_model_at_max is None else abs(rf2_model_at_max) * SYMMETRY_FACTOR_XZ
        line_force = None if max_rf2_full is None else max_rf2_full / FULL_Z_WIDTH

        cpress_max = max_contact_pressure(hold_frame)
        press_max, press_min = pressure_from_s(hold_frame, film)
        pevol_max, pevol_min, pevol_abs = pevol_stats(hold_frame, film)

        hold_thk_global = average_thickness(hold_frame, film, pairs, x_limit=None)
        hold_thk_contact = average_thickness(hold_frame, film, pairs, x_limit=CONTACT_HALF_X)
        hold_thk_center = average_thickness(hold_frame, film, pairs, x_limit=CENTER_HALF_X)
        residual_thk_global = average_thickness(unload_frame, film, pairs, x_limit=None)
        residual_thk_contact = average_thickness(unload_frame, film, pairs, x_limit=CONTACT_HALF_X)
        residual_thk_center = average_thickness(unload_frame, film, pairs, x_limit=CENTER_HALF_X)

        hold_profile = thickness_profile_along_z(hold_frame, film, pairs, x_target=0.0)
        unload_profile = thickness_profile_along_z(unload_frame, film, pairs, x_target=0.0)
        write_thickness_profile(hold_profile, unload_profile)

        allpd = final_history_value(odb, 'ALLPD')
        allie = final_history_value(odb, 'ALLIE')
        allsd = final_history_value(odb, 'ALLSD')
        allsd_ratio = None
        if allsd is not None and allie not in (None, 0.0):
            allsd_ratio = allsd / allie

        conclusion = judge_result(allpd, pevol_abs, residual_thk_contact,
                                  residual_thk_center, allsd, allie)

        rows = [
            ('odb_path', ODB_PATH),
            ('symmetry_factor_XZ', SYMMETRY_FACTOR_XZ),
            ('max_upper_RF2_loaded_steps_model_N_signed', rf2_model_at_max),
            ('max_upper_RF2_loaded_steps_full_equiv_N_abs', max_rf2_full),
            ('line_force_loaded_steps_abs_RF2_over_width_N_per_mm', line_force),
            ('max_upper_RF2_all_steps_model_N_signed_diagnostic', rf2_model_all_steps),
            ('max_CPRESS_MPa', cpress_max),
            ('max_PRESS_from_S_MPa', press_max),
            ('min_PRESS_from_S_MPa', press_min),
            ('hold_global_average_thickness_mm', hold_thk_global),
            ('hold_contact_X_0_to_2p6_average_thickness_mm', hold_thk_contact),
            ('hold_center_X_0_to_0p5_average_thickness_mm', hold_thk_center),
            ('unload_residual_global_average_thickness_mm', residual_thk_global),
            ('unload_residual_contact_X_0_to_2p6_average_thickness_mm', residual_thk_contact),
            ('unload_residual_center_X_0_to_0p5_average_thickness_mm', residual_thk_center),
            ('PEvol_max', pevol_max),
            ('PEvol_min', pevol_min),
            ('PEvol_abs_max', pevol_abs),
            ('ALLPD_final', allpd),
            ('ALLIE_final', allie),
            ('ALLSD_final', allsd),
            ('ALLSD_over_ALLIE', allsd_ratio),
            ('judgement', conclusion),
        ]

        with open(SUMMARY_CSV, 'w', newline='', encoding='utf-8-sig') as f:
            writer = csv.writer(f)
            writer.writerow(['metric', 'value'])
            for key, value in rows:
                writer.writerow([key, as_float_or_na(value)])

        print('完成：已导出汇总 CSV: %s' % SUMMARY_CSV)
        print('完成：已导出上辊 RP 历史 CSV: %s' % RP_HISTORY_CSV)
        print('完成：已导出极片厚度剖面 CSV: %s' % THICKNESS_PROFILE_CSV)
        print('判断：%s' % conclusion)
    finally:
        odb.close()


if __name__ == '__main__':
    main()
