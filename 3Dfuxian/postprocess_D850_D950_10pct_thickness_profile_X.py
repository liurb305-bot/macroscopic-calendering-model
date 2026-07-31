# -*- coding: utf-8 -*-
"""提取D850与D950、10%压下模型沿辊压方向X的Hold厚度曲线。"""

from odbAccess import openOdb
import csv
import os

import postprocess_selfsupport_yanshan_like_static_press as base


WORK_DIR = r'E:\abaqus\3Dfuxian'
COMBINED_CSV = os.path.join(
    WORK_DIR, 'D850_D950_no_unload_10pct_thickness_profile_X.csv')
Z_TARGET = 0.0

CASES = (
    {
        'label': 'D850_R425',
        'odb': r'E:\abaqus\3Dfuxian2.0\SelfSupport_YanshanParam_LocalStaticPress_NoUnload_D850_10pct.odb',
        'csv': r'E:\abaqus\3Dfuxian2.0\SelfSupport_YanshanParam_LocalStaticPress_NoUnload_D850_10pct_thickness_profile_X.csv',
    },
    {
        'label': 'D950_R475',
        'odb': r'E:\abaqus\3Dfuxian3.0\SelfSupport_YanshanParam_LocalStaticPress_NoUnload_D950_10pct.odb',
        'csv': r'E:\abaqus\3Dfuxian3.0\SelfSupport_YanshanParam_LocalStaticPress_NoUnload_D950_10pct_thickness_profile_X.csv',
    },
)


def thickness_profile_along_x(frame, instance, pairs, z_target=0.0):
    """在指定Z截面按当前Y坐标计算厚度，并镜像X半模型。"""
    u_map = base.displacement_map(frame, instance)
    half_profile = []
    for key, top_node, bottom_node in pairs:
        x_value, z_value = key
        if abs(z_value - z_target) > base.COORD_TOL:
            continue

        top_u2 = u_map.get(top_node.label, (0.0, 0.0, 0.0))[1]
        bottom_u2 = u_map.get(bottom_node.label, (0.0, 0.0, 0.0))[1]
        top_y = top_node.coordinates[1] + top_u2
        bottom_y = bottom_node.coordinates[1] + bottom_u2
        half_profile.append((x_value, top_y - bottom_y))

    if not half_profile:
        raise RuntimeError('未找到Z=%.6g截面的膜厚节点对。' % z_target)

    full_profile = []
    for x_value, thickness in half_profile:
        if abs(x_value) > base.COORD_TOL:
            full_profile.append((-x_value, thickness))
        full_profile.append((x_value, thickness))
    return sorted(full_profile, key=lambda item: item[0])


def extract_case(case):
    if not os.path.isfile(case['odb']):
        raise RuntimeError('未找到ODB：%s' % case['odb'])

    odb = openOdb(path=case['odb'], readOnly=True)
    try:
        film = base.find_instance(odb, 'Film-1')
        pairs = base.pair_surface_nodes(film)
        frame = base.final_frame(odb, 'Hold')
        return thickness_profile_along_x(
            frame, film, pairs, z_target=Z_TARGET)
    finally:
        odb.close()


def write_case(case, profile):
    initial_um = base.INITIAL_THICKNESS * 1000.0
    thickness_header = case['label'] + '_10pct_hold_thickness_um'
    with open(case['csv'], 'w', newline='', encoding='utf-8-sig') as out:
        writer = csv.writer(out)
        writer.writerow([
            'X_distance_from_deformation_center_mm',
            'initial_thickness_um',
            thickness_header,
            'compression_from_initial_um',
        ])
        for x_value, thickness in profile:
            thickness_um = thickness * 1000.0
            writer.writerow([
                base.as_float_or_na(x_value),
                base.as_float_or_na(initial_um),
                base.as_float_or_na(thickness_um),
                base.as_float_or_na(initial_um - thickness_um),
            ])


def main():
    profiles = {}
    for case in CASES:
        profile = extract_case(case)
        profiles[case['label']] = profile
        write_case(case, profile)
        center_um = min(profile, key=lambda item: abs(item[0]))[1] * 1000.0
        print('%s：点数=%d，中心厚度=%.9g um，CSV=%s' %
              (case['label'], len(profile), center_um, case['csv']))

    map850 = dict((round(x, 6), value) for x, value in profiles['D850_R425'])
    map950 = dict((round(x, 6), value) for x, value in profiles['D950_R475'])
    x_values = sorted(set(map850.keys()) | set(map950.keys()))
    initial_um = base.INITIAL_THICKNESS * 1000.0

    with open(COMBINED_CSV, 'w', newline='', encoding='utf-8-sig') as out:
        writer = csv.writer(out)
        writer.writerow([
            'X_distance_from_deformation_center_mm',
            'initial_thickness_um',
            'D850_R425_10pct_hold_thickness_um',
            'D950_R475_10pct_hold_thickness_um',
            'D950_minus_D850_um',
            'D850_compression_from_initial_um',
            'D950_compression_from_initial_um',
        ])
        for x_value in x_values:
            value850_um = map850[x_value] * 1000.0
            value950_um = map950[x_value] * 1000.0
            writer.writerow([
                base.as_float_or_na(x_value),
                base.as_float_or_na(initial_um),
                base.as_float_or_na(value850_um),
                base.as_float_or_na(value950_um),
                base.as_float_or_na(value950_um - value850_um),
                base.as_float_or_na(initial_um - value850_um),
                base.as_float_or_na(initial_um - value950_um),
            ])

    print('合并CSV：%s' % COMBINED_CSV)


if __name__ == '__main__':
    main()
