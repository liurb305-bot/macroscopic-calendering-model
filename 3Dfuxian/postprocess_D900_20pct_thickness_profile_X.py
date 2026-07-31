# -*- coding: utf-8 -*-
"""提取D900、20%压下模型沿辊压方向X的Hold厚度曲线。"""

from odbAccess import openOdb
import csv
import os

import postprocess_selfsupport_yanshan_like_static_press as base


MODEL_NAME = 'SelfSupport_YanshanParam_LocalStaticPress_NoUnload_20pct'
WORK_DIR = r'E:\abaqus\3Dfuxian'
ODB_PATH = os.path.join(WORK_DIR, MODEL_NAME + '.odb')
PROFILE_CSV = os.path.join(
    WORK_DIR, MODEL_NAME + '_thickness_profile_X.csv')
Z_TARGET = 0.0


def thickness_profile_along_x(frame, instance, pairs, z_target=0.0):
    """在指定Z截面计算厚度，并由X>=0半模型镜像为完整X曲线。"""
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


def main():
    if not os.path.isfile(ODB_PATH):
        raise RuntimeError('未找到ODB：%s' % ODB_PATH)

    odb = openOdb(path=ODB_PATH, readOnly=True)
    try:
        film = base.find_instance(odb, 'Film-1')
        pairs = base.pair_surface_nodes(film)
        hold_frame = base.final_frame(odb, 'Hold')
        profile = thickness_profile_along_x(
            hold_frame, film, pairs, z_target=Z_TARGET)

        initial_um = base.INITIAL_THICKNESS * 1000.0
        with open(PROFILE_CSV, 'w', newline='', encoding='utf-8-sig') as out:
            writer = csv.writer(out)
            writer.writerow([
                'X_distance_from_deformation_center_mm',
                'initial_thickness_um',
                'D900_20pct_hold_thickness_um',
                'compression_from_initial_um',
                'inside_nominal_contact_X_abs_le_2p6',
            ])
            for x_value, thickness in profile:
                thickness_um = thickness * 1000.0
                writer.writerow([
                    base.as_float_or_na(x_value),
                    base.as_float_or_na(initial_um),
                    base.as_float_or_na(thickness_um),
                    base.as_float_or_na(initial_um - thickness_um),
                    'YES' if abs(x_value) <= base.CONTACT_HALF_X else 'NO',
                ])

        center = min(profile, key=lambda item: abs(item[0]))[1] * 1000.0
        minimum = min(value for _, value in profile) * 1000.0
        maximum = max(value for _, value in profile) * 1000.0
        print('完成：%s' % PROFILE_CSV)
        print('点数=%d，中心厚度=%.9g um，最小=%.9g um，最大=%.9g um' %
              (len(profile), center, minimum, maximum))
    finally:
        odb.close()


if __name__ == '__main__':
    main()
