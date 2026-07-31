# -*- coding: utf-8 -*-
"""导出不卸载模型在 Hold 最后一帧的极片厚度剖面。"""

from odbAccess import openOdb
import csv
import os
import postprocess_selfsupport_yanshan_like_static_press as base


MODEL_NAME = 'SelfSupport_YanshanParam_LocalStaticPress_NoUnload'
WORK_DIR = r'E:\abaqus\3Dfuxian'
ODB_PATH = os.path.join(WORK_DIR, MODEL_NAME + '.odb')
PROFILE_CSV = os.path.join(WORK_DIR, MODEL_NAME + '_thickness_profile_Z.csv')


def main():
    odb = openOdb(path=ODB_PATH, readOnly=True)
    try:
        film = base.find_instance(odb, 'Film-1')
        pairs = base.pair_surface_nodes(film)
        hold_frame = base.final_frame(odb, 'Hold')
        profile = base.thickness_profile_along_z(
            hold_frame, film, pairs, x_target=0.0)

        with open(PROFILE_CSV, 'w', newline='', encoding='utf-8-sig') as out:
            writer = csv.writer(out)
            writer.writerow([
                'Z_distance_from_center_mm',
                'initial_thickness_um',
                'no_unload_hold_thickness_um',
                'change_from_initial_um',
            ])
            for z, thickness in profile:
                thickness_um = thickness * 1000.0
                writer.writerow([
                    base.as_float_or_na(z),
                    base.as_float_or_na(base.INITIAL_THICKNESS * 1000.0),
                    base.as_float_or_na(thickness_um),
                    base.as_float_or_na(
                        thickness_um - base.INITIAL_THICKNESS * 1000.0),
                ])
        print('完成：已导出 NoUnload 厚度剖面 CSV: %s' % PROFILE_CSV)
    finally:
        odb.close()


if __name__ == '__main__':
    main()
