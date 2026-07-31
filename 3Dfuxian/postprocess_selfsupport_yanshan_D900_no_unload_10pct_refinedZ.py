# -*- coding: utf-8 -*-
"""导出D900、10%压下、Z向加密、持续保载模型的厚度和主要结果。"""

from odbAccess import openOdb
import csv
import os

import postprocess_selfsupport_yanshan_like_static_press as base


MODEL_NAME = 'SelfSupport_YanshanParam_LocalStaticPress_NoUnload_D900_10pct_RefinedZ'
WORK_DIR = r'E:\abaqus\3Dfuxian'
ODB_PATH = os.path.join(WORK_DIR, MODEL_NAME + '.odb')
SUMMARY_CSV = os.path.join(WORK_DIR, MODEL_NAME + '_summary.csv')
RP_HISTORY_CSV = os.path.join(WORK_DIR, MODEL_NAME + '_upper_rp_history.csv')
PROFILE_CSV = os.path.join(WORK_DIR, MODEL_NAME + '_thickness_profile_Z.csv')


def tensor_component_stats(frame, instance, field_name, component_index):
    values = base.field_values_for_instance(frame, field_name, instance)
    if not values:
        return None, None
    data = [float(value.data[component_index]) for value in values]
    return max(data), min(data)


def mises_max(frame, instance):
    values = base.field_values_for_instance(frame, 'S', instance)
    mises = [float(value.mises) for value in values
             if getattr(value, 'mises', None) is not None]
    return max(mises) if mises else None


def write_profile(profile):
    with open(PROFILE_CSV, 'w', newline='', encoding='utf-8-sig') as out:
        writer = csv.writer(out)
        writer.writerow([
            'Z_distance_from_center_mm',
            'initial_thickness_um',
            'D900_refinedZ_no_unload_10pct_hold_thickness_um',
            'change_from_initial_um',
        ])
        initial_um = base.INITIAL_THICKNESS * 1000.0
        for z_value, thickness in profile:
            thickness_um = thickness * 1000.0
            writer.writerow([
                base.as_float_or_na(z_value),
                base.as_float_or_na(initial_um),
                base.as_float_or_na(thickness_um),
                base.as_float_or_na(thickness_um - initial_um),
            ])


def main():
    if not os.path.isfile(ODB_PATH):
        raise RuntimeError('未找到 ODB：%s' % ODB_PATH)

    odb = openOdb(path=ODB_PATH, readOnly=True)
    try:
        film = base.find_instance(odb, 'Film-1')
        pairs = base.pair_surface_nodes(film)
        hold_frame = base.final_frame(odb, 'Hold')

        profile = base.thickness_profile_along_z(
            hold_frame, film, pairs, x_target=0.0)
        write_profile(profile)

        base.RP_HISTORY_CSV = RP_HISTORY_CSV
        rp_rows = base.upper_rp_history(odb)
        base.write_rp_history(rp_rows)
        rf2_model = base.max_abs_rf2_model(
            rp_rows, allowed_steps=('Clamp_Down', 'Hold'))
        rf2_full = None if rf2_model is None else abs(rf2_model) * base.SYMMETRY_FACTOR_XZ

        press_max, press_min = base.pressure_from_s(hold_frame, film)
        pevol_max, pevol_min, pevol_abs = base.pevol_stats(hold_frame, film)
        le22_max, le22_min = tensor_component_stats(hold_frame, film, 'LE', 1)
        pe22_max, pe22_min = tensor_component_stats(hold_frame, film, 'PE', 1)

        allpd = base.final_history_value(odb, 'ALLPD')
        allie = base.final_history_value(odb, 'ALLIE')
        allsd = base.final_history_value(odb, 'ALLSD')
        allsd_ratio = None
        if allsd is not None and allie not in (None, 0.0):
            allsd_ratio = allsd / allie

        rows = [
            ('odb_path', ODB_PATH),
            ('roll_diameter_mm', 900.0),
            ('roll_radius_mm', 450.0),
            ('film_Z_mesh_size_mm', 0.50),
            ('roller_Z_mesh_size_mm', 2.0),
            ('prescribed_upper_roll_U2_mm', -0.015),
            ('target_nominal_gap_mm', 0.135),
            ('max_upper_RF2_model_N_signed', rf2_model),
            ('max_upper_RF2_full_equiv_N_abs', rf2_full),
            ('line_force_full_equiv_N_per_mm',
             None if rf2_full is None else rf2_full / base.FULL_Z_WIDTH),
            ('max_CPRESS_MPa', base.max_contact_pressure(hold_frame)),
            ('max_Mises_film_MPa', mises_max(hold_frame, film)),
            ('max_PRESS_from_S_MPa', press_max),
            ('min_PRESS_from_S_MPa', press_min),
            ('LE22_max', le22_max),
            ('LE22_min', le22_min),
            ('PE22_max', pe22_max),
            ('PE22_min', pe22_min),
            ('PEvol_max', pevol_max),
            ('PEvol_min', pevol_min),
            ('PEvol_abs_max', pevol_abs),
            ('hold_global_average_thickness_mm',
             base.average_thickness(hold_frame, film, pairs)),
            ('hold_contact_X_0_to_2p6_average_thickness_mm',
             base.average_thickness(hold_frame, film, pairs,
                                    x_limit=base.CONTACT_HALF_X)),
            ('hold_center_X_0_to_0p5_average_thickness_mm',
             base.average_thickness(hold_frame, film, pairs,
                                    x_limit=base.CENTER_HALF_X)),
            ('profile_min_thickness_mm', min(value for _, value in profile)),
            ('profile_max_thickness_mm', max(value for _, value in profile)),
            ('ALLPD_final', allpd),
            ('ALLIE_final', allie),
            ('ALLSD_final', allsd),
            ('ALLSD_over_ALLIE', allsd_ratio),
        ]

        with open(SUMMARY_CSV, 'w', newline='', encoding='utf-8-sig') as out:
            writer = csv.writer(out)
            writer.writerow(['metric', 'value'])
            for key, value in rows:
                writer.writerow([key, base.as_float_or_na(value)])

        print('完成：%s' % SUMMARY_CSV)
        print('完成：%s' % RP_HISTORY_CSV)
        print('完成：%s' % PROFILE_CSV)
    finally:
        odb.close()


if __name__ == '__main__':
    main()
