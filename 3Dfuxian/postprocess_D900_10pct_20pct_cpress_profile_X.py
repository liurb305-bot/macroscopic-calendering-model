# -*- coding: utf-8 -*-
"""提取D900在10%和20%压下时沿极片长度方向X的Hold接触压力。"""

from odbAccess import openOdb
import csv
import os

import postprocess_selfsupport_yanshan_like_static_press as base


WORK_DIR = r'E:\abaqus\3Dfuxian'
COMBINED_CSV = os.path.join(
    WORK_DIR, 'D900_10pct_vs_20pct_CPRESS_profile_X.csv')

CASES = (
    {
        'label': '10pct',
        'odb': os.path.join(
            WORK_DIR,
            'SelfSupport_YanshanParam_LocalStaticPress_NoUnload_D900_10pct_RefinedZ.odb'),
        'csv': os.path.join(
            WORK_DIR,
            'SelfSupport_YanshanParam_LocalStaticPress_NoUnload_D900_10pct_RefinedZ_CPRESS_profile_X.csv'),
    },
    {
        'label': '20pct',
        'odb': os.path.join(
            WORK_DIR,
            'SelfSupport_YanshanParam_LocalStaticPress_NoUnload_20pct.odb'),
        'csv': os.path.join(
            WORK_DIR,
            'SelfSupport_YanshanParam_LocalStaticPress_NoUnload_20pct_CPRESS_profile_X.csv'),
    },
)


def mean(values):
    return sum(values) / float(len(values)) if values else None


def scalar(value):
    try:
        return float(value.data)
    except TypeError:
        return float(value.data[0])


def extract_case(case):
    if not os.path.isfile(case['odb']):
        raise RuntimeError('未找到ODB：%s' % case['odb'])

    odb = openOdb(path=case['odb'], readOnly=True)
    try:
        film = base.find_instance(odb, 'Film-1')
        frame = base.final_frame(odb, 'Hold')
        if 'CPRESS' not in frame.fieldOutputs:
            raise RuntimeError('ODB中没有CPRESS字段：%s' % case['odb'])

        node_map = dict((node.label, node) for node in film.nodes)
        pressure_map = {}
        field = frame.fieldOutputs['CPRESS']
        for value in field.values:
            instance = getattr(value, 'instance', None)
            if instance is None or instance.name.upper() != film.name.upper():
                continue
            pressure_map[value.nodeLabel] = scalar(value)

        grouped = {}
        for node_label, pressure in pressure_map.items():
            node = node_map.get(node_label)
            if node is None:
                continue
            x_value, y_value, z_value = node.coordinates
            if abs(y_value - base.Y_TOP0) <= base.COORD_TOL:
                surface = 'top'
            elif abs(y_value - base.Y_BOTTOM0) <= base.COORD_TOL:
                surface = 'bottom'
            else:
                continue

            x_key = round(x_value, 6)
            bucket = grouped.setdefault(x_key, {
                'top_width': [], 'bottom_width': [],
                'top_center': [], 'bottom_center': [],
            })
            bucket[surface + '_width'].append(pressure)
            if abs(z_value) <= base.COORD_TOL:
                bucket[surface + '_center'].append(pressure)

        half_profile = []
        for x_value in sorted(grouped.keys()):
            bucket = grouped[x_value]
            top_center = mean(bucket['top_center'])
            bottom_center = mean(bucket['bottom_center'])
            top_width = mean(bucket['top_width'])
            bottom_width = mean(bucket['bottom_width'])
            center_values = [value for value in (top_center, bottom_center)
                             if value is not None]
            width_values = bucket['top_width'] + bucket['bottom_width']
            half_profile.append({
                'x': x_value,
                'top_center': top_center,
                'bottom_center': bottom_center,
                'center_average': mean(center_values),
                'top_width_average': top_width,
                'bottom_width_average': bottom_width,
                'width_average': mean(width_values),
                'width_maximum': max(width_values) if width_values else None,
            })

        full_profile = []
        for row in half_profile:
            if abs(row['x']) > base.COORD_TOL:
                mirrored = dict(row)
                mirrored['x'] = -row['x']
                full_profile.append(mirrored)
            full_profile.append(row)
        return sorted(full_profile, key=lambda row: row['x'])
    finally:
        odb.close()


def write_case(case, profile):
    with open(case['csv'], 'w', newline='', encoding='utf-8-sig') as out:
        writer = csv.writer(out)
        writer.writerow([
            'film_length_coordinate_mm',
            'X_distance_from_deformation_center_mm',
            'upper_surface_centerline_CPRESS_MPa',
            'lower_surface_centerline_CPRESS_MPa',
            'centerline_surface_average_CPRESS_MPa',
            'upper_surface_width_average_CPRESS_MPa',
            'lower_surface_width_average_CPRESS_MPa',
            'width_and_surface_average_CPRESS_MPa',
            'width_maximum_CPRESS_MPa',
        ])
        for row in profile:
            writer.writerow([
                base.as_float_or_na(row['x'] + 5.0),
                base.as_float_or_na(row['x']),
                base.as_float_or_na(row['top_center']),
                base.as_float_or_na(row['bottom_center']),
                base.as_float_or_na(row['center_average']),
                base.as_float_or_na(row['top_width_average']),
                base.as_float_or_na(row['bottom_width_average']),
                base.as_float_or_na(row['width_average']),
                base.as_float_or_na(row['width_maximum']),
            ])


def main():
    profiles = {}
    for case in CASES:
        profile = extract_case(case)
        profiles[case['label']] = profile
        write_case(case, profile)
        peak_center = max(row['center_average'] for row in profile)
        peak_width_average = max(row['width_average'] for row in profile)
        print('%s：点数=%d，中心线峰值=%.9g MPa，宽度平均峰值=%.9g MPa' %
              (case['label'], len(profile), peak_center, peak_width_average))
        print('CSV=%s' % case['csv'])

    maps = {}
    for label, profile in profiles.items():
        maps[label] = dict((round(row['x'], 6), row) for row in profile)
    x_values = sorted(set(maps['10pct'].keys()) | set(maps['20pct'].keys()))

    with open(COMBINED_CSV, 'w', newline='', encoding='utf-8-sig') as out:
        writer = csv.writer(out)
        writer.writerow([
            'film_length_coordinate_mm',
            'X_distance_from_deformation_center_mm',
            'CPRESS_10pct_centerline_surface_average_MPa',
            'CPRESS_20pct_centerline_surface_average_MPa',
            'CPRESS_10pct_width_and_surface_average_MPa',
            'CPRESS_20pct_width_and_surface_average_MPa',
            'CPRESS_10pct_width_maximum_MPa',
            'CPRESS_20pct_width_maximum_MPa',
        ])
        for x_value in x_values:
            row10 = maps['10pct'].get(x_value)
            row20 = maps['20pct'].get(x_value)
            writer.writerow([
                base.as_float_or_na(x_value + 5.0),
                base.as_float_or_na(x_value),
                base.as_float_or_na(
                    None if row10 is None else row10['center_average']),
                base.as_float_or_na(
                    None if row20 is None else row20['center_average']),
                base.as_float_or_na(
                    None if row10 is None else row10['width_average']),
                base.as_float_or_na(
                    None if row20 is None else row20['width_average']),
                base.as_float_or_na(
                    None if row10 is None else row10['width_maximum']),
                base.as_float_or_na(
                    None if row20 is None else row20['width_maximum']),
            ])

    print('合并CSV=%s' % COMBINED_CSV)


if __name__ == '__main__':
    main()
