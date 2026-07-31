# -*- coding: mbcs -*-
from odbAccess import openOdb
import csv

ODB_PATH = r'E:\abaqus\3Djingya\RollPress_QuasiStatic_LocalSym_50mm_RigidRoller.odb'
CSV_PATH = r'E:\abaqus\3Djingya\electrode_cpress_lastframe.csv'

odb = openOdb(ODB_PATH, readOnly=True)
try:
    frame = odb.steps['Press_Down'].frames[-1]
    key = [k for k in frame.fieldOutputs.keys() if k.strip().startswith('CPRESS')][0]
    cpress = frame.fieldOutputs[key]

    inst = odb.rootAssembly.instances['ACTIVELAYER-1']
    node_coords = dict((n.label, n.coordinates) for n in inst.nodes)

    rows = []
    for value in cpress.values:
        if value.instance is None or value.instance.name != 'ACTIVELAYER-1':
            continue
        node_label = getattr(value, 'nodeLabel', None)
        if node_label not in node_coords:
            continue
        x, y, z = node_coords[node_label]
        rows.append((node_label, x, y, z, float(value.data)))

    with open(CSV_PATH, 'w') as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow(['nodeLabel', 'x_mm', 'y_mm', 'z_mm', 'CPRESS_MPa'])
        writer.writerows(rows)

    if rows:
        vals = [r[4] for r in rows]
        print('CSV_EXPORTED %s rows=%d min=%g max=%g' % (CSV_PATH, len(rows), min(vals), max(vals)))
    else:
        print('CSV_EXPORTED %s rows=0' % CSV_PATH)
finally:
    odb.close()
