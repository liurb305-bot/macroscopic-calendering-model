# -*- coding: mbcs -*-
from odbAccess import openOdb

ODB_PATH = r'E:\abaqus\3Djingya\RollPress_QuasiStatic_LocalSym_50mm_RigidRoller.odb'

odb = openOdb(ODB_PATH, readOnly=True)
try:
    print('ODB:', ODB_PATH)
    print('STEPS:', ', '.join(odb.steps.keys()))
    for step_name, step in odb.steps.items():
        print('STEP %s frames=%d firstTime=%g lastTime=%g' %
              (step_name, len(step.frames), step.frames[0].frameValue, step.frames[-1].frameValue))

    unload_like = [name for name in odb.steps.keys()
                   if any(token in name.lower() for token in ('unload', 'release', 'springback', 'recover'))]
    print('UNLOAD_LIKE_STEPS:', unload_like if unload_like else 'NONE')

    # Report final RP-like U/RF history regions if available.
    for step_name, step in odb.steps.items():
        print('HISTORY_STEP:', step_name)
        for region_name, region in step.historyRegions.items():
            keys = region.historyOutputs.keys()
            if 'U2' in keys or 'RF2' in keys:
                u2 = region.historyOutputs['U2'].data[-1][1] if 'U2' in keys else None
                rf2 = region.historyOutputs['RF2'].data[-1][1] if 'RF2' in keys else None
                print('  REGION=%s U2_final=%s RF2_final=%s' % (region_name, u2, rf2))
finally:
    odb.close()
