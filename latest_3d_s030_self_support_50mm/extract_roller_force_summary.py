# -*- coding: utf-8 -*-
from odbAccess import openOdb
import os

WORKDIR = r"E:\abaqus\3Dnihe2.0\3Dnihe2.0"
JOB = "RP3D_Self_DPC_S030_50mm_Roll10mm_dt5e6_CoarseRoller_V2p2"
ODB = os.path.join(WORKDIR, JOB + ".odb")
OUT = os.path.join(WORKDIR, JOB + "_roller_force_summary.txt")

odb = openOdb(ODB, readOnly=True)
try:
    lines = []
    lines.append("job=%s" % JOB)
    for step_name, step in odb.steps.items():
        lines.append("")
        lines.append("[step] %s" % step_name)
        for region_name, region in sorted(step.historyRegions.items()):
            if "RF2" not in region.historyOutputs:
                continue
            data = region.historyOutputs["RF2"].data
            if not data:
                continue
            vals = [float(v) for _, v in data]
            times = [float(t) for t, _ in data]
            max_abs = max(abs(v) for v in vals)
            idx = max(range(len(vals)), key=lambda i: abs(vals[i]))
            lines.append("%s RF2 final=%.9g N maxabs=%.9g N at t=%.9g s value=%.9g N" %
                         (region_name, vals[-1], max_abs, times[idx], vals[idx]))
    with open(OUT, "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n".join(lines))
finally:
    odb.close()
