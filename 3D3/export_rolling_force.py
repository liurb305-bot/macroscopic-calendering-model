from odbAccess import openOdb
import csv
import os
import sys


WORKDIR = r"E:\abaqus\3D3"
ODB_NAME = "RollPress_3D_10pct.odb"
CSV_NAME = "rolling_force.csv"


def resolve_paths():
    odb_arg = sys.argv[1] if len(sys.argv) > 1 else ODB_NAME
    csv_arg = sys.argv[2] if len(sys.argv) > 2 else CSV_NAME
    if not odb_arg.lower().endswith(".odb"):
        odb_arg += ".odb"
    odb_path = odb_arg if os.path.isabs(odb_arg) else os.path.join(WORKDIR, odb_arg)
    csv_path = csv_arg if os.path.isabs(csv_arg) else os.path.join(WORKDIR, csv_arg)
    return odb_path, csv_path


def find_force_histories(odb):
    rows = []
    for step_name, step in odb.steps.items():
        for region_name, region in step.historyRegions.items():
            if "RF2" not in region.historyOutputs:
                continue
            rf2_data = region.historyOutputs["RF2"].data
            u2_data = region.historyOutputs["U2"].data if "U2" in region.historyOutputs else []
            rm3_data = region.historyOutputs["RM3"].data if "RM3" in region.historyOutputs else []
            ur3_data = region.historyOutputs["UR3"].data if "UR3" in region.historyOutputs else []
            vr3_data = region.historyOutputs["VR3"].data if "VR3" in region.historyOutputs else []

            u2 = dict(u2_data)
            rm3 = dict(rm3_data)
            ur3 = dict(ur3_data)
            vr3 = dict(vr3_data)
            for time_value, rf2 in rf2_data:
                rows.append({
                    "step": step_name,
                    "history_region": region_name,
                    "time_s": time_value,
                    "rf2_N": rf2,
                    "rolling_force_N": abs(rf2),
                    "u2_mm": u2.get(time_value, ""),
                    "rm3_Nmm": rm3.get(time_value, ""),
                    "ur3_rad": ur3.get(time_value, ""),
                    "vr3_rad_per_s": vr3.get(time_value, ""),
                })
    return rows


def main():
    odb_path, csv_path = resolve_paths()
    odb = openOdb(odb_path, readOnly=True)
    try:
        rows = find_force_histories(odb)
    finally:
        odb.close()

    if not rows:
        raise RuntimeError("No RF2 history output found in %s" % odb_path)

    fieldnames = ["step", "history_region", "time_s", "rf2_N", "rolling_force_N",
                  "u2_mm", "rm3_Nmm", "ur3_rad", "vr3_rad_per_s"]
    if sys.version_info[0] >= 3:
        fp = open(csv_path, "w", newline="")
    else:
        fp = open(csv_path, "wb")
    with fp:
        writer = csv.DictWriter(fp, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    print("Wrote %d rows to %s" % (len(rows), csv_path))


if __name__ == "__main__":
    main()
