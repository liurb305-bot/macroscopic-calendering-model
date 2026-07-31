# -*- coding: utf-8 -*-
"""两层极片稳定系数敏感性扫描。

仅从已验证的基础INP派生三个输入文件，只修改两个分析步的*Static行；
顺序提交计算，并生成统一汇总CSV。

运行方式：
    abaqus python run_two_layer_stabilization_sweep.py
"""

from __future__ import print_function

import csv
import os
import re
import subprocess
import sys


WORK_DIR = r"E:\abaqus\3Dyang"
BASE_JOB = "Yang_Macro_TwoLayerElectrode_Compression"
BASE_INP = os.path.join(WORK_DIR, BASE_JOB + ".inp")
SUMMARY_CSV = os.path.join(
    WORK_DIR, "Yang_Macro_TwoLayer_Stabilization_Sweep_Summary.csv"
)
NUM_CPUS = 4

JOB_SPECS = (
    {
        "job_name": "Yang_Macro_TwoLayerElectrode_Compression_Stab_1e_4",
        "factor_label": "1e-4",
        "factor_value": 1.0e-4,
        "static_line": "*Static, stabilize=0.0001, allsdtol=0, continue=NO",
    },
    {
        "job_name": "Yang_Macro_TwoLayerElectrode_Compression_Stab_5e_5",
        "factor_label": "5e-5",
        "factor_value": 5.0e-5,
        "static_line": "*Static, stabilize=0.00005, allsdtol=0, continue=NO",
    },
    {
        "job_name": "Yang_Macro_TwoLayerElectrode_Compression_NoStab",
        "factor_label": "0 (disabled)",
        "factor_value": 0.0,
        "static_line": "*Static",
    },
)

SUMMARY_COLUMNS = (
    "job_name",
    "stabilization_factor",
    "converged",
    "maximum_abs_RF2_N",
    "maximum_average_compressive_stress_MPa",
    "minimum_total_thickness_compression_mm",
    "residual_total_thickness_mm",
    "residual_active_layer_thickness_mm",
    "residual_al_collector_thickness_mm",
    "maximum_active_layer_PEEQ",
    "maximum_active_layer_Mises_MPa",
    "maximum_al_collector_Mises_MPa",
    "final_ALLPD_N_mm",
    "final_ALLIE_N_mm",
    "final_ALLSD_N_mm",
    "ALLSD_over_ALLIE",
    "ALLSD_over_ALLIE_below_1pct",
    "residual_thickness_change_vs_2e_4_pct",
    "maximum_RF2_change_vs_2e_4_pct",
    "active_PEEQ_ratio_vs_2e_4",
    "results_stable_vs_2e_4",
    "failure_step",
    "failure_increment",
    "abaqus_error",
    "selection_status",
    "sweep_decision",
)

METRIC_MAP = {
    "maximum_abs_RF2": "maximum_abs_RF2_N",
    "maximum_average_compressive_stress": "maximum_average_compressive_stress_MPa",
    "minimum_total_thickness_compression": "minimum_total_thickness_compression_mm",
    "residual_total_thickness_unloaded": "residual_total_thickness_mm",
    "residual_active_layer_thickness": "residual_active_layer_thickness_mm",
    "residual_al_collector_thickness": "residual_al_collector_thickness_mm",
    "maximum_active_layer_PEEQ": "maximum_active_layer_PEEQ",
    "maximum_active_layer_Mises": "maximum_active_layer_Mises_MPa",
    "maximum_al_collector_Mises": "maximum_al_collector_Mises_MPa",
    "final_ALLPD": "final_ALLPD_N_mm",
    "final_ALLIE": "final_ALLIE_N_mm",
    "final_ALLSD": "final_ALLSD_N_mm",
    "final_ALLSD_over_ALLIE": "ALLSD_over_ALLIE",
}


def read_text(path):
    with open(path, "r", encoding="utf-8", errors="ignore") as source_file:
        return source_file.read()


def write_text(path, text):
    with open(path, "w", encoding="utf-8", newline="") as target_file:
        target_file.write(text)


def normalized_static_lines(inp_text):
    """将Static过程行替换为占位符，用于证明派生INP没有其他变化。"""
    return re.sub(r"(?im)^\*Static[^\r\n]*$", "*Static::<CONTROLLED>", inp_text)


def create_variant_inputs():
    if not os.path.isfile(BASE_INP):
        raise RuntimeError("未找到基础INP：{}".format(BASE_INP))
    base_text = read_text(BASE_INP)
    pattern = re.compile(
        r"(?im)^\*Static,\s*stabilize=0\.0002[^\r\n]*$"
    )
    matches = pattern.findall(base_text)
    if len(matches) != 2:
        raise RuntimeError("基础INP应包含两个stabilize=0.0002的*Static行。")
    normalized_base = normalized_static_lines(base_text)

    for spec in JOB_SPECS:
        variant_text, replacement_count = pattern.subn(
            spec["static_line"], base_text
        )
        if replacement_count != 2:
            raise RuntimeError("{}未准确替换两个分析步。".format(spec["job_name"]))
        if normalized_static_lines(variant_text) != normalized_base:
            raise RuntimeError("{}除*Static行外出现了其他变化。".format(spec["job_name"]))
        inp_path = os.path.join(WORK_DIR, spec["job_name"] + ".inp")
        write_text(inp_path, variant_text)
        spec["inp_path"] = inp_path
        print("已生成受控输入：{}".format(inp_path))


def cleanup_job_outputs(job_name):
    extensions = (
        ".com", ".dat", ".env", ".ipm", ".lck", ".log", ".mdl", ".msg",
        ".odb", ".pac", ".prt", ".res", ".sim", ".sta", ".stt",
    )
    for extension in extensions:
        path = os.path.join(WORK_DIR, job_name + extension)
        if os.path.isfile(path):
            os.remove(path)
    result_csv = os.path.join(WORK_DIR, job_name + "_results.csv")
    if os.path.isfile(result_csv):
        os.remove(result_csv)


def submit_job(spec):
    cleanup_job_outputs(spec["job_name"])
    command = (
        'abaqus job="{job}" input="{inp}" cpus={cpus} interactive'
        .format(job=spec["job_name"], inp=spec["inp_path"], cpus=NUM_CPUS)
    )
    print("\n开始计算：{}，stabilization={}".format(
        spec["job_name"], spec["factor_label"]
    ))
    return_code = subprocess.call(command, cwd=WORK_DIR, shell=True)
    print("计算进程返回码：{}".format(return_code))


def analysis_converged(job_name):
    sta_path = os.path.join(WORK_DIR, job_name + ".sta")
    if not os.path.isfile(sta_path):
        return False
    return "THE ANALYSIS HAS COMPLETED SUCCESSFULLY" in read_text(sta_path).upper()


def parse_failure_location(job_name):
    sta_path = os.path.join(WORK_DIR, job_name + ".sta")
    last_step = ""
    last_increment = ""
    if os.path.isfile(sta_path):
        pattern = re.compile(r"^\s*(\d+)\s+(\d+)\s+\d+[A-Z]?\s+", re.MULTILINE)
        matches = pattern.findall(read_text(sta_path))
        if matches:
            last_step, last_increment = matches[-1]

    msg_path = os.path.join(WORK_DIR, job_name + ".msg")
    errors = []
    if os.path.isfile(msg_path):
        lines = read_text(msg_path).splitlines()
        for index, line in enumerate(lines):
            if "***ERROR" in line.upper():
                block = [line.strip()]
                for following in lines[index + 1:index + 4]:
                    if following.strip():
                        block.append(following.strip())
                errors.append(" ".join(block))
    error_text = " | ".join(errors)
    if len(error_text) > 1500:
        error_text = error_text[:1500] + "..."
    if not error_text:
        error_text = "No explicit ***ERROR line found; inspect .msg/.dat/.log."
    return last_step, last_increment, error_text


def run_standard_postprocessor(job_name):
    """复用已验证的两层后处理逻辑，保证所有Job的指标定义完全一致。"""
    import postprocess_two_layer_electrode_compression as postprocess
    postprocess.JOB_NAME = job_name
    postprocess.ODB_PATH = os.path.join(WORK_DIR, job_name + ".odb")
    postprocess.STA_PATH = os.path.join(WORK_DIR, job_name + ".sta")
    postprocess.CSV_PATH = os.path.join(WORK_DIR, job_name + "_results.csv")
    postprocess.process_odb()
    return postprocess.CSV_PATH


def metrics_from_result_csv(path):
    metrics = {}
    with open(path, "r", encoding="utf-8-sig", newline="") as csv_file:
        for row in csv.DictReader(csv_file):
            if row["record_type"] == "SUMMARY":
                metrics[row["metric"]] = row["value"]
    return metrics


def blank_result_row(job_name, factor_label, converged):
    row = {column: "" for column in SUMMARY_COLUMNS}
    row["job_name"] = job_name
    row["stabilization_factor"] = factor_label
    row["converged"] = str(bool(converged))
    return row


def fill_metrics(row, metrics):
    for source_metric, target_column in METRIC_MAP.items():
        if source_metric in metrics:
            row[target_column] = metrics[source_metric]
    ratio_text = row["ALLSD_over_ALLIE"]
    try:
        ratio = float(ratio_text)
        row["ALLSD_over_ALLIE_below_1pct"] = str(ratio < 0.01)
    except (TypeError, ValueError):
        row["ALLSD_over_ALLIE_below_1pct"] = "False"


def relative_change(value, baseline):
    if baseline == 0.0:
        return None
    return abs(value - baseline) / abs(baseline)


def evaluate_and_select(rows):
    baseline_row = next(row for row in rows if row["job_name"] == BASE_JOB)
    baseline_residual = float(baseline_row["residual_total_thickness_mm"])
    baseline_force = float(baseline_row["maximum_abs_RF2_N"])
    baseline_peeq = float(baseline_row["maximum_active_layer_PEEQ"])

    candidates = []
    factor_values = {BASE_JOB: 2.0e-4}
    factor_values.update({spec["job_name"]: spec["factor_value"] for spec in JOB_SPECS})
    for row in rows:
        if row["converged"] != "True":
            row["results_stable_vs_2e_4"] = "False"
            row["selection_status"] = "NONCONVERGED"
            continue
        residual_change = relative_change(
            float(row["residual_total_thickness_mm"]), baseline_residual
        )
        force_change = relative_change(float(row["maximum_abs_RF2_N"]), baseline_force)
        peeq_ratio = float(row["maximum_active_layer_PEEQ"]) / baseline_peeq
        row["residual_thickness_change_vs_2e_4_pct"] = 100.0 * residual_change
        row["maximum_RF2_change_vs_2e_4_pct"] = 100.0 * force_change
        row["active_PEEQ_ratio_vs_2e_4"] = peeq_ratio
        stable = residual_change <= 0.01 and force_change <= 0.05 and 0.5 <= peeq_ratio <= 2.0
        row["results_stable_vs_2e_4"] = str(stable)
        ratio_ok = row["ALLSD_over_ALLIE_below_1pct"] == "True"
        if row["job_name"] == BASE_JOB:
            row["selection_status"] = "BASELINE"
        elif stable and ratio_ok:
            row["selection_status"] = "ACCEPTABLE_CANDIDATE"
            candidates.append(row)
        else:
            row["selection_status"] = "NOT_ACCEPTABLE"

    if candidates:
        candidates.sort(
            key=lambda row: (
                float(row["ALLSD_over_ALLIE"]),
                factor_values[row["job_name"]],
            )
        )
        selected = candidates[0]
        selected["selection_status"] = "RECOMMENDED"
        decision = "Recommended job: {}".format(selected["job_name"])
        for row in candidates[1:]:
            row["selection_status"] = "ACCEPTABLE_NOT_SELECTED"
    else:
        decision = (
            "No reduced/no-stabilization job both converged, stayed close to the 2e-4 "
            "baseline, and achieved ALLSD/ALLIE < 1%; do not proceed to 2D rolling."
        )
    for row in rows:
        row["sweep_decision"] = decision
    return decision


def write_summary(rows):
    with open(SUMMARY_CSV, "w", encoding="utf-8-sig", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=SUMMARY_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)


def main():
    os.chdir(WORK_DIR)
    create_variant_inputs()

    rows = []
    if not analysis_converged(BASE_JOB):
        raise RuntimeError("2e-4基础模型没有成功完成，无法进行可靠对照。")
    baseline_csv = os.path.join(WORK_DIR, BASE_JOB + "_results.csv")
    if not os.path.isfile(baseline_csv):
        baseline_csv = run_standard_postprocessor(BASE_JOB)
    baseline_row = blank_result_row(BASE_JOB, "2e-4 (baseline)", True)
    fill_metrics(baseline_row, metrics_from_result_csv(baseline_csv))
    rows.append(baseline_row)

    for spec in JOB_SPECS:
        submit_job(spec)
        converged = analysis_converged(spec["job_name"])
        row = blank_result_row(
            spec["job_name"], spec["factor_label"], converged
        )
        if converged:
            result_csv = run_standard_postprocessor(spec["job_name"])
            fill_metrics(row, metrics_from_result_csv(result_csv))
        else:
            step, increment, error_text = parse_failure_location(spec["job_name"])
            row["failure_step"] = step
            row["failure_increment"] = increment
            row["abaqus_error"] = error_text
            row["ALLSD_over_ALLIE_below_1pct"] = "False"
        rows.append(row)

    decision = evaluate_and_select(rows)
    write_summary(rows)
    print("\n稳定系数扫描完成：{}".format(SUMMARY_CSV))
    print(decision)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("稳定系数扫描失败：{}".format(error), file=sys.stderr)
        sys.exit(1)

