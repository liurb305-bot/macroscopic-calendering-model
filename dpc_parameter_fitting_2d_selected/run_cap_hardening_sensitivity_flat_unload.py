# -*- coding: utf-8 -*-
"""
DPC Cap Hardening 参数敏感性验证脚本。

运行方式：
    abaqus cae noGUI=run_cap_hardening_sensitivity_flat_unload.py

脚本只复制现有平板压缩-卸载模型，不重新生成几何、不重新划分网格，
也不改变接触、边界条件、压板位移、弹性参数和 DPC 基础参数。
唯一改变是将 Cap Hardening 表第一列静水屈服压力乘以不同系数。
"""
from abaqus import *
from abaqusConstants import *
from driverUtils import executeOnCaeStartup
from odbAccess import openOdb
import csv
import math
import os
import re
import shutil
import struct
import zlib


executeOnCaeStartup()


# ----------------------------- 用户可改参数 -----------------------------
WORKDIR = r"E:\abaqus\2D3.1"
BASE_CAE = os.path.join(WORKDIR, "FlatUnload_2D_DPC_E50Nu015.cae")
SENSITIVITY_CAE = os.path.join(WORKDIR,
                               "FlatUnload_2D_DPC_E50Nu015_CapSensitivity.cae")
BASE_MODEL_NAME = "FlatUnload_2D_DPC_E50Nu015"
BASE_MATERIAL_NAME = "ActiveLayer_DPC"
BASE_JOB_PREFIX = "FlatUnload_2D_DPC_E50Nu015"
SCALES = (0.02, 0.05, 0.10, 0.20, 0.50, 1.00)

# 如果无法从 CAE 或 INP 中读取原始 Cap Hardening，就使用这里的表。
BASE_CAP_HARDENING = (
    (60.0, 0.00),
    (62.0, 0.05),
    (66.0, 0.10),
    (72.0, 0.15),
    (80.0, 0.18),
    (90.0, 0.20),
    (110.0, 0.22),
    (140.0, 0.24),
    (190.0, 0.26),
    (270.0, 0.28),
    (390.0, 0.30),
    (480.0, 0.31),
    (600.0, 0.32),
    (740.0, 0.33),
)

# DPC 基础参数保持不变；这里只用于在复制模型后重新写入 CapPlasticity。
DPC_CAP_PLASTICITY = (4.0, 65.0, 0.8, 60.0, 0.02, 1.0)

INSTANCE_NAME = "ACTIVELAYER-1"
INITIAL_THICKNESS_UM = 150.0
TARGET_THICKNESS_UM = 135.0
RF2_ZERO_TOL = 1.0e-3
CPRESS_ZERO_TOL = 1.0e-3
PERMANENT_NEAR_INITIAL_TOL_UM = 1.0
SOFT_TOO_THIN_TOL_UM = 0.5
ALLPD_ZERO_TOL = 1.0e-9
Y_TOL = 1.0e-7
NUM_CPUS = 1
RUN_JOBS = True


def scale_tag(scale):
    """把 0.02 变成 S002，1.00 变成 S100。"""
    return "S%03d" % int(round(scale * 100.0))


def read_base_cap_hardening_from_inp():
    """从现有 inp 中读取 Cap Hardening。读取失败时返回预设表。"""
    inp_path = os.path.join(WORKDIR, BASE_JOB_PREFIX + ".inp")
    if not os.path.exists(inp_path):
        return BASE_CAP_HARDENING
    rows = []
    in_table = False
    with open(inp_path, "r") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            if line.lower().startswith("*cap hardening"):
                in_table = True
                continue
            if in_table and line.startswith("*"):
                break
            if in_table:
                parts = [p.strip() for p in line.split(",") if p.strip()]
                if len(parts) >= 2:
                    rows.append((float(parts[0]), float(parts[1])))
    return tuple(rows) if rows else BASE_CAP_HARDENING


def open_base_model():
    """复制基准 CAE 到敏感性 CAE，并只在副本上操作。"""
    os.chdir(WORKDIR)
    shutil.copy2(BASE_CAE, SENSITIVITY_CAE)
    if BASE_MODEL_NAME not in mdb.models:
        openMdb(pathName=SENSITIVITY_CAE)
    if BASE_MODEL_NAME not in mdb.models:
        raise RuntimeError("Cannot find base model: %s" % BASE_MODEL_NAME)
    return mdb.models[BASE_MODEL_NAME]


def force_output_requests(model):
    """关键字型 CAE 可能没有输出请求仓库，此处只保留接口。"""
    return


def patch_inp_output_requests(job_name):
    """在写出的 inp 中补强 PRESS 输出，同时保留原有接触/RP/能量输出。"""
    inp_path = os.path.join(WORKDIR, job_name + ".inp")
    with open(inp_path, "r") as f:
        text = f.read()
    # Abaqus 的 PRESS 是应力不变量，用来判断静水压缩；PE 用于提取 PE11/PE22/PE33。
    text = re.sub(
        r"(\*Element Output, elset=Sheet_All, directions=YES\s*\n)[^\*\n]+",
        r"\1LE, PE, PEEQ, PRESS, S",
        text)
    # 确保上压板 RP 输出 U2/RF2；基准模型已有，这里再规范一次。
    text = re.sub(
        r"(\*Node Output, nset=RP_TopPlate\s*\n)[^\*\n]+",
        r"\1RF2, U2",
        text)
    # 能量输出只需保留所需项；额外能量项不影响结果。
    text = re.sub(
        r"(\*Energy Output\s*\n)[^\*\n]+",
        r"\1ALLPD, ALLIE, ALLKE",
        text)
    with open(inp_path, "w") as f:
        f.write(text)


def create_scaled_model(base_model, scale, base_cap_hardening):
    """复制基准模型，只修改 Cap Hardening 的静水屈服压力。"""
    tag = scale_tag(scale)
    model_name = BASE_MODEL_NAME + "_Cap" + tag
    job_name = BASE_JOB_PREFIX + "_Cap" + tag
    if model_name in mdb.models:
        del mdb.models[model_name]
    model = mdb.Model(name=model_name, objectToCopy=base_model)

    mat = model.materials[BASE_MATERIAL_NAME]
    scaled_table = tuple((p * scale, evp) for p, evp in base_cap_hardening)
    # 重新写入 DPC 基础参数和缩放后的硬化表；除硬化压力外参数保持不变。
    mat.CapPlasticity(table=(DPC_CAP_PLASTICITY,)).CapHardening(
        table=scaled_table)
    force_output_requests(model)

    if job_name in mdb.jobs:
        del mdb.jobs[job_name]
    job = mdb.Job(name=job_name,
                  model=model.name,
                  description="DPC Cap Hardening sensitivity scale %.3g" %
                  scale,
                  type=ANALYSIS,
                  explicitPrecision=SINGLE,
                  nodalOutputPrecision=SINGLE,
                  multiprocessingMode=DEFAULT,
                  numCpus=NUM_CPUS,
                  numDomains=NUM_CPUS)
    job.writeInput(consistencyChecking=ON)
    patch_inp_output_requests(job_name)
    return model_name, job_name, scaled_table


def submit_job(job_name):
    """顺序提交单核运算，避免本机 MPI/编码启动问题。"""
    if job_completed_successfully(job_name):
        print("Skipping completed %s" % job_name)
        return
    print("Submitting %s ..." % job_name)
    mdb.jobs[job_name].submit(consistencyChecking=OFF)
    mdb.jobs[job_name].waitForCompletion()
    print("Finished %s" % job_name)


def read_text_if_exists(path):
    if not os.path.exists(path):
        return ""
    try:
        with open(path, "r") as f:
            return f.read().lower()
    except Exception:
        return ""


def scan_stability_flags(job_name):
    """扫描求解文本文件，检查畸变、负体积、零时间增量等问题。"""
    text = ""
    for ext in (".sta", ".msg", ".dat"):
        text += "\n" + read_text_if_exists(os.path.join(WORKDIR,
                                                        job_name + ext))
    return {
        "distortion": ("distort" in text or
                       "warnelemdistorted" in text or
                       "excessdistortion" in text),
        "negative_volume": ("negative volume" in text or
                            "zero or negative volume" in text),
        "zero_time_increment": ("stable time increment is zero" in text or
                                "zero time increment" in text or
                                "time increment required is less" in text),
        "yield_stress_too_low": ("yield stress is too low" in text or
                                 "cap would be in tension space" in text),
        "aborted": ("aborted" in text or "exited with an error" in text),
    }


def job_completed_successfully(job_name):
    """已有成功 ODB 时跳过重复计算，避免重复等待。"""
    odb_path = os.path.join(WORKDIR, job_name + ".odb")
    if not os.path.exists(odb_path) or os.path.getsize(odb_path) < 1000000:
        return False
    text = ""
    for ext in (".sta", ".log", ".msg"):
        text += "\n" + read_text_if_exists(os.path.join(WORKDIR,
                                                        job_name + ext))
    return ("completed successfully" in text or
            ("job " in text and " completed" in text and
             "exited with errors" not in text))


def history_series(odb, region_fragment, variable):
    """按 step 顺序读取指定历史变量。"""
    out = []
    step_origin = 0.0
    for step_name, step in odb.steps.items():
        for region_name, region in step.historyRegions.items():
            if region_fragment in region_name.upper():
                if variable in region.historyOutputs:
                    for t, v in region.historyOutputs[variable].data:
                        out.append((step_name, step_origin + t, t, v))
        if step.frames:
            step_origin += step.frames[-1].frameValue
    return out


def energy_series(odb, variable):
    out = []
    step_origin = 0.0
    for step_name, step in odb.steps.items():
        for region in step.historyRegions.values():
            if variable in region.historyOutputs:
                for t, v in region.historyOutputs[variable].data:
                    out.append((step_name, step_origin + t, t, v))
        if step.frames:
            step_origin += step.frames[-1].frameValue
    return out


def nearest_value(series, step_name, step_time):
    candidates = [(t, v) for s, _, t, v in series if s == step_name]
    if not candidates:
        return None
    return min(candidates, key=lambda tv: abs(tv[0] - step_time))[1]


def top_bottom_thickness(inst, frame):
    """用上下表面节点平均坐标差计算当前厚度。"""
    u_field = frame.fieldOutputs["U"].getSubset(region=inst)
    disp = dict((v.nodeLabel, v.data) for v in u_field.values)
    nodes = list(inst.nodes)
    y_min = min(n.coordinates[1] for n in nodes)
    y_max = max(n.coordinates[1] for n in nodes)
    top = []
    bottom = []
    for node in nodes:
        ux, uy = disp.get(node.label, (0.0, 0.0))[:2]
        y_def = node.coordinates[1] + uy
        if abs(node.coordinates[1] - y_min) <= Y_TOL:
            bottom.append(y_def)
        elif abs(node.coordinates[1] - y_max) <= Y_TOL:
            top.append(y_def)
    if not top or not bottom:
        raise RuntimeError("Cannot find top/bottom surface nodes")
    avg_t = (sum(top) / len(top) - sum(bottom) / len(bottom)) * 1000.0
    min_t = (min(top) - max(bottom)) * 1000.0
    max_t = (max(top) - min(bottom)) * 1000.0
    return avg_t, min_t, max_t


def max_cpress(frame):
    if "CPRESS" in frame.fieldOutputs:
        values = [abs(v.data) for v in frame.fieldOutputs["CPRESS"].values]
        return max(values) if values else 0.0
    if "CSTRESS" in frame.fieldOutputs:
        vals = []
        for v in frame.fieldOutputs["CSTRESS"].values:
            data = v.data
            if isinstance(data, float):
                vals.append(abs(data))
            elif hasattr(data, "__len__") and len(data) > 0:
                vals.append(abs(data[0]))
        return max(vals) if vals else 0.0
    return 0.0


def stress_pressure_from_s(value):
    data = value.data
    if len(data) >= 3:
        return -(data[0] + data[1] + data[2]) / 3.0
    return 0.0


def max_press(frame, inst):
    if "PRESS" in frame.fieldOutputs:
        subset = frame.fieldOutputs["PRESS"].getSubset(region=inst)
        vals = [v.data for v in subset.values]
        return max(vals) if vals else 0.0
    if "S" in frame.fieldOutputs:
        subset = frame.fieldOutputs["S"].getSubset(region=inst)
        vals = [stress_pressure_from_s(v) for v in subset.values]
        return max(vals) if vals else 0.0
    return 0.0


def plastic_volumetric_strain(frame, inst):
    if "PE" not in frame.fieldOutputs:
        return 0.0, 0.0, 0.0
    vals = []
    subset = frame.fieldOutputs["PE"].getSubset(region=inst)
    for v in subset.values:
        data = v.data
        if len(data) >= 3:
            vals.append(data[0] + data[1] + data[2])
    if not vals:
        return 0.0, 0.0, 0.0
    return max(vals), min(vals), max(abs(v) for v in vals)


def classify_result(residual_t, allpd_max, max_abs_pv, flags):
    if flags.get("yield_stress_too_low", False):
        return "Cap曲线过软"
    if flags["aborted"] or flags["negative_volume"] or flags["zero_time_increment"]:
        return "严重畸变或计算失败"
    if residual_t < TARGET_THICKNESS_UM - SOFT_TOO_THIN_TOL_UM:
        return "Cap曲线过软"
    if flags["distortion"]:
        return "存在单元畸变风险"
    if (abs(allpd_max) <= ALLPD_ZERO_TOL and
            abs(residual_t - INITIAL_THICKNESS_UM) <= PERMANENT_NEAR_INITIAL_TOL_UM and
            max_abs_pv <= 1.0e-8):
        return "未发生永久压实"
    if residual_t < INITIAL_THICKNESS_UM - 2.0 and residual_t >= TARGET_THICKNESS_UM - SOFT_TOO_THIN_TOL_UM:
        return "数值候选参数"
    return "需人工复核"


def failure_summary(job_name, scale, status, flags):
    """失败工况也写入完整汇总列，保证 CSV 不缺字段。"""
    classification = "Cap曲线过软" if flags.get("yield_stress_too_low", False) else "严重畸变或计算失败"
    # 即使没有有效 ODB 帧，也输出空历史 CSV 和失败占位 PNG，便于批量检查。
    try:
        write_history_csv(os.path.join(WORKDIR,
                                       "cap%s_thickness_history.csv" %
                                       scale_tag(scale).lower()), [])
        write_failure_png(os.path.join(WORKDIR,
                                       "cap%s_thickness_time.png" %
                                       scale_tag(scale).lower()))
    except Exception:
        pass
    return {
        "scale": scale,
        "job_name": job_name,
        "status": status,
        "min_compression_thickness_um": "",
        "residual_thickness_um": "",
        "residual_basis_step": "",
        "residual_top_rf2_N": "",
        "residual_max_cpress_MPa": "",
        "max_cpress_MPa": "",
        "max_compressive_press_MPa": "",
        "max_signed_plastic_vol_strain": "",
        "min_signed_plastic_vol_strain": "",
        "max_abs_plastic_vol_strain": "",
        "ALLPD_max_abs": "",
        "ALLPD_final": "",
        "ALLIE_max_abs": "",
        "ALLIE_final": "",
        "ALLKE_max_abs": "",
        "ALLKE_final": "",
        "distortion": flags["distortion"],
        "negative_volume": flags["negative_volume"],
        "zero_time_increment": flags["zero_time_increment"],
        "yield_stress_too_low": flags.get("yield_stress_too_low", False),
        "aborted": True,
        "classification": classification,
    }


def extract_job_results(job_name, scale):
    """从 ODB 中提取厚度、压力、塑性体积应变和能量。"""
    odb_path = os.path.join(WORKDIR, job_name + ".odb")
    if not os.path.exists(odb_path):
        flags = scan_stability_flags(job_name)
        return failure_summary(job_name, scale, "NO_ODB", flags)

    odb = openOdb(odb_path, readOnly=True)
    rows = []
    max_cp = 0.0
    max_pr = 0.0
    max_pv = -1.0e100
    min_pv = 1.0e100
    max_abs_pv = 0.0
    try:
        inst = odb.rootAssembly.instances[INSTANCE_NAME]
        top_rf2 = history_series(odb, "TOPPLATE", "RF2")
        top_u2 = history_series(odb, "TOPPLATE", "U2")
        step_origin = 0.0
        for step_name, step in odb.steps.items():
            for i, frame in enumerate(step.frames):
                avg_t, min_t, max_t = top_bottom_thickness(inst, frame)
                cp = max_cpress(frame)
                pr = max_press(frame, inst)
                pv_max, pv_min, pv_abs = plastic_volumetric_strain(frame, inst)
                max_cp = max(max_cp, cp)
                max_pr = max(max_pr, pr)
                max_pv = max(max_pv, pv_max)
                min_pv = min(min_pv, pv_min)
                max_abs_pv = max(max_abs_pv, pv_abs)
                rf2 = nearest_value(top_rf2, step_name, frame.frameValue)
                u2 = nearest_value(top_u2, step_name, frame.frameValue)
                zero_unload = (step_name in ("Unload", "Free_Settle") and
                               rf2 is not None and abs(rf2) <= RF2_ZERO_TOL and
                               cp <= CPRESS_ZERO_TOL)
                rows.append({
                    "scale": scale,
                    "job_name": job_name,
                    "step": step_name,
                    "frame_index": i,
                    "step_time_s": frame.frameValue,
                    "total_time_s": step_origin + frame.frameValue,
                    "top_u2_mm": u2,
                    "top_rf2_N": rf2,
                    "avg_thickness_um": avg_t,
                    "min_thickness_um": min_t,
                    "max_thickness_um": max_t,
                    "max_cpress_MPa": cp,
                    "max_compressive_press_MPa": pr,
                    "max_signed_plastic_vol_strain": pv_max,
                    "min_signed_plastic_vol_strain": pv_min,
                    "max_abs_plastic_vol_strain": pv_abs,
                    "unloaded_zero_force_frame": zero_unload,
                })
            if step.frames:
                step_origin += step.frames[-1].frameValue

        if not rows:
            flags = scan_stability_flags(job_name)
            return failure_summary(job_name, scale, "NO_VALID_FRAMES", flags)

        min_row = min(rows, key=lambda r: r["avg_thickness_um"])
        zero_rows = [r for r in rows if r["unloaded_zero_force_frame"]]
        residual_row = zero_rows[-1] if zero_rows else rows[-1]

        energy = {}
        for var in ("ALLPD", "ALLIE", "ALLKE"):
            es = energy_series(odb, var)
            vals = [v for _, _, _, v in es]
            energy[var + "_max_abs"] = max([abs(v) for v in vals]) if vals else 0.0
            energy[var + "_final"] = vals[-1] if vals else 0.0

        csv_path = os.path.join(WORKDIR, "cap%s_thickness_history.csv" %
                                scale_tag(scale).lower())
        write_history_csv(csv_path, rows)
        write_curve_png(os.path.join(WORKDIR,
                                     "cap%s_thickness_time.png" %
                                     scale_tag(scale).lower()),
                        rows)

        flags = scan_stability_flags(job_name)
        classification = classify_result(
            residual_row["avg_thickness_um"],
            energy["ALLPD_max_abs"],
            max_abs_pv,
            flags)
        summary = {
            "scale": scale,
            "job_name": job_name,
            "status": "OK",
            "min_compression_thickness_um": min_row["avg_thickness_um"],
            "residual_thickness_um": residual_row["avg_thickness_um"],
            "residual_basis_step": residual_row["step"],
            "residual_top_rf2_N": residual_row["top_rf2_N"],
            "residual_max_cpress_MPa": residual_row["max_cpress_MPa"],
            "max_cpress_MPa": max_cp,
            "max_compressive_press_MPa": max_pr,
            "max_signed_plastic_vol_strain": max_pv,
            "min_signed_plastic_vol_strain": min_pv,
            "max_abs_plastic_vol_strain": max_abs_pv,
            "ALLPD_max_abs": energy["ALLPD_max_abs"],
            "ALLPD_final": energy["ALLPD_final"],
            "ALLIE_max_abs": energy["ALLIE_max_abs"],
            "ALLIE_final": energy["ALLIE_final"],
            "ALLKE_max_abs": energy["ALLKE_max_abs"],
            "ALLKE_final": energy["ALLKE_final"],
            "distortion": flags["distortion"],
            "negative_volume": flags["negative_volume"],
            "zero_time_increment": flags["zero_time_increment"],
            "yield_stress_too_low": flags.get("yield_stress_too_low", False),
            "aborted": flags["aborted"],
            "classification": classification,
        }
        return summary
    finally:
        odb.close()


def write_history_csv(path, rows):
    fields = ["scale", "job_name", "step", "frame_index", "step_time_s",
              "total_time_s", "top_u2_mm", "top_rf2_N",
              "avg_thickness_um", "min_thickness_um", "max_thickness_um",
              "max_cpress_MPa", "max_compressive_press_MPa",
              "max_signed_plastic_vol_strain",
              "min_signed_plastic_vol_strain",
              "max_abs_plastic_vol_strain",
              "unloaded_zero_force_frame"]
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_summary_csv(path, summaries):
    fields = ["scale", "job_name", "status",
              "min_compression_thickness_um", "residual_thickness_um",
              "residual_basis_step", "residual_top_rf2_N",
              "residual_max_cpress_MPa", "max_cpress_MPa",
              "max_compressive_press_MPa",
              "max_signed_plastic_vol_strain",
              "min_signed_plastic_vol_strain",
              "max_abs_plastic_vol_strain",
              "ALLPD_max_abs", "ALLPD_final",
              "ALLIE_max_abs", "ALLIE_final",
              "ALLKE_max_abs", "ALLKE_final",
              "distortion", "negative_volume", "zero_time_increment",
              "yield_stress_too_low", "aborted", "classification"]
    with open(path, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in summaries:
            writer.writerow(row)


# ----------------------------- 简易 PNG 绘图 -----------------------------
def _png_chunk(tag, data):
    return (struct.pack("!I", len(data)) + tag + data +
            struct.pack("!I", zlib.crc32(tag + data) & 0xffffffff))


def _write_png(path, width, height, pixels):
    raw = b"".join(b"\x00" + pixels[y * width * 3:(y + 1) * width * 3]
                   for y in range(height))
    data = b"\x89PNG\r\n\x1a\n"
    data += _png_chunk(b"IHDR", struct.pack("!IIBBBBB", width, height, 8, 2,
                                            0, 0, 0))
    data += _png_chunk(b"IDAT", zlib.compress(raw, 9))
    data += _png_chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(data)


def _set_px(pixels, width, height, x, y, color):
    if 0 <= x < width and 0 <= y < height:
        i = (y * width + x) * 3
        pixels[i:i + 3] = bytes(color)


def _line(pixels, width, height, x0, y0, x1, y1, color):
    x0, y0, x1, y1 = int(x0), int(y0), int(x1), int(y1)
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        for ox in (-1, 0, 1):
            for oy in (-1, 0, 1):
                _set_px(pixels, width, height, x0 + ox, y0 + oy, color)
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy


def write_curve_png(path, rows):
    """用纯 Python 画厚度-时间曲线，避免依赖 matplotlib/PIL。"""
    width, height = 1000, 650
    ml, mr, mt, mb = 85, 40, 45, 75
    pw, ph = width - ml - mr, height - mt - mb
    pixels = bytearray([255] * width * height * 3)
    times = [r["total_time_s"] for r in rows]
    vals = [r["avg_thickness_um"] for r in rows]
    xmin, xmax = min(times), max(times)
    ymin = min(130.0, min(vals) - 3.0)
    ymax = max(152.0, max(vals) + 3.0)

    def mx(x):
        return ml + (x - xmin) / (xmax - xmin) * pw if xmax > xmin else ml

    def my(y):
        return mt + (ymax - y) / (ymax - ymin) * ph if ymax > ymin else mt

    grid = (225, 225, 225)
    axis = (20, 20, 20)
    red = (220, 30, 30)
    blue = (40, 100, 210)
    gray = (125, 125, 125)
    for i in range(6):
        x = ml + i * pw / 5.0
        _line(pixels, width, height, x, mt, x, mt + ph, grid)
    for i in range(6):
        y = mt + i * ph / 5.0
        _line(pixels, width, height, ml, y, ml + pw, y, grid)
    _line(pixels, width, height, ml, mt, ml, mt + ph, axis)
    _line(pixels, width, height, ml, mt + ph, ml + pw, mt + ph, axis)
    _line(pixels, width, height, ml, my(INITIAL_THICKNESS_UM),
          ml + pw, my(INITIAL_THICKNESS_UM), gray)
    _line(pixels, width, height, ml, my(TARGET_THICKNESS_UM),
          ml + pw, my(TARGET_THICKNESS_UM), blue)
    pts = [(mx(t), my(v)) for t, v in zip(times, vals)]
    for p0, p1 in zip(pts[:-1], pts[1:]):
        _line(pixels, width, height, p0[0], p0[1], p1[0], p1[1], red)
    _write_png(path, width, height, pixels)


def write_failure_png(path):
    """失败工况没有帧数据时输出一个简单占位图。"""
    width, height = 1000, 650
    pixels = bytearray([255] * width * height * 3)
    red = (220, 30, 30)
    gray = (210, 210, 210)
    for x in range(80, 920, 80):
        _line(pixels, width, height, x, 60, x, 570, gray)
    for y in range(80, 580, 80):
        _line(pixels, width, height, 80, y, 920, y, gray)
    _line(pixels, width, height, 80, 60, 80, 570, (20, 20, 20))
    _line(pixels, width, height, 80, 570, 920, 570, (20, 20, 20))
    _line(pixels, width, height, 300, 180, 700, 480, red)
    _line(pixels, width, height, 700, 180, 300, 480, red)
    _write_png(path, width, height, pixels)


def write_all_curves_png(path, history_paths):
    all_rows = []
    for scale, hpath in history_paths:
        rows = []
        with open(hpath, newline="") as f:
            for r in csv.DictReader(f):
                rows.append((float(r["total_time_s"]),
                             float(r["avg_thickness_um"])))
        all_rows.append((scale, rows))
    if not all_rows:
        return
    width, height = 1000, 650
    ml, mr, mt, mb = 85, 40, 45, 75
    pw, ph = width - ml - mr, height - mt - mb
    pixels = bytearray([255] * width * height * 3)
    all_t = [t for _, rows in all_rows for t, _ in rows]
    all_v = [v for _, rows in all_rows for _, v in rows]
    xmin, xmax = min(all_t), max(all_t)
    ymin = min(130.0, min(all_v) - 3.0)
    ymax = max(152.0, max(all_v) + 3.0)

    def mx(x):
        return ml + (x - xmin) / (xmax - xmin) * pw

    def my(y):
        return mt + (ymax - y) / (ymax - ymin) * ph

    colors = [(220, 30, 30), (30, 120, 210), (40, 160, 70),
              (210, 140, 20), (140, 70, 190), (30, 30, 30)]
    for i in range(6):
        x = ml + i * pw / 5.0
        _line(pixels, width, height, x, mt, x, mt + ph, (225, 225, 225))
        y = mt + i * ph / 5.0
        _line(pixels, width, height, ml, y, ml + pw, y, (225, 225, 225))
    _line(pixels, width, height, ml, mt, ml, mt + ph, (20, 20, 20))
    _line(pixels, width, height, ml, mt + ph, ml + pw, mt + ph, (20, 20, 20))
    _line(pixels, width, height, ml, my(150.0), ml + pw, my(150.0),
          (125, 125, 125))
    _line(pixels, width, height, ml, my(135.0), ml + pw, my(135.0),
          (40, 100, 210))
    for idx, (_, rows) in enumerate(all_rows):
        color = colors[idx % len(colors)]
        pts = [(mx(t), my(v)) for t, v in rows]
        for p0, p1 in zip(pts[:-1], pts[1:]):
            _line(pixels, width, height, p0[0], p0[1], p1[0], p1[1],
                  color)
    _write_png(path, width, height, pixels)


def main():
    os.chdir(WORKDIR)
    base_model = open_base_model()
    base_cap = read_base_cap_hardening_from_inp()
    print("Base Cap Hardening rows: %d" % len(base_cap))

    # 按要求先运行 S=1.00，验证能复现基准，再运行其他系数。
    run_order = (1.00, 0.02, 0.05, 0.10, 0.20, 0.50)
    summaries = []
    history_paths = []

    for scale in run_order:
        _, job_name, scaled = create_scaled_model(base_model, scale, base_cap)
        print("Scale %.3g first cap pressure %.6g MPa" %
              (scale, scaled[0][0]))
        if RUN_JOBS:
            submit_job(job_name)
        summary = extract_job_results(job_name, scale)
        summaries.append(summary)
        hpath = os.path.join(WORKDIR, "cap%s_thickness_history.csv" %
                             scale_tag(scale).lower())
        if os.path.exists(hpath):
            history_paths.append((scale, hpath))

    summaries = sorted(summaries, key=lambda r: float(r["scale"]))
    write_summary_csv(os.path.join(WORKDIR,
                                   "cap_hardening_sensitivity_summary.csv"),
                      summaries)
    write_all_curves_png(os.path.join(
        WORKDIR, "cap_hardening_sensitivity_all_curves.png"), history_paths)
    mdb.saveAs(pathName=SENSITIVITY_CAE)
    print("Sensitivity finished.")
    for row in summaries:
        residual = row.get("residual_thickness_um", "")
        if residual == "":
            print("S=%.2f residual=N/A class=%s" %
                  (row["scale"], row["classification"]))
        else:
            print("S=%.2f residual=%.6f um class=%s" %
                  (row["scale"], float(residual), row["classification"]))


if __name__ == "__main__":
    main()
