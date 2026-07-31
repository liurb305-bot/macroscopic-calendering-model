# -*- coding: utf-8 -*-
from __future__ import print_function

"""
三维局部静态辊压等效模型。

运行方式：
    abaqus cae noGUI=create_selfsupport_dpc_local_static_press.py

本脚本自动生成完整INP，并将INP导入CAE后保存CAE文件；不自动提交Job。
说明：
1. 当前模型严格为单层干法自支撑膜，无集流体、无三层结构。
2. 刚体辊为局部离散刚体壳弧面，只保留接触附近区域。
3. Abaqus 2025 在当前环境中对 PartFromNodesAndElements 触发过内核异常，
   因此最终建模采用“完整INP关键字后备 + 导入CAE”的稳健路径。
4. DPC材料关键字完整写入，并在导出后自动检查 *Cap Plasticity 与 *Cap Hardening。
"""

import math
import os

from abaqus import mdb


# -----------------------------------------------------------------------------
# 1. 用户可修改参数区，单位体系：mm, N, MPa
# -----------------------------------------------------------------------------

WORK_DIR = r"E:\abaqus\3Dceshi3.1"
MODEL_NAME = "SelfSupport_DPC_LocalStaticPress_20pct_Ch3_1"
JOB_NAME = MODEL_NAME

FILM_LENGTH_X = 10.0
FILM_WIDTH_Z = 20.0
FILM_THICKNESS_Y = 0.150

ROLLER_RADIUS = 450.0
ROLLER_AXIAL_LENGTH_Z = 24.0
ROLLER_ARC_X_MIN = -6.0
ROLLER_ARC_X_MAX = 6.0
ROLLER_ARC_TARGET_SIZE = 0.075
ROLLER_Z_TARGET_SIZE = 0.75

TARGET_REDUCTION = 0.030
FRICTION = 0.05

FILM_FINE_X_MIN = -4.0
FILM_FINE_X_MAX = 4.0
FILM_FINE_X_SIZE = 0.075
FILM_COARSE_X_SIZE = 0.25
FILM_Z_SIZE = 1.0
FILM_Y_LAYERS = 5

INITIAL_INC = 0.01
MIN_INC = 1.0e-8
MAX_INC = 0.05
STEP_TIME = 1.0
ENABLE_AUTOMATIC_STABILIZATION = True
AUTOMATIC_STABILIZATION_FACTOR = 1.0e-4

ELASTIC_E = 20.0
ELASTIC_NU = 0.10

cohesion_d = 1.0
friction_angle_beta = 40.0
cap_eccentricity_R = 0.8
transition_surface_radius_alpha = 0.02
flow_stress_ratio_K = 1.0
CAP_HARDENING_TABLE = (
    (2.0, 0.00),
    (2.5, 0.02),
    (3.5, 0.05),
    (5.0, 0.08),
    (6.5, 0.10),
    (10.0, 0.15),
    (16.0, 0.20),
)


# -----------------------------------------------------------------------------
# 2. 网格和INP写入工具
# -----------------------------------------------------------------------------

def coords_by_size(start, end, target_size):
    n = int(math.ceil(abs(end - start) / target_size))
    if n < 1:
        n = 1
    step = (end - start) / float(n)
    return [start + i * step for i in range(n + 1)]


def merge_coords(*items):
    out = []
    for seq in items:
        for value in seq:
            if not out or abs(value - out[-1]) > 1.0e-9:
                out.append(value)
    return out


def write_label_lines(f, labels, per_line=16):
    labels = list(labels)
    for i in range(0, len(labels), per_line):
        f.write(", ".join(str(x) for x in labels[i:i + per_line]) + "\n")


def write_nodes(f, nodes):
    for label, x, y, z in nodes:
        f.write("%d, %.12g, %.12g, %.12g\n" % (label, x, y, z))


def write_elements(f, elements):
    for label, conn in elements:
        f.write("%d, %s\n" % (label, ", ".join(str(n) for n in conn)))


def check_initial_gap():
    top_center_y = ROLLER_RADIUS + FILM_THICKNESS_Y / 2.0
    bottom_center_y = -top_center_y
    initial_gap = (top_center_y - ROLLER_RADIUS) - (bottom_center_y + ROLLER_RADIUS)
    if abs(initial_gap - FILM_THICKNESS_Y) > 1.0e-9:
        raise RuntimeError("初始辊缝 %.12g 与膜厚 %.12g 不一致。" % (initial_gap, FILM_THICKNESS_Y))
    print("初始辊缝检查通过：gap=%.6f mm，膜厚=%.6f mm，无初始穿透。" %
          (initial_gap, FILM_THICKNESS_Y))


def build_film_mesh():
    x_left = coords_by_size(-FILM_LENGTH_X / 2.0, FILM_FINE_X_MIN, FILM_COARSE_X_SIZE)
    x_mid = coords_by_size(FILM_FINE_X_MIN, FILM_FINE_X_MAX, FILM_FINE_X_SIZE)
    x_right = coords_by_size(FILM_FINE_X_MAX, FILM_LENGTH_X / 2.0, FILM_COARSE_X_SIZE)
    xs = merge_coords(x_left, x_mid, x_right)
    ys = coords_by_size(-FILM_THICKNESS_Y / 2.0, FILM_THICKNESS_Y / 2.0,
                        FILM_THICKNESS_Y / float(FILM_Y_LAYERS))
    zs = coords_by_size(-FILM_WIDTH_Z / 2.0, FILM_WIDTH_Z / 2.0, FILM_Z_SIZE)

    nodes = []
    node = {}
    label = 1
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            for k, z in enumerate(zs):
                node[(i, j, k)] = label
                nodes.append((label, x, y, z))
                label += 1

    elements = []
    elem = {}
    label = 1
    for i in range(len(xs) - 1):
        for j in range(len(ys) - 1):
            for k in range(len(zs) - 1):
                conn = (
                    node[(i, j, k)],
                    node[(i + 1, j, k)],
                    node[(i + 1, j + 1, k)],
                    node[(i, j + 1, k)],
                    node[(i, j, k + 1)],
                    node[(i + 1, j, k + 1)],
                    node[(i + 1, j + 1, k + 1)],
                    node[(i, j + 1, k + 1)],
                )
                elem[(i, j, k)] = label
                elements.append((label, conn))
                label += 1

    mid_z = min(range(len(zs)), key=lambda kk: abs(zs[kk]))
    data = {
        "nodes": nodes,
        "elements": elements,
        "top_nodes": [node[(i, len(ys) - 1, k)] for i in range(len(xs)) for k in range(len(zs))],
        "bottom_nodes": [node[(i, 0, k)] for i in range(len(xs)) for k in range(len(zs))],
        "pin_x": [node[(0, 0, mid_z)]],
        "pin_z_a": [node[(0, 0, mid_z)]],
        "pin_z_b": [node[(len(xs) - 1, 0, mid_z)]],
        "top_face_elems": [elem[(i, len(ys) - 2, k)] for i in range(len(xs) - 1) for k in range(len(zs) - 1)],
        "bottom_face_elems": [elem[(i, 0, k)] for i in range(len(xs) - 1) for k in range(len(zs) - 1)],
    }
    print("Film mesh: nx=%d, ny=%d, nz=%d, elements=%d" %
          (len(xs) - 1, len(ys) - 1, len(zs) - 1, len(elements)))
    return data


def build_roller_mesh(is_top):
    center_y = ROLLER_RADIUS + FILM_THICKNESS_Y / 2.0
    if not is_top:
        center_y = -center_y
    xs = coords_by_size(ROLLER_ARC_X_MIN, ROLLER_ARC_X_MAX, ROLLER_ARC_TARGET_SIZE)
    zs = coords_by_size(-ROLLER_AXIAL_LENGTH_Z / 2.0, ROLLER_AXIAL_LENGTH_Z / 2.0,
                        ROLLER_Z_TARGET_SIZE)

    nodes = []
    node = {}
    label = 1
    for i, x in enumerate(xs):
        root = math.sqrt(ROLLER_RADIUS * ROLLER_RADIUS - x * x)
        y = center_y - root if is_top else center_y + root
        for k, z in enumerate(zs):
            node[(i, k)] = label
            nodes.append((label, x, y, z))
            label += 1

    elements = []
    label = 1
    for i in range(len(xs) - 1):
        for k in range(len(zs) - 1):
            if is_top:
                # side1法向约为 -Y，指向上辊下侧的极片。
                conn = (node[(i, k)], node[(i + 1, k)], node[(i + 1, k + 1)], node[(i, k + 1)])
            else:
                # 反向排序使side1法向约为 +Y，指向下辊上侧的极片。
                conn = (node[(i, k)], node[(i, k + 1)], node[(i + 1, k + 1)], node[(i + 1, k)])
            elements.append((label, conn))
            label += 1

    rp_label = len(nodes) + 1
    nodes.append((rp_label, 0.0, center_y, 0.0))
    arc_length = ROLLER_RADIUS * (math.asin(ROLLER_ARC_X_MAX / ROLLER_RADIUS) -
                                 math.asin(ROLLER_ARC_X_MIN / ROLLER_RADIUS))
    print("%s roller arc: projected X %.3f..%.3f, arc length %.4f, arc elems=%d, z elems=%d" %
          ("Top" if is_top else "Bottom", ROLLER_ARC_X_MIN, ROLLER_ARC_X_MAX,
           arc_length, len(xs) - 1, len(zs) - 1))
    return {"nodes": nodes, "elements": elements, "rp": [rp_label],
            "all_elems": [e[0] for e in elements]}


def write_part_film(f, film):
    f.write("*Part, name=Film\n")
    f.write("*Node\n")
    write_nodes(f, film["nodes"])
    f.write("*Element, type=C3D8R, elset=FILM_ALL_ELEMS\n")
    write_elements(f, film["elements"])
    for name in ("top_nodes", "bottom_nodes", "pin_x", "pin_z_a", "pin_z_b"):
        f.write("*Nset, nset=%s\n" % name.upper())
        write_label_lines(f, film[name])
    f.write("*Elset, elset=FILM_ALL_ELEMS\n")
    write_label_lines(f, [e[0] for e in film["elements"]])
    f.write("*Elset, elset=FILM_TOP_FACE_ELEMS\n")
    write_label_lines(f, film["top_face_elems"])
    f.write("*Elset, elset=FILM_BOTTOM_FACE_ELEMS\n")
    write_label_lines(f, film["bottom_face_elems"])
    f.write("*Solid Section, elset=FILM_ALL_ELEMS, material=DrySelfSupportFilm_DPC\n")
    f.write(",\n")
    f.write("*End Part\n")


def write_part_roller(f, part_name, roller):
    f.write("*Part, name=%s\n" % part_name)
    f.write("*Node\n")
    write_nodes(f, roller["nodes"])
    f.write("*Element, type=R3D4, elset=%s_ALL_ELEMS\n" % part_name.upper())
    write_elements(f, roller["elements"])
    f.write("*Nset, nset=%s_RP\n" % part_name.upper())
    write_label_lines(f, roller["rp"])
    f.write("*Elset, elset=%s_ALL_ELEMS\n" % part_name.upper())
    write_label_lines(f, roller["all_elems"])
    f.write("*End Part\n")


def write_material(f):
    initial_cap_position = CAP_HARDENING_TABLE[0][0]
    f.write("**\n** MATERIALS\n**\n")
    f.write("*Material, name=DrySelfSupportFilm_DPC\n")
    f.write("*Elastic\n")
    f.write("%g, %g\n" % (ELASTIC_E, ELASTIC_NU))
    f.write("** Drucker-Prager Cap材料参数映射：\n")
    f.write("** *Cap Plasticity: cohesion_d, friction_angle_beta, cap_eccentricity_R,\n")
    f.write("** initial_cap_position(按需求取Cap hardening第一行压力),\n")
    f.write("** transition_surface_radius_alpha, flow_stress_ratio_K\n")
    f.write("*Cap Plasticity\n")
    f.write("%g, %g, %g, %g, %g, %g\n" %
            (cohesion_d, friction_angle_beta, cap_eccentricity_R,
             initial_cap_position, transition_surface_radius_alpha, flow_stress_ratio_K))
    f.write("*Cap Hardening\n")
    for p, evp in CAP_HARDENING_TABLE:
        f.write("%g, %g\n" % (p, evp))


def write_assembly(f, film, top, bottom):
    f.write("**\n** ASSEMBLY\n**\n")
    f.write("*Assembly, name=Assembly\n")
    f.write("*Instance, name=Film-1, part=Film\n*End Instance\n")
    f.write("*Instance, name=TopRoller-1, part=TopRoller\n*End Instance\n")
    f.write("*Instance, name=BottomRoller-1, part=BottomRoller\n*End Instance\n")

    # 装配级节点集便于边界和ODB后处理。
    set_map = (
        ("FILM_TOP_NODES", "Film-1", film["top_nodes"]),
        ("FILM_BOTTOM_NODES", "Film-1", film["bottom_nodes"]),
        ("PIN_X_NODE", "Film-1", film["pin_x"]),
        ("PIN_Z_NODE_A", "Film-1", film["pin_z_a"]),
        ("PIN_Z_NODE_B", "Film-1", film["pin_z_b"]),
        ("TOPROLLER_RP", "TopRoller-1", top["rp"]),
        ("BOTTOMROLLER_RP", "BottomRoller-1", bottom["rp"]),
    )
    for name, inst, labels in set_map:
        f.write("*Nset, nset=%s, instance=%s\n" % (name, inst))
        write_label_lines(f, labels)

    f.write("*Surface, type=ELEMENT, name=FILM_TOP_SURF\n")
    f.write("Film-1.FILM_TOP_FACE_ELEMS, S5\n")
    f.write("*Surface, type=ELEMENT, name=FILM_BOTTOM_SURF\n")
    f.write("Film-1.FILM_BOTTOM_FACE_ELEMS, S3\n")

    # 局部刚体壳弧面：SPOS/SNEG都写入，避免壳面法向错误导致接触失败。
    f.write("*Surface, type=ELEMENT, name=TOPROLLER_CONTACT_SURF\n")
    f.write("TopRoller-1.TOPROLLER_ALL_ELEMS, SPOS\n")
    f.write("TopRoller-1.TOPROLLER_ALL_ELEMS, SNEG\n")
    f.write("*Surface, type=ELEMENT, name=BOTTOMROLLER_CONTACT_SURF\n")
    f.write("BottomRoller-1.BOTTOMROLLER_ALL_ELEMS, SPOS\n")
    f.write("BottomRoller-1.BOTTOMROLLER_ALL_ELEMS, SNEG\n")

    f.write("*Rigid Body, ref node=TOPROLLER_RP, elset=TopRoller-1.TOPROLLER_ALL_ELEMS\n")
    f.write("*Rigid Body, ref node=BOTTOMROLLER_RP, elset=BottomRoller-1.BOTTOMROLLER_ALL_ELEMS\n")
    f.write("*End Assembly\n")


def write_initial_bcs_and_contact(f):
    f.write("**\n** INTERACTIONS\n**\n")
    f.write("*Surface Interaction, name=RollerFilmContact\n")
    f.write("*Surface Behavior, pressure-overclosure=HARD\n")
    if abs(FRICTION) > 1.0e-12:
        f.write("*Friction\n")
        f.write("%g\n" % FRICTION)
    else:
        f.write("*Friction\n0.\n")
    f.write("*Contact Pair, interaction=RollerFilmContact, type=SURFACE TO SURFACE\n")
    f.write("FILM_TOP_SURF, TOPROLLER_CONTACT_SURF\n")
    f.write("*Contact Pair, interaction=RollerFilmContact, type=SURFACE TO SURFACE\n")
    f.write("FILM_BOTTOM_SURF, BOTTOMROLLER_CONTACT_SURF\n")

    f.write("**\n** INITIAL BOUNDARY CONDITIONS\n**\n")
    f.write("*Boundary\n")
    f.write("BOTTOMROLLER_RP, 1, 6, 0.\n")
    f.write("TOPROLLER_RP, 1, 1, 0.\n")
    f.write("TOPROLLER_RP, 3, 6, 0.\n")
    f.write("PIN_X_NODE, 1, 1, 0.\n")
    f.write("PIN_Z_NODE_A, 3, 3, 0.\n")
    f.write("PIN_Z_NODE_B, 3, 3, 0.\n")


def write_outputs(f):
    f.write("*Output, field\n")
    f.write("*Node Output\n")
    f.write("U\n")
    f.write("*Element Output, directions=YES\n")
    f.write("S, LE, PE, PEEQ, PRESS, EVOL\n")
    f.write("** 接触输出目标变量：CPRESS, CSHEAR1, CSHEAR2, COPEN；\n")
    f.write("** 当前求解器关键字形式使用PRESELECT，避免显式变量名不被该选项接受而中断。\n")
    f.write("*Contact Output, variable=PRESELECT\n")
    f.write("*Output, history\n")
    f.write("*Node Output, nset=TOPROLLER_RP\n")
    f.write("U2, RF2\n")
    f.write("*Energy Output\n")
    if ENABLE_AUTOMATIC_STABILIZATION:
        f.write("ALLIE, ALLPD, ALLSD\n")
    else:
        f.write("ALLIE, ALLPD\n")


def write_step(f, name, u2_value):
    f.write("** ----------------------------------------------------------------\n")
    f.write("*Step, name=%s, nlgeom=YES\n" % name)
    if ENABLE_AUTOMATIC_STABILIZATION:
        f.write("*Static, stabilize=%g\n" % AUTOMATIC_STABILIZATION_FACTOR)
    else:
        f.write("*Static\n")
    f.write("%g, %g, %g, %g\n" % (INITIAL_INC, STEP_TIME, MIN_INC, MAX_INC))
    f.write("*Boundary\n")
    f.write("TOPROLLER_RP, 2, 2, %.12g\n" % u2_value)
    write_outputs(f)
    f.write("*End Step\n")


def write_inp(inp_path):
    check_initial_gap()
    film = build_film_mesh()
    top = build_roller_mesh(is_top=True)
    bottom = build_roller_mesh(is_top=False)

    f = open(inp_path, "w")
    try:
        f.write("*Heading\n")
        f.write("** Job name: %s Model name: %s\n" % (JOB_NAME, MODEL_NAME))
        f.write("** 单层干法自支撑电极膜局部静态法向压下模型；无集流体、无转动、无进料、无显式动力学。\n")
        write_part_film(f, film)
        write_part_roller(f, "TopRoller", top)
        write_part_roller(f, "BottomRoller", bottom)
        write_assembly(f, film, top, bottom)
        write_material(f)
        write_initial_bcs_and_contact(f)
        write_step(f, "Clamp_Down", -TARGET_REDUCTION)
        write_step(f, "Hold", -TARGET_REDUCTION)
        write_step(f, "Unload", 0.0)
    finally:
        f.close()


def ensure_dpc_keywords_in_inp(inp_path):
    text = open(inp_path, "r").read()
    upper = text.upper()
    ok = ("*CAP PLASTICITY" in upper and "*CAP HARDENING" in upper)
    if ok:
        print("INP检查通过：已包含完整 *Cap Plasticity 和 *Cap Hardening。")
    else:
        print("严重警告：最终INP中没有完整DPC/Cap hardening关键字，可能只剩Elastic。")
    return ok


def import_inp_and_save_cae(inp_path, cae_path):
    if MODEL_NAME in mdb.models:
        del mdb.models[MODEL_NAME]
    try:
        mdb.ModelFromInputFile(name=MODEL_NAME, inputFileName=inp_path)
        mdb.saveAs(pathName=cae_path)
        print("已由INP导入并保存CAE：%s" % cae_path)
    except Exception as err:
        print("警告：INP已生成，但导入/保存CAE失败：%s" % err)
        print("请直接检查或在CAE中手动导入INP：%s" % inp_path)


def main():
    if not os.path.isdir(WORK_DIR):
        os.makedirs(WORK_DIR)
    os.chdir(WORK_DIR)
    inp_path = os.path.join(WORK_DIR, JOB_NAME + ".inp")
    cae_path = os.path.join(WORK_DIR, MODEL_NAME + ".cae")
    write_inp(inp_path)
    ensure_dpc_keywords_in_inp(inp_path)
    import_inp_and_save_cae(inp_path, cae_path)
    print("已生成INP：%s" % inp_path)
    print("脚本未提交Job。")


if __name__ == "__main__":
    main()
