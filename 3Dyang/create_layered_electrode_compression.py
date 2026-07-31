# -*- coding: utf-8 -*-
"""建立三层极片平板压缩模型，检查 INP，提交计算并自动后处理。

运行方式：
    abaqus cae noGUI=create_layered_electrode_compression.py
也可以在 Abaqus/CAE 中使用“文件 -> 运行脚本”。
"""

from __future__ import print_function

import os
import re
import sys

from abaqus import Mdb, mdb
from caeModules import *
from abaqusConstants import (
    ANALYSIS,
    ANALYTIC_RIGID_SURFACE,
    CARTESIAN,
    COMPLETED,
    DEFAULT,
    DEFORMABLE_BODY,
    DISSIPATED_ENERGY_FRACTION,
    FINITE,
    FIXED,
    FRICTIONLESS,
    HARD,
    HEX,
    NONE,
    OFF,
    OMIT,
    ON,
    PERCENTAGE,
    STANDARD,
    STRUCTURED,
    THREE_D,
    UNSET,
    XZPLANE,
)
import mesh
import regionToolset


# =============================================================================
# 关键参数区：mm、N、MPa 单位制
# =============================================================================
WORK_DIR = r"E:\abaqus\3Dyang"
MODEL_NAME = "Yang_Macro_LayeredElectrode_Compression"
JOB_NAME = MODEL_NAME
CAE_PATH = os.path.join(WORK_DIR, JOB_NAME + ".cae")
INP_PATH = os.path.join(WORK_DIR, JOB_NAME + ".inp")

# 几何：X=长度，Y=厚度，Z=宽度
X_LENGTH = 1.0
Z_WIDTH = 1.0
LOWER_ACTIVE_THICKNESS = 0.075
AL_THICKNESS = 0.015
UPPER_ACTIVE_THICKNESS = 0.075
TOTAL_THICKNESS = (
    LOWER_ACTIVE_THICKNESS + AL_THICKNESS + UPPER_ACTIVE_THICKNESS
)
Y_BOTTOM = -0.5 * TOTAL_THICKNESS
Y_LOWER_INTERFACE = -0.5 * AL_THICKNESS
Y_UPPER_INTERFACE = 0.5 * AL_THICKNESS
Y_TOP = 0.5 * TOTAL_THICKNESS
COMPRESSION_U2 = -0.015

# 活性层等效弹塑性材料
E_ACTIVE = 609.0
NU_ACTIVE = 0.10
ACTIVE_PLASTIC_TABLE = (
    (10.0, 0.00),
    (20.0, 0.10),
    (35.0, 0.20),
    (62.0, 0.30),
    (92.0, 0.40),
    (145.0, 0.50),
    (230.0, 0.63),
)

# 铝集流体仅允许线弹性
E_AL = 72000.0
NU_AL = 0.33
ACTIVE_MATERIAL_NAME = "Active_Layer"
AL_MATERIAL_NAME = "Al_Current_Collector"

# 接触与稳定化
FRICTION = 0.0
STABILIZATION_MAGNITUDE = 2.0e-4
PLATE_OVERHANG = 0.10
PLATE_X_LENGTH = X_LENGTH + 2.0 * PLATE_OVERHANG
PLATE_Z_WIDTH = Z_WIDTH + 2.0 * PLATE_OVERHANG

# 网格：10 × (3+2+3) × 10 = 800 个 C3D8R
N_ELEM_X = 10
N_ELEM_Z = 10
N_ELEM_ACTIVE_Y = 3
N_ELEM_AL_Y = 2

# 分析与计算资源
INITIAL_INCREMENT = 0.02
MAX_INCREMENT = 0.05
MIN_INCREMENT = 1.0e-8
MAX_NUM_INCREMENT = 1000
NUM_CPUS = 4

# 固定名称，供后处理脚本读取 ODB
ELECTRODE_PART_NAME = "LayeredElectrode"
ELECTRODE_INSTANCE_NAME = "LAYERED_ELECTRODE-1"
LOWER_ACTIVE_SET = "LOWER_ACTIVE"
AL_SET = "AL_COLLECTOR"
UPPER_ACTIVE_SET = "UPPER_ACTIVE"
BOTTOM_NODE_SET = "BOTTOM_SURFACE_NODES"
LOWER_INTERFACE_NODE_SET = "LOWER_ACTIVE_AL_INTERFACE_NODES"
UPPER_INTERFACE_NODE_SET = "AL_UPPER_ACTIVE_INTERFACE_NODES"
TOP_NODE_SET = "TOP_SURFACE_NODES"
TOP_RP_SET = "TOP_RP"


def validate_parameters():
    """检查几何、材料和网格参数。"""
    if not os.path.isdir(WORK_DIR):
        os.makedirs(WORK_DIR)
    if abs(TOTAL_THICKNESS - 0.165) > 1.0e-12:
        raise ValueError("三层厚度之和必须为 0.165 mm。")
    if abs(ACTIVE_PLASTIC_TABLE[0][1]) > 1.0e-15:
        raise ValueError("活性层塑性曲线第一行塑性应变必须为 0.00。")
    previous_strain = -1.0
    for stress, plastic_strain in ACTIVE_PLASTIC_TABLE:
        if stress <= 0.0 or plastic_strain <= previous_strain:
            raise ValueError("活性层塑性曲线必须具有正应力和严格递增的塑性应变。")
        previous_strain = plastic_strain
    if min(N_ELEM_X, N_ELEM_Z, N_ELEM_ACTIVE_Y, N_ELEM_AL_Y) < 1:
        raise ValueError("网格划分数必须为正整数。")
    if N_ELEM_ACTIVE_Y < 3 or N_ELEM_AL_Y not in (1, 2):
        raise ValueError("活性层至少3层单元，铝层必须为1或2层单元。")
    if FRICTION != 0.0:
        raise ValueError("本验证模型要求 friction=0.0。")


def create_layered_electrode(model):
    """创建一个分区实体，并为三个共享节点分区赋予不同截面。"""
    sketch_name = "__layered_electrode_profile__"
    sketch = model.ConstrainedSketch(
        name=sketch_name,
        sheetSize=4.0 * max(X_LENGTH, TOTAL_THICKNESS, Z_WIDTH),
    )
    sketch.rectangle(
        point1=(-0.5 * X_LENGTH, Y_BOTTOM),
        point2=(0.5 * X_LENGTH, Y_TOP),
    )
    part = model.Part(
        name=ELECTRODE_PART_NAME,
        dimensionality=THREE_D,
        type=DEFORMABLE_BODY,
    )
    part.BaseSolidExtrude(sketch=sketch, depth=Z_WIDTH)
    del model.sketches[sketch_name]

    # 先切下层界面，再在剩余上部实体中切上层界面。
    datum_lower = part.DatumPlaneByPrincipalPlane(
        principalPlane=XZPLANE, offset=Y_LOWER_INTERFACE
    )
    part.PartitionCellByDatumPlane(
        datumPlane=part.datums[datum_lower.id], cells=part.cells[:]
    )
    datum_upper = part.DatumPlaneByPrincipalPlane(
        principalPlane=XZPLANE, offset=Y_UPPER_INTERFACE
    )
    upper_combined_cell = part.cells.findAt(
        ((0.0, 0.5 * (Y_UPPER_INTERFACE + Y_TOP), 0.5 * Z_WIDTH),)
    )
    part.PartitionCellByDatumPlane(
        datumPlane=part.datums[datum_upper.id], cells=upper_combined_cell
    )

    lower_cell = part.cells.findAt(
        ((0.0, 0.5 * (Y_BOTTOM + Y_LOWER_INTERFACE), 0.5 * Z_WIDTH),)
    )
    al_cell = part.cells.findAt(((0.0, 0.0, 0.5 * Z_WIDTH),))
    upper_cell = part.cells.findAt(
        ((0.0, 0.5 * (Y_UPPER_INTERFACE + Y_TOP), 0.5 * Z_WIDTH),)
    )
    part.Set(name=LOWER_ACTIVE_SET, cells=lower_cell)
    part.Set(name=AL_SET, cells=al_cell)
    part.Set(name=UPPER_ACTIVE_SET, cells=upper_cell)

    active_material = model.Material(name=ACTIVE_MATERIAL_NAME)
    active_material.Elastic(table=((E_ACTIVE, NU_ACTIVE),))
    active_material.Plastic(table=ACTIVE_PLASTIC_TABLE)
    al_material = model.Material(name=AL_MATERIAL_NAME)
    al_material.Elastic(table=((E_AL, NU_AL),))

    model.HomogeneousSolidSection(
        name="ActiveLayerSection", material=ACTIVE_MATERIAL_NAME, thickness=None
    )
    model.HomogeneousSolidSection(
        name="AlCollectorSection", material=AL_MATERIAL_NAME, thickness=None
    )
    part.SectionAssignment(
        region=part.sets[LOWER_ACTIVE_SET], sectionName="ActiveLayerSection"
    )
    part.SectionAssignment(
        region=part.sets[UPPER_ACTIVE_SET], sectionName="ActiveLayerSection"
    )
    part.SectionAssignment(
        region=part.sets[AL_SET], sectionName="AlCollectorSection"
    )

    # 根据边方向和Y向分段长度分别设置单元数。
    for edge in part.edges:
        vertex_ids = edge.getVertices()
        if len(vertex_ids) != 2:
            raise RuntimeError("发现无法按端点识别方向的边。")
        point_0 = part.vertices[vertex_ids[0]].pointOn[0]
        point_1 = part.vertices[vertex_ids[1]].pointOn[0]
        delta = [abs(point_1[i] - point_0[i]) for i in range(3)]
        axis = delta.index(max(delta))
        if axis == 0:
            count = N_ELEM_X
        elif axis == 2:
            count = N_ELEM_Z
        else:
            edge_length = delta[1]
            count = N_ELEM_AL_Y if abs(edge_length - AL_THICKNESS) < 1.0e-8 else N_ELEM_ACTIVE_Y
        part.seedEdgeByNumber(edges=(edge,), number=count, constraint=FIXED)

    part.setMeshControls(regions=part.cells[:], elemShape=HEX, technique=STRUCTURED)
    element_type = mesh.ElemType(elemCode=mesh.C3D8R, elemLibrary=STANDARD)
    part.setElementType(regions=(part.cells[:],), elemTypes=(element_type,))
    part.generateMesh()

    # 外表面用于接触。
    top_face = part.faces.findAt(((0.0, Y_TOP, 0.5 * Z_WIDTH),))
    bottom_face = part.faces.findAt(((0.0, Y_BOTTOM, 0.5 * Z_WIDTH),))
    part.Surface(name="ELECTRODE_TOP", side1Faces=top_face)
    part.Surface(name="ELECTRODE_BOTTOM", side1Faces=bottom_face)

    # 四个节点面用于总厚度和三层独立厚度的当前坐标计算。
    tol = 1.0e-7
    node_planes = (
        (BOTTOM_NODE_SET, Y_BOTTOM),
        (LOWER_INTERFACE_NODE_SET, Y_LOWER_INTERFACE),
        (UPPER_INTERFACE_NODE_SET, Y_UPPER_INTERFACE),
        (TOP_NODE_SET, Y_TOP),
    )
    for set_name, y_value in node_planes:
        nodes = part.nodes.getByBoundingBox(
            xMin=-0.5 * X_LENGTH - tol,
            xMax=0.5 * X_LENGTH + tol,
            yMin=y_value - tol,
            yMax=y_value + tol,
            zMin=-tol,
            zMax=Z_WIDTH + tol,
        )
        if len(nodes) != (N_ELEM_X + 1) * (N_ELEM_Z + 1):
            raise RuntimeError("节点集 {} 的节点数不符合预期。".format(set_name))
        part.Set(name=set_name, nodes=nodes)

    # 两个角点只抑制X/Z刚体漂移，不限制Y向压缩。
    corner_a = part.nodes.getByBoundingBox(
        xMin=-0.5 * X_LENGTH - tol,
        xMax=-0.5 * X_LENGTH + tol,
        yMin=Y_BOTTOM - tol,
        yMax=Y_BOTTOM + tol,
        zMin=-tol,
        zMax=tol,
    )
    corner_b = part.nodes.getByBoundingBox(
        xMin=0.5 * X_LENGTH - tol,
        xMax=0.5 * X_LENGTH + tol,
        yMin=Y_BOTTOM - tol,
        yMax=Y_BOTTOM + tol,
        zMin=-tol,
        zMax=tol,
    )
    if len(corner_a) != 1 or len(corner_b) != 1:
        raise RuntimeError("未能唯一识别两个防漂移角点。")
    part.Set(name="DRIFT_CORNER_A", nodes=corner_a)
    part.Set(name="DRIFT_CORNER_B", nodes=corner_b)

    expected_elements = N_ELEM_X * N_ELEM_Z * (
        2 * N_ELEM_ACTIVE_Y + N_ELEM_AL_Y
    )
    if len(part.elements) != expected_elements:
        raise RuntimeError(
            "网格单元数错误：得到 {}，预期 {}。".format(
                len(part.elements), expected_elements
            )
        )
    return part


def create_rigid_plate(model, part_name, normal_toward_positive_y):
    """创建具有明确接触法向的解析刚体平板。"""
    sketch_name = "__{}_profile__".format(part_name)
    sketch = model.ConstrainedSketch(
        name=sketch_name, sheetSize=4.0 * max(PLATE_X_LENGTH, PLATE_Z_WIDTH)
    )
    if normal_toward_positive_y:
        point_1 = (-0.5 * PLATE_X_LENGTH, 0.0)
        point_2 = (0.5 * PLATE_X_LENGTH, 0.0)
    else:
        point_1 = (0.5 * PLATE_X_LENGTH, 0.0)
        point_2 = (-0.5 * PLATE_X_LENGTH, 0.0)
    sketch.Line(point1=point_1, point2=point_2)
    part = model.Part(
        name=part_name,
        dimensionality=THREE_D,
        type=ANALYTIC_RIGID_SURFACE,
    )
    part.AnalyticRigidSurfExtrude(sketch=sketch, depth=PLATE_Z_WIDTH)
    del model.sketches[sketch_name]
    part.ReferencePoint(point=(0.0, 0.0, 0.5 * PLATE_Z_WIDTH))
    part.Surface(name="PLATE_SURFACE", side1Faces=part.faces[:])
    return part


def parse_material_blocks(inp_text):
    """按 *Material 边界解析材料块，避免把其他材料的 Plastic 误判给铝。"""
    blocks = {}
    current_name = None
    current_lines = []
    material_pattern = re.compile(r"name\s*=\s*([^,\s]+)", re.IGNORECASE)
    for line in inp_text.splitlines():
        stripped = line.strip()
        if stripped.lower().startswith("*material"):
            if current_name is not None:
                blocks[current_name.lower()] = current_lines
            match = material_pattern.search(stripped)
            if not match:
                raise RuntimeError("发现没有 name 参数的 *Material 关键字。")
            current_name = match.group(1).strip('"\'')
            current_lines = [stripped]
        elif current_name is not None:
            current_lines.append(stripped)
    if current_name is not None:
        blocks[current_name.lower()] = current_lines
    return blocks


def check_inp_materials():
    """严格检查两种材料的关键字；失败时抛错并阻止提交。"""
    if not os.path.isfile(INP_PATH):
        raise RuntimeError("INP 未生成：{}".format(INP_PATH))
    with open(INP_PATH, "r", encoding="utf-8", errors="ignore") as inp_file:
        inp_text = inp_file.read()
    blocks = parse_material_blocks(inp_text)
    active_key = ACTIVE_MATERIAL_NAME.lower()
    al_key = AL_MATERIAL_NAME.lower()
    if active_key not in blocks or al_key not in blocks:
        raise RuntimeError("INP 中缺少 Active_Layer 或 Al_Current_Collector 材料块。")

    active_lines = blocks[active_key]
    al_lines = blocks[al_key]
    active_keywords = [line.lower() for line in active_lines if line.startswith("*")]
    al_keywords = [line.lower() for line in al_lines if line.startswith("*")]
    if not any(line.startswith("*elastic") for line in active_keywords):
        raise RuntimeError("Active_Layer 材料缺少 *Elastic。")
    if not any(line.startswith("*plastic") for line in active_keywords):
        raise RuntimeError("Active_Layer 材料缺少 *Plastic。")
    if not any(line.startswith("*elastic") for line in al_keywords):
        raise RuntimeError("Al_Current_Collector 材料缺少 *Elastic。")
    if any(line.startswith("*plastic") for line in al_keywords):
        raise RuntimeError("严重错误：Al_Current_Collector 中出现 *Plastic，停止提交。")

    plastic_index = next(
        index for index, line in enumerate(active_lines)
        if line.lower().startswith("*plastic")
    )
    first_plastic_data = None
    for line in active_lines[plastic_index + 1:]:
        if not line or line.startswith("**"):
            continue
        if line.startswith("*"):
            break
        first_plastic_data = line
        break
    if first_plastic_data is None:
        raise RuntimeError("Active_Layer 的 *Plastic 没有数据行。")
    values = [float(value.strip()) for value in first_plastic_data.split(",") if value.strip()]
    if len(values) < 2 or abs(values[1]) > 1.0e-15:
        raise RuntimeError("Active_Layer 第一行等效塑性应变不是 0.00。")
    print("INP材料检查通过：活性层含Elastic/Plastic，铝集流体仅含Elastic。")


def clean_previous_analysis_files():
    """删除同名旧求解输出，避免把旧 ODB 误认为本次结果。"""
    extensions = (
        ".com", ".dat", ".env", ".ipm", ".lck", ".log", ".mdl", ".msg",
        ".odb", ".pac", ".prt", ".res", ".sim", ".sta", ".stt",
    )
    for extension in extensions:
        path = os.path.join(WORK_DIR, JOB_NAME + extension)
        if os.path.isfile(path):
            os.remove(path)
    result_path = os.path.join(WORK_DIR, JOB_NAME + "_results.csv")
    if os.path.isfile(result_path):
        os.remove(result_path)


def analysis_completed_successfully():
    """Abaqus 2025 noGUI 中 job.status 可能为 None，因此以 .sta 为准。"""
    sta_path = os.path.join(WORK_DIR, JOB_NAME + ".sta")
    if not os.path.isfile(sta_path):
        return False
    with open(sta_path, "r", encoding="utf-8", errors="ignore") as sta_file:
        return "THE ANALYSIS HAS COMPLETED SUCCESSFULLY" in sta_file.read().upper()


def build_submit_and_postprocess():
    validate_parameters()
    os.chdir(WORK_DIR)
    clean_previous_analysis_files()

    Mdb()
    mdb.models.changeKey(fromName="Model-1", toName=MODEL_NAME)
    model = mdb.models[MODEL_NAME]
    electrode_part = create_layered_electrode(model)
    bottom_plate_part = create_rigid_plate(
        model, "BottomRigidPlate", normal_toward_positive_y=True
    )
    top_plate_part = create_rigid_plate(
        model, "TopRigidPlate", normal_toward_positive_y=False
    )

    assembly = model.rootAssembly
    assembly.DatumCsysByDefault(CARTESIAN)
    electrode_instance = assembly.Instance(
        name=ELECTRODE_INSTANCE_NAME, part=electrode_part, dependent=ON
    )
    assembly.translate(
        instanceList=(ELECTRODE_INSTANCE_NAME,),
        vector=(0.0, 0.0, -0.5 * Z_WIDTH),
    )
    bottom_instance = assembly.Instance(
        name="BOTTOM_PLATE-1", part=bottom_plate_part, dependent=ON
    )
    top_instance = assembly.Instance(
        name="TOP_PLATE-1", part=top_plate_part, dependent=ON
    )
    assembly.translate(
        instanceList=("BOTTOM_PLATE-1",),
        vector=(0.0, Y_BOTTOM, -0.5 * PLATE_Z_WIDTH),
    )
    assembly.translate(
        instanceList=("TOP_PLATE-1",),
        vector=(0.0, Y_TOP, -0.5 * PLATE_Z_WIDTH),
    )

    bottom_rp_id = list(bottom_plate_part.referencePoints.keys())[0]
    top_rp_id = list(top_plate_part.referencePoints.keys())[0]
    bottom_rp_region = regionToolset.Region(
        referencePoints=(bottom_instance.referencePoints[bottom_rp_id],)
    )
    top_rp_region = regionToolset.Region(
        referencePoints=(top_instance.referencePoints[top_rp_id],)
    )
    assembly.Set(
        name="BOTTOM_RP",
        referencePoints=(bottom_instance.referencePoints[bottom_rp_id],),
    )
    assembly.Set(
        name=TOP_RP_SET,
        referencePoints=(top_instance.referencePoints[top_rp_id],),
    )
    model.RigidBody(
        name="RigidBody-Bottom",
        refPointRegion=bottom_rp_region,
        surfaceRegion=bottom_instance.surfaces["PLATE_SURFACE"],
    )
    model.RigidBody(
        name="RigidBody-Top",
        refPointRegion=top_rp_region,
        surfaceRegion=top_instance.surfaces["PLATE_SURFACE"],
    )

    contact_property = model.ContactProperty("PlateElectrodeContact")
    contact_property.NormalBehavior(
        pressureOverclosure=HARD,
        allowSeparation=ON,
        constraintEnforcementMethod=DEFAULT,
    )
    contact_property.TangentialBehavior(formulation=FRICTIONLESS)
    model.SurfaceToSurfaceContactStd(
        name="Contact-Top",
        createStepName="Initial",
        main=top_instance.surfaces["PLATE_SURFACE"],
        secondary=electrode_instance.surfaces["ELECTRODE_TOP"],
        sliding=FINITE,
        interactionProperty="PlateElectrodeContact",
        adjustMethod=NONE,
        initialClearance=OMIT,
    )
    model.SurfaceToSurfaceContactStd(
        name="Contact-Bottom",
        createStepName="Initial",
        main=bottom_instance.surfaces["PLATE_SURFACE"],
        secondary=electrode_instance.surfaces["ELECTRODE_BOTTOM"],
        sliding=FINITE,
        interactionProperty="PlateElectrodeContact",
        adjustMethod=NONE,
        initialClearance=OMIT,
    )

    step_1 = "Step-1 Compression"
    step_2 = "Step-2 Unload"
    for step_name, previous_step in ((step_1, "Initial"), (step_2, step_1)):
        model.StaticStep(
            name=step_name,
            previous=previous_step,
            nlgeom=ON,
            timePeriod=1.0,
            initialInc=INITIAL_INCREMENT,
            maxInc=MAX_INCREMENT,
            minInc=MIN_INCREMENT,
            maxNumInc=MAX_NUM_INCREMENT,
            stabilizationMethod=DISSIPATED_ENERGY_FRACTION,
            stabilizationMagnitude=STABILIZATION_MAGNITUDE,
        )

    model.EncastreBC(
        name="BC-BottomPlate-Fixed",
        createStepName="Initial",
        region=assembly.sets["BOTTOM_RP"],
    )
    model.DisplacementBC(
        name="BC-TopPlate",
        createStepName="Initial",
        region=assembly.sets[TOP_RP_SET],
        u1=0.0, u2=0.0, u3=0.0,
        ur1=0.0, ur2=0.0, ur3=0.0,
    )
    model.boundaryConditions["BC-TopPlate"].setValuesInStep(
        stepName=step_1, u2=COMPRESSION_U2
    )
    model.boundaryConditions["BC-TopPlate"].setValuesInStep(
        stepName=step_2, u2=0.0
    )
    model.DisplacementBC(
        name="BC-Drift-A",
        createStepName="Initial",
        region=electrode_instance.sets["DRIFT_CORNER_A"],
        u1=0.0, u2=UNSET, u3=0.0,
    )
    model.DisplacementBC(
        name="BC-Drift-B",
        createStepName="Initial",
        region=electrode_instance.sets["DRIFT_CORNER_B"],
        u1=UNSET, u2=UNSET, u3=0.0,
    )

    if "F-Output-1" in model.fieldOutputRequests:
        del model.fieldOutputRequests["F-Output-1"]
    if "H-Output-1" in model.historyOutputRequests:
        del model.historyOutputRequests["H-Output-1"]
    model.FieldOutputRequest(
        name="F-LayeredElectrode",
        createStepName=step_1,
        variables=("S", "U", "LE", "PE", "PEEQ", "EVOL"),
        frequency=1,
    )
    model.HistoryOutputRequest(
        name="H-TopRP",
        createStepName=step_1,
        variables=("U2", "RF2"),
        region=assembly.sets[TOP_RP_SET],
        frequency=1,
    )
    model.HistoryOutputRequest(
        name="H-Energy",
        createStepName=step_1,
        variables=("ALLIE", "ALLPD", "ALLSD"),
        frequency=1,
    )

    assembly.regenerate()
    job = mdb.Job(
        name=JOB_NAME,
        model=MODEL_NAME,
        description="Three-layer electrode compression and unloading validation",
        type=ANALYSIS,
        memory=90,
        memoryUnits=PERCENTAGE,
        numCpus=NUM_CPUS,
        numDomains=NUM_CPUS,
    )
    mdb.saveAs(pathName=CAE_PATH)
    job.writeInput(consistencyChecking=OFF)
    print("CAE已保存：{}".format(CAE_PATH))
    print("INP已导出：{}".format(INP_PATH))

    # 只有材料块检查完全通过才允许提交。
    check_inp_materials()
    print("开始提交 {}（{}核）...".format(JOB_NAME, NUM_CPUS))
    job.submit(consistencyChecking=OFF)
    job.waitForCompletion()
    if not analysis_completed_successfully():
        raise RuntimeError(
            "Job未成功完成，状态为 {}。请检查同名 .sta 和 .msg。".format(job.status)
        )
    print("Job计算完成，开始自动后处理。")

    if WORK_DIR not in sys.path:
        sys.path.insert(0, WORK_DIR)
    import postprocess_layered_electrode_compression as postprocess
    postprocess.process_odb()
    print("三层极片压缩验证完整流程结束。")


if __name__ == "__main__":
    build_submit_and_postprocess()
