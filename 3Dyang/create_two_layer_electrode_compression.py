# -*- coding: utf-8 -*-
"""自动建立、检查、计算并后处理两层极片平板压缩模型。

运行方式：
    abaqus cae noGUI=create_two_layer_electrode_compression.py
也可在 Abaqus/CAE 中通过“文件 -> 运行脚本”直接运行。
"""

from __future__ import print_function

import os
import re
import sys

from abaqus import Mdb, mdb
from caeModules import *
from abaqusConstants import (
    ANALYSIS, ANALYTIC_RIGID_SURFACE, CARTESIAN, COMPUTED, DEFAULT,
    DEFORMABLE_BODY, DISSIPATED_ENERGY_FRACTION, FINITE, FIXED,
    FRICTIONLESS, HARD, HEX, NONE, OFF, OMIT, ON, PERCENTAGE,
    STANDARD, STRUCTURED, THREE_D, UNSET,
)
import mesh
import regionToolset


# =============================================================================
# 关键参数区：mm、N、MPa
# =============================================================================
WORK_DIR = r"E:\abaqus\3Dyang"
MODEL_NAME = "Yang_Macro_TwoLayerElectrode_Compression"
JOB_NAME = MODEL_NAME
CAE_PATH = os.path.join(WORK_DIR, JOB_NAME + ".cae")
INP_PATH = os.path.join(WORK_DIR, JOB_NAME + ".inp")

X_LENGTH = 1.0
Z_WIDTH = 1.0
TOTAL_THICKNESS = 0.150
ACTIVE_THICKNESS = 0.133
AL_THICKNESS = 0.017
Y_BOTTOM = -0.075
Y_INTERFACE = -0.058
Y_TOP = 0.075
COMPRESSION_U2 = -0.015

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
E_AL = 72000.0
NU_AL = 0.33
ACTIVE_MATERIAL_NAME = "Active_Layer"
AL_MATERIAL_NAME = "Al_Current_Collector"

FRICTION = 0.0
STABILIZATION_MAGNITUDE = 2.0e-4
PLATE_OVERHANG = 0.10
PLATE_X_LENGTH = X_LENGTH + 2.0 * PLATE_OVERHANG
PLATE_Z_WIDTH = Z_WIDTH + 2.0 * PLATE_OVERHANG

N_ELEM_X = 10
N_ELEM_Z = 10
N_ELEM_ACTIVE_Y = 5
N_ELEM_AL_Y = 2
EXPECTED_ELEMENT_COUNT = N_ELEM_X * N_ELEM_Z * (
    N_ELEM_ACTIVE_Y + N_ELEM_AL_Y
)

INITIAL_INCREMENT = 0.02
MAX_INCREMENT = 0.05
MIN_INCREMENT = 1.0e-8
MAX_NUM_INCREMENT = 1000
NUM_CPUS = 4

ACTIVE_PART_NAME = "ActiveLayer"
AL_PART_NAME = "AlCollector"
ACTIVE_INSTANCE_NAME = "ACTIVE_LAYER-1"
AL_INSTANCE_NAME = "AL_COLLECTOR-1"
ACTIVE_ELEMENT_SET = "ACTIVE_LAYER_ALL"
AL_ELEMENT_SET = "AL_COLLECTOR_ALL"
ACTIVE_TOP_NODE_SET = "ACTIVE_TOP_NODES"
ACTIVE_INTERFACE_NODE_SET = "ACTIVE_INTERFACE_NODES"
AL_INTERFACE_NODE_SET = "AL_INTERFACE_NODES"
AL_BOTTOM_NODE_SET = "AL_BOTTOM_NODES"
TOP_RP_SET = "TOP_RP"


def validate_parameters():
    """检查几何、塑性曲线、网格和计算参数。"""
    if not os.path.isdir(WORK_DIR):
        os.makedirs(WORK_DIR)
    if abs(ACTIVE_THICKNESS + AL_THICKNESS - TOTAL_THICKNESS) > 1.0e-12:
        raise ValueError("活性层与集流体厚度之和必须为0.150 mm。")
    if abs(Y_INTERFACE - Y_BOTTOM - AL_THICKNESS) > 1.0e-12:
        raise ValueError("集流体Y坐标与厚度不一致。")
    if abs(Y_TOP - Y_INTERFACE - ACTIVE_THICKNESS) > 1.0e-12:
        raise ValueError("活性层Y坐标与厚度不一致。")
    if abs(ACTIVE_PLASTIC_TABLE[0][1]) > 1.0e-15:
        raise ValueError("活性层塑性曲线第一行塑性应变必须为0.00。")
    previous_strain = -1.0
    for stress, plastic_strain in ACTIVE_PLASTIC_TABLE:
        if stress <= 0.0 or plastic_strain <= previous_strain:
            raise ValueError("活性层塑性曲线必须具有正应力和严格递增的塑性应变。")
        previous_strain = plastic_strain
    if N_ELEM_ACTIVE_Y < 5 or N_ELEM_AL_Y not in (1, 2):
        raise ValueError("活性层至少5层单元，集流体必须为1或2层单元。")
    if FRICTION != 0.0:
        raise ValueError("本模型要求friction=0.0。")


def seed_structured_mesh(part, y_element_count):
    """按X/Y/Z方向分别播种并生成C3D8R结构化网格。"""
    direction_counts = (N_ELEM_X, y_element_count, N_ELEM_Z)
    for edge in part.edges:
        vertex_ids = edge.getVertices()
        if len(vertex_ids) != 2:
            raise RuntimeError("发现无法按端点识别方向的边。")
        point_0 = part.vertices[vertex_ids[0]].pointOn[0]
        point_1 = part.vertices[vertex_ids[1]].pointOn[0]
        delta = [abs(point_1[index] - point_0[index]) for index in range(3)]
        axis = delta.index(max(delta))
        part.seedEdgeByNumber(
            edges=(edge,), number=direction_counts[axis], constraint=FIXED
        )
    part.setMeshControls(regions=part.cells[:], elemShape=HEX, technique=STRUCTURED)
    element_type = mesh.ElemType(elemCode=mesh.C3D8R, elemLibrary=STANDARD)
    part.setElementType(regions=(part.cells[:],), elemTypes=(element_type,))
    part.generateMesh()


def nodes_on_y_plane(part, y_value, set_name):
    """创建完整X-Z平面的节点集并检查节点数量。"""
    tolerance = 1.0e-7
    nodes = part.nodes.getByBoundingBox(
        xMin=-0.5 * X_LENGTH - tolerance,
        xMax=0.5 * X_LENGTH + tolerance,
        yMin=y_value - tolerance,
        yMax=y_value + tolerance,
        zMin=-tolerance,
        zMax=Z_WIDTH + tolerance,
    )
    expected = (N_ELEM_X + 1) * (N_ELEM_Z + 1)
    if len(nodes) != expected:
        raise RuntimeError(
            "节点集{}包含{}个节点，预期{}个。".format(set_name, len(nodes), expected)
        )
    part.Set(name=set_name, nodes=nodes)


def create_active_layer(model):
    """创建活性层独立实体；其界面节点不与集流体共享。"""
    sketch_name = "__active_profile__"
    sketch = model.ConstrainedSketch(name=sketch_name, sheetSize=4.0)
    sketch.rectangle(
        point1=(-0.5 * X_LENGTH, Y_INTERFACE),
        point2=(0.5 * X_LENGTH, Y_TOP),
    )
    part = model.Part(
        name=ACTIVE_PART_NAME, dimensionality=THREE_D, type=DEFORMABLE_BODY
    )
    part.BaseSolidExtrude(sketch=sketch, depth=Z_WIDTH)
    del model.sketches[sketch_name]

    material = model.Material(name=ACTIVE_MATERIAL_NAME)
    material.Elastic(table=((E_ACTIVE, NU_ACTIVE),))
    material.Plastic(table=ACTIVE_PLASTIC_TABLE)
    model.HomogeneousSolidSection(
        name="ActiveLayerSection", material=ACTIVE_MATERIAL_NAME, thickness=None
    )
    all_cells = part.Set(name=ACTIVE_ELEMENT_SET, cells=part.cells[:])
    part.SectionAssignment(region=all_cells, sectionName="ActiveLayerSection")

    seed_structured_mesh(part, N_ELEM_ACTIVE_Y)
    if len(part.elements) != N_ELEM_X * N_ELEM_Z * N_ELEM_ACTIVE_Y:
        raise RuntimeError("活性层单元数不符合预期。")

    top_face = part.faces.findAt(((0.0, Y_TOP, 0.5 * Z_WIDTH),))
    interface_face = part.faces.findAt(((0.0, Y_INTERFACE, 0.5 * Z_WIDTH),))
    part.Surface(name="ACTIVE_TOP", side1Faces=top_face)
    part.Surface(name="ACTIVE_INTERFACE", side1Faces=interface_face)
    nodes_on_y_plane(part, Y_TOP, ACTIVE_TOP_NODE_SET)
    nodes_on_y_plane(part, Y_INTERFACE, ACTIVE_INTERFACE_NODE_SET)
    return part


def create_al_collector(model):
    """创建仅含线弹性材料的集流体独立实体。"""
    sketch_name = "__al_profile__"
    sketch = model.ConstrainedSketch(name=sketch_name, sheetSize=4.0)
    sketch.rectangle(
        point1=(-0.5 * X_LENGTH, Y_BOTTOM),
        point2=(0.5 * X_LENGTH, Y_INTERFACE),
    )
    part = model.Part(
        name=AL_PART_NAME, dimensionality=THREE_D, type=DEFORMABLE_BODY
    )
    part.BaseSolidExtrude(sketch=sketch, depth=Z_WIDTH)
    del model.sketches[sketch_name]

    material = model.Material(name=AL_MATERIAL_NAME)
    material.Elastic(table=((E_AL, NU_AL),))
    model.HomogeneousSolidSection(
        name="AlCollectorSection", material=AL_MATERIAL_NAME, thickness=None
    )
    all_cells = part.Set(name=AL_ELEMENT_SET, cells=part.cells[:])
    part.SectionAssignment(region=all_cells, sectionName="AlCollectorSection")

    seed_structured_mesh(part, N_ELEM_AL_Y)
    if len(part.elements) != N_ELEM_X * N_ELEM_Z * N_ELEM_AL_Y:
        raise RuntimeError("集流体单元数不符合预期。")

    interface_face = part.faces.findAt(((0.0, Y_INTERFACE, 0.5 * Z_WIDTH),))
    bottom_face = part.faces.findAt(((0.0, Y_BOTTOM, 0.5 * Z_WIDTH),))
    part.Surface(name="AL_INTERFACE", side1Faces=interface_face)
    part.Surface(name="AL_BOTTOM", side1Faces=bottom_face)
    nodes_on_y_plane(part, Y_INTERFACE, AL_INTERFACE_NODE_SET)
    nodes_on_y_plane(part, Y_BOTTOM, AL_BOTTOM_NODE_SET)

    tolerance = 1.0e-7
    corner_a = part.nodes.getByBoundingBox(
        xMin=-0.5 * X_LENGTH - tolerance,
        xMax=-0.5 * X_LENGTH + tolerance,
        yMin=Y_BOTTOM - tolerance,
        yMax=Y_BOTTOM + tolerance,
        zMin=-tolerance,
        zMax=tolerance,
    )
    corner_b = part.nodes.getByBoundingBox(
        xMin=0.5 * X_LENGTH - tolerance,
        xMax=0.5 * X_LENGTH + tolerance,
        yMin=Y_BOTTOM - tolerance,
        yMax=Y_BOTTOM + tolerance,
        zMin=-tolerance,
        zMax=tolerance,
    )
    if len(corner_a) != 1 or len(corner_b) != 1:
        raise RuntimeError("未能唯一识别集流体底部两个防漂移角点。")
    part.Set(name="DRIFT_CORNER_A", nodes=corner_a)
    part.Set(name="DRIFT_CORNER_B", nodes=corner_b)
    return part


def create_rigid_plate(model, part_name, normal_toward_positive_y):
    """创建具有正确接触法向的解析刚体平板。"""
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
    """将INP按*Material分块，防止跨材料误判Plastic。"""
    blocks = {}
    current_name = None
    current_lines = []
    name_pattern = re.compile(r"name\s*=\s*([^,\s]+)", re.IGNORECASE)
    for line in inp_text.splitlines():
        stripped = line.strip()
        if stripped.lower().startswith("*material"):
            if current_name is not None:
                blocks[current_name.lower()] = current_lines
            match = name_pattern.search(stripped)
            if not match:
                raise RuntimeError("发现无名称的*Material关键字。")
            current_name = match.group(1).strip('"\'')
            current_lines = [stripped]
        elif current_name is not None:
            current_lines.append(stripped)
    if current_name is not None:
        blocks[current_name.lower()] = current_lines
    return blocks


def count_c3d8r_elements(inp_lines):
    """统计所有C3D8R数据行，支持两个独立Part的多个Element块。"""
    count = 0
    in_c3d8r_block = False
    for line in inp_lines:
        stripped = line.strip()
        if stripped.startswith("*"):
            in_c3d8r_block = (
                stripped.lower().startswith("*element")
                and "type=c3d8r" in stripped.lower().replace(" ", "")
            )
        elif in_c3d8r_block and stripped and not stripped.startswith("**"):
            count += 1
    return count


def check_inp_before_submit():
    """检查材料、Tie、单元、稳定化和输出；失败则禁止提交。"""
    if not os.path.isfile(INP_PATH):
        raise RuntimeError("未找到导出的INP。")
    with open(INP_PATH, "r", encoding="utf-8", errors="ignore") as inp_file:
        inp_text = inp_file.read()
    inp_lines = inp_text.splitlines()
    blocks = parse_material_blocks(inp_text)
    active_key = ACTIVE_MATERIAL_NAME.lower()
    al_key = AL_MATERIAL_NAME.lower()
    if active_key not in blocks or al_key not in blocks:
        raise RuntimeError("INP必须包含Active_Layer和Al_Current_Collector。")

    active_lines = blocks[active_key]
    al_lines = blocks[al_key]
    active_keywords = [line.lower() for line in active_lines if line.startswith("*")]
    al_keywords = [line.lower() for line in al_lines if line.startswith("*")]
    if not any(line.startswith("*elastic") for line in active_keywords):
        raise RuntimeError("Active_Layer缺少*Elastic。")
    if not any(line.startswith("*plastic") for line in active_keywords):
        raise RuntimeError("Active_Layer缺少*Plastic。")
    if not any(line.startswith("*elastic") for line in al_keywords):
        raise RuntimeError("Al_Current_Collector缺少*Elastic。")
    if any(line.startswith("*plastic") for line in al_keywords):
        raise RuntimeError("严重错误：Al_Current_Collector中出现*Plastic，停止提交。")

    plastic_index = next(
        index for index, line in enumerate(active_lines)
        if line.lower().startswith("*plastic")
    )
    first_data = None
    for line in active_lines[plastic_index + 1:]:
        if not line or line.startswith("**"):
            continue
        if line.startswith("*"):
            break
        first_data = line
        break
    if first_data is None:
        raise RuntimeError("Active_Layer的*Plastic没有数据。")
    values = [float(value.strip()) for value in first_data.split(",") if value.strip()]
    if len(values) < 2 or abs(values[1]) > 1.0e-15:
        raise RuntimeError("Active_Layer第一行塑性应变不是0.00。")

    tie_count = 0
    for line in inp_lines:
        if line.strip().lower().startswith("*tie"):
            tie_count += 1
    if tie_count != 1:
        raise RuntimeError("INP必须且只能包含一个*Tie，当前为{}个。".format(tie_count))
    element_count = count_c3d8r_elements(inp_lines)
    if element_count != EXPECTED_ELEMENT_COUNT:
        raise RuntimeError(
            "C3D8R单元数为{}，预期{}。".format(element_count, EXPECTED_ELEMENT_COUNT)
        )
    stabilized_steps = 0
    for line in inp_lines:
        if (
            line.strip().lower().startswith("*static")
            and "stabilize=0.0002" in line.strip().lower()
        ):
            stabilized_steps += 1
    if stabilized_steps != 2:
        raise RuntimeError("两个分析步均必须包含stabilize=0.0002。")
    upper_text = inp_text.upper()
    if "ALLIE, ALLPD, ALLSD" not in upper_text:
        raise RuntimeError("INP缺少ALLIE/ALLPD/ALLSD历史输出。")
    for variable in ("EVOL", "LE", "PE", "PEEQ", "S"):
        if variable not in upper_text:
            raise RuntimeError("INP缺少场输出{}。".format(variable))
    print("INP检查通过：材料、Tie、700个C3D8R、稳定化和输出均正确。")


def clean_previous_analysis_files():
    """清除同名旧求解输出，避免误用旧ODB。"""
    extensions = (
        ".com", ".dat", ".env", ".ipm", ".lck", ".log", ".mdl", ".msg",
        ".odb", ".pac", ".prt", ".res", ".sim", ".sta", ".stt",
    )
    for extension in extensions:
        path = os.path.join(WORK_DIR, JOB_NAME + extension)
        if os.path.isfile(path):
            os.remove(path)
    csv_path = os.path.join(WORK_DIR, JOB_NAME + "_results.csv")
    if os.path.isfile(csv_path):
        os.remove(csv_path)


def analysis_completed_successfully():
    """以Abaqus状态文件为准，规避noGUI下job.status返回None。"""
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
    active_part = create_active_layer(model)
    al_part = create_al_collector(model)
    bottom_plate_part = create_rigid_plate(
        model, "BottomRigidPlate", normal_toward_positive_y=True
    )
    top_plate_part = create_rigid_plate(
        model, "TopRigidPlate", normal_toward_positive_y=False
    )

    assembly = model.rootAssembly
    assembly.DatumCsysByDefault(CARTESIAN)
    active_instance = assembly.Instance(
        name=ACTIVE_INSTANCE_NAME, part=active_part, dependent=ON
    )
    al_instance = assembly.Instance(
        name=AL_INSTANCE_NAME, part=al_part, dependent=ON
    )
    assembly.translate(
        instanceList=(ACTIVE_INSTANCE_NAME, AL_INSTANCE_NAME),
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

    # 两个独立实体的界面使用Tie，不共享节点、不接触、不允许滑移或分离。
    model.Tie(
        name="Tie-Active-Al",
        main=al_instance.surfaces["AL_INTERFACE"],
        secondary=active_instance.surfaces["ACTIVE_INTERFACE"],
        positionToleranceMethod=COMPUTED,
        adjust=OFF,
        tieRotations=ON,
        thickness=ON,
    )

    contact_property = model.ContactProperty("PlateElectrodeContact")
    contact_property.NormalBehavior(
        pressureOverclosure=HARD,
        allowSeparation=ON,
        constraintEnforcementMethod=DEFAULT,
    )
    contact_property.TangentialBehavior(formulation=FRICTIONLESS)
    model.SurfaceToSurfaceContactStd(
        name="Contact-Top-Active",
        createStepName="Initial",
        main=top_instance.surfaces["PLATE_SURFACE"],
        secondary=active_instance.surfaces["ACTIVE_TOP"],
        sliding=FINITE,
        interactionProperty="PlateElectrodeContact",
        adjustMethod=NONE,
        initialClearance=OMIT,
    )
    model.SurfaceToSurfaceContactStd(
        name="Contact-Bottom-Al",
        createStepName="Initial",
        main=bottom_instance.surfaces["PLATE_SURFACE"],
        secondary=al_instance.surfaces["AL_BOTTOM"],
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
        region=al_instance.sets["DRIFT_CORNER_A"],
        u1=0.0, u2=UNSET, u3=0.0,
    )
    model.DisplacementBC(
        name="BC-Drift-B",
        createStepName="Initial",
        region=al_instance.sets["DRIFT_CORNER_B"],
        u1=UNSET, u2=UNSET, u3=0.0,
    )

    if "F-Output-1" in model.fieldOutputRequests:
        del model.fieldOutputRequests["F-Output-1"]
    if "H-Output-1" in model.historyOutputRequests:
        del model.historyOutputRequests["H-Output-1"]
    model.FieldOutputRequest(
        name="F-TwoLayerElectrode",
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
        description="Two-layer active-layer/current-collector compression validation",
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

    check_inp_before_submit()
    print("开始提交{}（{}核）...".format(JOB_NAME, NUM_CPUS))
    job.submit(consistencyChecking=OFF)
    job.waitForCompletion()
    if not analysis_completed_successfully():
        raise RuntimeError("Job未成功完成，请检查同名.sta和.msg。")

    if WORK_DIR not in sys.path:
        sys.path.insert(0, WORK_DIR)
    import postprocess_two_layer_electrode_compression as postprocess
    postprocess.process_odb()
    print("两层极片压缩验证完整流程结束。")


if __name__ == "__main__":
    build_submit_and_postprocess()
