# -*- coding: utf-8 -*-
"""建立活性层上下平板压缩—卸载模型，保存 CAE 并导出 INP。

运行方式：
    abaqus cae noGUI=create_active_layer_compression.py

本脚本只建模和写入输入文件，不提交分析作业。
"""

from __future__ import print_function

import os

from abaqus import Mdb, mdb
from caeModules import *
from abaqusConstants import (
    ANALYSIS,
    ANALYTIC_RIGID_SURFACE,
    CARTESIAN,
    DEFAULT,
    DEFORMABLE_BODY,
    FINITE,
    FIXED,
    FRICTIONLESS,
    HARD,
    HEX,
    ISOTROPIC,
    NONE,
    OFF,
    OMIT,
    ON,
    PENALTY,
    PERCENTAGE,
    SINGLE,
    STANDARD,
    STRUCTURED,
    THREE_D,
    UNSET,
)
import mesh
import regionToolset


# =============================================================================
# 关键参数区：后续标定时优先修改本区
# =============================================================================
WORK_DIR = r"E:\abaqus\3Dyang"
MODEL_NAME = "Yang_Macro_ActiveLayer_Compression"
JOB_NAME = MODEL_NAME

CAE_PATH = os.path.join(WORK_DIR, JOB_NAME + ".cae")
INP_PATH = os.path.join(WORK_DIR, JOB_NAME + ".inp")

# mm、N、MPa 单位制；X=长度、Y=厚度、Z=宽度
X_LENGTH = 1.0
Y_THICKNESS = 0.150
Z_WIDTH = 1.0
COMPRESSION_U2 = -0.015

# 解析刚体平板比试样每侧多出 0.10 mm
PLATE_OVERHANG = 0.10
PLATE_X_LENGTH = X_LENGTH + 2.0 * PLATE_OVERHANG
PLATE_Z_WIDTH = Z_WIDTH + 2.0 * PLATE_OVERHANG

# 活性层等效弹塑性参数
E_ACTIVE = 609.0
NU_ACTIVE = 0.10
PLASTIC_TABLE = (
    (10.0, 0.00),
    (20.0, 0.10),
    (35.0, 0.20),
    (62.0, 0.30),
    (92.0, 0.40),
    (145.0, 0.50),
    (230.0, 0.63),
)

# 接触参数；设为正值（例如 0.05 或 0.10）时自动启用罚函数摩擦
FRICTION = 0.0

# 结构化网格划分数：对应约 0.10、0.03、0.10 mm
N_ELEM_X = 10
N_ELEM_Y = 5
N_ELEM_Z = 10

# 分析步增量参数（未启用自动稳定）
INITIAL_INCREMENT = 0.02
MAX_INCREMENT = 0.05
MIN_INCREMENT = 1.0e-8
MAX_NUM_INCREMENT = 1000

# 固定名称供后处理脚本定位
ACTIVE_PART_NAME = "ActiveLayer"
ACTIVE_INSTANCE_NAME = "ACTIVE_LAYER-1"
TOP_PLATE_PART_NAME = "TopRigidPlate"
BOTTOM_PLATE_PART_NAME = "BottomRigidPlate"
TOP_PLATE_INSTANCE_NAME = "TOP_PLATE-1"
BOTTOM_PLATE_INSTANCE_NAME = "BOTTOM_PLATE-1"
TOP_NODE_SET_NAME = "TOP_SURFACE_NODES"
BOTTOM_NODE_SET_NAME = "BOTTOM_SURFACE_NODES"
TOP_RP_SET_NAME = "TOP_RP"


def validate_parameters():
    """在建模前检查最容易导致错误的输入参数。"""
    if not os.path.isdir(WORK_DIR):
        os.makedirs(WORK_DIR)
    if abs(PLASTIC_TABLE[0][1]) > 1.0e-15:
        raise ValueError("塑性曲线第一行的等效塑性应变必须为 0.00。")
    previous_strain = -1.0
    for stress, plastic_strain in PLASTIC_TABLE:
        if stress <= 0.0:
            raise ValueError("塑性曲线中的屈服应力必须为正值。")
        if plastic_strain <= previous_strain:
            raise ValueError("塑性曲线中的等效塑性应变必须严格递增。")
        previous_strain = plastic_strain
    if min(N_ELEM_X, N_ELEM_Y, N_ELEM_Z) < 1:
        raise ValueError("三个方向的网格划分数均必须大于零。")
    if N_ELEM_Y < 5:
        raise ValueError("厚度方向至少需要 5 层单元。")
    if FRICTION < 0.0:
        raise ValueError("摩擦系数不能为负值。")


def create_active_layer(model):
    """创建、赋材并划分活性层结构化 C3D8R 网格。"""
    sketch_name = "__active_layer_profile__"
    sketch = model.ConstrainedSketch(
        name=sketch_name,
        sheetSize=4.0 * max(X_LENGTH, Y_THICKNESS, Z_WIDTH),
    )
    sketch.rectangle(
        point1=(-0.5 * X_LENGTH, -0.5 * Y_THICKNESS),
        point2=(0.5 * X_LENGTH, 0.5 * Y_THICKNESS),
    )
    part = model.Part(
        name=ACTIVE_PART_NAME, dimensionality=THREE_D, type=DEFORMABLE_BODY
    )
    part.BaseSolidExtrude(sketch=sketch, depth=Z_WIDTH)
    del model.sketches[sketch_name]

    material = model.Material(name="ActiveLayerMaterial")
    material.Elastic(table=((E_ACTIVE, NU_ACTIVE),))
    material.Plastic(table=PLASTIC_TABLE)

    model.HomogeneousSolidSection(
        name="ActiveLayerSection", material="ActiveLayerMaterial", thickness=None
    )
    all_cells = part.Set(name="ACTIVE_LAYER_ALL", cells=part.cells[:])
    part.SectionAssignment(region=all_cells, sectionName="ActiveLayerSection")

    # 按边的主方向分别指定单元数，确保厚度方向恰好为 5 层。
    direction_counts = (N_ELEM_X, N_ELEM_Y, N_ELEM_Z)
    for edge in part.edges:
        vertex_ids = edge.getVertices()
        if len(vertex_ids) != 2:
            raise RuntimeError("发现无法按端点识别方向的边。")
        p0 = part.vertices[vertex_ids[0]].pointOn[0]
        p1 = part.vertices[vertex_ids[1]].pointOn[0]
        delta = [abs(p1[i] - p0[i]) for i in range(3)]
        axis = delta.index(max(delta))
        part.seedEdgeByNumber(
            edges=(edge,), number=direction_counts[axis], constraint=FIXED
        )

    part.setMeshControls(regions=part.cells[:], elemShape=HEX, technique=STRUCTURED)
    elem_type = mesh.ElemType(elemCode=mesh.C3D8R, elemLibrary=STANDARD)
    part.setElementType(regions=(part.cells[:],), elemTypes=(elem_type,))
    part.generateMesh()

    # 面与节点集使用明确的几何坐标创建，供接触和 ODB 厚度计算使用。
    top_face = part.faces.findAt(((0.0, 0.5 * Y_THICKNESS, 0.5 * Z_WIDTH),))
    bottom_face = part.faces.findAt(((0.0, -0.5 * Y_THICKNESS, 0.5 * Z_WIDTH),))
    part.Surface(name="ACTIVE_TOP", side1Faces=top_face)
    part.Surface(name="ACTIVE_BOTTOM", side1Faces=bottom_face)

    tol = 1.0e-7
    top_nodes = part.nodes.getByBoundingBox(
        xMin=-0.5 * X_LENGTH - tol,
        xMax=0.5 * X_LENGTH + tol,
        yMin=0.5 * Y_THICKNESS - tol,
        yMax=0.5 * Y_THICKNESS + tol,
        zMin=-tol,
        zMax=Z_WIDTH + tol,
    )
    bottom_nodes = part.nodes.getByBoundingBox(
        xMin=-0.5 * X_LENGTH - tol,
        xMax=0.5 * X_LENGTH + tol,
        yMin=-0.5 * Y_THICKNESS - tol,
        yMax=-0.5 * Y_THICKNESS + tol,
        zMin=-tol,
        zMax=Z_WIDTH + tol,
    )
    part.Set(name=TOP_NODE_SET_NAME, nodes=top_nodes)
    part.Set(name=BOTTOM_NODE_SET_NAME, nodes=bottom_nodes)

    # 两个角点仅消除 X/Z 平面内刚体漂移；绝不施加 U2 约束。
    corner_a = part.nodes.getByBoundingBox(
        xMin=-0.5 * X_LENGTH - tol,
        xMax=-0.5 * X_LENGTH + tol,
        yMin=-0.5 * Y_THICKNESS - tol,
        yMax=-0.5 * Y_THICKNESS + tol,
        zMin=-tol,
        zMax=tol,
    )
    corner_b = part.nodes.getByBoundingBox(
        xMin=0.5 * X_LENGTH - tol,
        xMax=0.5 * X_LENGTH + tol,
        yMin=-0.5 * Y_THICKNESS - tol,
        yMax=-0.5 * Y_THICKNESS + tol,
        zMin=-tol,
        zMax=tol,
    )
    if len(corner_a) != 1 or len(corner_b) != 1:
        raise RuntimeError("未能唯一识别用于防漂移约束的两个角点节点。")
    part.Set(name="DRIFT_CORNER_A", nodes=corner_a)
    part.Set(name="DRIFT_CORNER_B", nodes=corner_b)
    return part


def create_rigid_plate(model, part_name, normal_toward_positive_y):
    """创建解析刚体平板，并用线段方向明确控制接触法向。"""
    sketch_name = "__{}_profile__".format(part_name)
    sketch = model.ConstrainedSketch(
        name=sketch_name, sheetSize=4.0 * max(PLATE_X_LENGTH, PLATE_Z_WIDTH)
    )
    if normal_toward_positive_y:
        # 从 -X 指向 +X 时，解析面的外法向朝 +Y（用于下板）。
        point_1 = (-0.5 * PLATE_X_LENGTH, 0.0)
        point_2 = (0.5 * PLATE_X_LENGTH, 0.0)
    else:
        # 反向线段使外法向朝 -Y（用于上板）。
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


def build_model():
    validate_parameters()
    os.chdir(WORK_DIR)

    # 清空默认数据库，保证重复运行时结果可复现。
    Mdb()
    mdb.models.changeKey(fromName="Model-1", toName=MODEL_NAME)
    model = mdb.models[MODEL_NAME]

    active_part = create_active_layer(model)
    bottom_plate_part = create_rigid_plate(
        model, BOTTOM_PLATE_PART_NAME, normal_toward_positive_y=True
    )
    top_plate_part = create_rigid_plate(
        model, TOP_PLATE_PART_NAME, normal_toward_positive_y=False
    )

    assembly = model.rootAssembly
    assembly.DatumCsysByDefault(CARTESIAN)

    active_instance = assembly.Instance(
        name=ACTIVE_INSTANCE_NAME, part=active_part, dependent=ON
    )
    # 实体拉伸时 Z 为 0～Z_WIDTH，将其平移到 Z=±Z_WIDTH/2。
    assembly.translate(
        instanceList=(ACTIVE_INSTANCE_NAME,), vector=(0.0, 0.0, -0.5 * Z_WIDTH)
    )

    bottom_instance = assembly.Instance(
        name=BOTTOM_PLATE_INSTANCE_NAME, part=bottom_plate_part, dependent=ON
    )
    top_instance = assembly.Instance(
        name=TOP_PLATE_INSTANCE_NAME, part=top_plate_part, dependent=ON
    )
    assembly.translate(
        instanceList=(BOTTOM_PLATE_INSTANCE_NAME,),
        vector=(0.0, -0.5 * Y_THICKNESS, -0.5 * PLATE_Z_WIDTH),
    )
    assembly.translate(
        instanceList=(TOP_PLATE_INSTANCE_NAME,),
        vector=(0.0, 0.5 * Y_THICKNESS, -0.5 * PLATE_Z_WIDTH),
    )

    top_rp_id = list(top_plate_part.referencePoints.keys())[0]
    bottom_rp_id = list(bottom_plate_part.referencePoints.keys())[0]
    top_rp_region = regionToolset.Region(
        referencePoints=(top_instance.referencePoints[top_rp_id],)
    )
    bottom_rp_region = regionToolset.Region(
        referencePoints=(bottom_instance.referencePoints[bottom_rp_id],)
    )
    assembly.Set(
        name=TOP_RP_SET_NAME,
        referencePoints=(top_instance.referencePoints[top_rp_id],),
    )
    assembly.Set(
        name="BOTTOM_RP",
        referencePoints=(bottom_instance.referencePoints[bottom_rp_id],),
    )

    # 解析刚体表面与各自 RP 关联。
    model.RigidBody(
        name="RigidBody-Top",
        refPointRegion=top_rp_region,
        surfaceRegion=top_instance.surfaces["PLATE_SURFACE"],
    )
    model.RigidBody(
        name="RigidBody-Bottom",
        refPointRegion=bottom_rp_region,
        surfaceRegion=bottom_instance.surfaces["PLATE_SURFACE"],
    )

    # 接触属性：法向硬接触；零摩擦与正摩擦使用不同的合法 API 形式。
    contact_property = model.ContactProperty("PlateActiveContact")
    contact_property.NormalBehavior(
        pressureOverclosure=HARD,
        allowSeparation=ON,
        constraintEnforcementMethod=DEFAULT,
    )
    if FRICTION == 0.0:
        contact_property.TangentialBehavior(formulation=FRICTIONLESS)
    else:
        contact_property.TangentialBehavior(
            formulation=PENALTY,
            directionality=ISOTROPIC,
            table=((FRICTION,),),
        )

    model.SurfaceToSurfaceContactStd(
        name="Contact-Top",
        createStepName="Initial",
        main=top_instance.surfaces["PLATE_SURFACE"],
        secondary=active_instance.surfaces["ACTIVE_TOP"],
        sliding=FINITE,
        interactionProperty="PlateActiveContact",
        adjustMethod=NONE,
        initialClearance=OMIT,
    )
    model.SurfaceToSurfaceContactStd(
        name="Contact-Bottom",
        createStepName="Initial",
        main=bottom_instance.surfaces["PLATE_SURFACE"],
        secondary=active_instance.surfaces["ACTIVE_BOTTOM"],
        sliding=FINITE,
        interactionProperty="PlateActiveContact",
        adjustMethod=NONE,
        initialClearance=OMIT,
    )

    step_1 = "Step-1 Compression"
    step_2 = "Step-2 Unload"
    model.StaticStep(
        name=step_1,
        previous="Initial",
        nlgeom=ON,
        timePeriod=1.0,
        initialInc=INITIAL_INCREMENT,
        maxInc=MAX_INCREMENT,
        minInc=MIN_INCREMENT,
        maxNumInc=MAX_NUM_INCREMENT,
    )
    model.StaticStep(
        name=step_2,
        previous=step_1,
        nlgeom=ON,
        timePeriod=1.0,
        initialInc=INITIAL_INCREMENT,
        maxInc=MAX_INCREMENT,
        minInc=MIN_INCREMENT,
        maxNumInc=MAX_NUM_INCREMENT,
    )

    model.EncastreBC(
        name="BC-BottomPlate-Fixed",
        createStepName="Initial",
        region=assembly.sets["BOTTOM_RP"],
    )
    model.DisplacementBC(
        name="BC-TopPlate",
        createStepName="Initial",
        region=assembly.sets[TOP_RP_SET_NAME],
        u1=0.0,
        u2=0.0,
        u3=0.0,
        ur1=0.0,
        ur2=0.0,
        ur3=0.0,
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
        region=active_instance.sets["DRIFT_CORNER_A"],
        u1=0.0,
        u2=UNSET,
        u3=0.0,
    )
    model.DisplacementBC(
        name="BC-Drift-B",
        createStepName="Initial",
        region=active_instance.sets["DRIFT_CORNER_B"],
        u1=UNSET,
        u2=UNSET,
        u3=0.0,
    )

    # 用明确的请求替换默认输出，确保 ODB 中包含后处理所需数据。
    if "F-Output-1" in model.fieldOutputRequests:
        del model.fieldOutputRequests["F-Output-1"]
    if "H-Output-1" in model.historyOutputRequests:
        del model.historyOutputRequests["H-Output-1"]
    model.FieldOutputRequest(
        name="F-ActiveLayer",
        createStepName=step_1,
        variables=("S", "U", "LE", "PE", "PEEQ", "EVOL"),
        frequency=1,
    )
    model.HistoryOutputRequest(
        name="H-TopRP",
        createStepName=step_1,
        variables=("U2", "RF2"),
        region=assembly.sets[TOP_RP_SET_NAME],
        frequency=1,
    )
    model.HistoryOutputRequest(
        name="H-Energy",
        createStepName=step_1,
        variables=("ALLIE", "ALLPD"),
        frequency=1,
    )

    assembly.regenerate()
    job = mdb.Job(
        name=JOB_NAME,
        model=MODEL_NAME,
        description="Active-layer material compression and unloading calibration model",
        type=ANALYSIS,
        memory=90,
        memoryUnits=PERCENTAGE,
        nodalOutputPrecision=SINGLE,
    )

    mdb.saveAs(pathName=CAE_PATH)
    job.writeInput(consistencyChecking=OFF)
    print("CAE 已保存：{}".format(CAE_PATH))
    print("INP 已导出：{}".format(INP_PATH))

    # 按要求自动检查材料关键字，不区分大小写。
    if not os.path.isfile(INP_PATH):
        print("\n" + "!" * 78)
        print("警告：未找到导出的 INP 文件，无法检查材料定义！")
        print("!" * 78 + "\n")
        return
    with open(INP_PATH, "r", encoding="utf-8", errors="ignore") as inp_file:
        inp_text = inp_file.read().lower()
    has_elastic = "*elastic" in inp_text
    has_plastic = "*plastic" in inp_text
    if not (has_elastic and has_plastic):
        print("\n" + "!" * 78)
        print("严重警告：INP 材料定义检查失败！")
        print("  *Elastic: {}".format("存在" if has_elastic else "缺失"))
        print("  *Plastic: {}".format("存在" if has_plastic else "缺失"))
        print("!" * 78 + "\n")
    else:
        print("INP 检查通过：已包含 *Elastic 和 *Plastic。")
    print("按设计未提交 Job；如需计算，请由用户另行提交 {}。".format(JOB_NAME))


if __name__ == "__main__":
    build_model()
