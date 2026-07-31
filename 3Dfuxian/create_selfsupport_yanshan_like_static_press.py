# -*- coding: utf-8 -*-
"""
建立单层自支撑膜三维局部静态辊压等效模型。

说明：
1. 不联网、不读取论文；所有参数均来自任务说明。
2. 极片改为单层自支撑膜，不建立集流体或三层结构。
3. 轧辊为弹性实体 1/8 截取模型，不使用刚体壳面。
4. 脚本只保存 CAE 并导出 INP，不自动提交 Job。
"""

from abaqus import *
from abaqusConstants import *
from caeModules import *
import mesh
import os
import math


# =============================================================================
# 关键参数区：后续修改参数优先改这里
# =============================================================================

MODEL_NAME = 'SelfSupport_YanshanParam_LocalStaticPress'
JOB_NAME = MODEL_NAME
WORK_DIR = r'E:\abaqus\3Dfuxian'
INCLUDE_UNLOAD_STEP = True
FIELD_OUTPUT_FREQUENCY = LAST_INCREMENT

# 单层自支撑膜尺寸，单位 mm
X_LENGTH = 10.0
Y_THICKNESS = 0.150
Z_WIDTH = 100.0
HALF_X = X_LENGTH / 2.0
HALF_Z = Z_WIDTH / 2.0

# 轧辊几何和材料
ROLL_RADIUS = 450.0
ROLL_WIDTH = 110.0
HALF_ROLL_WIDTH = ROLL_WIDTH / 2.0
ROLL_E = 193000.0
ROLL_NU = 0.20

# 上下辊初始圆心位置；辊缝初始为 0.150 mm
UPPER_ROLL_CY = ROLL_RADIUS + Y_THICKNESS / 2.0
LOWER_ROLL_CY = -(ROLL_RADIUS + Y_THICKNESS / 2.0)
CLAMP_DISPLACEMENT = -0.015
CLAMP_MAX_NUM_INC = 120
CLAMP_INITIAL_INC = 0.025
CLAMP_MIN_INC = 1.0e-8
CLAMP_MAX_INC = 0.05

# 接触和网格参数
FRICTION_COEFF = 0.15
FILM_X_SIZE = 0.10
FILM_Y_SIZE = 0.03
FILM_Z_SIZE = 1.00
FILM_ANCHOR_TOL = 1.0e-6
ROLLER_CONTACT_X = 4.0
ROLLER_CONTACT_X_SIZE = 0.20
ROLLER_CONTACT_Z_SIZE = 10.0
ROLLER_GLOBAL_SIZE = 45.0
ROLLER_BODY_X_MAX = 80.0
ROLLER_NORMAL_ETA = (
    0.0, 0.001, 0.004, 0.01, 0.03, 0.06, 0.10, 0.14,
    0.18, 0.22, 0.26, 0.30, 0.34, 0.38, 0.42, 0.46,
    0.50, 0.54, 0.58, 0.62, 0.66, 0.70, 0.74, 0.78,
    0.82, 0.86, 0.90, 0.94, 0.98, 1.0
)
USE_UNLOAD_STABILIZATION = True
UNLOAD_STABILIZATION_FACTOR = 1.0e-1
UNLOAD_ALLSDTOL = 0.0
RELEASE_UPPER_CONTACT_IN_UNLOAD = True
USE_LOW_FRICTION_LOWER_SUPPORT_IN_UNLOAD = False
UNLOAD_SUPPORT_FRICTION = 0.01

# 自支撑膜材料参数：采用任务给定的文献涂层 DPC 参数作为对照，不代表真实干法膜参数
FILM_E = 6500.0
FILM_NU = 0.01
COHESION_D = 4.0
FRICTION_ANGLE_BETA = 65.0
CAP_ECCENTRICITY_R = 0.8
INITIAL_CAP_POSITION = 60.0
TRANSITION_SURFACE_RADIUS_ALPHA = 0.02
FLOW_STRESS_RATIO_K = 1.0

# Cap hardening：静水屈服压力 MPa, 体积塑性应变
CAP_HARDENING_TABLE = (
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


def ensure_work_dir():
    if not os.path.isdir(WORK_DIR):
        os.makedirs(WORK_DIR)
    os.chdir(WORK_DIR)


def reset_model():
    """清理同名模型，避免重复运行脚本时混入旧对象。"""
    if MODEL_NAME in mdb.models:
        del mdb.models[MODEL_NAME]
    model = mdb.Model(name=MODEL_NAME)
    if 'Model-1' in mdb.models and len(mdb.models['Model-1'].parts) == 0:
        del mdb.models['Model-1']
    return model


def make_materials(model):
    """建立膜材料和弹性钢辊材料。"""
    film_mat = model.Material(name='SelfSupportingFilm_DPC_reference')
    film_mat.Elastic(table=((FILM_E, FILM_NU),))
    # Drucker-Prager Cap 参数来自任务给定表格，仅用于对照复现流程。
    # *Cap Plasticity 顺序：d, beta, R, 初始cap位置, alpha, K。
    film_mat.CapPlasticity(
        table=((COHESION_D, FRICTION_ANGLE_BETA, CAP_ECCENTRICITY_R,
                INITIAL_CAP_POSITION, TRANSITION_SURFACE_RADIUS_ALPHA,
                FLOW_STRESS_RATIO_K),)
    )
    film_mat.capPlasticity.CapHardening(table=CAP_HARDENING_TABLE)

    roll_mat = model.Material(name='ElasticSteel_Roll')
    roll_mat.Elastic(table=((ROLL_E, ROLL_NU),))

    model.HomogeneousSolidSection(name='FilmSection',
                                  material='SelfSupportingFilm_DPC_reference')
    model.HomogeneousSolidSection(name='RollSection',
                                  material='ElasticSteel_Roll')


def seed_film_edges(part):
    """分别控制膜的 X/Y/Z 方向网格尺寸，厚度方向保证 5 层。"""
    y_top = Y_THICKNESS / 2.0
    y_bot = -Y_THICKNESS / 2.0
    z_mid = HALF_Z / 2.0
    x_mid = HALF_X / 2.0

    x_edges = part.edges.findAt(
        ((x_mid, y_bot, 0.0),),
        ((x_mid, y_bot, HALF_Z),),
        ((x_mid, y_top, 0.0),),
        ((x_mid, y_top, HALF_Z),),
    )
    part.seedEdgeBySize(edges=tuple(x_edges), size=FILM_X_SIZE,
                        deviationFactor=0.1, minSizeFactor=0.1)

    y_edges = part.edges.findAt(
        ((0.0, 0.0, 0.0),),
        ((0.0, 0.0, HALF_Z),),
        ((HALF_X, 0.0, 0.0),),
        ((HALF_X, 0.0, HALF_Z),),
    )
    part.seedEdgeBySize(edges=tuple(y_edges), size=FILM_Y_SIZE,
                        deviationFactor=0.1, minSizeFactor=0.1)

    z_edges = part.edges.findAt(
        ((0.0, y_bot, z_mid),),
        ((0.0, y_top, z_mid),),
        ((HALF_X, y_bot, z_mid),),
        ((HALF_X, y_top, z_mid),),
    )
    part.seedEdgeBySize(edges=tuple(z_edges), size=FILM_Z_SIZE,
                        deviationFactor=0.1, minSizeFactor=0.1)


def make_film_part(model):
    """建立 X>=0、Z>=0 的半长半宽自支撑膜实体。"""
    sketch = model.ConstrainedSketch(name='Sketch_Film', sheetSize=200.0)
    sketch.rectangle(point1=(0.0, -Y_THICKNESS / 2.0),
                     point2=(HALF_X, Y_THICKNESS / 2.0))
    part = model.Part(name='Film', dimensionality=THREE_D, type=DEFORMABLE_BODY)
    part.BaseSolidExtrude(sketch=sketch, depth=HALF_Z)
    del model.sketches['Sketch_Film']

    all_cells = part.Set(cells=part.cells, name='ALL_FILM')
    part.SectionAssignment(region=all_cells, sectionName='FilmSection')

    part.Surface(side1Faces=part.faces.findAt(((HALF_X / 2.0, Y_THICKNESS / 2.0, HALF_Z / 2.0),)),
                 name='FILM_TOP')
    part.Surface(side1Faces=part.faces.findAt(((HALF_X / 2.0, -Y_THICKNESS / 2.0, HALF_Z / 2.0),)),
                 name='FILM_BOTTOM')
    part.Set(faces=part.faces.findAt(((0.0, 0.0, HALF_Z / 2.0),)), name='FILM_XSYM')
    part.Set(faces=part.faces.findAt(((HALF_X / 2.0, 0.0, 0.0),)), name='FILM_ZSYM')

    seed_film_edges(part)
    part.setMeshControls(regions=part.cells, technique=STRUCTURED)
    elem_type = mesh.ElemType(elemCode=C3D8R, elemLibrary=STANDARD,
                              kinematicSplit=AVERAGE_STRAIN,
                              hourglassControl=ENHANCED, distortionControl=DEFAULT)
    part.setElementType(regions=(part.cells,), elemTypes=(elem_type,))
    part.generateMesh()

    # 卸载后辊-膜接触可能完全脱开，膜需要一个单节点 U2 防整体漂移约束。
    # 锚点放在远离中心接触区的外角点，不固定端面，也不限制膜整体厚向压缩。
    anchor_nodes = part.nodes.getByBoundingBox(
        xMin=HALF_X - FILM_ANCHOR_TOL, xMax=HALF_X + FILM_ANCHOR_TOL,
        yMin=-Y_THICKNESS / 2.0 - FILM_ANCHOR_TOL,
        yMax=-Y_THICKNESS / 2.0 + FILM_ANCHOR_TOL,
        zMin=HALF_Z - FILM_ANCHOR_TOL, zMax=HALF_Z + FILM_ANCHOR_TOL
    )
    if len(anchor_nodes) != 1:
        raise RuntimeError('FILM_Y_DRIFT_ANCHOR 节点数量不是 1，请检查膜网格。')
    part.Set(nodes=anchor_nodes, name='FILM_Y_DRIFT_ANCHOR')
    return part


def arc_y(center_y, sign, x):
    """sign=-1 为上辊下侧圆弧，sign=+1 为下辊上侧圆弧。"""
    return center_y + sign * math.sqrt(ROLL_RADIUS * ROLL_RADIUS - x * x)


def coordinate_points(ranges):
    points = []
    for start, end, step in ranges:
        if not points:
            points.append(start)
        elif abs(points[-1] - start) > 1.0e-8:
            points.append(start)
        value = start
        while value + step < end - 1.0e-8:
            value += step
            points.append(value)
        if abs(points[-1] - end) > 1.0e-8:
            points.append(end)
    return points


def make_roll_part(model, name, center_y, upper=True):
    """
    建立单根轧辊 1/8 弹性实体的规则扫掠网格。

    这里使用 orphan mesh 直接生成 C3D8R 扫掠单元：
    - 接触外表面严格按 R=450 mm 圆柱面取点；
    - X=0 和 Z=0 为对称面；
    - 接触区 X=0~6 mm 沿 X 约 0.15 mm，沿 Z 约 1 mm；
    - 远离接触区逐渐粗化，避免自由四面体带来的 WarnElemDistorted。
    """
    part = model.Part(name=name, dimensionality=THREE_D, type=DEFORMABLE_BODY)

    x_points = coordinate_points((
        (0.0, ROLLER_CONTACT_X, ROLLER_CONTACT_X_SIZE),
        (ROLLER_CONTACT_X, 20.0, 2.0),
        (20.0, 80.0, 10.0),
        (80.0, ROLLER_BODY_X_MAX, 20.0),
    ))
    z_points = coordinate_points(((0.0, HALF_ROLL_WIDTH, ROLLER_CONTACT_Z_SIZE),))
    eta_points = list(ROLLER_NORMAL_ETA)

    node_map = {}
    label = 1
    sign = -1.0 if upper else +1.0
    for i, x in enumerate(x_points):
        surf_y = arc_y(center_y, sign, x)
        for j, eta in enumerate(eta_points):
            y = surf_y + eta * (center_y - surf_y)
            for k, z in enumerate(z_points):
                node_map[(i, j, k)] = part.Node(coordinates=(x, y, z), label=label)
                label += 1

    contact_labels = []
    bearing_labels = []
    elem_label = 1
    n_x = len(x_points) - 1
    n_eta = len(eta_points) - 1
    n_z = len(z_points) - 1
    for i in range(n_x):
        for j in range(n_eta):
            for k in range(n_z):
                nodes = (
                    node_map[(i, j, k)],
                    node_map[(i + 1, j, k)],
                    node_map[(i + 1, j, k + 1)],
                    node_map[(i, j, k + 1)],
                    node_map[(i, j + 1, k)],
                    node_map[(i + 1, j + 1, k)],
                    node_map[(i + 1, j + 1, k + 1)],
                    node_map[(i, j + 1, k + 1)],
                )
                if upper:
                    nodes = (
                        node_map[(i, j, k)],
                        node_map[(i, j, k + 1)],
                        node_map[(i + 1, j, k + 1)],
                        node_map[(i + 1, j, k)],
                        node_map[(i, j + 1, k)],
                        node_map[(i, j + 1, k + 1)],
                        node_map[(i + 1, j + 1, k + 1)],
                        node_map[(i + 1, j + 1, k)],
                    )
                elem = part.Element(nodes=nodes, elemShape=HEX8, label=elem_label)
                if j == 0:
                    contact_labels.append(elem_label)
                if k == n_z - 1:
                    bearing_labels.append(elem_label)
                elem_label += 1

    all_elems = part.Set(elements=part.elements, name='ALL_' + name.upper())
    part.SectionAssignment(region=all_elems, sectionName='RollSection')
    elem_type = mesh.ElemType(elemCode=C3D8R, elemLibrary=STANDARD,
                              kinematicSplit=AVERAGE_STRAIN,
                              hourglassControl=DEFAULT, distortionControl=DEFAULT)
    part.setElementType(regions=(part.elements,), elemTypes=(elem_type,))

    contact_elements = part.elements.sequenceFromLabels(labels=tuple(contact_labels))
    bearing_elements = part.elements.sequenceFromLabels(labels=tuple(bearing_labels))
    part.Surface(face1Elements=contact_elements, name=name.upper() + '_CONTACT')
    if upper:
        part.Surface(face4Elements=bearing_elements, name=name.upper() + '_BEARING_END')
    else:
        part.Surface(face5Elements=bearing_elements, name=name.upper() + '_BEARING_END')

    xsym_labels = [node_map[(0, j, k)].label
                   for j in range(len(eta_points))
                   for k in range(len(z_points))]
    zsym_labels = [node_map[(i, j, 0)].label
                   for i in range(len(x_points))
                   for j in range(len(eta_points))]
    xsym_nodes = part.nodes.sequenceFromLabels(labels=tuple(xsym_labels))
    zsym_nodes = part.nodes.sequenceFromLabels(labels=tuple(zsym_labels))
    part.Set(nodes=xsym_nodes, name=name.upper() + '_XSYM')
    part.Set(nodes=zsym_nodes, name=name.upper() + '_ZSYM')

    return part


def make_assembly(model, film, upper_roll, lower_roll):
    assembly = model.rootAssembly
    assembly.DatumCsysByDefault(CARTESIAN)
    film_i = assembly.Instance(name='Film-1', part=film, dependent=ON)
    upper_i = assembly.Instance(name='UpperRoll-1', part=upper_roll, dependent=ON)
    lower_i = assembly.Instance(name='LowerRoll-1', part=lower_roll, dependent=ON)

    upper_rp_obj = assembly.ReferencePoint(point=(0.0, UPPER_ROLL_CY, HALF_ROLL_WIDTH))
    lower_rp_obj = assembly.ReferencePoint(point=(0.0, LOWER_ROLL_CY, HALF_ROLL_WIDTH))
    upper_rp = assembly.referencePoints[upper_rp_obj.id]
    lower_rp = assembly.referencePoints[lower_rp_obj.id]
    upper_set = assembly.Set(referencePoints=(upper_rp,), name='UPPER_ROLL_RP')
    lower_set = assembly.Set(referencePoints=(lower_rp,), name='LOWER_ROLL_RP')

    return assembly, film_i, upper_i, lower_i, upper_set, lower_set


def make_interactions_and_bcs(model, assembly, film_i, upper_i, lower_i, upper_set, lower_set):
    # 接触属性
    prop = model.ContactProperty('Roll_Film_Friction')
    prop.NormalBehavior(pressureOverclosure=HARD, allowSeparation=ON,
                        constraintEnforcementMethod=DEFAULT)
    prop.TangentialBehavior(formulation=PENALTY, directionality=ISOTROPIC,
                            slipRateDependency=OFF, pressureDependency=OFF,
                            temperatureDependency=OFF, dependencies=0,
                            table=((FRICTION_COEFF,),), shearStressLimit=None,
                            maximumElasticSlip=FRACTION, fraction=0.005,
                            elasticSlipStiffness=None)

    model.SurfaceToSurfaceContactStd(
        name='UpperRoll_to_Film',
        createStepName='Initial',
        main=upper_i.surfaces['UPPERROLL_CONTACT'],
        secondary=film_i.surfaces['FILM_TOP'],
        sliding=FINITE,
        thickness=ON,
        interactionProperty='Roll_Film_Friction',
        adjustMethod=NONE,
        initialClearance=OMIT,
        datumAxis=None,
        clearanceRegion=None
    )
    model.SurfaceToSurfaceContactStd(
        name='LowerRoll_to_Film',
        createStepName='Initial',
        main=lower_i.surfaces['LOWERROLL_CONTACT'],
        secondary=film_i.surfaces['FILM_BOTTOM'],
        sliding=FINITE,
        thickness=ON,
        interactionProperty='Roll_Film_Friction',
        adjustMethod=NONE,
        initialClearance=OMIT,
        datumAxis=None,
        clearanceRegion=None
    )

    # 卸载步释放上辊；下辊默认继续采用给定的 mu=0.15 接触作为支承。
    # 如需低摩擦卸载对照，可在参数区开启切换并修改 UNLOAD_SUPPORT_FRICTION。
    if INCLUDE_UNLOAD_STEP and RELEASE_UPPER_CONTACT_IN_UNLOAD:
        model.interactions['UpperRoll_to_Film'].deactivate('Unload')
    if INCLUDE_UNLOAD_STEP and USE_LOW_FRICTION_LOWER_SUPPORT_IN_UNLOAD:
        support_prop = model.ContactProperty('Unload_LowFriction_Support')
        support_prop.NormalBehavior(pressureOverclosure=HARD, allowSeparation=ON,
                                    constraintEnforcementMethod=DEFAULT)
        support_prop.TangentialBehavior(
            formulation=PENALTY, directionality=ISOTROPIC,
            slipRateDependency=OFF, pressureDependency=OFF,
            temperatureDependency=OFF, dependencies=0,
            table=((UNLOAD_SUPPORT_FRICTION,),), shearStressLimit=None,
            maximumElasticSlip=FRACTION, fraction=0.005,
            elasticSlipStiffness=None)
        model.interactions['LowerRoll_to_Film'].setValuesInStep(
            stepName='Unload',
            interactionProperty='Unload_LowFriction_Support'
        )

    # RP 只耦合端部轴承/加载面，不耦合接触弧面。
    for cname, rp_set, surf in (
        ('Upper_RP_to_BearingEnd', upper_set, upper_i.surfaces['UPPERROLL_BEARING_END']),
        ('Lower_RP_to_BearingEnd', lower_set, lower_i.surfaces['LOWERROLL_BEARING_END']),
    ):
        model.Coupling(name=cname, controlPoint=rp_set, surface=surf,
                       influenceRadius=WHOLE_SURFACE, couplingType=DISTRIBUTING,
                       weightingMethod=UNIFORM, localCsys=None,
                       u1=ON, u2=ON, u3=ON,
                       ur1=OFF, ur2=OFF, ur3=OFF)

    # X=0 与 Z=0 对称边界。
    for inst, prefix in ((film_i, 'FILM'), (upper_i, 'UPPERROLL'), (lower_i, 'LOWERROLL')):
        model.XsymmBC(name='BC_' + prefix + '_XSYM',
                      createStepName='Initial',
                      region=inst.sets[prefix + '_XSYM'])
        model.ZsymmBC(name='BC_' + prefix + '_ZSYM',
                      createStepName='Initial',
                      region=inst.sets[prefix + '_ZSYM'])

    # 膜只在一个角点额外约束 U2，防止卸载接触脱开后的 Y 向刚体漂移。
    model.DisplacementBC(name='BC_Film_Y_Drift_Anchor', createStepName='Initial',
                         region=film_i.sets['FILM_Y_DRIFT_ANCHOR'],
                         u1=UNSET, u2=0.0, u3=UNSET,
                         amplitude=UNSET, distributionType=UNIFORM,
                         fieldName='', localCsys=None)

    # continuum distributed coupling 只激活平动自由度；RP 只约束 U1/U2/U3。
    # 下辊 RP 平动固定；上辊 RP 控制法向压下并约束 U1/U3。
    model.DisplacementBC(name='BC_Lower_RP_Fixed', createStepName='Initial',
                         region=lower_set, u1=0.0, u2=0.0, u3=0.0,
                         amplitude=UNSET, distributionType=UNIFORM,
                         fieldName='', localCsys=None)
    upper_bc = model.DisplacementBC(name='BC_Upper_RP_Press', createStepName='Initial',
                                    region=upper_set, u1=0.0, u2=0.0, u3=0.0,
                                    amplitude=UNSET, distributionType=UNIFORM,
                                    fieldName='', localCsys=None)
    upper_bc.setValuesInStep(stepName='Clamp_Down', u2=CLAMP_DISPLACEMENT)
    upper_bc.setValuesInStep(stepName='Hold', u2=CLAMP_DISPLACEMENT)
    if INCLUDE_UNLOAD_STEP:
        upper_bc.setValuesInStep(stepName='Unload', u2=0.0)


def make_steps(model):
    model.StaticStep(name='Clamp_Down', previous='Initial', nlgeom=ON,
                     timePeriod=1.0, maxNumInc=CLAMP_MAX_NUM_INC,
                     initialInc=CLAMP_INITIAL_INC,
                     minInc=CLAMP_MIN_INC, maxInc=CLAMP_MAX_INC)
    model.StaticStep(name='Hold', previous='Clamp_Down', nlgeom=ON,
                     timePeriod=1.0, maxNumInc=20, initialInc=1.0,
                     minInc=1.0e-8, maxInc=1.0)
    if INCLUDE_UNLOAD_STEP:
        model.StaticStep(name='Unload', previous='Hold', nlgeom=ON,
                         timePeriod=1.0, maxNumInc=1000, initialInc=0.0125,
                         minInc=1.0e-8, maxInc=0.025)


def make_outputs(model, upper_set):
    for key in list(model.fieldOutputRequests.keys()):
        del model.fieldOutputRequests[key]
    for key in list(model.historyOutputRequests.keys()):
        del model.historyOutputRequests[key]

    model.FieldOutputRequest(name='FieldOutputs',
                             createStepName='Clamp_Down',
                             variables=('S', 'U', 'LE', 'PE', 'PEEQ', 'EVOL'),
                             frequency=FIELD_OUTPUT_FREQUENCY)
    model.FieldOutputRequest(name='ContactOutputs',
                             createStepName='Clamp_Down',
                             variables=('CSTRESS', 'CDISP'),
                             frequency=FIELD_OUTPUT_FREQUENCY)
    # Abaqus/Standard 用 CSTRESS/CDISP 请求接触输出；ODB 中通常可读取 CPRESS、CSHEAR、COPEN 等分量。
    model.HistoryOutputRequest(name='Energy_History',
                               createStepName='Clamp_Down',
                               variables=('ALLIE', 'ALLPD', 'ALLSD'),
                               region=MODEL)
    model.HistoryOutputRequest(name='Upper_RP_U2_RF2',
                               createStepName='Clamp_Down',
                               variables=('U2', 'RF2'),
                               region=upper_set,
                               sectionPoints=DEFAULT,
                               rebar=EXCLUDE)


def patch_inp_contact_output(inp_path):
    """将 CAE 生成的接触预选输出改为 Standard 接受的接触组合变量。"""
    with open(inp_path, 'r') as f:
        text = f.read()
    text = text.replace('*Contact Output, variable=PRESELECT',
                        '*Contact Output\nCSTRESS, CDISP')
    with open(inp_path, 'w') as f:
        f.write(text)


def patch_inp_distributing_coupling(inp_path):
    """显式使用连续体分布耦合，避免实体单元云图触发结构转动耦合警告。"""
    with open(inp_path, 'r') as f:
        text = f.read()
    text = text.replace(
        '*Distributing, weighting method=UNIFORM',
        '*Distributing, coupling=CONTINUUM, rotational coupling=CONTINUUM, weighting method=UNIFORM'
    )
    with open(inp_path, 'w') as f:
        f.write(text)


def patch_inp_unload_stabilization(inp_path):
    """只在卸载步启用低耗散自动稳定，用于帮助接触释放收敛。"""
    if not INCLUDE_UNLOAD_STEP or not USE_UNLOAD_STABILIZATION:
        return
    with open(inp_path, 'r') as f:
        text = f.read()

    marker = '*Step, name=Unload'
    start = text.find(marker)
    if start < 0:
        raise RuntimeError('未找到 Unload 步，无法写入自动稳定参数。')
    static_pos = text.find('*Static', start)
    if static_pos < 0:
        raise RuntimeError('未找到 Unload 步的 *Static 行，无法写入自动稳定参数。')
    line_end = text.find('\n', static_pos)
    if line_end < 0:
        line_end = len(text)

    old_line = text[static_pos:line_end].strip()
    new_line = '*Static, stabilize=%g, allsdtol=%g' % (
        UNLOAD_STABILIZATION_FACTOR, UNLOAD_ALLSDTOL)
    if old_line.lower().startswith('*static, stabilize='):
        text = text[:static_pos] + new_line + text[line_end:]
    elif old_line.lower() == '*static':
        text = text[:static_pos] + new_line + text[line_end:]
    else:
        raise RuntimeError('Unload 步 *Static 行格式异常：%s' % old_line)

    with open(inp_path, 'w') as f:
        f.write(text)


def print_and_check_material_segment(inp_path):
    """打印并检查最终 INP 材料段，防止 DPC 关键字被误写。"""
    with open(inp_path, 'r') as f:
        text = f.read()

    start = text.find('** MATERIALS')
    end = text.find('** INTERACTION PROPERTIES', start)
    if start < 0 or end < 0:
        raise RuntimeError('未能在 INP 中定位材料段。')

    block = text[start:end].strip()
    print('\n===== FINAL INP MATERIAL SEGMENT BEGIN =====')
    print(block)
    print('===== FINAL INP MATERIAL SEGMENT END =====\n')

    upper_block = block.upper()
    if '*DRUCKER PRAGER' in upper_block:
        raise RuntimeError('材料段仍包含 *Drucker Prager，请检查材料定义。')
    if '*DRUCKER PRAGER HARDENING' in upper_block:
        raise RuntimeError('材料段仍包含 *Drucker Prager Hardening，请检查材料定义。')
    for keyword in ('*ELASTIC', '*CAP PLASTICITY', '*CAP HARDENING'):
        if keyword not in upper_block:
            raise RuntimeError('材料段缺少关键字：%s' % keyword)

    lines = block.splitlines()
    cap_line = None
    for i, line in enumerate(lines):
        if line.strip().upper().startswith('*CAP PLASTICITY'):
            for data_line in lines[i + 1:]:
                stripped = data_line.strip()
                if stripped and not stripped.startswith('*') and not stripped.startswith('**'):
                    cap_line = stripped
                    break
            break
    if cap_line is None:
        raise RuntimeError('未找到 *Cap Plasticity 数据行。')

    values = [float(item.strip()) for item in cap_line.split(',') if item.strip()]
    expected = [COHESION_D, FRICTION_ANGLE_BETA, CAP_ECCENTRICITY_R,
                INITIAL_CAP_POSITION, TRANSITION_SURFACE_RADIUS_ALPHA,
                FLOW_STRESS_RATIO_K]
    if len(values) < len(expected):
        raise RuntimeError('*Cap Plasticity 数据列数不足：%s' % cap_line)
    for actual, target in zip(values[:len(expected)], expected):
        if abs(actual - target) > 1.0e-8:
            raise RuntimeError('*Cap Plasticity 参数顺序/数值错误：%s' % cap_line)

    print('材料段检查通过：只包含 Elastic/Cap Plasticity/Cap Hardening，未出现 Drucker Prager。')


def main():
    ensure_work_dir()
    model = reset_model()
    make_materials(model)

    film = make_film_part(model)
    upper_roll = make_roll_part(model, 'UpperRoll', UPPER_ROLL_CY, upper=True)
    lower_roll = make_roll_part(model, 'LowerRoll', LOWER_ROLL_CY, upper=False)

    assembly, film_i, upper_i, lower_i, upper_set, lower_set = make_assembly(
        model, film, upper_roll, lower_roll
    )
    make_steps(model)
    make_interactions_and_bcs(model, assembly, film_i, upper_i, lower_i, upper_set, lower_set)
    make_outputs(model, upper_set)

    job = mdb.Job(name=JOB_NAME, model=MODEL_NAME, description='单层自支撑膜三维局部静态辊压等效模型',
                  type=ANALYSIS, atTime=None, waitMinutes=0, waitHours=0,
                  queue=None, memory=90, memoryUnits=PERCENTAGE,
                  getMemoryFromAnalysis=True, explicitPrecision=SINGLE,
                  nodalOutputPrecision=SINGLE, echoPrint=OFF, modelPrint=OFF,
                  contactPrint=OFF, historyPrint=OFF, userSubroutine='',
                  scratch='', resultsFormat=ODB, multiprocessingMode=DEFAULT,
                  numCpus=4, numDomains=4, numGPUs=0)

    cae_path = os.path.join(WORK_DIR, MODEL_NAME + '.cae')
    inp_path = os.path.join(WORK_DIR, JOB_NAME + '.inp')
    mdb.saveAs(pathName=cae_path)
    job.writeInput(consistencyChecking=OFF)
    patch_inp_contact_output(inp_path)
    patch_inp_distributing_coupling(inp_path)
    patch_inp_unload_stabilization(inp_path)
    print_and_check_material_segment(inp_path)

    print('完成：已保存 CAE: %s' % cae_path)
    print('完成：已导出 INP: %s' % inp_path)
    print('注意：脚本不会自动提交 Job，请在 Abaqus 中手动提交。')


if __name__ == '__main__':
    main()
