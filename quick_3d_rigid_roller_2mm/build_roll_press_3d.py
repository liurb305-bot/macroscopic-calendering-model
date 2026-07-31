from abaqus import *
from abaqusConstants import *
from caeModules import *
from driverUtils import executeOnCaeStartup
import assembly
import mesh
import os
import regionToolset

executeOnCaeStartup()


# ---------------------------------------------------------------------------
# Roll pressing model parameters
# Unit system: mm, tonne, second, N, MPa
# ---------------------------------------------------------------------------
WORKDIR = r"E:\abaqus\3D100mm"
MODEL_NAME = "RollPress_3D_SelfSupport_DiffSpeed_DPC_S00008B5_50mm_Roll2mm_RigidRoller_Stable"
JOB_NAME = "RollPress_3D_SelfSupport_DiffSpeed_DPC_S00008B5_50mm_Roll2mm_RigidRoller_Stable"

# Electrode geometry
SHEET_LENGTH = 50.0
SHEET_WIDTH = 100.0
TOTAL_THICKNESS = 0.150
ACTIVE_THICKNESS = TOTAL_THICKNESS
ACTIVE_INITIAL_REL_DENSITY = 0.55

# Drucker-Prager/Cap parameters for the active coating.  The values below are
# the S00008B5 candidate fitted by 2D roll pressing and then screened in 3D.
# Units: MPa for cohesion/cap pressure, degree for friction angle.
DCP_COHESION = 0.0032
DCP_FRICTION_ANGLE = 5.0
DCP_ECCENTRICITY = 0.8
DCP_INITIAL_CAP_POSITION = 0.048
DCP_TRANSITION_SURFACE_RADIUS = 0.02
DCP_FLOW_STRESS_RATIO = 1.0
DCP_CAP_HARDENING = (
    (0.048, 0.00),
    (0.0496, 0.05),
    (0.0528, 0.10),
    (0.0576, 0.15),
    (0.064, 0.18),
    (0.072, 0.20),
    (0.088, 0.22),
    (0.112, 0.24),
    (0.152, 0.26),
    (0.216, 0.28),
    (0.312, 0.30),
    (0.384, 0.31),
    (0.480, 0.32),
    (0.592, 0.33),
)

# The strip leading edge starts at the nip center (x=0) and feeds in +x.
SHEET_X_BACK = -SHEET_LENGTH
SHEET_X_FRONT = 0.0

# Roller geometry, estimated from the supplied figure.
ROLLER_RADIUS = 50.0
ROLLER_FACE_WIDTH = 120.0
ROLLER_TOTAL_WIDTH = 140.0
ROLLER_SHOULDER_WIDTH = 0.5 * (ROLLER_TOTAL_WIDTH - ROLLER_FACE_WIDTH)

# Process
REDUCTION_RATIO = 0.10
TARGET_THICKNESS = TOTAL_THICKNESS * (1.0 - REDUCTION_RATIO)
UPPER_ROLLER_DROP = TOTAL_THICKNESS - TARGET_THICKNESS
UPPER_LINE_SPEED = 0.8 * 1000.0 / 60.0  # 0.8 m/min -> mm/s
LOWER_LINE_SPEED = 0.5 * 1000.0 / 60.0  # 0.5 m/min -> mm/s
UPPER_ROLLER_OMEGA = UPPER_LINE_SPEED / ROLLER_RADIUS
LOWER_ROLLER_OMEGA = LOWER_LINE_SPEED / ROLLER_RADIUS
ENTRY_VELOCITY = UPPER_LINE_SPEED
TENSION_STRESS = 0.01  # MPa = N/mm^2; reduced for the very soft fitted DPC film

# Analysis controls. The staged setup first closes the nip without rotation,
# holds it briefly, then starts rolling with a smooth speed ramp.
CLAMP_DOWN_TIME = 0.005
HOLD_TIME = 0.02
ROLLING_RAMP_TIME = 0.01
ROLLING_TIME = 0.15
MASS_SCALING_MODE = "LOCAL_ACTIVE"  # NONE or LOCAL_ACTIVE
MASS_SCALING_TARGET_DT = 5.0e-06
MASS_SCALING_SET_NAME = "MassScale_Active_Critical"
MASS_SCALING_X_MIN = -SHEET_LENGTH
MASS_SCALING_X_MAX = 0.0
BULK_VISCOSITY_LINEAR = 0.12
BULK_VISCOSITY_QUADRATIC = 1.2
NUM_CPUS = 6

# Mesh targets. The strip is refined only over the material length that will
# pass through the nip during the simulated half-sheet rolling event.
SHEET_X_PARTITIONS = (-35.0, -20.0, -12.0, -5.0, -1.0)
SHEET_X_COARSE_SIZE = 2.5
SHEET_X_TRANSITION_SIZE = 1.5
SHEET_X_FINE_SIZE = 0.6
SHEET_X_NIP_SIZE = 0.30
SHEET_WIDTH_SIZE = 2.5
ACTIVE_THICKNESS_ELEMS = 5
ROLLER_GLOBAL_SIZE = 12.0
ROLLER_FACE_SIZE = 5.0

TOL = 1.0e-6


def reset_model():
    os.chdir(WORKDIR)
    for job_name in list(mdb.jobs.keys()):
        if job_name.startswith(MODEL_NAME + "_smoke"):
            del mdb.jobs[job_name]
    existing_models = list(mdb.models.keys())
    holder_name = "__tmp_delete_holder__"
    if MODEL_NAME in existing_models and len(existing_models) == 1:
        mdb.Model(name=holder_name, modelType=STANDARD_EXPLICIT)
        existing_models = list(mdb.models.keys())
    for model_name in existing_models:
        if ((model_name.startswith("__tmp_") and model_name != holder_name) or
                model_name.startswith(MODEL_NAME + "_smoke")):
            del mdb.models[model_name]
    if MODEL_NAME in list(mdb.models.keys()):
        del mdb.models[MODEL_NAME]
    model = mdb.Model(name=MODEL_NAME, modelType=STANDARD_EXPLICIT)
    if "Model-1" in list(mdb.models.keys()) and len(mdb.models["Model-1"].parts) == 0:
        del mdb.models["Model-1"]
    if holder_name in list(mdb.models.keys()):
        del mdb.models[holder_name]
    return model


def make_materials(model):
    active = model.Material(name="ActiveLayer_Porous_Calibratable")
    active.Density(table=((2.55e-9,),))
    active.Elastic(table=((6500.0, 0.01),))
    # Drucker-Prager/Cap model for porous coating compaction.
    active.CapPlasticity(table=((DCP_COHESION,
                                 DCP_FRICTION_ANGLE,
                                 DCP_ECCENTRICITY,
                                 DCP_INITIAL_CAP_POSITION,
                                 DCP_TRANSITION_SURFACE_RADIUS,
                                 DCP_FLOW_STRESS_RATIO),)).CapHardening(
        table=DCP_CAP_HARDENING)

    model.HomogeneousSolidSection(name="Sec_ActiveLayer",
                                  material=active.name,
                                  thickness=None)


def _edge_axis_and_mid(part, edge):
    coords = [part.vertices[i].pointOn[0] for i in edge.getVertices()]
    xs = [c[0] for c in coords]
    ys = [c[1] for c in coords]
    zs = [c[2] for c in coords]
    dx = max(xs) - min(xs)
    dy = max(ys) - min(ys)
    dz = max(zs) - min(zs)
    mid = (0.5 * (max(xs) + min(xs)),
           0.5 * (max(ys) + min(ys)),
           0.5 * (max(zs) + min(zs)))
    if dx >= dy and dx >= dz:
        return "x", mid
    if dz >= dx and dz >= dy:
        return "z", mid
    return "y", mid


def _sheet_x_seed_size(midx):
    if midx >= -5.0:
        return SHEET_X_NIP_SIZE
    if midx >= -20.0:
        return SHEET_X_FINE_SIZE
    if midx >= -35.0:
        return SHEET_X_TRANSITION_SIZE
    return SHEET_X_COARSE_SIZE


def make_sheet_part(model, name, thickness, section_name, thickness_elems):
    sketch = model.ConstrainedSketch(name=name + "_sketch", sheetSize=300.0)
    sketch.rectangle(point1=(SHEET_X_BACK, 0.0), point2=(SHEET_X_FRONT, thickness))
    part = model.Part(name=name, dimensionality=THREE_D, type=DEFORMABLE_BODY)
    part.BaseSolidExtrude(sketch=sketch, depth=SHEET_WIDTH)
    del model.sketches[sketch.name]

    for x_pos in SHEET_X_PARTITIONS:
        datum = part.DatumPlaneByPrincipalPlane(principalPlane=YZPLANE,
                                                offset=x_pos)
        part.PartitionCellByDatumPlane(datumPlane=part.datums[datum.id],
                                       cells=part.cells)

    part.SectionAssignment(region=regionToolset.Region(cells=part.cells),
                           sectionName=section_name)

    # Named surfaces in part coordinates. z=0..SHEET_WIDTH; assembly translates
    # them to z=-SHEET_WIDTH/2..+SHEET_WIDTH/2. Bounding boxes are used because
    # the x partitions split top/bottom into multiple faces.
    top_faces = part.faces.getByBoundingBox(
        xMin=SHEET_X_BACK - TOL, xMax=SHEET_X_FRONT + TOL,
        yMin=thickness - TOL, yMax=thickness + TOL,
        zMin=-TOL, zMax=SHEET_WIDTH + TOL)
    bottom_faces = part.faces.getByBoundingBox(
        xMin=SHEET_X_BACK - TOL, xMax=SHEET_X_FRONT + TOL,
        yMin=-TOL, yMax=TOL,
        zMin=-TOL, zMax=SHEET_WIDTH + TOL)
    front_faces = part.faces.getByBoundingBox(
        xMin=SHEET_X_FRONT - TOL, xMax=SHEET_X_FRONT + TOL,
        yMin=-TOL, yMax=thickness + TOL,
        zMin=-TOL, zMax=SHEET_WIDTH + TOL)
    back_faces = part.faces.getByBoundingBox(
        xMin=SHEET_X_BACK - TOL, xMax=SHEET_X_BACK + TOL,
        yMin=-TOL, yMax=thickness + TOL,
        zMin=-TOL, zMax=SHEET_WIDTH + TOL)
    part.Surface(name="Top", side1Faces=top_faces)
    part.Surface(name="Bottom", side1Faces=bottom_faces)
    part.Surface(name="Front", side1Faces=front_faces)
    part.Surface(name="Back", side1Faces=back_faces)

    elem = mesh.ElemType(elemCode=C3D8R, elemLibrary=EXPLICIT)
    part.setElementType(regions=(part.cells,), elemTypes=(elem,))
    part.setMeshControls(regions=part.cells, elemShape=HEX, technique=STRUCTURED)

    x_edges = {}
    width_edges = []
    thickness_edges = []
    for edge in part.edges:
        axis, mid = _edge_axis_and_mid(part, edge)
        if axis == "x":
            size = _sheet_x_seed_size(mid[0])
            x_edges.setdefault(size, []).append(edge)
        elif axis == "z":
            width_edges.append(edge)
        else:
            thickness_edges.append(edge)

    for size, edges in x_edges.items():
        part.seedEdgeBySize(edges=edges, size=size, constraint=FINER)
    part.seedEdgeBySize(edges=width_edges, size=SHEET_WIDTH_SIZE, constraint=FINER)
    part.seedEdgeByNumber(edges=thickness_edges, number=thickness_elems, constraint=FINER)
    part.generateMesh()
    return part


def make_roller_part(model):
    sketch = model.ConstrainedSketch(name="Roller_sketch", sheetSize=160.0)
    sketch.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(ROLLER_RADIUS, 0.0))
    part = model.Part(name="RigidRoller_CylindricalShell",
                      dimensionality=THREE_D,
                      type=DISCRETE_RIGID_SURFACE)
    part.BaseShellExtrude(sketch=sketch, depth=ROLLER_TOTAL_WIDTH)
    del model.sketches[sketch.name]

    part.Surface(name="ContactFace", side1Faces=part.faces)
    elem = mesh.ElemType(elemCode=R3D4, elemLibrary=EXPLICIT)
    part.setElementType(regions=(part.faces,), elemTypes=(elem,))
    part.seedPart(size=ROLLER_GLOBAL_SIZE, deviationFactor=0.1, minSizeFactor=0.1)
    part.generateMesh()
    return part


def make_assembly(model, active_part, roller_part):
    root = model.rootAssembly
    root.DatumCsysByDefault(CARTESIAN)

    active = root.Instance(name="ActiveLayer-1", part=active_part, dependent=ON)
    lower = root.Instance(name="LowerRoller-1", part=roller_part, dependent=ON)
    upper = root.Instance(name="UpperRoller-1", part=roller_part, dependent=ON)

    # Center the self-supporting coating width about z=0; y=0..TOTAL_THICKNESS.
    root.translate(instanceList=(active.name,), vector=(0.0, 0.0, -0.5 * SHEET_WIDTH))

    lower_center_y = -ROLLER_RADIUS
    upper_center_y = TOTAL_THICKNESS + ROLLER_RADIUS
    root.translate(instanceList=(lower.name,),
                   vector=(0.0, lower_center_y, -0.5 * ROLLER_TOTAL_WIDTH))
    root.translate(instanceList=(upper.name,),
                   vector=(0.0, upper_center_y, -0.5 * ROLLER_TOTAL_WIDTH))

    # Reference points at roller axes.
    upper_rp_obj = root.ReferencePoint(point=(0.0, upper_center_y, 0.0))
    lower_rp_obj = root.ReferencePoint(point=(0.0, lower_center_y, 0.0))
    upper_rp = root.referencePoints[upper_rp_obj.id]
    lower_rp = root.referencePoints[lower_rp_obj.id]

    root.Set(name="RP_UpperRoller", referencePoints=(upper_rp,))
    root.Set(name="RP_LowerRoller", referencePoints=(lower_rp,))
    root.Set(name="Sheet_All", cells=active.cells)
    root.Set(name="Sheet_Active", cells=active.cells)
    mass_scale_elements = active.elements.getByBoundingBox(
        xMin=MASS_SCALING_X_MIN - TOL, xMax=MASS_SCALING_X_MAX + TOL,
        yMin=-TOL, yMax=ACTIVE_THICKNESS + TOL,
        zMin=-0.5 * SHEET_WIDTH - TOL, zMax=0.5 * SHEET_WIDTH + TOL)
    if len(mass_scale_elements) == 0:
        mass_scale_elements = active.elements
    root.Set(name=MASS_SCALING_SET_NAME, elements=mass_scale_elements)

    # Combined strip end surfaces for front/back tensile stress.
    active_front = active.faces.findAt(((SHEET_X_FRONT, 0.5 * ACTIVE_THICKNESS, 0.0),))
    active_back = active.faces.findAt(((SHEET_X_BACK, 0.5 * ACTIVE_THICKNESS, 0.0),))
    root.Surface(name="Sheet_FrontFace", side1Faces=active_front)
    root.Surface(name="Sheet_BackFace", side1Faces=active_back)

    return root, active, upper, lower


def make_interactions(model, root, active, upper, lower):
    prop = model.ContactProperty(name="Contact_Roller_Active_mu0p1")
    prop.NormalBehavior(pressureOverclosure=HARD,
                        allowSeparation=ON,
                        constraintEnforcementMethod=DEFAULT)
    prop.TangentialBehavior(formulation=PENALTY,
                            directionality=ISOTROPIC,
                            slipRateDependency=OFF,
                            pressureDependency=OFF,
                            temperatureDependency=OFF,
                            dependencies=0,
                            table=((0.1,),),
                            maximumElasticSlip=FRACTION,
                            fraction=0.005)

    general_contact = model.ContactExp(name="GeneralContact_RollerFilm",
                                       createStepName="Initial")
    general_contact.includedPairs.setValuesInStep(stepName="Initial",
                                                  useAllstar=ON)
    general_contact.contactPropertyAssignments.appendInStep(
        stepName="Initial",
        assignments=((GLOBAL, SELF, prop.name),))

    # The rollers are discrete rigid cylindrical surfaces controlled by their
    # reference points; no internal roller mesh or elastic deformation is used.
    model.RigidBody(name="Rigid_UpperRoller",
                    refPointRegion=root.sets["RP_UpperRoller"],
                    bodyRegion=regionToolset.Region(faces=upper.faces))
    model.RigidBody(name="Rigid_LowerRoller",
                    refPointRegion=root.sets["RP_LowerRoller"],
                    bodyRegion=regionToolset.Region(faces=lower.faces))


def make_steps_bcs_loads_outputs(model, root):
    model.ExplicitDynamicsStep(
        name="Clamp_Down",
        previous="Initial",
        timePeriod=CLAMP_DOWN_TIME,
        nlgeom=ON,
        linearBulkViscosity=BULK_VISCOSITY_LINEAR,
        quadBulkViscosity=BULK_VISCOSITY_QUADRATIC)
    model.ExplicitDynamicsStep(
        name="Hold_Clamp",
        previous="Clamp_Down",
        timePeriod=HOLD_TIME,
        nlgeom=ON,
        linearBulkViscosity=BULK_VISCOSITY_LINEAR,
        quadBulkViscosity=BULK_VISCOSITY_QUADRATIC)
    model.ExplicitDynamicsStep(
        name="Rolling",
        previous="Hold_Clamp",
        timePeriod=ROLLING_TIME,
        nlgeom=ON,
        linearBulkViscosity=BULK_VISCOSITY_LINEAR,
        quadBulkViscosity=BULK_VISCOSITY_QUADRATIC)

    total_time = CLAMP_DOWN_TIME + HOLD_TIME + ROLLING_TIME
    model.SmoothStepAmplitude(name="Amp_Clamp_Drop",
                              timeSpan=TOTAL,
                              data=((0.0, 0.0),
                                    (CLAMP_DOWN_TIME, 1.0),
                                    (total_time, 1.0)))
    model.SmoothStepAmplitude(name="Amp_Rolling_Spin",
                              timeSpan=STEP,
                              data=((0.0, 0.0),
                                    (ROLLING_RAMP_TIME, 1.0),
                                    (ROLLING_TIME, 1.0)))

    upper_rp = root.sets["RP_UpperRoller"]
    lower_rp = root.sets["RP_LowerRoller"]
    sheet_all = root.sets["Sheet_All"]

    # Roller support DOFs: translations and UR1/UR2 fixed; UR3 available for spin.
    model.DisplacementBC(name="BC_Lower_RP_Support",
                         createStepName="Initial",
                         region=lower_rp,
                         u1=0.0, u2=0.0, u3=0.0,
                         ur1=0.0, ur2=0.0, ur3=UNSET)
    model.DisplacementBC(name="BC_Upper_RP_LateralSupport",
                         createStepName="Initial",
                         region=upper_rp,
                         u1=0.0, u2=UNSET, u3=0.0,
                         ur1=0.0, ur2=0.0, ur3=UNSET)
    model.DisplacementBC(name="BC_Upper_RP_Drop",
                         createStepName="Clamp_Down",
                         region=upper_rp,
                         u1=UNSET, u2=-UPPER_ROLLER_DROP, u3=UNSET,
                         ur1=UNSET, ur2=UNSET, ur3=UNSET,
                         amplitude="Amp_Clamp_Drop")

    # Hold the rollers stationary during clamp/hold, then release UR3 when
    # rolling starts so the velocity boundary can drive the differential speeds.
    model.DisplacementBC(name="BC_Upper_NoSpin_ClampHold",
                         createStepName="Clamp_Down",
                         region=upper_rp,
                         u1=UNSET, u2=UNSET, u3=UNSET,
                         ur1=UNSET, ur2=UNSET, ur3=0.0)
    model.DisplacementBC(name="BC_Lower_NoSpin_ClampHold",
                         createStepName="Clamp_Down",
                         region=lower_rp,
                         u1=UNSET, u2=UNSET, u3=UNSET,
                         ur1=UNSET, ur2=UNSET, ur3=0.0)
    model.boundaryConditions["BC_Upper_NoSpin_ClampHold"].deactivate("Rolling")
    model.boundaryConditions["BC_Lower_NoSpin_ClampHold"].deactivate("Rolling")

    # Opposite signs give +x surface velocity at the lower top and upper bottom.
    # Rotation starts only after the clamp and hold stages are complete.
    model.VelocityBC(name="BC_Upper_Rotation",
                     createStepName="Rolling",
                     region=upper_rp,
                     vr3=UPPER_ROLLER_OMEGA,
                     amplitude="Amp_Rolling_Spin")
    model.VelocityBC(name="BC_Lower_Rotation",
                     createStepName="Rolling",
                     region=lower_rp,
                     vr3=-LOWER_ROLLER_OMEGA,
                     amplitude="Amp_Rolling_Spin")

    # Stabilizing front/back tensile stress placeholders.
    model.SurfaceTraction(name="Load_Front_Tension",
                          createStepName="Clamp_Down",
                          region=root.surfaces["Sheet_FrontFace"],
                          magnitude=TENSION_STRESS,
                          directionVector=((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                          traction=GENERAL,
                          follower=OFF,
                          resultant=OFF)
    model.SurfaceTraction(name="Load_Back_Tension",
                          createStepName="Clamp_Down",
                          region=root.surfaces["Sheet_BackFace"],
                          magnitude=TENSION_STRESS,
                          directionVector=((0.0, 0.0, 0.0), (-1.0, 0.0, 0.0)),
                          traction=GENERAL,
                          follower=OFF,
                          resultant=OFF)

    if "F-Output-1" in model.fieldOutputRequests:
        model.fieldOutputRequests["F-Output-1"].suppress()
    model.FieldOutputRequest(name="F_Sheet_StressStrain",
                             createStepName="Clamp_Down",
                             region=sheet_all,
                             variables=("S", "LE", "PE", "PEEQ", "U", "V", "A", "RF", "CSTRESS"),
                             numIntervals=80)
    model.HistoryOutputRequest(name="H_UpperRoller_RP",
                               createStepName="Clamp_Down",
                               variables=("RF1", "RF2", "RF3", "RM3", "U2", "UR3", "VR3"),
                               region=upper_rp,
                               numIntervals=250)
    model.HistoryOutputRequest(name="H_LowerRoller_RP",
                               createStepName="Clamp_Down",
                               variables=("RF1", "RF2", "RF3", "RM3", "U2", "UR3", "VR3"),
                               region=lower_rp,
                               numIntervals=250)
    model.HistoryOutputRequest(name="H_Energy",
                               createStepName="Clamp_Down",
                               variables=("ALLIE", "ALLKE", "ALLWK", "ALLSE", "ALLPD",
                                          "ALLAE", "ALLMW"),
                               numIntervals=250)


def apply_local_mass_scaling_keyword(model):
    if MASS_SCALING_MODE != "LOCAL_ACTIVE":
        return
    if MASS_SCALING_TARGET_DT is None or MASS_SCALING_TARGET_DT <= 0.0:
        raise ValueError("MASS_SCALING_TARGET_DT must be positive for LOCAL_ACTIVE mode")

    model.keywordBlock.synchVersions(storeNodesAndElements=False)
    step_seen = False
    insert_position = None
    for i, block in enumerate(model.keywordBlock.sieBlocks):
        stripped = block.strip()
        if stripped.startswith("*Step") and "Clamp_Down" in stripped:
            step_seen = True
            continue
        if step_seen and stripped.startswith("*Bulk Viscosity"):
            insert_position = i + 1
            break
        if step_seen and stripped.startswith("*Step"):
            break

    if insert_position is None:
        raise RuntimeError("Could not locate Clamp_Down bulk viscosity block for local mass scaling")

    keyword = ("*Fixed Mass Scaling, type=below min, dt=%.12g, elset=%s" %
               (MASS_SCALING_TARGET_DT, MASS_SCALING_SET_NAME))
    model.keywordBlock.insert(position=insert_position, text=keyword)


def make_job(model):
    if JOB_NAME in mdb.jobs:
        del mdb.jobs[JOB_NAME]
    job = mdb.Job(name=JOB_NAME,
                  model=model.name,
                  description="3D explicit staged clamp-then-roll model for a self-supporting coating film",
                  type=ANALYSIS,
                  explicitPrecision=DOUBLE,
                  nodalOutputPrecision=SINGLE,
                  multiprocessingMode=DEFAULT,
                  numCpus=NUM_CPUS,
                  numDomains=NUM_CPUS)
    return job


def main():
    model = reset_model()
    make_materials(model)
    active = make_sheet_part(model, "ActiveLayer", ACTIVE_THICKNESS, "Sec_ActiveLayer",
                             ACTIVE_THICKNESS_ELEMS)
    roller = make_roller_part(model)
    root, active_i, upper_i, lower_i = make_assembly(model, active, roller)
    make_interactions(model, root, active_i, upper_i, lower_i)
    make_steps_bcs_loads_outputs(model, root)
    apply_local_mass_scaling_keyword(model)
    job = make_job(model)
    mdb.saveAs(pathName=os.path.join(WORKDIR, MODEL_NAME + ".cae"))
    job.writeInput(consistencyChecking=ON)
    inp_path = os.path.join(WORKDIR, JOB_NAME + ".inp")
    with open(inp_path, "r") as inp_file:
        inp_text = inp_file.read()
    inp_text = inp_text.replace("*Contact, op=NEW", "*Contact")
    with open(inp_path, "w") as inp_file:
        inp_file.write(inp_text)
    print("Created model: %s" % MODEL_NAME)
    print("Created job:   %s" % JOB_NAME)
    print("CAE path:      %s" % os.path.join(WORKDIR, MODEL_NAME + ".cae"))
    print("INP path:      %s" % inp_path)
    print("Target thickness: %.6f mm; upper roller drop: %.6f mm" %
          (TARGET_THICKNESS, UPPER_ROLLER_DROP))
    print("Self-supporting active coating thickness: %.6f mm" % ACTIVE_THICKNESS)
    print("Active coating model: Drucker-Prager/Cap placeholder")
    print("DCP: cohesion=%.6f MPa; friction angle=%.6f deg; eccentricity=%.6f; cap pos=%.6f; R=%.6f; K=%.6f" %
          (DCP_COHESION, DCP_FRICTION_ANGLE, DCP_ECCENTRICITY,
           DCP_INITIAL_CAP_POSITION, DCP_TRANSITION_SURFACE_RADIUS,
           DCP_FLOW_STRESS_RATIO))
    print("Upper line speed: %.6f mm/s; omega: %.6f rad/s" %
          (UPPER_LINE_SPEED, UPPER_ROLLER_OMEGA))
    print("Lower line speed: %.6f mm/s; omega: %.6f rad/s" %
          (LOWER_LINE_SPEED, LOWER_ROLLER_OMEGA))
    print("Clamp down time: %.6f s; hold time: %.6f s; rolling ramp time: %.6f s; rolling time: %.6f s" %
          (CLAMP_DOWN_TIME, HOLD_TIME, ROLLING_RAMP_TIME, ROLLING_TIME))
    if MASS_SCALING_MODE == "LOCAL_ACTIVE":
        print("Local active mass scaling set: %s; target dt: %.3e s" %
              (MASS_SCALING_SET_NAME, MASS_SCALING_TARGET_DT))
    else:
        print("Mass scaling: NONE")


if __name__ == "__main__":
    main()
