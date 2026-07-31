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
WORKDIR = r"E:\abaqus\3D5+3.0"
MODEL_NAME = "RollPress_3D_DoubleCoat_DiffSpeed_DCP_V3p0_AnalyticRigid"
JOB_NAME = "RollPress_3D_DoubleCoat_DiffSpeed_DCP_V3p0_AnalyticRigid"

# Electrode geometry
SHEET_LENGTH = 200.0
SHEET_WIDTH = 100.0
TOTAL_THICKNESS = 0.150
ACTIVE_THICKNESS = TOTAL_THICKNESS
ACTIVE_INITIAL_REL_DENSITY = 0.55

# Drucker-Prager/Cap parameters for the active coating, calibrated from the
# provided DCP parameter set.
# Units: MPa for cohesion/cap pressure, degree for friction angle.
DCP_COHESION = 4.0
DCP_FRICTION_ANGLE = 65.0
DCP_ECCENTRICITY = 0.8
DCP_INITIAL_CAP_POSITION = 60.0
DCP_TRANSITION_SURFACE_RADIUS = 0.02
DCP_FLOW_STRESS_RATIO = 1.0
DCP_CAP_HARDENING = (
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
TENSION_STRESS = 0.5  # MPa = N/mm^2

# Analysis controls. A reduced fixed mass scaling target keeps the full explicit
# rolling run below the impractical increment count seen with no mass scaling.
BITE_TIME = 5.0
ROLLING_TIME = (0.5 * SHEET_LENGTH) / UPPER_LINE_SPEED
MASS_SCALING_DT = 1.0e-6
BULK_VISCOSITY_LINEAR = 0.12
BULK_VISCOSITY_QUADRATIC = 1.2
NUM_CPUS = 6

# Mesh targets. The strip is refined only over the material length that will
# pass through the nip during the simulated half-sheet rolling event.
SHEET_X_PARTITIONS = (-160.0, -120.0)
SHEET_X_COARSE_SIZE = 2.0
SHEET_X_TRANSITION_SIZE = 0.75
SHEET_X_CONTACT_SIZE = 0.35
SHEET_WIDTH_SIZE = 2.0
ACTIVE_THICKNESS_ELEMS = 23
ROLLER_ANALYTIC_WIDTH = ROLLER_FACE_WIDTH

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
    if midx >= -120.0:
        return SHEET_X_CONTACT_SIZE
    if midx >= -160.0:
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


def make_roller_part(model, name, upper):
    sketch = model.ConstrainedSketch(name=name + "_sketch", sheetSize=160.0)
    if upper:
        direction = CLOCKWISE
        arc_points = ((ROLLER_RADIUS, 0.0),
                      (0.0, -ROLLER_RADIUS),
                      (-ROLLER_RADIUS, 0.0))
    else:
        direction = COUNTERCLOCKWISE
        arc_points = ((ROLLER_RADIUS, 0.0),
                      (0.0, ROLLER_RADIUS),
                      (-ROLLER_RADIUS, 0.0))
    sketch.ArcByCenterEnds(center=(0.0, 0.0),
                           point1=arc_points[0],
                           point2=arc_points[1],
                           direction=direction)
    sketch.ArcByCenterEnds(center=(0.0, 0.0),
                           point1=arc_points[1],
                           point2=arc_points[2],
                           direction=direction)

    part = model.Part(name=name,
                      dimensionality=THREE_D,
                      type=ANALYTIC_RIGID_SURFACE)
    part.AnalyticRigidSurfExtrude(sketch=sketch, depth=ROLLER_ANALYTIC_WIDTH)
    del model.sketches[sketch.name]

    part.ReferencePoint(point=(0.0, 0.0, 0.5 * ROLLER_ANALYTIC_WIDTH))
    part.Surface(name="ContactFace", side1Faces=part.faces)
    return part


def _instance_reference_point(instance):
    rp_keys = list(instance.referencePoints.keys())
    if len(rp_keys) != 1:
        raise RuntimeError("Expected exactly one reference point on %s" %
                           instance.name)
    return instance.referencePoints[rp_keys[0]]


def make_assembly(model, active_part, lower_roller_part, upper_roller_part):
    root = model.rootAssembly
    root.DatumCsysByDefault(CARTESIAN)

    active = root.Instance(name="ActiveLayer-1", part=active_part, dependent=ON)
    lower = root.Instance(name="LowerRoller-1", part=lower_roller_part, dependent=ON)
    upper = root.Instance(name="UpperRoller-1", part=upper_roller_part, dependent=ON)

    # Center the self-supporting coating width about z=0; y=0..TOTAL_THICKNESS.
    root.translate(instanceList=(active.name,), vector=(0.0, 0.0, -0.5 * SHEET_WIDTH))

    lower_center_y = -ROLLER_RADIUS
    upper_center_y = TOTAL_THICKNESS + ROLLER_RADIUS
    root.translate(instanceList=(lower.name,),
                   vector=(0.0, lower_center_y, -0.5 * ROLLER_ANALYTIC_WIDTH))
    root.translate(instanceList=(upper.name,),
                   vector=(0.0, upper_center_y, -0.5 * ROLLER_ANALYTIC_WIDTH))

    # Use the analytic rigid part reference points so prescribed motion is tied
    # directly to each roller surface.
    upper_rp = _instance_reference_point(upper)
    lower_rp = _instance_reference_point(lower)

    root.Set(name="RP_UpperRoller", referencePoints=(upper_rp,))
    root.Set(name="RP_LowerRoller", referencePoints=(lower_rp,))
    root.Set(name="Sheet_All", cells=active.cells)
    root.Set(name="Sheet_Active", cells=active.cells)

    # Combined strip end surfaces for front/back tensile stress.
    active_front = active.faces.findAt(((SHEET_X_FRONT, 0.5 * ACTIVE_THICKNESS, 0.0),))
    active_back = active.faces.findAt(((SHEET_X_BACK, 0.5 * ACTIVE_THICKNESS, 0.0),))
    root.Surface(name="Sheet_FrontFace", side1Faces=active_front)
    root.Surface(name="Sheet_BackFace", side1Faces=active_back)

    return root, active, upper, lower


def make_interactions(model, root, active, upper, lower):
    prop_top = model.ContactProperty(name="Contact_TopRoller_Active_mu0p1")
    prop_top.NormalBehavior(pressureOverclosure=HARD,
                            allowSeparation=ON,
                            constraintEnforcementMethod=DEFAULT)
    prop_top.TangentialBehavior(formulation=PENALTY,
                                directionality=ISOTROPIC,
                                slipRateDependency=OFF,
                                pressureDependency=OFF,
                                temperatureDependency=OFF,
                                dependencies=0,
                                table=((0.1,),),
                                maximumElasticSlip=FRACTION,
                                fraction=0.005)

    prop_bottom = model.ContactProperty(name="Contact_BottomRoller_Active_mu0p1")
    prop_bottom.NormalBehavior(pressureOverclosure=HARD,
                               allowSeparation=ON,
                               constraintEnforcementMethod=DEFAULT)
    prop_bottom.TangentialBehavior(formulation=PENALTY,
                                   directionality=ISOTROPIC,
                                   slipRateDependency=OFF,
                                   pressureDependency=OFF,
                                   temperatureDependency=OFF,
                                   dependencies=0,
                                   table=((0.1,),),
                                   maximumElasticSlip=FRACTION,
                                   fraction=0.005)

    model.SurfaceToSurfaceContactExp("TopRoller_to_ActiveFilm",
                                     "Initial",
                                     upper.surfaces["ContactFace"],
                                     active.surfaces["Top"],
                                     FINITE,
                                     prop_top.name,
                                     "",
                                     ON,
                                     KINEMATIC)
    model.SurfaceToSurfaceContactExp("BottomRoller_to_ActiveFilm",
                                     "Initial",
                                     lower.surfaces["ContactFace"],
                                     active.surfaces["Bottom"],
                                     FINITE,
                                     prop_bottom.name,
                                     "",
                                     ON,
                                     KINEMATIC)


def make_steps_bcs_loads_outputs(model, root):
    model.ExplicitDynamicsStep(
        name="Bite_Clamp",
        previous="Initial",
        timePeriod=BITE_TIME,
        nlgeom=ON,
        linearBulkViscosity=BULK_VISCOSITY_LINEAR,
        quadBulkViscosity=BULK_VISCOSITY_QUADRATIC,
        massScaling=((SEMI_AUTOMATIC, MODEL, AT_BEGINNING, 0.0,
                      MASS_SCALING_DT, BELOW_MIN, 0, 0, 0.0, 0.0, 0, None),))
    model.ExplicitDynamicsStep(
        name="Rolling",
        previous="Bite_Clamp",
        timePeriod=ROLLING_TIME,
        nlgeom=ON,
        linearBulkViscosity=BULK_VISCOSITY_LINEAR,
        quadBulkViscosity=BULK_VISCOSITY_QUADRATIC,
        massScaling=((SEMI_AUTOMATIC, MODEL, AT_BEGINNING, 0.0,
                      MASS_SCALING_DT, BELOW_MIN, 0, 0, 0.0, 0.0, 0, None),))

    model.SmoothStepAmplitude(name="Amp_Bite_Ramp",
                              timeSpan=TOTAL,
                              data=((0.0, 0.0),
                                    (BITE_TIME, 1.0),
                                    (BITE_TIME + ROLLING_TIME, 1.0)))

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
                         createStepName="Bite_Clamp",
                         region=upper_rp,
                         u1=UNSET, u2=-UPPER_ROLLER_DROP, u3=UNSET,
                         ur1=UNSET, ur2=UNSET, ur3=UNSET,
                         amplitude="Amp_Bite_Ramp")

    # Opposite signs give +x surface velocity at the lower top and upper bottom.
    model.VelocityBC(name="BC_Upper_Rotation",
                     createStepName="Bite_Clamp",
                     region=upper_rp,
                     vr3=UPPER_ROLLER_OMEGA,
                     amplitude="Amp_Bite_Ramp")
    model.VelocityBC(name="BC_Lower_Rotation",
                     createStepName="Bite_Clamp",
                     region=lower_rp,
                     vr3=-LOWER_ROLLER_OMEGA,
                     amplitude="Amp_Bite_Ramp")

    # Initial strip feed speed for nip entry.
    model.Velocity(name="IC_Sheet_EntryVelocity",
                   region=sheet_all,
                   velocity1=ENTRY_VELOCITY,
                   velocity2=0.0,
                   velocity3=0.0,
                   omega=0.0,
                   axisBegin=(0.0, 0.0, 0.0),
                   axisEnd=(0.0, 0.0, 1.0))

    # Stabilizing front/back tensile stress placeholders.
    model.SurfaceTraction(name="Load_Front_Tension",
                          createStepName="Bite_Clamp",
                          region=root.surfaces["Sheet_FrontFace"],
                          magnitude=TENSION_STRESS,
                          directionVector=((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)),
                          traction=GENERAL,
                          follower=OFF,
                          resultant=OFF)
    model.SurfaceTraction(name="Load_Back_Tension",
                          createStepName="Bite_Clamp",
                          region=root.surfaces["Sheet_BackFace"],
                          magnitude=TENSION_STRESS,
                          directionVector=((0.0, 0.0, 0.0), (-1.0, 0.0, 0.0)),
                          traction=GENERAL,
                          follower=OFF,
                          resultant=OFF)

    if "F-Output-1" in model.fieldOutputRequests:
        model.fieldOutputRequests["F-Output-1"].suppress()
    model.FieldOutputRequest(name="F_Sheet_StressStrain",
                             createStepName="Bite_Clamp",
                             region=sheet_all,
                             variables=("S", "LE", "PE", "PEEQ", "U", "V", "A", "RF", "CSTRESS"),
                             numIntervals=120)
    model.HistoryOutputRequest(name="H_UpperRoller_RP",
                               createStepName="Bite_Clamp",
                               variables=("RF1", "RF2", "RF3", "RM3", "U2", "UR3", "VR3"),
                               region=upper_rp,
                               numIntervals=1000)
    model.HistoryOutputRequest(name="H_LowerRoller_RP",
                               createStepName="Bite_Clamp",
                               variables=("RF1", "RF2", "RF3", "RM3", "U2", "UR3", "VR3"),
                               region=lower_rp,
                               numIntervals=1000)
    model.HistoryOutputRequest(name="H_Energy",
                               createStepName="Bite_Clamp",
                               variables=("ALLIE", "ALLKE", "ALLWK", "ALLSE", "ALLPD"),
                               numIntervals=1000)


def make_job(model):
    if JOB_NAME in mdb.jobs:
        del mdb.jobs[JOB_NAME]
    job = mdb.Job(name=JOB_NAME,
                  model=model.name,
                  description="3D explicit self-supporting coating roll pressing model with analytic rigid rollers",
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
    lower_roller = make_roller_part(model, "AnalyticLowerRoller", upper=False)
    upper_roller = make_roller_part(model, "AnalyticUpperRoller", upper=True)
    root, active_i, upper_i, lower_i = make_assembly(model, active,
                                                     lower_roller,
                                                     upper_roller)
    make_interactions(model, root, active_i, upper_i, lower_i)
    make_steps_bcs_loads_outputs(model, root)
    job = make_job(model)
    mdb.saveAs(pathName=os.path.join(WORKDIR, MODEL_NAME + ".cae"))
    job.writeInput(consistencyChecking=ON)
    print("Created model: %s" % MODEL_NAME)
    print("Created job:   %s" % JOB_NAME)
    print("CAE path:      %s" % os.path.join(WORKDIR, MODEL_NAME + ".cae"))
    print("INP path:      %s" % os.path.join(WORKDIR, JOB_NAME + ".inp"))
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
    print("Entry velocity: %.6f mm/s; rolling step time: %.6f s" %
          (ENTRY_VELOCITY, ROLLING_TIME))
    print("Analytic rigid roller contact width: %.6f mm" % ROLLER_ANALYTIC_WIDTH)
    print("Mass scaling target dt: %.3e s" % MASS_SCALING_DT)


if __name__ == "__main__":
    main()
