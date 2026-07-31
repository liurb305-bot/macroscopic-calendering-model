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
WORKDIR = r"E:\abaqus\3D3"
MODEL_NAME = "RollPress_3D_10pct"
JOB_NAME = "RollPress_3D_10pct"

# Electrode geometry
SHEET_LENGTH = 200.0
SHEET_WIDTH = 100.0
TOTAL_THICKNESS = 0.150
COLLECTOR_THICKNESS = 0.017
ACTIVE_THICKNESS = 0.133
ACTIVE_INITIAL_REL_DENSITY = 0.55

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
LINE_SPEED = 0.8 * 1000.0 / 60.0  # 0.8 m/min -> mm/s
ROLLER_OMEGA = LINE_SPEED / ROLLER_RADIUS
ENTRY_VELOCITY = LINE_SPEED
TENSION_STRESS = 0.5  # MPa = N/mm^2, placeholder stabilizing tension

# Analysis controls. The true rolling speed gives a long explicit event; target
# time increment keeps the model runnable and is documented as mass scaling.
BITE_TIME = 0.25
ROLLING_TIME = (0.5 * SHEET_LENGTH) / LINE_SPEED
MASS_SCALING_DT = 5.0e-5
NUM_CPUS = 1

# Mesh targets
SHEET_LENGTH_ELEMS = 200
SHEET_WIDTH_ELEMS = 50
ACTIVE_THICKNESS_ELEMS = 5
COLLECTOR_THICKNESS_ELEMS = 1
ROLLER_GLOBAL_SIZE = 6.0
ROLLER_FACE_SIZE = 4.0

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
    roller = model.Material(name="Cr3_ZincSteel_Roller_E220GPa")
    roller.Density(table=((7.85e-9,),))
    roller.Elastic(table=((220000.0, 0.30),))

    active = model.Material(name="ActiveLayer_Porous_Calibratable")
    active.Density(table=((1.98e-9,),))
    active.Elastic(table=((1000.0, 0.25),))
    # Placeholder crushable coating response; replace with calibrated data when available.
    active.Plastic(table=((2.0, 0.000),
                          (5.0, 0.020),
                          (12.0, 0.050),
                          (25.0, 0.100),
                          (55.0, 0.180),
                          (100.0, 0.300)))

    collector = model.Material(name="CollectorLayer_AlFoil_Calibratable")
    collector.Density(table=((2.70e-9,),))
    collector.Elastic(table=((70000.0, 0.33),))
    collector.Plastic(table=((80.0, 0.000),
                             (110.0, 0.020),
                             (145.0, 0.060),
                             (180.0, 0.120)))

    model.HomogeneousSolidSection(name="Sec_Roller_ZincSteel",
                                  material=roller.name,
                                  thickness=None)
    model.HomogeneousSolidSection(name="Sec_ActiveLayer",
                                  material=active.name,
                                  thickness=None)
    model.HomogeneousSolidSection(name="Sec_Collector",
                                  material=collector.name,
                                  thickness=None)


def make_sheet_part(model, name, thickness, section_name, thickness_elems):
    sketch = model.ConstrainedSketch(name=name + "_sketch", sheetSize=300.0)
    sketch.rectangle(point1=(SHEET_X_BACK, 0.0), point2=(SHEET_X_FRONT, thickness))
    part = model.Part(name=name, dimensionality=THREE_D, type=DEFORMABLE_BODY)
    part.BaseSolidExtrude(sketch=sketch, depth=SHEET_WIDTH)
    del model.sketches[sketch.name]

    part.SectionAssignment(region=regionToolset.Region(cells=part.cells),
                           sectionName=section_name)

    # Named surfaces in part coordinates. z=0..SHEET_WIDTH; assembly translates
    # them to z=-SHEET_WIDTH/2..+SHEET_WIDTH/2.
    mid_x = 0.5 * (SHEET_X_BACK + SHEET_X_FRONT)
    mid_z = 0.5 * SHEET_WIDTH
    top_face = part.faces.findAt(((mid_x, thickness, mid_z),))
    bottom_face = part.faces.findAt(((mid_x, 0.0, mid_z),))
    front_face = part.faces.findAt(((SHEET_X_FRONT, 0.5 * thickness, mid_z),))
    back_face = part.faces.findAt(((SHEET_X_BACK, 0.5 * thickness, mid_z),))
    part.Surface(name="Top", side1Faces=top_face)
    part.Surface(name="Bottom", side1Faces=bottom_face)
    part.Surface(name="Front", side1Faces=front_face)
    part.Surface(name="Back", side1Faces=back_face)

    # Structured hexahedral mesh.
    elem = mesh.ElemType(elemCode=C3D8R, elemLibrary=EXPLICIT)
    part.setElementType(regions=(part.cells,), elemTypes=(elem,))
    part.setMeshControls(regions=part.cells, elemShape=HEX, technique=STRUCTURED)

    length_edges = part.edges.findAt(
        ((0.5 * (SHEET_X_BACK + SHEET_X_FRONT), 0.0, 0.0),),
        ((0.5 * (SHEET_X_BACK + SHEET_X_FRONT), thickness, 0.0),),
        ((0.5 * (SHEET_X_BACK + SHEET_X_FRONT), 0.0, SHEET_WIDTH),),
        ((0.5 * (SHEET_X_BACK + SHEET_X_FRONT), thickness, SHEET_WIDTH),),
    )
    width_edges = part.edges.findAt(
        ((SHEET_X_BACK, 0.0, mid_z),),
        ((SHEET_X_FRONT, 0.0, mid_z),),
        ((SHEET_X_BACK, thickness, mid_z),),
        ((SHEET_X_FRONT, thickness, mid_z),),
    )
    thickness_edges = part.edges.findAt(
        ((SHEET_X_BACK, 0.5 * thickness, 0.0),),
        ((SHEET_X_FRONT, 0.5 * thickness, 0.0),),
        ((SHEET_X_BACK, 0.5 * thickness, SHEET_WIDTH),),
        ((SHEET_X_FRONT, 0.5 * thickness, SHEET_WIDTH),),
    )
    part.seedEdgeByNumber(edges=length_edges, number=SHEET_LENGTH_ELEMS, constraint=FINER)
    part.seedEdgeByNumber(edges=width_edges, number=SHEET_WIDTH_ELEMS, constraint=FINER)
    part.seedEdgeByNumber(edges=thickness_edges, number=thickness_elems, constraint=FINER)
    part.generateMesh()
    return part


def make_roller_part(model):
    sketch = model.ConstrainedSketch(name="Roller_sketch", sheetSize=160.0)
    sketch.CircleByCenterPerimeter(center=(0.0, 0.0), point1=(ROLLER_RADIUS, 0.0))
    part = model.Part(name="ElasticRoller_WithShoulders",
                      dimensionality=THREE_D,
                      type=DEFORMABLE_BODY)
    part.BaseSolidExtrude(sketch=sketch, depth=ROLLER_TOTAL_WIDTH)
    del model.sketches[sketch.name]

    # Partition the axial shoulders from the effective roller face.
    left_plane = part.DatumPlaneByPrincipalPlane(principalPlane=XYPLANE,
                                                 offset=ROLLER_SHOULDER_WIDTH)
    right_plane = part.DatumPlaneByPrincipalPlane(
        principalPlane=XYPLANE,
        offset=ROLLER_TOTAL_WIDTH - ROLLER_SHOULDER_WIDTH)
    part.PartitionCellByDatumPlane(datumPlane=part.datums[left_plane.id],
                                   cells=part.cells)
    part.PartitionCellByDatumPlane(datumPlane=part.datums[right_plane.id],
                                   cells=part.cells)

    part.SectionAssignment(region=regionToolset.Region(cells=part.cells),
                           sectionName="Sec_Roller_ZincSteel")

    z_mid = 0.5 * ROLLER_TOTAL_WIDTH
    contact_face = part.faces.findAt(((ROLLER_RADIUS, 0.0, z_mid),))
    end_face_1 = part.faces.findAt(((0.25 * ROLLER_RADIUS, 0.0, 0.0),))
    end_face_2 = part.faces.findAt(((0.25 * ROLLER_RADIUS, 0.0, ROLLER_TOTAL_WIDTH),))
    part.Surface(name="ContactFace", side1Faces=contact_face)
    part.Surface(name="EndFaces", side1Faces=end_face_1 + end_face_2)

    # Free tetrahedral mesh is robust for the deformable cylindrical roller.
    elem = mesh.ElemType(elemCode=C3D4, elemLibrary=EXPLICIT)
    part.setElementType(regions=(part.cells,), elemTypes=(elem,))
    part.setMeshControls(regions=part.cells, elemShape=TET, technique=FREE)
    part.seedPart(size=ROLLER_GLOBAL_SIZE, deviationFactor=0.1, minSizeFactor=0.1)
    central_edges = part.edges.getByBoundingBox(
        xMin=-ROLLER_RADIUS - TOL, xMax=ROLLER_RADIUS + TOL,
        yMin=-ROLLER_RADIUS - TOL, yMax=ROLLER_RADIUS + TOL,
        zMin=ROLLER_SHOULDER_WIDTH - TOL,
        zMax=ROLLER_TOTAL_WIDTH - ROLLER_SHOULDER_WIDTH + TOL)
    part.seedEdgeBySize(edges=central_edges, size=ROLLER_FACE_SIZE, constraint=FINER)
    part.generateMesh()
    return part


def make_assembly(model, active_part, collector_part, roller_part):
    root = model.rootAssembly
    root.DatumCsysByDefault(CARTESIAN)

    active = root.Instance(name="ActiveLayer-1", part=active_part, dependent=ON)
    collector = root.Instance(name="CollectorLayer-1", part=collector_part, dependent=ON)
    lower = root.Instance(name="LowerRoller-1", part=roller_part, dependent=ON)
    upper = root.Instance(name="UpperRoller-1", part=roller_part, dependent=ON)

    # Center the width about z=0 and stack the layers in y.
    root.translate(instanceList=(active.name,), vector=(0.0, COLLECTOR_THICKNESS, -0.5 * SHEET_WIDTH))
    root.translate(instanceList=(collector.name,), vector=(0.0, 0.0, -0.5 * SHEET_WIDTH))

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
    root.Set(name="Sheet_All",
             cells=active.cells + collector.cells)

    # Combined strip end surfaces for front/back tensile stress.
    active_front = active.faces.findAt(((SHEET_X_FRONT, COLLECTOR_THICKNESS + 0.5 * ACTIVE_THICKNESS, 0.0),))
    collector_front = collector.faces.findAt(((SHEET_X_FRONT, 0.5 * COLLECTOR_THICKNESS, 0.0),))
    active_back = active.faces.findAt(((SHEET_X_BACK, COLLECTOR_THICKNESS + 0.5 * ACTIVE_THICKNESS, 0.0),))
    collector_back = collector.faces.findAt(((SHEET_X_BACK, 0.5 * COLLECTOR_THICKNESS, 0.0),))
    root.Surface(name="Sheet_FrontFace", side1Faces=active_front + collector_front)
    root.Surface(name="Sheet_BackFace", side1Faces=active_back + collector_back)

    return root, active, collector, upper, lower


def make_interactions(model, root, active, collector, upper, lower):
    # Tie current collector to active coating.
    model.Tie("Tie_Collector_to_Active",
              collector.surfaces["Top"],
              active.surfaces["Bottom"],
              adjust=ON,
              positionToleranceMethod=COMPUTED,
              tieRotations=ON,
              thickness=ON)

    prop_top = model.ContactProperty(name="Contact_TopRoller_Active_mu0p3")
    prop_top.NormalBehavior(pressureOverclosure=HARD,
                            allowSeparation=ON,
                            constraintEnforcementMethod=DEFAULT)
    prop_top.TangentialBehavior(formulation=PENALTY,
                                directionality=ISOTROPIC,
                                slipRateDependency=OFF,
                                pressureDependency=OFF,
                                temperatureDependency=OFF,
                                dependencies=0,
                                table=((0.3,),),
                                maximumElasticSlip=FRACTION,
                                fraction=0.005)

    prop_bottom = model.ContactProperty(name="Contact_BottomRoller_Collector_mu0p1")
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

    model.SurfaceToSurfaceContactExp("TopRoller_to_Active",
                                     "Initial",
                                     upper.surfaces["ContactFace"],
                                     active.surfaces["Top"],
                                     FINITE,
                                     prop_top.name,
                                     "",
                                     ON,
                                     PENALTY)
    model.SurfaceToSurfaceContactExp("BottomRoller_to_Collector",
                                     "Initial",
                                     lower.surfaces["ContactFace"],
                                     collector.surfaces["Bottom"],
                                     FINITE,
                                     prop_bottom.name,
                                     "",
                                     ON,
                                     PENALTY)

    # Kinematic coupling at rigid shoulder/end support regions.
    model.Coupling(name="Couple_Upper_EndFaces_to_RP",
                   surface=upper.surfaces["EndFaces"],
                   controlPoint=root.sets["RP_UpperRoller"],
                   influenceRadius=WHOLE_SURFACE,
                   couplingType=KINEMATIC,
                   u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON)
    model.Coupling(name="Couple_Lower_EndFaces_to_RP",
                   surface=lower.surfaces["EndFaces"],
                   controlPoint=root.sets["RP_LowerRoller"],
                   influenceRadius=WHOLE_SURFACE,
                   couplingType=KINEMATIC,
                   u1=ON, u2=ON, u3=ON, ur1=ON, ur2=ON, ur3=ON)


def make_steps_bcs_loads_outputs(model, root):
    model.ExplicitDynamicsStep(
        name="Bite_Clamp",
        previous="Initial",
        timePeriod=BITE_TIME,
        nlgeom=ON,
        massScaling=((SEMI_AUTOMATIC, MODEL, AT_BEGINNING, 0.0,
                      MASS_SCALING_DT, BELOW_MIN, 0, 0, 0.0, 0.0, 0, None),))
    model.ExplicitDynamicsStep(
        name="Rolling",
        previous="Bite_Clamp",
        timePeriod=ROLLING_TIME,
        nlgeom=ON,
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
                     vr3=ROLLER_OMEGA,
                     amplitude="Amp_Bite_Ramp")
    model.VelocityBC(name="BC_Lower_Rotation",
                     createStepName="Bite_Clamp",
                     region=lower_rp,
                     vr3=-ROLLER_OMEGA,
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
        model.fieldOutputRequests["F-Output-1"].setValues(
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
                  description="3D explicit roll pressing model, 10 percent thickness reduction",
                  type=ANALYSIS,
                  explicitPrecision=SINGLE,
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
    collector = make_sheet_part(model, "CollectorLayer", COLLECTOR_THICKNESS,
                                "Sec_Collector", COLLECTOR_THICKNESS_ELEMS)
    roller = make_roller_part(model)
    root, active_i, collector_i, upper_i, lower_i = make_assembly(model, active, collector, roller)
    make_interactions(model, root, active_i, collector_i, upper_i, lower_i)
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
    print("Line speed: %.6f mm/s; omega: %.6f rad/s; rolling step time: %.6f s" %
          (LINE_SPEED, ROLLER_OMEGA, ROLLING_TIME))
    print("Mass scaling target dt: %.3e s" % MASS_SCALING_DT)


if __name__ == "__main__":
    main()
