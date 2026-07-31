from abaqus import *
from abaqusConstants import *
from caeModules import *
from driverUtils import executeOnCaeStartup
import mesh
import os
import re
import regionToolset

executeOnCaeStartup()


WORKDIR = r"E:\abaqus\2D3.1"
MODEL_NAME = "FlatUnload_2D_DPC_E50Nu015"
JOB_NAME = MODEL_NAME

# Units: mm, tonne, second, N, MPa
SHEET_LENGTH = 6.0
SHEET_X_BACK = -3.0
SHEET_X_FRONT = 3.0
TOTAL_THICKNESS = 0.150
TARGET_THICKNESS = 0.135
COMPRESS_DISP = TOTAL_THICKNESS - TARGET_THICKNESS
UNLOAD_LIFT = 0.002
PLATE_HALF_LENGTH = 3.2

ACTIVE_DENSITY = 2.55e-9
ACTIVE_E = 50.0
ACTIVE_NU = 0.15
DPC_COHESION = 4.0
DPC_FRICTION_ANGLE = 65.0
DPC_ECCENTRICITY = 0.8
DPC_INITIAL_CAP_POSITION = 60.0
DPC_TRANSITION_SURFACE_RADIUS = 0.02
DPC_FLOW_STRESS_RATIO = 1.0
DPC_CAP_HARDENING = (
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

FRICTION_COEFF = 0.1
COMPRESS_TIME = 0.20
HOLD_TIME = 0.05
UNLOAD_TIME = 0.20
SETTLE_TIME = 0.10
MASS_SCALING_DT = 1.0e-6
BULK_VISCOSITY_LINEAR = 0.12
BULK_VISCOSITY_QUADRATIC = 1.2
THICKNESS_ELEMS = 24
X_SEED_SIZE = 0.05
TOL = 1.0e-7
NUM_CPUS = 6


def reset_model():
    os.chdir(WORKDIR)
    if MODEL_NAME in mdb.models:
        del mdb.models[MODEL_NAME]
    model = mdb.Model(name=MODEL_NAME, modelType=STANDARD_EXPLICIT)
    if "Model-1" in mdb.models and len(mdb.models["Model-1"].parts) == 0:
        del mdb.models["Model-1"]
    return model


def make_material(model):
    mat = model.Material(name="ActiveLayer_DPC")
    mat.Density(table=((ACTIVE_DENSITY,),))
    mat.Elastic(table=((ACTIVE_E, ACTIVE_NU),))
    mat.CapPlasticity(table=((DPC_COHESION,
                              DPC_FRICTION_ANGLE,
                              DPC_ECCENTRICITY,
                              DPC_INITIAL_CAP_POSITION,
                              DPC_TRANSITION_SURFACE_RADIUS,
                              DPC_FLOW_STRESS_RATIO),)).CapHardening(
        table=DPC_CAP_HARDENING)
    model.HomogeneousSolidSection(name="Sec_ActiveLayer",
                                  material=mat.name,
                                  thickness=None)


def make_sheet_part(model):
    sketch = model.ConstrainedSketch(name="Sheet_sketch", sheetSize=20.0)
    sketch.rectangle(point1=(SHEET_X_BACK, 0.0),
                     point2=(SHEET_X_FRONT, TOTAL_THICKNESS))
    part = model.Part(name="ActiveLayer",
                      dimensionality=TWO_D_PLANAR,
                      type=DEFORMABLE_BODY)
    part.BaseShell(sketch=sketch)
    del model.sketches[sketch.name]

    part.SectionAssignment(region=regionToolset.Region(faces=part.faces),
                           sectionName="Sec_ActiveLayer")
    elem = mesh.ElemType(elemCode=CPE4R, elemLibrary=EXPLICIT)
    part.setElementType(regions=(part.faces,), elemTypes=(elem,))
    part.setMeshControls(regions=part.faces, elemShape=QUAD,
                         technique=STRUCTURED)

    x_edges = []
    y_edges = []
    for edge in part.edges:
        p = edge.pointOn[0]
        if abs(p[1]) < TOL or abs(p[1] - TOTAL_THICKNESS) < TOL:
            x_edges.append(edge)
        else:
            y_edges.append(edge)
    part.seedEdgeBySize(edges=x_edges, size=X_SEED_SIZE, constraint=FINER)
    part.seedEdgeByNumber(edges=y_edges, number=THICKNESS_ELEMS,
                          constraint=FINER)
    part.generateMesh()
    return part


def make_rigid_plate_part(model, name, contact_side):
    sketch = model.ConstrainedSketch(name=name + "_sketch", sheetSize=20.0)
    sketch.Line(point1=(-PLATE_HALF_LENGTH, 0.0),
                point2=(PLATE_HALF_LENGTH, 0.0))
    part = model.Part(name=name,
                      dimensionality=TWO_D_PLANAR,
                      type=ANALYTIC_RIGID_SURFACE)
    part.AnalyticRigidSurf2DPlanar(sketch=sketch)
    del model.sketches[sketch.name]
    part.ReferencePoint(point=(0.0, 0.0, 0.0))
    if contact_side == "side1":
        part.Surface(name="PlateSurface", side1Edges=part.edges)
    else:
        part.Surface(name="PlateSurface", side2Edges=part.edges)
    return part


def make_assembly(model, sheet_part, top_part, bottom_part):
    root = model.rootAssembly
    root.DatumCsysByDefault(CARTESIAN)
    sheet = root.Instance(name="ActiveLayer-1", part=sheet_part, dependent=ON)
    top = root.Instance(name="TopPlate-1", part=top_part, dependent=ON)
    bottom = root.Instance(name="BottomPlate-1", part=bottom_part,
                           dependent=ON)
    root.translate(instanceList=(top.name,), vector=(0.0, TOTAL_THICKNESS,
                                                     0.0))

    top_rp = top.referencePoints[top_part.referencePoints.keys()[0]]
    bottom_rp = bottom.referencePoints[bottom_part.referencePoints.keys()[0]]
    root.Set(name="RP_TopPlate", referencePoints=(top_rp,))
    root.Set(name="RP_BottomPlate", referencePoints=(bottom_rp,))
    root.Set(name="Sheet_All", faces=sheet.faces)
    root.Set(name="Bottom_Left_Node", nodes=sheet.nodes.getByBoundingBox(
        xMin=SHEET_X_BACK - TOL, xMax=SHEET_X_BACK + TOL,
        yMin=-TOL, yMax=TOL))
    root.Set(name="Top_Nodes", nodes=sheet.nodes.getByBoundingBox(
        xMin=SHEET_X_BACK - TOL, xMax=SHEET_X_FRONT + TOL,
        yMin=TOTAL_THICKNESS - TOL, yMax=TOTAL_THICKNESS + TOL))
    root.Set(name="Bottom_Nodes", nodes=sheet.nodes.getByBoundingBox(
        xMin=SHEET_X_BACK - TOL, xMax=SHEET_X_FRONT + TOL,
        yMin=-TOL, yMax=TOL))
    root.Surface(name="Sheet_Top", side1Edges=sheet.edges.getByBoundingBox(
        xMin=SHEET_X_BACK - TOL, xMax=SHEET_X_FRONT + TOL,
        yMin=TOTAL_THICKNESS - TOL, yMax=TOTAL_THICKNESS + TOL))
    root.Surface(name="Sheet_Bottom", side1Edges=sheet.edges.getByBoundingBox(
        xMin=SHEET_X_BACK - TOL, xMax=SHEET_X_FRONT + TOL,
        yMin=-TOL, yMax=TOL))
    return root, sheet, top, bottom


def make_interactions(model, root, top, bottom):
    prop = model.ContactProperty(name="Plate_Film_mu0p1")
    prop.NormalBehavior(pressureOverclosure=HARD,
                        allowSeparation=ON,
                        constraintEnforcementMethod=DEFAULT)
    prop.TangentialBehavior(formulation=PENALTY,
                            directionality=ISOTROPIC,
                            table=((FRICTION_COEFF,),),
                            maximumElasticSlip=FRACTION,
                            fraction=0.005)
    model.SurfaceToSurfaceContactExp("TopPlate_to_Film",
                                     "Initial",
                                     top.surfaces["PlateSurface"],
                                     root.surfaces["Sheet_Top"],
                                     FINITE,
                                     prop.name,
                                     "",
                                     ON,
                                     KINEMATIC)
    model.SurfaceToSurfaceContactExp("BottomPlate_to_Film",
                                     "Initial",
                                     bottom.surfaces["PlateSurface"],
                                     root.surfaces["Sheet_Bottom"],
                                     FINITE,
                                     prop.name,
                                     "",
                                     ON,
                                     KINEMATIC)


def make_steps_bcs_outputs(model, root):
    model.ExplicitDynamicsStep(name="Compress_Down",
                               previous="Initial",
                               timePeriod=COMPRESS_TIME,
                               nlgeom=ON,
                               linearBulkViscosity=BULK_VISCOSITY_LINEAR,
                               quadBulkViscosity=BULK_VISCOSITY_QUADRATIC,
                               massScaling=((SEMI_AUTOMATIC, MODEL,
                                             AT_BEGINNING, 0.0,
                                             MASS_SCALING_DT, BELOW_MIN,
                                             0, 0, 0.0, 0.0, 0, None),))
    model.ExplicitDynamicsStep(name="Hold_135um",
                               previous="Compress_Down",
                               timePeriod=HOLD_TIME,
                               nlgeom=ON,
                               linearBulkViscosity=BULK_VISCOSITY_LINEAR,
                               quadBulkViscosity=BULK_VISCOSITY_QUADRATIC)
    model.ExplicitDynamicsStep(name="Unload",
                               previous="Hold_135um",
                               timePeriod=UNLOAD_TIME,
                               nlgeom=ON,
                               linearBulkViscosity=BULK_VISCOSITY_LINEAR,
                               quadBulkViscosity=BULK_VISCOSITY_QUADRATIC)
    model.ExplicitDynamicsStep(name="Free_Settle",
                               previous="Unload",
                               timePeriod=SETTLE_TIME,
                               nlgeom=ON,
                               linearBulkViscosity=BULK_VISCOSITY_LINEAR,
                               quadBulkViscosity=BULK_VISCOSITY_QUADRATIC)

    t1 = COMPRESS_TIME
    t2 = t1 + HOLD_TIME
    t3 = t2 + UNLOAD_TIME
    t4 = t3 + SETTLE_TIME
    model.SmoothStepAmplitude(name="Amp_Top_U2_Total",
                              timeSpan=TOTAL,
                              data=((0.0, 0.0),
                                    (t1, -COMPRESS_DISP),
                                    (t2, -COMPRESS_DISP),
                                    (t3, UNLOAD_LIFT),
                                    (t4, UNLOAD_LIFT)))

    top_rp = root.sets["RP_TopPlate"]
    bottom_rp = root.sets["RP_BottomPlate"]
    model.DisplacementBC(name="BC_BottomPlate_Fixed",
                         createStepName="Initial",
                         region=bottom_rp,
                         u1=0.0, u2=0.0, ur3=0.0)
    model.DisplacementBC(name="BC_TopPlate_XR_Fixed",
                         createStepName="Initial",
                         region=top_rp,
                         u1=0.0, u2=UNSET, ur3=0.0)
    model.DisplacementBC(name="BC_TopPlate_U2_History",
                         createStepName="Compress_Down",
                         region=top_rp,
                         u1=UNSET, u2=1.0, ur3=UNSET,
                         amplitude="Amp_Top_U2_Total")
    model.DisplacementBC(name="BC_Sheet_X_Anchor",
                         createStepName="Initial",
                         region=root.sets["Bottom_Left_Node"],
                         u1=0.0, u2=UNSET, ur3=UNSET)

    if "F-Output-1" in model.fieldOutputRequests:
        model.fieldOutputRequests["F-Output-1"].suppress()
    model.FieldOutputRequest(name="F_Sheet",
                             createStepName="Compress_Down",
                             region=root.sets["Sheet_All"],
                             variables=("S", "LE", "PE", "PEEQ", "U", "V",
                                        "A", "CSTRESS"),
                             numIntervals=160)
    model.HistoryOutputRequest(name="H_TopPlate_RP",
                               createStepName="Compress_Down",
                               variables=("RF1", "RF2", "U2", "V2"),
                               region=top_rp,
                               numIntervals=500)
    model.HistoryOutputRequest(name="H_BottomPlate_RP",
                               createStepName="Compress_Down",
                               variables=("RF1", "RF2", "U2", "V2"),
                               region=bottom_rp,
                               numIntervals=500)
    model.HistoryOutputRequest(name="H_Energy",
                               createStepName="Compress_Down",
                               variables=("ALLAE", "ALLIE", "ALLKE", "ALLMW",
                                          "ALLPD", "ALLSE", "ALLWK"),
                               numIntervals=500)


def make_job(model):
    if JOB_NAME in mdb.jobs:
        del mdb.jobs[JOB_NAME]
    return mdb.Job(name=JOB_NAME,
                   model=model.name,
                   description="2D DPC flat compression unload residual thickness check",
                   type=ANALYSIS,
                   explicitPrecision=SINGLE,
                   nodalOutputPrecision=SINGLE,
                   multiprocessingMode=DEFAULT,
                   numCpus=NUM_CPUS,
                   numDomains=NUM_CPUS)


def main():
    model = reset_model()
    make_material(model)
    sheet = make_sheet_part(model)
    top = make_rigid_plate_part(model, "TopRigidPlate", "side2")
    bottom = make_rigid_plate_part(model, "BottomRigidPlate", "side1")
    root, sheet_i, top_i, bottom_i = make_assembly(model, sheet, top, bottom)
    make_interactions(model, root, top_i, bottom_i)
    make_steps_bcs_outputs(model, root)
    job = make_job(model)
    mdb.saveAs(pathName=os.path.join(WORKDIR, MODEL_NAME + ".cae"))
    job.writeInput(consistencyChecking=ON)

    inp_path = os.path.join(WORKDIR, JOB_NAME + ".inp")
    with open(inp_path, "r") as f:
        inp_text = f.read()
    inp_text = inp_text.replace(
        "** PARTS\n",
        "** SECTION CONTROLS\n"
        "*SECTION CONTROLS, NAME=FilmControls, HOURGLASS=ENHANCED, "
        "DISTORTION CONTROL=YES\n"
        "** PARTS\n",
        1)
    inp_text = re.sub(
        r"(\*Solid Section, elset=[^,\n]+, material=ActiveLayer_DPC)(\s*\n)",
        r"\1, controls=FilmControls\2",
        inp_text,
        count=1)
    with open(inp_path, "w") as f:
        f.write(inp_text)
    print("Created flat unload DPC model: %s" % MODEL_NAME)
    print("Initial thickness %.6f mm" % TOTAL_THICKNESS)
    print("Target compressed thickness %.6f mm" % TARGET_THICKNESS)
    print("Top plate compress displacement %.6f mm" % COMPRESS_DISP)
    print("Unload lift %.6f mm" % UNLOAD_LIFT)
    print("Material E %.6f MPa nu %.6f" % (ACTIVE_E, ACTIVE_NU))
    print("Mass scaling target dt %.3e s" % MASS_SCALING_DT)


if __name__ == "__main__":
    main()
