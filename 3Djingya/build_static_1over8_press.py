from abaqus import *
from abaqusConstants import *
from caeModules import *
from driverUtils import executeOnCaeStartup
import mesh
import os
import regionToolset

executeOnCaeStartup()


# ---------------------------------------------------------------------------
# Local quasi-static explicit normal press model parameters
# Unit system: mm, tonne, second, N, MPa
# Coordinates: X = roller axis, Y = press direction, Z = rolling direction
# ---------------------------------------------------------------------------
WORKDIR = r"E:\abaqus\3Djingya"
MODEL_NAME = "RollPress_QuasiStatic_LocalSym_50mm_RigidRoller"
JOB_NAME = "RollPress_QuasiStatic_LocalSym_50mm_RigidRoller"

# Original full electrode dimensions from the 3D50mm model.
FULL_SHEET_WIDTH_X = 100.0
FULL_SHEET_LENGTH_Z = 50.0
TOTAL_THICKNESS = 0.150
ACTIVE_THICKNESS = TOTAL_THICKNESS
ACTIVE_INITIAL_REL_DENSITY = 0.55

# Symmetric local model dimensions. The model keeps X>=0 and Z>=0.
SHEET_HALF_WIDTH_X = 0.5 * FULL_SHEET_WIDTH_X
SHEET_HALF_LENGTH_Z = 0.5 * FULL_SHEET_LENGTH_Z

# Drucker-Prager/Cap parameters copied from the baseline 3D50mm model.
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

# Roller geometry copied from the baseline model. Only the X>=0 half is built.
ROLLER_RADIUS = 50.0
ROLLER_FACE_WIDTH = 120.0
ROLLER_TOTAL_WIDTH = 140.0
ROLLER_MODELED_WIDTH_X = 0.5 * ROLLER_TOTAL_WIDTH

# Pressing target copied from the 10% reduction baseline.
UPPER_ROLLER_DROP = 0.015
CONTACT_FRICTION_COEFF = 0.1

# Explicit quasi-static step controls. The pseudo-time is only a numerical
# loading path for local normal pressing; no rolling speed is modeled.
PRESS_TIME = 1.0e-3
NUM_CPUS = 6

# Mesh targets. Fine zoning is near the symmetry nip plane Z=0.
SHEET_Z_PARTITIONS = (1.0, 5.0, 12.5)
SHEET_X_SIZE = 2.5
SHEET_Z_COARSE_SIZE = 2.5
SHEET_Z_TRANSITION_SIZE = 1.5
SHEET_Z_FINE_SIZE = 0.6
SHEET_Z_NIP_SIZE = 0.30
ACTIVE_THICKNESS_ELEMS = 5
ROLLER_GLOBAL_SIZE = 12.0
ROLLER_EDGE_SEED_NUMBER = 64

TOL = 1.0e-6


def reset_model():
    if not os.path.isdir(WORKDIR):
        os.makedirs(WORKDIR)
    os.chdir(WORKDIR)

    if JOB_NAME in mdb.jobs:
        del mdb.jobs[JOB_NAME]

    existing_models = list(mdb.models.keys())
    holder_name = "__tmp_delete_holder__"
    if MODEL_NAME in existing_models and len(existing_models) == 1:
        mdb.Model(name=holder_name, modelType=STANDARD_EXPLICIT)
        existing_models = list(mdb.models.keys())

    for model_name in existing_models:
        if model_name == MODEL_NAME:
            del mdb.models[model_name]
        elif model_name.startswith(MODEL_NAME + "_"):
            del mdb.models[model_name]

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


def _sheet_z_seed_size(midz):
    if midz <= 1.0:
        return SHEET_Z_NIP_SIZE
    if midz <= 5.0:
        return SHEET_Z_FINE_SIZE
    if midz <= 12.5:
        return SHEET_Z_TRANSITION_SIZE
    return SHEET_Z_COARSE_SIZE


def make_sheet_part(model):
    sketch = model.ConstrainedSketch(name="ActiveLayer_sketch", sheetSize=160.0)
    sketch.rectangle(point1=(0.0, 0.0),
                     point2=(SHEET_HALF_WIDTH_X, ACTIVE_THICKNESS))
    part = model.Part(name="ActiveLayer",
                      dimensionality=THREE_D,
                      type=DEFORMABLE_BODY)
    part.BaseSolidExtrude(sketch=sketch, depth=SHEET_HALF_LENGTH_Z)
    del model.sketches[sketch.name]

    for z_pos in SHEET_Z_PARTITIONS:
        datum = part.DatumPlaneByPrincipalPlane(principalPlane=XYPLANE,
                                                offset=z_pos)
        part.PartitionCellByDatumPlane(datumPlane=part.datums[datum.id],
                                       cells=part.cells)

    part.SectionAssignment(region=regionToolset.Region(cells=part.cells),
                           sectionName="Sec_ActiveLayer")

    top_faces = part.faces.getByBoundingBox(
        xMin=-TOL, xMax=SHEET_HALF_WIDTH_X + TOL,
        yMin=ACTIVE_THICKNESS - TOL, yMax=ACTIVE_THICKNESS + TOL,
        zMin=-TOL, zMax=SHEET_HALF_LENGTH_Z + TOL)
    bottom_faces = part.faces.getByBoundingBox(
        xMin=-TOL, xMax=SHEET_HALF_WIDTH_X + TOL,
        yMin=-TOL, yMax=TOL,
        zMin=-TOL, zMax=SHEET_HALF_LENGTH_Z + TOL)
    x_sym_faces = part.faces.getByBoundingBox(
        xMin=-TOL, xMax=TOL,
        yMin=-TOL, yMax=ACTIVE_THICKNESS + TOL,
        zMin=-TOL, zMax=SHEET_HALF_LENGTH_Z + TOL)
    z_sym_faces = part.faces.getByBoundingBox(
        xMin=-TOL, xMax=SHEET_HALF_WIDTH_X + TOL,
        yMin=-TOL, yMax=ACTIVE_THICKNESS + TOL,
        zMin=-TOL, zMax=TOL)

    part.Surface(name="Top", side2Faces=top_faces)
    part.Surface(name="Bottom", side2Faces=bottom_faces)
    part.Set(name="XSymFace", faces=x_sym_faces)
    part.Set(name="ZSymFace", faces=z_sym_faces)

    elem = mesh.ElemType(elemCode=C3D8R, elemLibrary=EXPLICIT)
    part.setElementType(regions=(part.cells,), elemTypes=(elem,))
    part.setMeshControls(regions=part.cells, elemShape=HEX, technique=STRUCTURED)

    x_edges = []
    z_edges = {}
    thickness_edges = []
    for edge in part.edges:
        axis, mid = _edge_axis_and_mid(part, edge)
        if axis == "x":
            x_edges.append(edge)
        elif axis == "z":
            size = _sheet_z_seed_size(mid[2])
            z_edges.setdefault(size, []).append(edge)
        else:
            thickness_edges.append(edge)

    part.seedEdgeBySize(edges=x_edges, size=SHEET_X_SIZE, constraint=FINER)
    for size, edges in z_edges.items():
        part.seedEdgeBySize(edges=edges, size=size, constraint=FINER)
    part.seedEdgeByNumber(edges=thickness_edges,
                          number=ACTIVE_THICKNESS_ELEMS,
                          constraint=FINER)
    part.generateMesh()
    return part


def make_roller_part(model):
    sketch = model.ConstrainedSketch(name="Roller_sketch", sheetSize=160.0)
    sketch.CircleByCenterPerimeter(center=(0.0, 0.0),
                                   point1=(ROLLER_RADIUS, 0.0))
    part = model.Part(name="RigidRoller_CylindricalShell",
                      dimensionality=THREE_D,
                      type=DISCRETE_RIGID_SURFACE)
    part.BaseShellExtrude(sketch=sketch, depth=ROLLER_MODELED_WIDTH_X)
    del model.sketches[sketch.name]

    part.Surface(name="ContactFace", side1Faces=part.faces, side2Faces=part.faces)
    elem = mesh.ElemType(elemCode=R3D4, elemLibrary=EXPLICIT)
    part.setElementType(regions=(part.faces,), elemTypes=(elem,))
    part.seedPart(size=ROLLER_GLOBAL_SIZE,
                  deviationFactor=0.1,
                  minSizeFactor=0.1)
    part.seedEdgeByNumber(edges=part.edges,
                          number=ROLLER_EDGE_SEED_NUMBER,
                          constraint=FINER)
    part.generateMesh()
    return part


def make_assembly(model, sheet_part, roller_part):
    root = model.rootAssembly
    root.DatumCsysByDefault(CARTESIAN)

    sheet = root.Instance(name="ActiveLayer-1", part=sheet_part, dependent=ON)
    lower = root.Instance(name="LowerRoller-1", part=roller_part, dependent=ON)
    upper = root.Instance(name="UpperRoller-1", part=roller_part, dependent=ON)

    # The roller part is extruded along local Z; rotate its axis onto assembly X.
    root.rotate(instanceList=(lower.name, upper.name),
                axisPoint=(0.0, 0.0, 0.0),
                axisDirection=(0.0, 1.0, 0.0),
                angle=90.0)

    lower_center_y = -ROLLER_RADIUS
    upper_center_y = ACTIVE_THICKNESS + ROLLER_RADIUS
    root.translate(instanceList=(lower.name,),
                   vector=(0.0, lower_center_y, 0.0))
    root.translate(instanceList=(upper.name,),
                   vector=(0.0, upper_center_y, 0.0))

    upper_rp_obj = root.ReferencePoint(point=(0.0, upper_center_y, 0.0))
    lower_rp_obj = root.ReferencePoint(point=(0.0, lower_center_y, 0.0))
    upper_rp = root.referencePoints[upper_rp_obj.id]
    lower_rp = root.referencePoints[lower_rp_obj.id]

    root.Set(name="RP_UpperRoller", referencePoints=(upper_rp,))
    root.Set(name="RP_LowerRoller", referencePoints=(lower_rp,))
    root.Set(name="Sheet_All", cells=sheet.cells)

    sheet_top = sheet.faces.getByBoundingBox(
        xMin=-TOL, xMax=SHEET_HALF_WIDTH_X + TOL,
        yMin=ACTIVE_THICKNESS - TOL, yMax=ACTIVE_THICKNESS + TOL,
        zMin=-TOL, zMax=SHEET_HALF_LENGTH_Z + TOL)
    sheet_bottom = sheet.faces.getByBoundingBox(
        xMin=-TOL, xMax=SHEET_HALF_WIDTH_X + TOL,
        yMin=-TOL, yMax=TOL,
        zMin=-TOL, zMax=SHEET_HALF_LENGTH_Z + TOL)
    x_sym = sheet.faces.getByBoundingBox(
        xMin=-TOL, xMax=TOL,
        yMin=-TOL, yMax=ACTIVE_THICKNESS + TOL,
        zMin=-TOL, zMax=SHEET_HALF_LENGTH_Z + TOL)
    z_sym = sheet.faces.getByBoundingBox(
        xMin=-TOL, xMax=SHEET_HALF_WIDTH_X + TOL,
        yMin=-TOL, yMax=ACTIVE_THICKNESS + TOL,
        zMin=-TOL, zMax=TOL)

    root.Surface(name="Sheet_Top", side2Faces=sheet_top)
    root.Surface(name="Sheet_Bottom", side2Faces=sheet_bottom)
    root.Surface(name="UpperRoller_Contact", side1Faces=upper.faces, side2Faces=upper.faces)
    root.Surface(name="LowerRoller_Contact", side1Faces=lower.faces, side2Faces=lower.faces)
    root.Set(name="Sheet_XSymFace", faces=x_sym)
    root.Set(name="Sheet_ZSymFace", faces=z_sym)

    return root, sheet, upper, lower


def make_interactions(model, root, upper, lower):
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
                            table=((CONTACT_FRICTION_COEFF,),),
                            maximumElasticSlip=FRACTION,
                            fraction=0.005)

    model.RigidBody(name="Rigid_UpperRoller",
                    refPointRegion=root.sets["RP_UpperRoller"],
                    bodyRegion=regionToolset.Region(faces=upper.faces))
    model.RigidBody(name="Rigid_LowerRoller",
                    refPointRegion=root.sets["RP_LowerRoller"],
                    bodyRegion=regionToolset.Region(faces=lower.faces))

    general_contact = model.ContactExp(name="GeneralContact_RollerSheet",
                                       createStepName="Initial")
    general_contact.includedPairs.setValuesInStep(stepName="Initial",
                                                  useAllstar=ON)
    general_contact.contactPropertyAssignments.appendInStep(
        stepName="Initial",
        assignments=((GLOBAL, SELF, prop.name),))


def make_steps_bcs_outputs(model, root):
    model.ExplicitDynamicsStep(name="Press_Down",
                               previous="Initial",
                               timePeriod=PRESS_TIME,
                               nlgeom=ON,
                               improvedDtMethod=ON)

    model.SmoothStepAmplitude(name="Amp_Upper_Drop",
                              timeSpan=STEP,
                              data=((0.0, 0.0), (PRESS_TIME, 1.0)))

    upper_rp = root.sets["RP_UpperRoller"]
    lower_rp = root.sets["RP_LowerRoller"]

    model.DisplacementBC(name="BC_Lower_RP_Fixed",
                         createStepName="Initial",
                         region=lower_rp,
                         u1=0.0, u2=0.0, u3=0.0,
                         ur1=0.0, ur2=0.0, ur3=0.0)
    model.DisplacementBC(name="BC_Upper_RP_Guide",
                         createStepName="Initial",
                         region=upper_rp,
                         u1=0.0, u2=UNSET, u3=0.0,
                         ur1=0.0, ur2=0.0, ur3=0.0)
    model.DisplacementBC(name="BC_Upper_RP_Drop",
                         createStepName="Press_Down",
                         region=upper_rp,
                         u1=UNSET, u2=-UPPER_ROLLER_DROP, u3=UNSET,
                         ur1=UNSET, ur2=UNSET, ur3=UNSET,
                         amplitude="Amp_Upper_Drop")

    model.XsymmBC(name="BC_Sheet_XSym",
                  createStepName="Initial",
                  region=root.sets["Sheet_XSymFace"])
    model.ZsymmBC(name="BC_Sheet_ZSym",
                  createStepName="Initial",
                  region=root.sets["Sheet_ZSymFace"])

    if "F-Output-1" in model.fieldOutputRequests:
        model.fieldOutputRequests["F-Output-1"].suppress()
    model.FieldOutputRequest(name="F_Sheet_StressStrainContact",
                             createStepName="Press_Down",
                             region=root.sets["Sheet_All"],
                             variables=("S", "LE", "PE", "PEEQ", "U", "RF",
                                        "V", "A", "CSTRESS"),
                             numIntervals=40)
    model.HistoryOutputRequest(name="H_UpperRoller_RP",
                               createStepName="Press_Down",
                               variables=("RF1", "RF2", "RF3",
                                          "RM1", "RM2", "RM3",
                                          "U1", "U2", "U3", "UR1", "V2"),
                               region=upper_rp,
                               numIntervals=100)
    model.HistoryOutputRequest(name="H_LowerRoller_RP",
                               createStepName="Press_Down",
                               variables=("RF1", "RF2", "RF3",
                                          "RM1", "RM2", "RM3",
                                          "U1", "U2", "U3", "UR1", "V2"),
                               region=lower_rp,
                               numIntervals=100)
    model.HistoryOutputRequest(name="H_Energy",
                               createStepName="Press_Down",
                               variables=("ALLIE", "ALLKE", "ALLWK", "ALLSE",
                                          "ALLPD", "ALLAE"),
                               numIntervals=100)


def make_job(model):
    if JOB_NAME in mdb.jobs:
        del mdb.jobs[JOB_NAME]
    return mdb.Job(name=JOB_NAME,
                   model=model.name,
                   description="Local static symmetric normal press model with rigid rollers",
                   type=ANALYSIS,
                   nodalOutputPrecision=SINGLE,
                   numCpus=NUM_CPUS,
                   numDomains=NUM_CPUS)


def main():
    model = reset_model()
    make_materials(model)
    sheet_part = make_sheet_part(model)
    roller_part = make_roller_part(model)
    root, sheet, upper, lower = make_assembly(model, sheet_part, roller_part)
    make_interactions(model, root, upper, lower)
    make_steps_bcs_outputs(model, root)
    job = make_job(model)

    cae_path = os.path.join(WORKDIR, MODEL_NAME + ".cae")
    inp_path = os.path.join(WORKDIR, JOB_NAME + ".inp")
    mdb.saveAs(pathName=cae_path)
    job.writeInput(consistencyChecking=ON)

    print("Created model: %s" % MODEL_NAME)
    print("Created job:   %s" % JOB_NAME)
    print("CAE path:      %s" % cae_path)
    print("INP path:      %s" % inp_path)
    print("Coordinates: X=roller axis, Y=press direction, Z=rolling direction")
    print("Sheet model size: X %.6f mm, Z %.6f mm, thickness %.6f mm" %
          (SHEET_HALF_WIDTH_X, SHEET_HALF_LENGTH_Z, ACTIVE_THICKNESS))
    print("Roller radius: %.6f mm; modeled roller width: %.6f mm" %
          (ROLLER_RADIUS, ROLLER_MODELED_WIDTH_X))
    print("Upper roller Y drop: %.6f mm" % UPPER_ROLLER_DROP)
    print("Friction coefficient: %.6f" % CONTACT_FRICTION_COEFF)


if __name__ == "__main__":
    main()
