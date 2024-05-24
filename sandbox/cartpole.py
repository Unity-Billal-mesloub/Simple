import pinocchio as pin
import numpy as np
import hppfcl
import tap
from pinocchio.visualize import MeshcatVisualizer
import meshcat
from simulation_args import SimulationArgs
import simple

GREY = np.array([192, 201, 229, 255]) / 255


class SimulationArgs(tap.Tap):
    horizon: int = 1000
    dt: float = 1e-3
    contact_solver: str = "ADMM"  # set to PGS or ADMM
    maxit: int = 1000  # solver maxit
    tol: float = 1e-6  # absolute constraint solver tol
    tol_rel: float = 1e-6  # relative constraint solver tol
    solve_ncp: int = 1  # set to 0 to solve ccp
    warm_start: int = 1  # warm start the solver?
    mu_prox: float = 1e-4  # prox value for admm
    material: str = "metal"  # contact friction
    compliance: float = 0
    Kp: float = 0  # baumgarte proportional term
    Kd: float = 0  # baumgarte derivative term
    seed: int = 1234
    limits: bool = False  # set to activate limits
    floor: bool = False  # add a floor
    cos_torque: bool = False
    dont_stop: bool = False


args = SimulationArgs().parse_args()
allowed_solvers = ["ADMM", "PGS"]
if args.contact_solver not in allowed_solvers:
    print(
        f"Error: unsupported simulator. Avalaible simulators: {allowed_solvers}. Exiting"
    )
    exit(1)
np.random.seed(args.seed)
pin.seed(args.seed)


def create_cartpole(N, add_floor):
    model = pin.Model()
    geom_model = pin.GeometryModel()

    if add_floor:
        # add floor
        floor_collision_shape = hppfcl.Halfspace(0, 0, 1, 0)
        M = pin.SE3.Identity()
        floor_collision_object = pin.GeometryObject(
            "floor", 0, 0, M, floor_collision_shape
        )
        color = GREY
        color[3] = 0.5
        floor_collision_object.meshColor = color
        geom_floor = geom_model.addGeometryObject(floor_collision_object)

    parent_id = 0

    cart_radius = 0.1
    cart_length = 5 * cart_radius
    cart_mass = 1.0
    joint_name = "joint_cart"

    geometry_placement = pin.SE3.Identity()
    geometry_placement.rotation = pin.Quaternion(
        np.array([0.0, 0.0, 1.0]), np.array([0.0, 1.0, 0.0])
    ).toRotationMatrix()

    joint_id = model.addJoint(
        parent_id, pin.JointModelPY(), pin.SE3.Identity(), joint_name
    )

    body_inertia = pin.Inertia.FromCylinder(cart_mass, cart_radius, cart_length)
    body_placement = geometry_placement
    model.appendBodyToJoint(
        joint_id, body_inertia, body_placement
    )  # We need to rotate the inertia as it is expressed in the LOCAL frame of the geometry

    shape_cart = hppfcl.Cylinder(cart_radius, cart_length)

    geom_cart = pin.GeometryObject(
        "shape_cart", joint_id, geometry_placement, shape_cart
    )
    geom_cart.meshColor = np.array([1.0, 0.1, 0.1, 1.0])
    geom_model.addGeometryObject(geom_cart)

    parent_id = joint_id
    joint_placement = pin.SE3.Identity()
    body_mass = 1.0
    body_radius = 0.1
    for k in range(N):
        joint_name = "joint_" + str(k + 1)
        joint_id = model.addJoint(
            parent_id, pin.JointModelRX(), joint_placement, joint_name
        )

        body_inertia = pin.Inertia.FromSphere(body_mass, body_radius)
        body_placement = joint_placement.copy()
        body_placement.translation[2] = 1.0
        model.appendBodyToJoint(joint_id, body_inertia, body_placement)

        geom1_name = "ball_" + str(k + 1)
        shape1 = hppfcl.Sphere(body_radius)
        geom1_obj = pin.GeometryObject(geom1_name, joint_id, body_placement, shape1)
        geom1_obj.meshColor = np.ones((4))
        geom_ball = geom_model.addGeometryObject(geom1_obj)
        if add_floor:
            geom_model.addCollisionPair(pin.CollisionPair(geom_floor, geom_ball))

        geom2_name = "bar_" + str(k + 1)
        shape2 = hppfcl.Cylinder(body_radius / 4.0, body_placement.translation[2])
        shape2_placement = body_placement.copy()
        shape2_placement.translation[2] /= 2.0

        geom2_obj = pin.GeometryObject(geom2_name, joint_id, shape2_placement, shape2)
        geom2_obj.meshColor = np.array([0.0, 0.0, 0.0, 1.0])
        geom_model.addGeometryObject(geom2_obj)

        # update parent id to add next pendulum
        parent_id = joint_id
        joint_placement = body_placement.copy()

    end_frame = pin.Frame(
        "end_effector_frame",
        model.getJointId("joint_" + str(N)),
        0,
        body_placement,
        pin.FrameType(3),
    )
    model.addFrame(end_frame)
    geom_model.collision_pairs = []
    model.qinit = np.zeros(model.nq)
    model.qinit[1] = 0.0 * np.pi
    model.qref = pin.neutral(model)
    return model, geom_model


# ============================================================================
# SCENE CREATION
# ============================================================================
# Create model
model, geom_model = create_cartpole(1, args.floor)

for gobj in geom_model.geometryObjects:
    if args.material == "ice":
        gobj.physicsMaterial.materialType = pin.PhysicsMaterialType.ICE
    elif args.material == "plastic":
        gobj.physicsMaterial.materialType = pin.PhysicsMaterialType.PLASTIC
    elif args.material == "wood":
        gobj.physicsMaterial.materialType = pin.PhysicsMaterialType.WOOD
    elif args.material == "metal":
        gobj.physicsMaterial.materialType = pin.PhysicsMaterialType.METAL
    elif args.material == "concrete":
        gobj.physicsMaterial.materialType = pin.PhysicsMaterialType.CONCRETE

    # Compliance
    gobj.physicsMaterial.compliance = args.compliance


# Initial state
if args.limits:
    for i in range(model.nq):
        model.lowerPositionLimit[i] = -0.8
        model.upperPositionLimit[i] = 0.8
else:
    for i in range(model.nq):
        model.lowerPositionLimit[i] = np.finfo("d").min
        model.upperPositionLimit[i] = np.finfo("d").max
q0 = pin.neutral(model)
v0 = np.zeros(model.nv)
tau0 = np.ones(model.nv)
print("q0 = ", q0)
print("v0 = ", v0)
print(f"{model.lowerPositionLimit}")
print(f"{model.upperPositionLimit}")

# visualize the trajectory
viewer = meshcat.Visualizer(zmq_url="tcp://127.0.0.1:6000")
viewer.delete()
vizer: MeshcatVisualizer = MeshcatVisualizer(model, geom_model, geom_model)
vizer.initViewer(viewer=viewer, open=False, loadModel=True)
vizer.display(q0)

# ============================================================================
# SIMULATION
# ============================================================================
sim = simple.Simulator(model, geom_model)
# PGS
sim.pgs_constraint_solver_settings.absolute_precision = args.tol
sim.pgs_constraint_solver_settings.relative_precision = args.tol_rel
sim.pgs_constraint_solver_settings.max_iter = args.maxit
# ADMM
sim.admm_constraint_solver_settings.absolute_precision = args.tol
sim.admm_constraint_solver_settings.relative_precision = args.tol_rel
sim.admm_constraint_solver_settings.max_iter = args.maxit
sim.admm_constraint_solver_settings.mu = args.mu_prox
#
sim.warm_start_contact_forces = args.warm_start
sim.constraints_problem.is_ncp = args.solve_ncp
sim.constraints_problem.Kp = args.Kp
sim.constraints_problem.Kd = args.Kd
dt = args.dt
T = args.horizon

q = q0.copy()
v = v0.copy()
tau = tau0.copy()
zero_torque = np.zeros(model.nv)
sim.step(q, v, tau0, dt)
qprev = q.copy()
vprev = v.copy()
q = sim.qnew.copy()
v = sim.qnew.copy()
vizer.display(q)
input("[Press enter to simulate]")
for t in range(T):
    tau = np.zeros(model.nv)
    if args.cos_torque:
        tau[0] = 10 * np.cos(10 * float(t) / float(T))
    if args.contact_solver == "ADMM":
        sim.step(q, v, tau, dt)
    if args.contact_solver == "PGS":
        sim.stepPGS(q, v, tau, dt)
    if sim.constraints_problem.constraints_problem_size() > 0 and not args.dont_stop:
        print(f"{t=}")
        print(f"{qprev=}")
        print(f"{vprev=}")
        print(f"{q=}")
        print(f"{v=}")
        print(f"{sim.constraints_problem.constraints_problem_size()=}")
        print(f"{sim.constraints_problem.constraints_forces()=}")
        input("[Press enter to continue]")
    qprev = q.copy()
    vprev = v.copy()
    q = sim.qnew.copy()
    v = sim.vnew.copy()
    vizer.display(q)
