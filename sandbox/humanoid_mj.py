import pinocchio as pin
import numpy as np
import simple
from sim_utils import (
    # addFloor,
    addMaterialAndCompliance,
    addSystemCollisionPairs,
    plotContactSolver,
    runMujocoXML,
    setupSimulatorFromArgs,
    subSample,
    createVisualizer,
    SimulationArgs,
    printSimulationPerfStats,
)
import time

args = SimulationArgs().parse_args()
allowed_solvers = ["ADMM", "PGS"]
if args.contact_solver not in allowed_solvers:
    print(
        f"Error: unsupported simulator. Avalaible simulators: {allowed_solvers}. Exiting"
    )
    exit(1)
np.random.seed(args.seed)
pin.seed(args.seed)
model_path = "./sandbox/robots/humanoid.xml"

if not args.mujoco:
    # Create model
    print("Loading mj robot description...")
    model = pin.buildModelFromMJCF(model_path)
    geom_model = pin.buildGeomFromMJCF(model, model_path, pin.COLLISION)
    visual_model = pin.buildGeomFromMJCF(model, model_path, pin.VISUAL)

    addMaterialAndCompliance(geom_model, args.material, args.compliance)

    # Initial state
    q0 = model.referenceConfigurations["qpos0"]

    for joint in model.joints:
        print(joint)

    if args.random_init_vel:
        v0 = np.random.randn(model.nv)
    else:
        v0 = np.zeros(model.nv)
    for i in range(model.nq):
        model.lowerPositionLimit[i] = np.finfo("d").min
        model.upperPositionLimit[i] = np.finfo("d").max
    # for i in range(model.nq):
    #     model.positionLimitMargin[i] = np.finfo("d").max
    model.lowerDryFrictionLimit[:] = 0
    model.upperDryFrictionLimit[:] = 0

    # add floor and collision pairs
    # addFloor(geom_model, visual_model)
    addSystemCollisionPairs(model, geom_model, q0)

    # visualize the trajectory
    vizer, _ = createVisualizer(model, geom_model, visual_model)
    vizer.display(q0)
    print(f"{q0=}")
    print(f"{v0=}")

    # simulation
    _data = model.createData()
    rmass = pin.computeTotalMass(model, _data)
    print(f"Robot mass = {rmass}")

    sim = simple.Simulator(model, geom_model)
    setupSimulatorFromArgs(sim, args)
    dt = args.dt
    T = args.horizon

    N = 1
    input("[Press ENTER to simulate]")
    for i in range(N):
        q = q0.copy()
        qs = [q0.copy()]
        v = v0.copy()
        zero_torque = np.zeros(model.nv)
        step_timings = np.zeros(T)
        sim.reset()
        for t in range(T):
            start_time = time.time()
            if args.contact_solver == "ADMM":
                sim.step(q, v, zero_torque, dt)
            if args.contact_solver == "PGS":
                sim.stepPGS(q, v, zero_torque, dt)
            end_time = time.time()
            step_timings[t] = end_time - start_time

            plotContactSolver(sim, args, t, q, v)

            q = sim.qnew.copy()
            v = sim.vnew.copy()
            qs.append(q)
            if args.display and args.debug:
                vizer.display(q)

    printSimulationPerfStats(step_timings)

    if args.display:
        fps = 60.0
        dt_vis = 1.0 / fps
        qs = subSample(qs, dt * T, fps)
        while True:
            for q in qs:
                step_start = time.time()
                vizer.display(q)
                time_until_next_step = dt_vis - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
else:
    runMujocoXML(model_path, args)
