import pinocchio as pin
import numpy as np
from simulation_args import SimulationArgs
import simple
from robot_descriptions.loaders.pinocchio import load_robot_description as plr
from sim_utils import (
    addMaterialAndCompliance,
    addFloor,
    addSystemCollisionPairs,
    SimulationArgs,
    createVisualizer,
    printSimulationPerfStats,
    setupSimulatorFromArgs,
    plotContactSolver,
    subSample,
    runMujocoXML,
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
model_path = "cassie_mj_description"

if not args.mujoco:
    # Create model
    print("Loading mj robot description...")
    robot_mj = plr("cassie_mj_description")
    model = robot_mj.model
    geom_model = robot_mj.collision_model
    visual_model = robot_mj.visual_model
    constraint_models_dict = robot_mj.constraint_models

    for key, value in constraint_models_dict.items():
        print("")
        print(f"---Constraints of type {key}---")
        for constraint in value:
            print(f"{constraint.joint1_id=}")
            print(f"{constraint.joint1_placement.translation=}")
            print(f"{constraint.joint2_id=}")
            print(f"{constraint.joint2_placement.translation=}")
            print("")

    addMaterialAndCompliance(geom_model, args.material, args.compliance)

    print(f"{model.damping=}")

    # Initial state
    # q0 = model.referenceConfigurations["qpos0"]
    q0 = model.referenceConfigurations["home"]

    if args.random_init_vel:
        v0 = np.random.randn(model.nv)
    else:
        v0 = np.zeros(model.nv)
    # for i in range(model.nq):
    #     model.lowerPositionLimit[i] = np.finfo("d").min
    #     model.upperPositionLimit[i] = np.finfo("d").max
    # model.lowerDryFrictionLimit[:] = -0
    # model.upperDryFrictionLimit[:] = 0
    model.lowerDryFrictionLimit[:] = -1.0
    model.upperDryFrictionLimit[:] = 1.0
    model.lowerDryFrictionLimit[:6] = -0
    model.upperDryFrictionLimit[:6] = 0
    # for i in range(model.nq):
    #     model.lowerPositionLimit[i] = -1.0
    #     model.upperPositionLimit[i] = 1.0
    # print(f"{model.lowerPositionLimit=}")
    # print(f"{model.upperPositionLimit=}")

    # add floor and collision pairs
    addFloor(geom_model, visual_model)
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

    sim = simple.Simulator(model, geom_model, **constraint_models_dict)
    # sim = simple.Simulator(model, geom_model)
    setupSimulatorFromArgs(sim, args)
    dt = args.dt
    T = args.horizon

    input("[Press ENTER to simulate]")
    q = q0.copy()
    qs = [q0.copy()]
    v = v0.copy()
    zero_torque = np.zeros(model.nv)
    step_timings = np.zeros(T)
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
        if args.display or args.debug:
            vizer.display(q)

    printSimulationPerfStats(step_timings)

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
