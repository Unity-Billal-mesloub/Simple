import pinocchio as pin
import numpy as np
import simple
import time
from sim_utils import (
    addSystemCollisionPairs,
    SimulationArgs,
    createVisualizer,
    setupSimulatorFromArgs,
    plotContactSolver,
    subSample,
    runMujocoXML,
    printSimulationPerfStats,
)

args = SimulationArgs().parse_args()
model_path = "./sandbox/robots/pendulum.xml"

if not args.mujoco:
    model = pin.buildModelFromMJCF(model_path)
    constraint_models = pin.buildConstraintModelsFromMJCF(model, model_path)
    geom_model = pin.buildGeomFromMJCF(model, model_path, pin.COLLISION)
    visual_model = pin.buildGeomFromMJCF(model, model_path, pin.VISUAL)

    q0 = model.referenceConfigurations["qpos0"]
    q0[0] = 0.4
    v0 = np.zeros(model.nv)
    print(f"{q0=}")
    print(f"{v0=}")

    addSystemCollisionPairs(model, geom_model, q0)
    # geom_model.removeAllCollisionPairs()

    for inertia in model.inertias:
        print(inertia)

    for i in range(model.nq):
        model.lowerPositionLimit[i] = -0.2
        model.upperPositionLimit[i] = 0.6
    # for i in range(model.nq):
    #     model.lowerPositionLimit[i] = np.finfo("d").min
    #     model.upperPositionLimit[i] = np.finfo("d").max
    # model.lowerDryFrictionLimit[:] = -10000
    # model.upperDryFrictionLimit[:] = 10000

    # visualize the trajectory
    vizer, _ = createVisualizer(model, geom_model, visual_model)
    vizer.display(q0)

    # sim = simple.Simulator(model, geom_model, constraint_models)
    sim = simple.Simulator(model, geom_model)
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
