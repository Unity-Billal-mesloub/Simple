import pinocchio as pin
import simple
import numpy as np
from hppfcl import (
    Halfspace,
    Sphere,
    HeightFieldAABB,
    CollisionRequest,
    CollisionResult,
    collide,
)

try:
    import example_robot_data as erd

    has_erd = True
except:
    print("Cannot find example robot data. Skipping simulator tests.")
    has_erd = False


def setBallsAndHalfSpace(length=[0.2], mass=[1.0]):
    assert len(length) == len(mass) or len(length) == 1 or len(mass) == 1
    N = max(len(length), len(mass))
    if len(length) == 1:
        length = length * N
    if len(mass) == 1:
        mass = mass * N
    model = pin.Model()
    geom_model = pin.GeometryModel()

    # create plane for floor
    n = np.array([0.0, 0.0, 1])
    plane_shape = Halfspace(n, 0)
    T = pin.SE3(np.eye(3), np.zeros(3))
    plane = pin.GeometryObject("plane", 0, 0, T, plane_shape)
    plane.meshColor = np.array([0.5, 0.5, 0.5, 1.0])
    plane_id = geom_model.addGeometryObject(plane)
    ball_ids = []
    for n_ball in range(N):
        a = length[n_ball]
        m = mass[n_ball]
        freeflyer = pin.JointModelFreeFlyer()
        joint = model.addJoint(0, freeflyer, pin.SE3.Identity(), "ball_" + str(n_ball))
        model.appendBodyToJoint(
            joint, pin.Inertia.FromSphere(m, a / 2), pin.SE3.Identity()
        )
        ball_shape = Sphere(a / 2)
        geom_ball = pin.GeometryObject(
            "ball_" + str(n_ball), joint, joint, pin.SE3.Identity(), ball_shape
        )
        geom_ball.meshColor = np.array([1.0, 0.2, 0.2, 1.0])
        ball_id = geom_model.addGeometryObject(geom_ball)
        for id in ball_ids:
            col_pair = pin.CollisionPair(id, ball_id)
            geom_model.addCollisionPair(col_pair)
        ball_ids += [ball_id]
        col_pair = pin.CollisionPair(plane_id, ball_id)
        geom_model.addCollisionPair(col_pair)

    model.qref = pin.neutral(model)
    model.qinit = model.qref.copy()
    for n_ball in range(N):
        model.qinit[7 * n_ball] += a
        model.qinit[7 * n_ball + 2] += 0.1

    data = model.createData()
    geom_data = geom_model.createData()
    for req in geom_data.collisionRequests:
        req.security_margin = 1e-3
    return model, data, geom_model, geom_data


def test_init():
    model = pin.Model()
    geom_model = pin.GeometryModel()
    data = model.createData()
    geom_data = geom_model.createData()
    sim = simple.Simulator(model, data, geom_model, geom_data)
    assert True


def test_init2():
    model = pin.Model()
    geom_model = pin.GeometryModel()
    sim = simple.Simulator(model, geom_model)
    assert True


def test_simulator_handles():
    model = pin.Model()
    geom_model = pin.GeometryModel()
    data = model.createData()
    geom_data = geom_model.createData()

    sim = simple.Simulator(model, data, geom_model, geom_data)

    model2 = pin.buildSampleModelHumanoid()
    model2.lowerPositionLimit = -np.ones((model2.nq, 1))
    model2.upperPositionLimit = np.ones((model2.nq, 1))
    geom_model2 = pin.buildSampleGeometryModelHumanoid(model2)
    data2 = model2.createData()
    geom_data2 = geom_model2.createData()
    sim = simple.Simulator(model2, data2, geom_model2, geom_data2)
    q = pin.randomConfiguration(model2)
    pin.updateGeometryPlacements(sim.model, sim.data, sim.geom_model, sim.geom_data, q)

    assert sim.model == model2
    assert not (sim.model == model)
    assert sim.geom_model == geom_model2
    assert not (sim.geom_model == geom_model)
    assert sim.geom_data == geom_data2
    assert not (sim.geom_data == geom_data)

    model3 = model2.copy()
    freeflyer = pin.JointModelFreeFlyer()
    joint = model2.addJoint(0, freeflyer, pin.SE3.Identity(), "ball_0")
    assert sim.model.njoints == model3.njoints + 1
    gemo_model3 = geom_model2.copy()
    ball_shape = Sphere(0.1 / 2)
    geom_ball = pin.GeometryObject(
        "ball_0", joint, joint, pin.SE3.Identity(), ball_shape
    )
    ball_id = geom_model2.addGeometryObject(geom_ball)
    assert sim.geom_model.ngeoms == gemo_model3.ngeoms + 1


def test_init_empty():
    model = pin.Model()
    geom_model = pin.GeometryModel()
    data = model.createData()
    geom_data = geom_model.createData()
    sim = simple.Simulator(model, data, geom_model, geom_data)
    q = pin.neutral(model)
    v = np.zeros(model.nv)
    tau = np.zeros(model.nv)
    dt = 1e-3
    sim.step(q, v, tau, dt)
    assert True


def test_step():
    model = pin.buildSampleModelManipulator()
    geom_model = pin.buildSampleGeometryModelManipulator(model)
    data = model.createData()
    geom_data = geom_model.createData()

    sim = simple.Simulator(model, data, geom_model, geom_data)

    q = pin.randomConfiguration(model)
    v = np.random.random(model.nv)
    tau = np.random.random(model.nv)
    dt = 1e-3
    sim.step(q, v, tau, dt)

    fext = [pin.Force(np.random.random(6)) for i in range(model.njoints)]
    sim.step(q, v, tau, fext, dt)
    assert True


def test_step_balls():
    mass = [12.0]
    length = [1.5]
    model, data, geom_model, geom_data = setBallsAndHalfSpace(length, mass)

    q = pin.neutral(model)
    q[2] = length[0] / 2.0 + 1.0
    v = np.zeros(model.nv)
    tau = np.zeros(model.nv)
    dt = 1e-3
    sim = simple.Simulator(model, data, geom_model, geom_data)
    sim.step(q, v, tau, dt)
    assert sim.constraints_problem.getNumberOfContacts() == 0

    q[2] = 0.0
    v = np.zeros(model.nv)
    sim.step(q, v, tau, dt)
    assert sim.constraints_problem.getNumberOfContacts() == 1
    assert np.linalg.norm(sim.vnew) < 1e-6
    mass = [12.0, 0.1]
    length = [1.5, 0.5]
    model, data, geom_model, geom_data = setBallsAndHalfSpace(length, mass)
    sim = simple.Simulator(model, data, geom_model, geom_data)

    q = pin.neutral(model)
    q[0] = 0.0
    q[7] = length[1] / 2.0 + 1.0
    q[2] = 0.0
    q[9] = 0.0
    v = np.zeros(model.nv)
    tau = np.zeros(model.nv)
    sim.step(q, v, tau, dt)
    assert np.linalg.norm(sim.vnew) < 1e-6
    num_contacts = sim.constraints_problem.getNumberOfContacts()
    assert num_contacts == 2
    lam = sim.constraints_problem.frictional_point_constraints_forces()[
        : 3 * num_contacts
    ]
    assert len(lam) == 3 * num_contacts
    sim.step(q, v, tau, dt)
    assert True


def test_step_balls_with_constraints():
    nballs = 2
    mass = [23.0] * nballs
    length = [2.0] * nballs
    model, data, geom_model, geom_data = setBallsAndHalfSpace(length, mass)
    model.lowerPositionLimit = np.ones(model.nq) * -100.0
    model.upperPositionLimit = np.ones(model.nq) * 100.0

    q0 = pin.neutral(model)
    for i in range(nballs):
        q0[7 * i] = i * length[i] * 1.5
        q0[7 * i + 2] = 10.0
    q = q0.copy()
    v = np.zeros(model.nv)
    tau = np.zeros(model.nv)
    dt = 1e-3
    sim = simple.Simulator(model, data, geom_model, geom_data)
    sim.step(q, v, tau, dt)
    vnew = v + dt * pin.aba(model, data, q, v, tau)
    assert sim.constraints_problem.constraints_problem_size() == nballs * 3 * 2
    assert np.linalg.norm(sim.vnew - vnew) < 1e-6

    model, data, geom_model, geom_data = setBallsAndHalfSpace(length, mass)
    model.lowerPositionLimit = np.ones(model.nq) * -100.0
    model.upperPositionLimit = np.ones(model.nq) * 100.0
    for i in range(nballs // 2):
        model.lowerPositionLimit[7 * i + 2] = 10.0
    q = q0.copy()
    v = np.zeros(model.nv)
    tau = np.zeros(model.nv)
    dt = 1e-3
    sim = simple.Simulator(model, data, geom_model, geom_data)
    sim.step(q, v, tau, dt)
    vnew = v + dt * pin.aba(model, data, q, v, tau)
    assert sim.constraints_problem.constraints_problem_size() == nballs * 3 * 2
    assert (
        np.linalg.norm(sim.vnew[(nballs // 2) * 6 :] - vnew[(nballs // 2) * 6 :]) < 1e-6
    )
    assert np.linalg.norm(sim.vnew[: (nballs // 2) * 6]) < 1e-6


def addSystemCollisionPairs(model, geom_model, qref):
    """
    Add the right collision pairs of a model, given qref.
    qref is here as a `T-pose`. The function uses this pose to determine which objects are in collision
    in this ref pose. If objects are in collision, they are not added as collision pairs, as they are considered
    to always be in collision.
    """
    data = model.createData()
    geom_data = geom_model.createData()
    pin.updateGeometryPlacements(model, data, geom_model, geom_data, qref)
    geom_model.removeAllCollisionPairs()
    num_col_pairs = 0
    for i in range(len(geom_model.geometryObjects)):
        for j in range(i + 1, len(geom_model.geometryObjects)):
            # Don't add collision pair if same object
            if i != j:
                gobj_i: pin.GeometryObject = geom_model.geometryObjects[i]
                gobj_j: pin.GeometryObject = geom_model.geometryObjects[j]
                if gobj_i.name == "floor" or gobj_j.name == "floor":
                    num_col_pairs += 1
                    col_pair = pin.CollisionPair(i, j)
                    geom_model.addCollisionPair(col_pair)
                else:
                    if gobj_i.parentJoint != gobj_j.parentJoint:
                        # Compute collision between the geometries. Only add the collision pair if there is no collision.
                        M1 = geom_data.oMg[i]
                        M2 = geom_data.oMg[j]
                        colreq = CollisionRequest()
                        colreq.security_margin = 1e-2  # 1cm of clearance
                        colres = CollisionResult()
                        collide(
                            gobj_i.geometry, M1, gobj_j.geometry, M2, colreq, colres
                        )
                        if not colres.isCollision():
                            num_col_pairs += 1
                            col_pair = pin.CollisionPair(i, j)
                            geom_model.addCollisionPair(col_pair)


def addFloor(geom_model: pin.GeometryModel):
    floor_collision_shape = Halfspace(0, 0, 1, 0)
    M = pin.SE3.Identity()
    floor_collision_object = pin.GeometryObject("floor", 0, 0, M, floor_collision_shape)
    geom_model.addGeometryObject(floor_collision_object)


def test_manipulator_limits():
    model = pin.buildSampleModelManipulator()
    geom_model = pin.buildSampleGeometryModelManipulator(model)
    q0 = pin.neutral(model)
    model.lowerPositionLimit = q0.copy()
    model.upperPositionLimit = q0.copy()
    sim = simple.Simulator(model, geom_model)
    v = np.zeros(model.nv)
    tau = np.zeros(model.nv)
    dt = 1e-3
    sim.step(q0, v, tau, dt)
    assert sim.constraints_problem.getNumberOfContacts() == 0
    assert sim.constraints_problem.constraints_problem_size() == 2 * model.nq
    assert np.linalg.norm(sim.vnew) < 1e-6


def test_humanoid_limits():
    model = pin.buildSampleModelHumanoid()
    geom_model = pin.buildSampleGeometryModelHumanoid(model)
    q0 = pin.neutral(model)
    model.lowerPositionLimit = q0.copy()
    model.upperPositionLimit = q0.copy()
    sim = simple.Simulator(model, geom_model)
    v = np.zeros(model.nv)
    tau = np.zeros(model.nv)
    dt = 1e-3
    sim.step(q0, v, tau, dt)
    assert sim.constraints_problem.getNumberOfContacts() == 0
    assert sim.constraints_problem.constraints_problem_size() == 2 * (model.nq - 4)
    assert sim.admm_constraint_solver.getAbsoluteConvergenceResidual() < 1e-6


def test_freeflyer_limits():
    model = pin.Model()
    geom_model = pin.GeometryModel()
    a = 0.1
    m = 3.8
    freeflyer = pin.JointModelFreeFlyer()
    joint = model.addJoint(0, freeflyer, pin.SE3.Identity(), "ball")
    model.appendBodyToJoint(joint, pin.Inertia.FromSphere(m, a / 2), pin.SE3.Identity())
    ball_shape = Sphere(a / 2)
    geom_ball = pin.GeometryObject("ball", joint, joint, pin.SE3.Identity(), ball_shape)
    geom_ball.meshColor = np.array([1.0, 0.2, 0.2, 1.0])
    ball_id = geom_model.addGeometryObject(geom_ball)
    q0 = pin.neutral(model)
    model.lowerPositionLimit = q0.copy()
    model.upperPositionLimit = q0.copy()
    sim = simple.Simulator(model, geom_model)
    v = np.zeros(model.nv)
    tau = np.zeros(model.nv)
    dt = 1e-3
    sim.step(q0, v, tau, dt)
    assert sim.constraints_problem.getNumberOfContacts() == 0
    assert sim.constraints_problem.constraints_problem_size() == 2 * (model.nq - 4)
    assert np.linalg.norm(sim.vnew) < 1e-6


def test_composite_limits():
    model = pin.Model()
    geom_model = pin.GeometryModel()
    a = 0.1
    m = 3.8
    composite = pin.JointModelComposite(2)
    composite.addJoint(pin.JointModelRX())
    composite.addJoint(pin.JointModelRY())
    joint = model.addJoint(0, composite, pin.SE3.Identity(), "ball")
    model.appendBodyToJoint(joint, pin.Inertia.FromSphere(m, a / 2), pin.SE3.Identity())
    ball_shape = Sphere(a / 2)
    geom_ball = pin.GeometryObject("ball", joint, joint, pin.SE3.Identity(), ball_shape)
    geom_ball.meshColor = np.array([1.0, 0.2, 0.2, 1.0])
    ball_id = geom_model.addGeometryObject(geom_ball)
    q0 = pin.neutral(model)
    model.lowerPositionLimit = q0.copy()
    model.upperPositionLimit = q0.copy()
    sim = simple.Simulator(model, geom_model)
    v = np.zeros(model.nv)
    tau = np.zeros(model.nv)
    dt = 1e-3
    sim.step(q0, v, tau, dt)
    assert sim.constraints_problem.getNumberOfContacts() == 0
    assert sim.constraints_problem.constraints_problem_size() == 2 * model.nq
    assert np.linalg.norm(sim.vnew) < 1e-6
    model = pin.Model()
    geom_model = pin.GeometryModel()
    a = 0.1
    m = 3.8
    composite = pin.JointModelComposite(2)
    composite.addJoint(pin.JointModelPX())
    composite.addJoint(pin.JointModelPY())
    joint = model.addJoint(0, composite, pin.SE3.Identity(), "ball")
    model.appendBodyToJoint(joint, pin.Inertia.FromSphere(m, a / 2), pin.SE3.Identity())
    ball_shape = Sphere(a / 2)
    geom_ball = pin.GeometryObject("ball", joint, joint, pin.SE3.Identity(), ball_shape)
    geom_ball.meshColor = np.array([1.0, 0.2, 0.2, 1.0])
    ball_id = geom_model.addGeometryObject(geom_ball)
    q0 = pin.neutral(model)
    model.lowerPositionLimit = q0.copy()
    model.upperPositionLimit = q0.copy()
    sim = simple.Simulator(model, geom_model)
    v = np.zeros(model.nv)
    tau = np.zeros(model.nv)
    dt = 1e-3
    sim.step(q0, v, tau, dt)
    assert sim.constraints_problem.getNumberOfContacts() == 0
    assert sim.constraints_problem.constraints_problem_size() == 2 * model.nq
    assert np.linalg.norm(sim.vnew) < 1e-6


def test_mujoco_humanoid_limits():
    import os

    path = os.path.abspath(__file__)
    path = path.split("/")
    path = "/".join(path[:-2])
    path = os.path.join(path, "test_data/mujoco_humanoid.xml")
    model = pin.buildModelFromMJCF(path)
    geom_model = pin.buildGeomFromMJCF(model, path, pin.COLLISION)
    addFloor(geom_model)
    q0 = pin.neutral(model)
    addSystemCollisionPairs(model, geom_model, q0)

    sim = simple.Simulator(model, geom_model)
    # First we test without contact but with limits
    q = q0.copy()
    q[2] += 1.0
    v = np.zeros(model.nv)
    tau = np.zeros(model.nv)
    dt = 1e-3
    sim.step(q, v, tau, dt)
    data = model.createData()
    vnew = v + dt * pin.aba(model, data, q, v, tau)
    assert np.linalg.norm(sim.vnew - vnew) < 1e-6
    assert sim.constraints_problem.getNumberOfContacts() == 0
    assert sim.constraints_problem.constraints_problem_size() == (model.nq - 4) * 2
    # Now we test without contact but with tight limits
    model.lowerPositionLimit = q0.copy()
    model.lowerPositionLimit[2] += 1.0
    model.upperPositionLimit = model.lowerPositionLimit.copy()
    sim = simple.Simulator(model, geom_model)
    q = model.lowerPositionLimit.copy()
    v = np.zeros(model.nv)
    tau = np.zeros(model.nv)
    dt = 1e-3
    sim.step(q, v, tau, dt)
    assert sim.constraints_problem.getNumberOfContacts() == 0
    assert sim.constraints_problem.constraints_problem_size() == (model.nq - 4) * 2
    assert sim.admm_constraint_solver.getAbsoluteConvergenceResidual() < 1e-6
    # Now we test with contact but without limits
    model.lowerPositionLimit = -np.ones(model.nq) * np.finfo("d").max
    model.upperPositionLimit = np.ones(model.nq) * np.finfo("d").max
    sim = simple.Simulator(model, geom_model)
    q = q0.copy()
    v = np.zeros(model.nv)
    tau = np.zeros(model.nv)
    dt = 1e-3
    sim.step(q, v, tau, dt)

    assert (
        sim.constraints_problem.constraints_problem_size()
        == sim.constraints_problem.getNumberOfContacts() * 3
    )
    assert sim.admm_constraint_solver.getAbsoluteConvergenceResidual() < 1e-6
    # Now we test with contact and with large limits
    model.lowerPositionLimit = -np.ones(model.nq) * 10000.0
    model.upperPositionLimit = np.ones(model.nq) * 10000.0
    sim = simple.Simulator(model, geom_model)
    q = q0.copy()
    v = np.zeros(model.nv)
    tau = np.zeros(model.nv)
    dt = 1e-3
    sim.step(q, v, tau, dt)
    assert (
        sim.constraints_problem.constraints_problem_size()
        == (model.nq - 4) * 2 + 3 * sim.constraints_problem.getNumberOfContacts()
    )
    # TODO Now we test with contact and limits
    model = pin.buildModelFromMJCF(path)
    sim = simple.Simulator(model, geom_model)
    q = q0.copy()
    v = np.zeros(model.nv)
    tau = np.zeros(model.nv)
    dt = 1e-3
    sim.step(q, v, tau, dt)


if has_erd:

    def setSolo():
        # robot model
        robot = erd.load("solo12")
        model = robot.model.copy()
        model.qref = np.array(
            [
                0.09906518,
                0.20099078,
                0.32502457,
                0.19414175,
                -0.00524735,
                -0.97855773,
                0.06860185,
                0.00968163,
                0.60963582,
                -1.61206407,
                -0.02543309,
                0.66709088,
                -1.50870083,
                0.32405118,
                -1.15305599,
                1.56867351,
                -0.39097222,
                -1.29675892,
                1.39741073,
            ]
        )
        model.qref = pin.normalize(model, model.qref)

        # Create height field ground
        def ground(xy):
            return (
                np.sin(xy[0] * 3) / 5
                + np.cos(xy[1] ** 2 * 3) / 20
                + np.sin(xy[1] * xy[0] * 5) / 10
            )

        xg = np.arange(-2, 2, 0.02)
        nx = xg.shape[0]
        xy_g = np.meshgrid(xg, xg)
        xy_g = np.stack(xy_g)
        elev_g = np.zeros((nx, nx))
        elev_g[:, :] = ground(xy_g)
        sx = xg[-1] - xg[0]
        sy = xg[-1] - xg[0]
        elev_g[:, :] = elev_g[::-1, :]

        # Create geometry model
        geom_model = robot.collision_model
        hfield = HeightFieldAABB(sx, sy, elev_g, np.min(elev_g))
        Mhfield = pin.SE3.Identity()
        ground = pin.GeometryObject("ground", 0, Mhfield, hfield)
        ground_id = geom_model.addGeometryObject(ground)

        # Add collision pairs between the ground and the robot
        geom_model.removeAllCollisionPairs()

        a = 0.01910275
        frames_names = ["HR_FOOT", "HL_FOOT", "FR_FOOT", "FL_FOOT"]

        for name in frames_names:
            frame_id = model.getFrameId(name)
            frame = model.frames[frame_id]
            joint_id = frame.parentJoint
            frame_placement = frame.placement

            shape_name = name + "_shape"
            shape = Sphere(a)
            geometry = pin.GeometryObject(shape_name, joint_id, frame_placement, shape)
            geometry.meshColor = np.array([1.0, 0.2, 0.2, 1.0])

            geom_id = geom_model.addGeometryObject(geometry)

            foot_plane = pin.CollisionPair(
                ground_id, geom_id
            )  # order should be inverted ?
            geom_model.addCollisionPair(foot_plane)

        # Create data
        data = model.createData()
        geom_data = geom_model.createData()
        for req in geom_data.collisionRequests:
            req.security_margin = 1e-3
        return model, data, geom_model, geom_data

    def test_step_solo():
        model, data, geom_model, geom_data = setSolo()
        q = model.qref.copy()
        v = np.zeros(model.nv)
        tau = np.zeros(model.nv)
        dt = 1e-3
        sim = simple.Simulator(model, data, geom_model, geom_data)
        sim.step(q, v, tau, dt)
        assert sim.constraints_problem.getNumberOfContacts() == 4


# test_init()
# test_init2()
# test_simulator_handles()
# test_init_empty()
# test_step()
# test_step_balls()
# test_step_balls_with_constraints()
# test_humanoid_limits()
# test_freeflyer_limits()
# test_composite_limits()
# test_manipulator_limits()
# test_mujoco_humanoid_limits()
# print("All tests passed")
