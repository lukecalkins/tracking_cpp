# Example: incremental 2D target tracking with ISAM2 (range + bearing)
import numpy as np
import gtsa m
from math import pi

# --- configuration / noise models ---
prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1]))   # strong prior on Point2 (x,y)
motion_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.5, 0.5]))  # process noise for BetweenFactor(Point2)
bearing_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.5])) # [bearing(rad), range(m)]

# Sensor pose (known) at origin
sensor_pose = gtsam.Pose2(0.0, 0.0, 0.0)
sensor_key = gtsam.Symbol('S', 0)

# ISAM2 parameters and object
isam_params = gtsam.ISAM2Params()
isam = gtsam.ISAM2(isam_params)

# keys helper
def P(t): return gtsam.Symbol('p', t)   # target Point2 at time t

# ground-truth (for synthetic example)
true_positions = [gtsam.Point2(1.0 + 0.5*t, 2.0 + 0.2*t) for t in range(10)]

# we'll simulate measurements (bearing,range) with noise
rng = np.random.RandomState(1)

# initialize containers
current_estimates = gtsam.Values()

# We'll keep track of last predicted displacement as our "control" odometry (e.g. constant velocity)
last_pos = None

for t, true_pos in enumerate(true_positions):
    # 1) create incremental factor graph and initial estimates for this timestep
    graph = gtsam.NonlinearFactorGraph()
    initial = gtsam.Values()

    # If first time step, add prior on p0 and prior on sensor pose (sensor pose is known firmly)
    if t == 0:
        graph.add(gtsam.PriorFactorPoint2(P(0), true_pos, prior_noise))
        # add sensor pose as a prior (so BearingRangeFactor can use it)
        graph.add(gtsam.PriorFactorPose2(sensor_key, sensor_pose, gtsam.noiseModel.Diagonal.Sigmas(
            np.array([1e-9,1e-9,1e-9]))))  # very tight prior
        initial.insert(P(0), gtsam.Point2(true_pos.x() + 0.2, true_pos.y() - 0.2))  # slightly off init guess
        current_estimates.insert(sensor_key, sensor_pose)
        isam.update(graph, initial)
        # get current estimate and continue
        current = isam.calculateEstimate()
        last_pos = current.atPoint2(P(0))
        continue

    # 2) Add motion (BetweenFactor) between p_{t-1} and p_t using a simple predicted displacement
    # here we simulate a "control" odometry as difference between true positions (in real use this would come from a motion model)
    predicted_disp = np.array([true_positions[t].x() - true_positions[t-1].x(),
                               true_positions[t].y() - true_positions[t-1].y()])
    # BetweenFactor for Point2 expects a Point2 measurement: the relative translation
    graph.add(gtsam.BetweenFactorPoint2(P(t-1), P(t), gtsam.Point2(predicted_disp[0], predicted_disp[1]), motion_noise))

    # 3) Simulate a range+bearing measurement from sensor to target (with noise)
    dx = true_pos.x() - sensor_pose.x()
    dy = true_pos.y() - sensor_pose.y()
    true_bearing = np.arctan2(dy, dx) - sensor_pose.theta()
    true_range = np.hypot(dx, dy)
    meas_bearing = true_bearing + rng.normal(scale=0.05)    # add noise
    meas_range = true_range + rng.normal(scale=0.5)

    # Add BearingRangeFactor: (Pose2 sensor, Point2 target)
    graph.add(gtsam.BearingRangeFactor2D(sensor_key, P(t),
                                         gtsam.Rot2(meas_bearing), float(meas_range),
                                         bearing_noise))

    # 4) Provide an initial guess for the new state (p_t)
    # we can predict p_t = previous_est + predicted_disp (use last estimate from isam)
    current = isam.calculateEstimate()
    prev_est = current.atPoint2(P(t-1))
    init_guess = gtsam.Point2(prev_est.x() + predicted_disp[0] + rng.normal(scale=0.2),
                              prev_est.y() + predicted_disp[1] + rng.normal(scale=0.2))
    initial.insert(P(t), init_guess)

    # 5) Update ISAM2 incrementally
    isam.update(graph, initial)

    # 6) get posterior estimate for current point
    current = isam.calculateEstimate()
    est_pt = current.atPoint2(P(t))
    print(f"t={t} estimate: x={est_pt.x():.3f}, y={est_pt.y():.3f}  true: x={true_pos.x():.3f}, y={true_pos.y():.3f}")

    last_pos = est_pt
