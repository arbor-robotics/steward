from numpy.random import uniform, randn
import numpy as np
import scipy
from matplotlib import pyplot as plt
from filterpy.monte_carlo import systematic_resample
from numpy.linalg import norm
from numpy.random import randn
import scipy.stats
import rclpy
from rclpy.node import Node, ParameterDescriptor, ParameterType
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from tf2_ros import TransformBroadcaster
import utm
import pandas as pd

# ROS message definitions
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from gps_msgs.msg import GPSFix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from tqdm import tqdm, trange

"""
ADAPTED FROM ROGER LABBE: https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/12-Particle-Filters.ipynb
"""


def create_uniform_particles(x_range, y_range, hdg_range, N):
    particles = np.empty((N, 3))
    particles[:, 0] = uniform(x_range[0], x_range[1], size=N)
    particles[:, 1] = uniform(y_range[0], y_range[1], size=N)
    particles[:, 2] = uniform(hdg_range[0], hdg_range[1], size=N)
    particles[:, 2] %= 2 * np.pi
    return particles


def create_gaussian_particles(mean, std, N):
    particles = np.empty((N, 3))
    particles[:, 0] = mean[0] + (randn(N) * std[0])
    particles[:, 1] = mean[1] + (randn(N) * std[1])
    particles[:, 2] = mean[2] + (randn(N) * std[2])
    particles[:, 2] %= 2 * np.pi
    return particles


def updateFromMotionCommand(particles, u, std, dt=1.0):
    """move according to control input u (heading change, velocity)
    with noise Q (std heading change, std velocity)`"""

    N = len(particles)
    # update heading
    particles[:, 2] += u[0] + (randn(N) * std[0])
    particles[:, 2] %= 2 * np.pi

    # move in the (noisy) commanded direction
    dist = (u[1] * dt) + (randn(N) * std[1])
    particles[:, 0] += np.cos(particles[:, 2]) * dist
    particles[:, 1] += np.sin(particles[:, 2]) * dist


def updateFromLandmarkDistances(particles, weights, z, sensor_std_error, landmarks):
    for i, landmark in enumerate(landmarks):
        distance = np.linalg.norm(particles[:, 0:2] - landmark, axis=1)

        # This says, how likely is it that this particle
        # is REALLY this far away, given a distance measurement z?
        weights *= scipy.stats.norm(distance, sensor_std_error).pdf(z[i])

    weights += 1.0e-300  # avoid round-off to zero
    weights /= sum(weights)  # normalize


def updateFromGnssPosition(particles: np.ndarray, weights, z, sensor_std_error):

    weights *= scipy.stats.norm(particles[:, :2]).pdf(z)

    # for i, landmark in enumerate(landmarks):
    # distance = np.linalg.norm(particles[:, 0:2] - landmark, axis=1)

    # This says, how likely is it that this particle
    # is REALLY this far away, given a distance measurement z?
    # weights *= scipy.stats.norm(distance, sensor_std_error).pdf(z[i])

    weights += 1.0e-300  # avoid round-off to zero
    weights /= sum(weights)  # normalize


def trueTrackToEnuRads(track_deg: float):
    enu_yaw = track_deg

    enu_yaw -= 90

    enu_yaw = 360 - enu_yaw

    if enu_yaw < 0:
        enu_yaw += 360
    elif enu_yaw > 360:
        enu_yaw -= 360

    enu_yaw *= np.pi / 180.0
    return enu_yaw


def estimate(particles, weights):
    """returns mean and variance of the weighted particles"""

    pos = particles[:, 0:2]
    mean = np.average(pos, weights=weights, axis=0)
    var = np.average((pos - mean) ** 2, weights=weights, axis=0)
    return mean, var


def neff(weights):
    return 1.0 / np.sum(np.square(weights))


def resample_from_index(particles, weights, indexes):
    particles[:] = particles[indexes]
    weights.resize(len(particles))
    weights.fill(1.0 / len(weights))


def run_pf1(
    N,
    iters=18,
    sensor_std_err=0.1,
    plot_particles=False,
    xlim=(0, 20),
    ylim=(0, 20),
    initial_x=None,
):
    """Run the particle filter for a number of iterations

    Args:
        N (int): The number of particles
        iters (int, optional): The number of iterations. Defaults to 18.
        sensor_std_err (float, optional): Standard deviation of the sensor reading. Defaults to 0.1.
        plot_particles (bool, optional): Defaults to False.
        xlim (tuple, optional): _description_. Defaults to (0, 20).
        ylim (tuple, optional): _description_. Defaults to (0, 20).
        initial_x (_type_, optional): Estimate of initial starting position. Defaults to None.
    """
    landmarks = np.array([[-1, 2], [5, 10], [12, 14], [18, 21]])
    NL = len(landmarks)

    plt.figure()

    # create particles and weights
    if initial_x is not None:
        particles = create_gaussian_particles(
            mean=initial_x, std=(5, 5, np.pi / 4), N=N
        )
    else:
        particles = create_uniform_particles((0, 20), (0, 20), (0, 6.28), N)
    weights = np.ones(N) / N

    if plot_particles:
        alpha = 0.20
        if N > 5000:
            alpha *= np.sqrt(5000) / np.sqrt(N)
        plt.scatter(particles[:, 0], particles[:, 1], alpha=alpha, color="g")

    xs = []
    robot_pos = np.array([0.0, 0.0])
    for _ in range(iters):
        robot_pos += (1, 1)

        # distance from robot to each landmark
        zs = norm(landmarks - robot_pos, axis=1) + (randn(NL) * sensor_std_err)

        # move diagonally forward to (x+1, x+1)
        updateFromMotionCommand(particles, u=(0.00, 1.414), std=(0.2, 0.05))

        # incorporate measurements
        updateFromLandmarkDistances(
            particles,
            weights,
            z=zs,
            sensor_std_error=sensor_std_err,
            landmarks=landmarks,
        )

        # resample if too few effective particles
        if neff(weights) < N / 2:
            indexes = systematic_resample(weights)
            resample_from_index(particles, weights, indexes)
            assert np.allclose(weights, 1 / N)
        mu, var = estimate(particles, weights)
        xs.append(mu)

        if plot_particles:
            plt.scatter(particles[:, 0], particles[:, 1], color="k", marker=",", s=1)
        p1 = plt.scatter(robot_pos[0], robot_pos[1], marker="+", color="k", s=180, lw=3)
        p2 = plt.scatter(mu[0], mu[1], marker="s", color="r")

    xs = np.array(xs)
    # plt.plot(xs[:, 0], xs[:, 1])
    plt.legend([p1, p2], ["Actual", "PF"], loc=4, numpoints=1)
    plt.xlim(*xlim)
    plt.ylim(*ylim)
    print("final position error, variance:\n\t", mu - np.array([iters, iters]), var)
    plt.show()


def plotParticles(particles: np.ndarray, weights, limit=50):
    fig = plt.figure()

    quiver_u = np.cos(particles[:limit, 2])
    quiver_v = np.sin(particles[:limit, 2])
    plt.quiver(
        particles[:limit, 0], particles[:limit, 1], quiver_u, quiver_v, weights[:limit]
    )
    plt.colorbar()
    fig.savefig("particles.png")


def updateFromTwist(particles, u, std, dt=1.0):
    """move according to control input u (heading change, velocity)
    with noise Q (std heading change, std velocity)`"""

    N = len(particles)
    # update heading
    particles[:, 2] += u[0] + (randn(N) * std[0])
    particles[:, 2] %= 2 * np.pi

    # move in the (noisy) commanded direction
    dist = (u[1] * dt) + (randn(N) * std[1])
    particles[:, 0] += np.cos(particles[:, 2]) * dist
    particles[:, 1] += np.sin(particles[:, 2]) * dist


def updateFromGnss(particles, weights, z, sensor_std_error):

    # z = [pos_x, pos_y]

    # This says, how likely is it that this particle
    # is REALLY this far away, given a distance measurement z?
    likelihooods = np.mean(
        scipy.stats.norm(particles[:, :2], sensor_std_error).pdf(z), axis=1
    )
    weights *= likelihooods

    weights += 1.0e-300  # avoid round-off to zero
    weights /= sum(weights)  # normalize


# Start by loading our sensor data
imu_df: pd.DataFrame = pd.read_pickle("data/pickles/imu.pkl")
print(imu_df.columns)
twists_df: pd.DataFrame = pd.read_pickle("data/pickles/twists.pkl")
print(twists_df.columns)
fixes_df: pd.DataFrame = pd.read_pickle("data/pickles/fixes.pkl")
print(fixes_df.columns)

# Filter readings by time
imu_df["t"] -= imu_df["t"].min()
fixes_df["t"] -= fixes_df["t"].min()
twists_df["t"] -= twists_df["t"].min()

start_time = 0.0
end_time = 195.0

fixes_df = fixes_df[fixes_df["t"] > start_time]
fixes_df = fixes_df[fixes_df["t"] < end_time]
imu_df = imu_df[imu_df["t"] > start_time]
imu_df = imu_df[imu_df["t"] < end_time]
twists_df = twists_df[twists_df["t"] > start_time]
twists_df = twists_df[twists_df["t"] < end_time]

# Plot our sensor data

print(fixes_df["pos_x"])
plt.gca().set_aspect("equal")
colors = fixes_df["t"].to_numpy()
yaw_np = fixes_df["yaw"].to_numpy()
quiver_u = np.cos(yaw_np)
quiver_v = np.sin(yaw_np)

plt.quiver(
    fixes_df["pos_x"].to_numpy(),
    fixes_df["pos_y"].to_numpy(),
    quiver_u,
    quiver_v,
    colors,
)
plt.colorbar()

# plt.plot(fixes_df["t"].to_numpy(), fixes_df["speed"].to_numpy())
# plt.show()

# Finally, run the filter
initial_x = [
    fixes_df["pos_x"].iloc[0],
    fixes_df["pos_y"].iloc[0],
    fixes_df["yaw"].iloc[0],
]
initial_std = [5, 5, np.pi / 4]
u_std = [0.2, 0.05]
sensor_std = [5.0, 5.0]
N = 500

print(f"Initial x: {initial_x}")
particles = create_gaussian_particles(mean=initial_x, std=initial_std, N=N)
weights = np.ones(N) / N
plotParticles(particles, weights)

xs = []


def atTime(df: pd.DataFrame, t: float):
    return df[df["t"] > t].iloc[0]


dt = 0.1
timesteps = np.arange(start_time, end_time, dt)


for t in tqdm(timesteps):
    # 1. Update motion from IMU
    twist = atTime(twists_df, t)
    fix = atTime(fixes_df, t)

    u = [twist["linear_vel_z"], fix["speed"]]
    updateFromTwist(particles, u, [u_std[0], fix["speed_err"]], dt)

    # 2. Update from GNSS
    z = [fix["pos_x"], fix["pos_y"]]
    sensor_std_error = [fix["x_err"], fix["y_err"]]
    updateFromGnss(particles, weights, z, sensor_std_error)

    # 3. Resample
    if neff(weights) < N / 2:
        indexes = systematic_resample(weights)
        resample_from_index(particles, weights, indexes)
        assert np.allclose(weights, 1 / N)
    mu, var = estimate(particles, weights)
    xs.append(mu)

    # if t % 10 == 0:
    #     plotParticles(particles, weights)

xs = np.asarray(xs)

fig, (ax1) = plt.subplots(1, 1)

ax1.quiver(
    fixes_df["pos_x"].to_numpy(),
    fixes_df["pos_y"].to_numpy(),
    quiver_u,
    quiver_v,
    colors,
)
# ax1.colorbar()
# ax1.
ax1.plot(xs[:, 0], xs[:, 1], c="red")

fig.savefig("result.png")
plt.show()
print(xs)
