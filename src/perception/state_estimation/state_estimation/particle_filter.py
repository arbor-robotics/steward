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
    particles = np.empty((N, 4))
    particles[:, 0] = mean[0] + (randn(N) * std[0])
    particles[:, 1] = mean[1] + (randn(N) * std[1])
    particles[:, 2] = mean[2] + (randn(N) * std[2])
    particles[:, 2] %= 2 * np.pi
    particles[:, 3] = mean[3] + (randn(N) * std[3])
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

    # pos = particles[:, 0:2]
    mean = np.average(particles, weights=weights, axis=0)
    var = np.average((particles - mean) ** 2, weights=weights, axis=0)
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


def plotParticles(particles: np.ndarray, weights, xs: list, limit=50):
    fig, (ax1) = plt.subplots(1, 1)
    xs = np.asarray(xs)

    plt.gca().set_aspect("equal")

    quiver_u = np.cos(particles[:limit, 2])
    quiver_v = np.sin(particles[:limit, 2])
    ax1.quiver(
        particles[:limit, 0], particles[:limit, 1], quiver_u, quiver_v, weights[:limit]
    )
    # plt.colorbar()

    ax1.plot(fixes_df["pos_x"].to_numpy(), fixes_df["pos_y"].to_numpy(), c="red")
    # ax1.colorbar()
    # ax1.

    if len(xs) > 0:
        ax = ax1.plot(xs[:, 0], xs[:, 1], c="blue", alpha=0.5)
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


def updateFromImu(particles, dyaw, ddx, std, dt):
    """move according to imu reading (dyaw, ddx)
    with noise Q (std dyaw, std ddx)`"""

    N = len(particles)

    # Update heading
    delta_yaw = (dyaw + (randn(N) * std[0])) * dt
    particles[:, 2] += delta_yaw
    particles[:, 2] %= 2 * np.pi

    # Update speed
    delta_speed = (ddx + (randn(N) * std[1])) * dt
    # plt.hist(delta_speed)
    # plt.show()
    particles[:, 3] += delta_speed

    # move in the (noisy) commanded direction
    dist = particles[:, 3] * dt
    particles[:, 0] += np.cos(particles[:, 2]) * dist
    particles[:, 1] += np.sin(particles[:, 2]) * dist

    print(
        f"My yaw is {particles[0,2]}, speed {particles[0,3]}. I'm moving in {np.cos(particles[0, 2]) * dist[0]}, {(np.sin(particles[:, 2]) * dist)[0]}"
    )


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
print(fixes_df.shape)
wheels_df: pd.DataFrame = pd.read_pickle("data/pickles/wheels.pkl")
print(wheels_df.shape)


# Filter readings by time
imu_df["t"] -= imu_df["t"].min()
fixes_df["t"] -= fixes_df["t"].min()
twists_df["t"] -= twists_df["t"].min()
wheels_df["t"] -= wheels_df["t"].min()

start_time = 0.0
end_time = 195.0

fixes_df = fixes_df[fixes_df["t"] > start_time]
fixes_df = fixes_df[fixes_df["t"] < end_time]
imu_df = imu_df[imu_df["t"] > start_time]
imu_df = imu_df[imu_df["t"] < end_time]
twists_df = twists_df[twists_df["t"] > start_time]
twists_df = twists_df[twists_df["t"] < end_time]
wheels_df = wheels_df[wheels_df["t"] > start_time]
wheels_df = wheels_df[wheels_df["t"] < end_time]

# Plot our sensor data

fig, ((ax1, ax2), (ax3, ax4), (ax5, ax6)) = plt.subplots(3, 2)
fig.set_size_inches(12, 8)

print(fixes_df["pos_x"])
colors = fixes_df["t"].to_numpy()
yaw_np = fixes_df["yaw"].to_numpy()
quiver_u = np.cos(yaw_np)
quiver_v = np.sin(yaw_np)

ax1: plt.Axes
ax1.quiver(
    fixes_df["pos_x"].to_numpy(),
    fixes_df["pos_y"].to_numpy(),
    quiver_u,
    quiver_v,
    colors,
)
ax1.set_title("GPS position")


def moving_average(a, n=3):
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1 :] / n


smooth_x_acc = moving_average(imu_df["linear_acc_x"].to_numpy(), n=100)
ax2.plot(imu_df["t"].to_numpy()[: len(smooth_x_acc)], smooth_x_acc, c="red")

smooth_y_acc = moving_average(imu_df["linear_acc_y"].to_numpy(), n=100)
ax2.plot(imu_df["t"].to_numpy()[: len(smooth_y_acc)], smooth_y_acc, c="green")
ax2.plot(
    imu_df["t"].to_numpy()[: len(smooth_y_acc)], np.zeros_like(smooth_y_acc), c="black"
)

smooth_z_acc = moving_average(imu_df["linear_acc_z"].to_numpy(), n=100)
ax2.plot(imu_df["t"].to_numpy()[: len(smooth_z_acc)], smooth_z_acc, c="blue")
ax2.plot(
    imu_df["t"].to_numpy()[: len(smooth_z_acc)],
    np.ones_like(smooth_z_acc) * -9.81,
    c="black",
)


ax2: plt.Axes
ax2.plot(imu_df["t"].to_numpy(), imu_df["linear_acc_x"].to_numpy(), c="red", alpha=0.5)
ax2.plot(
    imu_df["t"].to_numpy(), imu_df["linear_acc_y"].to_numpy(), c="green", alpha=0.5
)
ax2.plot(imu_df["t"].to_numpy(), imu_df["linear_acc_z"].to_numpy(), c="blue", alpha=0.5)
ax2.set_title("Linear acc")

ax3: plt.Axes
ax3.plot(
    twists_df["t"].to_numpy(), twists_df["linear_vel_x"].to_numpy(), c="red", alpha=0.5
)
ax3.plot(
    twists_df["t"].to_numpy(),
    twists_df["linear_vel_y"].to_numpy(),
    c="green",
    alpha=0.5,
)
ax3.plot(
    twists_df["t"].to_numpy(), twists_df["linear_vel_z"].to_numpy(), c="blue", alpha=0.5
)
ax3.set_title("Linear vel")

plt.savefig("sensor_data.png")

ax4: plt.Axes
ax4.plot(imu_df["t"].to_numpy(), imu_df["angular_vel_x"].to_numpy(), c="red", alpha=0.5)
ax4.plot(
    imu_df["t"].to_numpy(), imu_df["angular_vel_y"].to_numpy(), c="green", alpha=0.5
)
ax4.plot(
    imu_df["t"].to_numpy(), imu_df["angular_vel_z"].to_numpy(), c="blue", alpha=0.5
)
ax4.set_title("Angular vel")

ax5.plot(
    wheels_df["t"].to_numpy(), wheels_df["linear_vel_x"].to_numpy(), c="red", alpha=0.5
)
smooth_x_vel = moving_average(wheels_df["linear_vel_x"].to_numpy(), n=100)
ax2.plot(wheels_df["t"].to_numpy()[: len(smooth_x_vel)], smooth_x_vel, c="red")
ax5.set_title("Wheel twists")
print(wheels_df["linear_vel_x"])


plt.savefig("sensor_data.png")

# plt.plot(fixes_df["t"].to_numpy(), fixes_df["speed"].to_numpy())
# plt.show()

# Sanity check of IMU data
x = []

exit()

# Finally, run the filter
initial_x = [
    fixes_df["pos_x"].iloc[0],
    fixes_df["pos_y"].iloc[0],
    fixes_df["yaw"].iloc[0],
    fixes_df["speed"].iloc[0],
]
initial_std = [5.0, 5.0, np.pi / 4, 1.0]
u_std = [0.2, 0.05]
sensor_std = [5.0, 5.0]
N = 500

print(f"Initial x: {initial_x}")
particles = create_gaussian_particles(mean=initial_x, std=initial_std, N=N)
weights = np.ones(N) / N
xs = []
plotParticles(particles, weights, xs)


def atTime(df: pd.DataFrame, t: float, tol=0.1):
    try:
        row = df[df["t"] > t - tol].iloc[0]
        if row["t"] - t > 2 * tol:
            return None
        return row
    except IndexError as e:
        return None


dt = 0.1
timesteps = np.arange(start_time, end_time, dt)


for t in tqdm(timesteps):
    # 1. Update motion from IMU
    twist = atTime(twists_df, t)
    fix = atTime(fixes_df, t)
    imu = atTime(imu_df, t)

    wheel = atTime(wheels_df, t)

    dyaw = imu["angular_vel_z"]
    ddx = imu["linear_acc_x"]

    u = [imu["angular_vel_z"] * 1.0, fix["speed"]]
    # updateFromTwist(particles, u, [u_std[0], fix["speed_err"]], dt)
    updateFromImu(
        particles,
        dyaw * 1.0,
        ddx,
        [
            1.0,
        ],
        dt,
    )

    # 2. Update from GNSS
    z = [fix["pos_x"], fix["pos_y"]]
    sensor_std_error = np.asarray([fix["x_err"], fix["y_err"]]) * 2
    updateFromGnss(particles, weights, z, sensor_std_error)

    # 3. Resample
    if neff(weights) < N / 2:
        indexes = systematic_resample(weights)
        resample_from_index(particles, weights, indexes)
        assert np.allclose(weights, 1 / N)
    mu, var = estimate(particles, weights)
    xs.append(mu)

    plotParticles(particles, weights, xs)

xs = np.asarray(xs)

fig, (ax1) = plt.subplots(1, 1)
plt.gca().set_aspect("equal")

ax1.plot(fixes_df["pos_x"].to_numpy(), fixes_df["pos_y"].to_numpy(), c="red")
# ax1.colorbar()
# ax1.
colors = xs[:, 3]

ax = ax1.plot(xs[:, 0], xs[:, 1], c="blue", alpha=0.5)
# fig.colorbar(ax)

fig.savefig("result.png")
# plt.show()
print(xs)
