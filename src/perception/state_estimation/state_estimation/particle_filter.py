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

# ROS message definitions
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from gps_msgs.msg import GPSFix
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

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
    # landmarks = np.array([[-1, 2], [5, 10], [12, 14], [18, 21]])
    # NL = len(landmarks)

    # plt.figure()

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
        # zs = norm(landmarks - robot_pos, axis=1) + (randn(NL) * sensor_std_err)

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


# run_pf1(N=5000, plot_particles=False, initial_x=(1, 1, 1))


class ParticleFilterLocalizer(Node):
    def __init__(self):
        super().__init__("particle_filter_localizer")

        sensor_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.setUpParameters()

        self.ego_pos_hist = []
        self.ego_vel_hist = []
        self.imu_hist = []

        lat, lon, alt = self.get_parameter("map_origin_lat_lon_alt_degrees").value
        self.x0, self.y0, _, __ = utm.from_latlon(lat, lon)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.num_particles = 100

        self.particles = None
        self.weights = np.ones(self.num_particles) / self.num_particles
        self.initial_guess_std = [5, 5, np.pi / 4]

        # TODO: Parameterize topic name
        self.gnss_odom_sub = self.create_subscription(
            GPSFix, "/gnss/gpsfix", self.gpsFixCb, sensor_qos_profile
        )

        self.imu_sub = self.create_subscription(
            Imu, "/gnss/imu", self.imuCb, sensor_qos_profile
        )

        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmdVelCb, 10
        )

    def cmdVelCb(self, msg: Twist):
        u = (msg.angular.z, msg.linear.x)
        updateFromMotionCommand(self.particles, u=u, std=(0.2, 0.05))
        self.plotParticles()

    def plotParticles(self):
        fig = plt.figure()
        plt.scatter(self.particles[:, 0], self.particles[:, 1], alpha=0.2, color="g")
        plt.savefig("particles.png")
        plt.close(fig)

    def gpsFixCb(self, msg: GPSFix):

        ego_x, ego_y, _, __ = utm.from_latlon(msg.latitude, msg.longitude)

        ego_x = ego_x - self.x0
        ego_y = ego_y - self.y0

        unfiltered_yaw = trueTrackToEnuRads(msg.track)

        if self.particles is None:
            initial_x = [ego_x, ego_y, unfiltered_yaw]
            self.particles = create_gaussian_particles(
                mean=initial_x, std=self.initial_guess_std, N=self.num_particles
            )
            self.plotParticles()

        if len(self.ego_pos_hist) > 0:
            previous_yaw = self.ego_pos_hist[-1][2]
        else:
            previous_yaw = unfiltered_yaw

        if previous_yaw - unfiltered_yaw > 0.5:
            print("YAW JUMP")
            if msg.speed > 0.6:
                ego_yaw = unfiltered_yaw
                print(f"Speed was {msg.speed}, accepting jump")
            else:
                ego_yaw = previous_yaw
        else:
            ego_yaw = unfiltered_yaw

        self.ego_pos_hist.append([ego_x, ego_y, ego_yaw])
        self.ego_vel_hist.append(msg.speed)

        if len(self.ego_pos_hist) % 10 == 0:
            points = np.asarray(self.ego_pos_hist)

            # plt.figure()
            fig, ([ax1, ax2, ax3], [ax4, ax5, ax6]) = plt.subplots(2, 3)
            ax1.scatter(points[:, 0], points[:, 1])
            ax1.set_title(
                f"std: {np.std(points, axis=0)}, acc: {msg.err_horz}, {msg.err_vert}"
            )

            ax2.plot(range(len(points)), points[:, 2])

            imu_hist = np.asarray(self.imu_hist)

            ax3.plot(range(len(imu_hist)), imu_hist[:, 0], c="red")
            ax3.plot(range(len(imu_hist)), imu_hist[:, 1], c="green")
            ax3.plot(range(len(imu_hist)), imu_hist[:, 2], c="blue")
            ax3.set_title("Linear accel")

            ax4.plot(range(len(self.ego_vel_hist)), self.ego_vel_hist, c="blue")

            ax5.plot(range(len(self.ego_vel_hist)), self.ego_vel_hist, c="blue")
            ax5.plot(imu_hist[::10, 1], c="green")
            ax5.plot(range(len(points)), points[:, 2], c="red")

            plt.savefig("gnss.png")
            plt.close(fig)

            if len(self.ego_pos_hist) > 100:
                self.ego_pos_hist = self.ego_pos_hist[10:]
            if len(self.ego_vel_hist) > 100:
                self.ego_vel_hist = self.ego_vel_hist[10:]
            if len(self.imu_hist) > 100:
                self.imu_hist = self.imu_hist[10:]

    def imuCb(self, msg: Imu):

        linacc = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ]
        angvel = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ]

        self.imu_hist.append(linacc + angvel)

    def setUpParameters(self):
        param_desc = ParameterDescriptor()
        param_desc.type = ParameterType.PARAMETER_DOUBLE_ARRAY
        self.declare_parameter(
            "map_origin_lat_lon_alt_degrees",
            [40.4431653, -79.9402844, 288.0961589],
        )

    def poseCb(self, msg: PoseWithCovarianceStamped):
        t = TransformStamped()

        t.header = msg.header
        t.child_frame_id = "base_link"  # TODO: Change to 'gnss'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    broadcaster = ParticleFilterLocalizer()

    rclpy.spin(broadcaster)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    broadcaster.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
