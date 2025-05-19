#!/usr/bin/env python3

""" 
The monte carlo implementation has been taken from https://github.com/yz9/Monte-Carlo-Localization/tree/master and adapted to our use case
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

import rclpy
from rclpy.duration import Duration
import rclpy.time
import tf_transformations
from std_msgs.msg import String, Header, ColorRGBA
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from math import sqrt, cos, sin, pi, atan2
from math import pi, log, exp
import random
import numpy as np

from .sub_class.particle import Particle, ParticleFilter

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from std_msgs.msg import Float32MultiArray


GLOBAL_SIZE_X = 50.0
GLOBAL_SIZE_Y = 50.0
MAP_RES = 0.1



class Monte_Carlo(Node):
    def __init__(self):
        super().__init__('monte_carlo')

        self.declare_parameter('robot_prefix', '/crazyflie')
        robot_prefix = self.get_parameter('robot_prefix').value
        self.declare_parameter('num_particles', 50)
        num_particles= self.get_parameter('num_particles').value
        self.belief =  [0.0,0.0]
        self.xs = []
        self.ys = []

        self.declare_parameter('xmin', -20)
        xmin= self.get_parameter('xmin').value
        self.declare_parameter('xmax', 20)
        xmax= self.get_parameter('xmax').value
        self.declare_parameter('ymin', -20)
        ymin= self.get_parameter('ymin').value
        self.declare_parameter('ymax', 20)
        ymax= self.get_parameter('ymax').value

        self.declare_parameter('dynamics_translation_noise_std_dev', 0.45)
        self.dynamics_translation_noise_std_dev= self.get_parameter('dynamics_translation_noise_std_dev').value
        self.declare_parameter('dynamics_orientation_noise_std_dev', 0.03)
        self.dynamics_orientation_noise_std_dev= self.get_parameter('dynamics_orientation_noise_std_dev').value
        self.declare_parameter('beam_range_measurement_noise_std_dev', 0.3)
        self.beam_range_measurement_noise_std_dev= self.get_parameter('beam_range_measurement_noise_std_dev').value

        l = [-1] * int(GLOBAL_SIZE_X / MAP_RES) * \
            int(GLOBAL_SIZE_Y / MAP_RES)
        grid_map = np.array(l, dtype='int8')
        grid_map = grid_map.reshape((int(GLOBAL_SIZE_X*10), int(GLOBAL_SIZE_Y*10)))

        for i in range(10):
            for j in range(10):
                grid_map[245+i][245+j] = 0

        m = list(grid_map.reshape(-1))

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = MAP_RES
        msg.info.width = int(GLOBAL_SIZE_X / MAP_RES)
        msg.info.height = int(GLOBAL_SIZE_Y / MAP_RES)
        msg.info.origin.position.x = - GLOBAL_SIZE_X / 2.0
        msg.info.origin.position.y = - GLOBAL_SIZE_Y / 2.0
        msg.data = m

        self.ogm = msg

        self.q_baselink_baselaser = np.array([1.0, 0, 0, 0])
        self.R_baselink_baselaser = tf_transformations.quaternion_matrix(self.q_baselink_baselaser)[0:3,0:3]
        self.p_baselink_baselaser = np.array([0.337, 0.0, 0.308])


        self.pf = ParticleFilter(num_particles, self.ogm, xmin, xmax, ymin, ymax, 0, 0, 0, 0,
                            self.dynamics_translation_noise_std_dev,
                            self.dynamics_orientation_noise_std_dev,
                            self.beam_range_measurement_noise_std_dev)

        self.pf.init_particles()
        self.last_scan = None


        self.map_subscriber = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry, robot_prefix + '/odom', self.odometry_callback, 10)

        self.laser_sub = self.create_subscription(
            LaserScan, robot_prefix+'/scan4',self.laser_scan_callback, 10)

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.laser_points_marker_pub = self.create_publisher(Marker, robot_prefix+'/debug/laser_points',10)
        self.particles_pub = self.create_publisher(MarkerArray, robot_prefix+'/particle_filter/particles',  10)
        self.belief_pub = self.create_publisher(Float32MultiArray, robot_prefix+'/belief',  10)

        self.get_logger().info(f'Starting monte carlo for {robot_prefix}')

    # Get new odometry scans
    def odometry_callback(self, msg):
        self.pf.handle_odometry(msg)

    # Get updated map
    def map_callback(self, msg):
        self.ogm = msg
        self.pf.set_ogm(self.ogm)

    def get_2d_laser_points_marker(self, timestamp, frame_id, pts_in_map, marker_id, rgba):
        msg = Marker()
        msg.header.stamp = timestamp
        msg.header.frame_id = frame_id
        msg.ns = 'laser_points'
        msg.id = marker_id
        msg.type = 6
        msg.action = 0
        msg.points = [Point(x=pt[0], y=pt[1], z=pt[2]) for pt in pts_in_map]
        msg.colors = [rgba for pt in pts_in_map]

        for pt in pts_in_map:
            assert((not np.isnan(pt).any()) and np.isfinite(pt).all())

        msg.scale.x = 0.1
        msg.scale.y = 0.1
        msg.scale.z = 0.1
        return msg

    def laser_scan_callback(self, msg):
        self.pf.laser_min_angle = msg.angle_min
        self.pf.laser_max_angle = msg.angle_max
        self.pf.laser_min_range = msg.range_min
        self.pf.laser_max_range = msg.range_max

        dt_since_last_scan = 0

        if self.last_scan:
            dt_since_last_scan = (msg.header.stamp.nanosec - self.last_scan.header.stamp.nanosec)/1000000000.0

        self.publish_laser_pts(msg)
        self.pf.handle_observation(msg, dt_since_last_scan)

        self.pf.dx = 0
        self.pf.dy = 0
        self.pf.dyaw = 0

        self.last_scan = msg
    

    def publish_laser_pts(self, msg):
        """Publishes the currently received laser scan points from the robot, after we subsampled
        them in order to comparse them with the expected laser scan from each particle."""
        if self.pf.robot_odom is None:
            return

        subsampled_ranges, subsampled_angles = self.pf.subsample_laser_scan(msg)

        N = len(subsampled_ranges)
        x = self.pf.robot_odom.pose.pose.position.x
        y = self.pf.robot_odom.pose.pose.position.y
        _, _ , yaw_in_map = tf_transformations.euler_from_quaternion(np.array([self.pf.robot_odom.pose.pose.orientation.x,
                                                               self.pf.robot_odom.pose.pose.orientation.y,
                                                               self.pf.robot_odom.pose.pose.orientation.z,
                                                               self.pf.robot_odom.pose.pose.orientation.w]))

        pts_in_map = [ (x + r*cos(theta + yaw_in_map),
                        y + r*sin(theta + yaw_in_map),
                        0.3) for r,theta in zip(subsampled_ranges, subsampled_angles)]

        lpmarker = self.get_2d_laser_points_marker(msg.header.stamp, 'map', pts_in_map, 30000, ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))
        self.laser_points_marker_pub.publish(lpmarker)
    
    def get_particle_marker(self, timestamp, particle, marker_id):
        """Returns an rviz marker that visualizes a single particle"""
        msg = Marker()
        msg.header.stamp = timestamp
        msg.header.frame_id = 'map'
        msg.ns = 'particles'
        msg.id = marker_id
        msg.type = 0  # arrow
        msg.action = 0 # add/modify
        msg.lifetime =  Duration(seconds=2).to_msg()

        yaw_in_map = particle.theta
        vx = cos(yaw_in_map)
        vy = sin(yaw_in_map)

        msg.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)

        msg.points.append(Point(x=particle.x, y=particle.y, z=0.2))
        msg.points.append(Point(x=particle.x + 0.3*vx, y=particle.y + 0.3*vy, z=0.2))
        self.xs.append(particle.x)
        self.ys.append(particle.y)
        

        msg.scale.x = 0.05
        msg.scale.y = 0.15
        msg.scale.z = 0.1
        return msg

    # Calculate the mean placement as belief of position and draw markers
    def publish_particle_markers(self):
        """ Publishes the particles of the particle filter in rviz"""
        ma = MarkerArray()
        ts = self.get_clock().now().to_msg()
        for i in range(len(self.pf.particles)):
            ma.markers.append(self.get_particle_marker(ts, self.pf.particles[i], i))

        self.belief[0] = sum(self.xs)/len(self.xs)
        self.belief[1] = sum(self.ys)/len(self.ys)
        self.xs = []
        self.ys = []

        msg = Float32MultiArray()
        msg.data = self.belief

        self.belief_pub.publish(msg)
        self.particles_pub.publish(ma)

    def timer_callback(self):

        self.publish_particle_markers()



def main(args=None):

    rclpy.init(args=args)
    monte = Monte_Carlo()
    rclpy.spin(monte)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
