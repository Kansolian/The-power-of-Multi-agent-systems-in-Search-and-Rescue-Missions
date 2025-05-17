import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

import tf_transformations
import math
import numpy as np

GLOBAL_SIZE_X = 50.0
GLOBAL_SIZE_Y = 50.0
MAP_RES = 0.1
class global_Map(Node):
    def __init__(self):
        super().__init__('global')
        self.declare_parameter('numDrones', 1)
        self.n = self.get_parameter('numDrones').value

        self.map_subscriber = self.create_subscription(
            OccupancyGrid, '/mapLocal', self.scan_subscribe_callback, 10)

        self.tfbr = StaticTransformBroadcaster(self)

        self.position_update = False

        self.map = [-1] * int(GLOBAL_SIZE_X / MAP_RES) * \
            int(GLOBAL_SIZE_Y / MAP_RES)
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map',
                                                   qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST,))



    def scan_subscribe_callback(self, msg):
        oldmap = msg.data

        # integrate the map of two drones and divide em by two, aggegating the information of all drones over time
        self.map =  np.ceil(np.add(self.map, oldmap) /2).astype(int)
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = MAP_RES
        msg.info.width = int(GLOBAL_SIZE_X / MAP_RES)
        msg.info.height = int(GLOBAL_SIZE_Y / MAP_RES)
        msg.info.origin.position.x = - GLOBAL_SIZE_X / 2.0
        msg.info.origin.position.y = - GLOBAL_SIZE_Y / 2.0
        msg.data = self.map
        self.map_publisher.publish(msg)

        


        





def main(args=None):

    rclpy.init(args=args)
    globa = global_Map()
    rclpy.spin(globa)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
