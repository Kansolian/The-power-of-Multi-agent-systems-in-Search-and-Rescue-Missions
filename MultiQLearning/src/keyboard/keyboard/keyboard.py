import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

import sys

from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16

from geometry_msgs.msg import Twist
from tf2_ros import StaticTransformBroadcaster
from std_srvs.srv import Trigger

import tf_transformations
import math
import numpy as np
import time
import random 

boxes = [(21.0, -5.0 , 0.25),(15.0, 6.0 , 0.5),(5.0,0.3,0.4),(-2.0,7.2,0.2),(-2.2, -2.6, 0.25),(-5.6,-5.9,0.1),(20.2, 1.3,0.4),(11.59,0.67,0.5)]
strat = [0,1,2,3,4,5]

class Keyboard(Node):

    def __init__(self):
        super().__init__('keyboard')

        self.found = None
        self.remove = None
        self.pub = self.create_publisher(Int16, 'boxes', 5)
        self.sub = self.create_subscription(Int16, 'searched', self.searched_callback, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        if self.found:
            msg = Int16()
            msg.data = self.remove
            
            self.pub.publish(msg)
            self.found = False

    def searched_callback(self, msg):
        self.found = True
        self.remove = msg.data






def main(args=None):
    rclpy.init(args=args)

    key = Keyboard()
    rclpy.spin(key)
    Key.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()