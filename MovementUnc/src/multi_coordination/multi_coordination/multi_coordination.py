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
import pickle

boxes = [(21.0, -5.0 , 0.25),(15.0, 6.0 , 0.5),(5.0,0.3,0.4),(-2.0,7.2,0.2),(-2.2, -2.6, 0.25),(-5.6,-5.9,0.1),(20.2, 1.3,0.4),(11.59,0.67,0.5)]


class Coordination(Node):

    def __init__(self):
        super().__init__('Multicoord')

        self.declare_parameter('numDrones', 3)
        self.numDrones = self.get_parameter('numDrones').value
        self.declare_parameter('trials', 5)
        self.trials = self.get_parameter('trials').value
        self.counter = self.trials

        # Load data if already recorded
        '''
        with open('/home/coders/MultiQSub/Exp/MultiCoor.pkl', 'rb') as f:
            self.startTimes = pickle.load(f)
        '''

        self.found = None
        self.remove = None

        self.startTimes = [[0, self.get_clock().now().nanoseconds * 1e-9]]
        self.reset = []
        self.pub = self.create_publisher(Int16, 'boxes', 5)
        self.sub = self.create_subscription(Int16, 'searched', self.searched_callback, 10)

        self.startpub = self.create_publisher(Int16, 'startCycle', 10)
        self.startSub = self.create_subscription(Int16, 'startCommand', self.start_callback, 10)
        self.timer = self.create_timer(0.01, self.timer_callback)


    # Main function
    def timer_callback(self):
        if self.found:
            msg = Int16()
            msg.data = self.remove
            
            self.pub.publish(msg)
            self.get_logger().info(f'Searched {self.remove+1}')
            self.found = False

        # Send reset command
        if len(self.reset) == self.numDrones:
            self.reset = []
            self.counter -=1
            msg = Int16()
            msg.data = random.choice([0,1,2,3,4,5,6,7])
            self.startpub.publish(msg)
            self.startTimes.append([self.trials - self.counter, self.get_clock().now().nanoseconds * 1e-9])
            self.get_logger().info(f'Iteration: {self.trials - self.counter}')
            self.get_logger().info(f'New hidder: {msg.data+1}')

            with open('/home/coders/MovementUnc/Exp/MultiCoor.pkl', 'wb') as f:
                pickle.dump(self.startTimes, f)

    # Boxes successfully searched
    def searched_callback(self, msg):
        self.found = True
        self.remove = msg.data


    # Get drones ready on starting point
    def start_callback(self, msg):
        i = msg.data

        self.reset.append(i)






def main(args=None):
    rclpy.init(args=args)

    coor = Coordination()
    rclpy.spin(coor)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()