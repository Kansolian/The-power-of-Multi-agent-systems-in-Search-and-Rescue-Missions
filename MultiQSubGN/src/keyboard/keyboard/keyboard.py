import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

import sys

from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry

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
strat = [0,1,2,3,4,5]

# The name is of no bigger importance
class Keyboard(Node):

    def __init__(self):
        super().__init__('keyboard')

        self.declare_parameter('numDrones', 3)
        self.numDrones = self.get_parameter('numDrones').value
        self.declare_parameter('trials', 5)
        self.trials = self.get_parameter('trials').value
        self.counter = 0

        
        self.drones = [[0,0],[0,0],[0,0]]

        self.pub = self.create_publisher(Float32MultiArray, 'substitute', 10)
        self.sub = self.create_subscription(Odometry, 'crazyflie0/odom', self.odom_callback, 10)
        self.sub1 = self.create_subscription(Odometry, 'crazyflie1/odom', self.odom1_callback, 10)
        self.sub2 = self.create_subscription(Odometry, 'crazyflie2/odom', self.odom2_callback, 10)

        self.counter = 0
        self.timer = self.create_timer(0.1, self.timer_callback)



    #After 10 seconds calculate the middle of all active drones and send it to the substitute
    def timer_callback(self):
        if self.counter == 100:
            coordinates = [0,0]
            for i in self.drones:
                coordinates[0] += i[0]
                coordinates[1] += i[1]


            coordinates = [x / 3 for x in coordinates]


            msg = Float32MultiArray()
            msg.data = coordinates
            self.pub.publish(msg)
            self.counter = 0
        else:
            self.counter += 1


    #Get the position of all Drones
    def odom_callback(self, msg):
        self.drones[0] = [msg.pose.pose.position.x,msg.pose.pose.position.y]

    def odom1_callback(self, msg):
        self.drones[1] = [msg.pose.pose.position.x,msg.pose.pose.position.y]

    def odom2_callback(self, msg):
        self.drones[2] = [msg.pose.pose.position.x,msg.pose.pose.position.y]







def main(args=None):
    rclpy.init(args=args)

    key = Keyboard()
    rclpy.spin(key)
    key.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()