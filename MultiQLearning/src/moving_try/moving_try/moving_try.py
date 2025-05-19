#!/usr/bin/env python3

""" 
This Movement class is loosly based on both the bitcraze cflib point cloud example
 https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/multiranger/multiranger_pointcloud.py
 and the webots epuck simple mapper example:
 https://github.com/cyberbotics/webots_ros2

 Originally from https://github.com/knmcguire/crazyflie_ros2_experimental/
 """

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from tf2_ros import StaticTransformBroadcaster
from std_srvs.srv import Trigger
from std_msgs.msg import Float32MultiArray

import tf_transformations
import math
import numpy as np
import time
import random 

GLOBAL_SIZE_X = 30.0
GLOBAL_SIZE_Y = 30.0
MAP_RES = 0.1

import itertools
from itertools import combinations
import numpy as np
import math
import pickle




import itertools
import numpy as np
import math


search_strat = [0,1,2,3,4,5,6,7]
og_boxes = [(21.0, -5.0 , 0.25),(15.0, 6.0 , 0.5),(5.0,0.3,0.4),(-2.0,7.2,0.2),(-2.2, -2.6, 0.25),(-5.6,-5.9,0.1),(20.2, -0.2,0.4),(11.59,0.67,0.5)]
        

class Moving(Node):
    def __init__(self):

        super().__init__('Movement')
        
        #Overall Setup
        self.declare_parameter('robot_prefix', '/crazyflie')
        robot_prefix = self.get_parameter('robot_prefix').value
        self.declare_parameter('delay', 5.0)
        self.delay = self.get_parameter('delay').value

        self.declare_parameter('hidder', -1)
        self.hidder= self.get_parameter('hidder').value
        self.declare_parameter('strategy',[0,1,2])
        self.strategy = self.get_parameter('strategy').value
        self.declare_parameter('n',1)
        self.n = self.get_parameter('n').value
        
        self.counter = self.n
        self.og_strategy = self.strategy[:]
        self.robot_prefix = robot_prefix

        self.get_logger().info(f"Hidder is set on {self.hidder} using the strategy {self.strategy}")

        self.hidders_found = 0

        #Movement Variables
        self.stop = False
        self.start_pos = False
        self.position = [0.0, 0.0, 0.0]
        self.angles = [0.0, 0.0, 0.0]
        self.ranges = [0.0, 0.0, 0.0, 0.0]
        self.height_sensor = [0.0]
        self.lean_sensor = [0.0]
        self.avoiding = False
        self.count = 5
        self.target = False
        self.boxes = [(21.0, -5.0 , 0.25),(15.0, 6.0 , 0.5),(5.0,0.3,0.4),(-2.0,7.2,0.2),(-2.2, -2.6, 0.25),(-5.6,-5.9,0.1),(20.2, -0.2,0.4),(11.59,0.67,0.5)]
        self.fail = False
        self.succ = False
        self.start = True
        self.finder = False
        self.oldSearch = -1

        self.declare_parameter('starting_x', 0.0)
        self.starting_x= self.get_parameter('starting_x').value
        self.declare_parameter('starting_y', 0.0)
        self.starting_y= self.get_parameter('starting_y').value
        self.declare_parameter('max_turn_rate', 0.5)
        max_turn_rate = self.get_parameter('max_turn_rate').value
        self.declare_parameter('max_forward_speed', 0.5)
        max_forward_speed = self.get_parameter('max_forward_speed').value

        self.odom_subscriber = self.create_subscription(
            Odometry, robot_prefix + '/odom', self.odom_subscribe_callback, 10)
        self.ranges_subscriber = self.create_subscription(
            LaserScan, robot_prefix + '/scan', self.scan_subscribe_callback, 10)
        self.ranges_subscriber = self.create_subscription(
            LaserScan, robot_prefix + '/scan2', self.scan_subscribe_callback2, 10)
        self.ranges_subscriber = self.create_subscription(
            LaserScan, robot_prefix + '/scan3', self.scan_subscribe_callback3, 10)
        self.searcher_subscriber = self.create_subscription(
            Int16, 'boxes', self.searcher_subscribe_callback, 10)

        self.reset_subscriber = self.create_subscription(
            Int16, 'resetCycle', self.reset_subscribe_callback, 10)
        self.reset_pub = self.create_publisher(
            Int16, 'resetCycle', 10)

        self.start_subscriber = self.create_subscription(
            Int16, 'startCycle', self.start_subscribe_callback, 10)
        self.start_pub = self.create_publisher(
            Int16, 'startCommand', 10)
        
        self.belief_subscriber = self.create_subscription(
            Float32MultiArray, robot_prefix+'/belief', self.belief_callback, 10)

        self.searched_publisher = self.create_publisher(Int16, 'searched', 10)
        self.twist_publisher = self.create_publisher(Twist, robot_prefix+'/cmd_vel', 10)


        #Saving
        self.fileName = 'ExperimentSim3R'+self.robot_prefix
        self.historyQ = []
        self.historyList = []

        #Q Learning variables and Initialization
        self.n_states = 255
        self.n_actions = 8
        self.qTable = np.zeros((self.n_states, self.n_actions))
        self.current_state = [0,1,2,3,4,5,6,7]
        self.next_state = self.current_state[:]


        self.lr = 0.8
        self.disFactor = 0.95
        self.explProb = 0.1
        self.total_Reward = 0

        self.all_states =[]
        for i in range(1,9):
            combi = combinations(range(8), i)
            for j in list(combi):
                self.all_states.append(list(j))

        



        '''
        with open('/home/coders/MultiQLearning/Exp/'+self.fileName+'QTable.pkl', 'rb') as f:
            self.qTable = pickle.load(f)
        
        
        with open('/home/coders/MultiQLearning/Exp/'+self.fileName+'.pkl', 'rb') as f:
            self.historyList = pickle.load(f)

        
        with open('/home/coders/MultiQLearning/Exp/'+self.fileName+'QEntries.pkl', 'rb') as f:
            self.historyQ = pickle.load(f)
        '''


        # Movement Timer
        self.timer = self.create_timer(0.01, self.timer_callback)

        # Give a take off command but wait for the delay to start the runs
        self.wait_for_start = True
        self.start_clock = self.get_clock().now().nanoseconds * 1e-9
        msg = Twist()
        msg.linear.z = 1.5
        self.twist_publisher.publish(msg)






    def stop_wall_following_cb(self, request, response):
        self.get_logger().info('Stopping')
        self.timer.cancel()
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = -0.2
        msg.angular.z = 0.0
        self.twist_publisher.publish(msg)

        response.success = True

        return response

    # Update the Q-Table
    def update_Table(self, reward, reset, conti):
        spot = self.strategy[0]
        if spot in self.next_state and conti:
            self.next_state.remove(spot)

        # If the goal has been acquired
        if len(self.next_state) == 0:
            self.strategy = [0]
            self.boxes.insert(0,(self.starting_x, self.starting_y, 1.1))
            return 
        
        #When training update
        if reset:
            self.qTable[self.all_states.index(self.current_state), spot] += self.lr * (reward + self.disFactor * np.max(self.qTable[self.all_states.index(self.next_state)]) - self.qTable[self.all_states.index(self.current_state), spot])
        
        # Reset or choose next target
        if reset:
            self.strategy = [0]
            self.boxes.insert(0,(self.starting_x, self.starting_y, 1.1))
        else:
            # searched spot, new target spot selection
            if np.random.rand() < self.explProb:
                self.strategy = [random.choice(self.next_state)]
            else:
                self.strategy = [np.argmax(self.qTable[self.all_states.index(self.next_state)])]
            self.og_strategy.append(self.strategy[0])


    # Search function
    def search(self):
        rand = random.random()

        targetP = self.boxes[self.strategy[0]][2]
        indexS = self.strategy[0]

        # if drone was called back, prep reset of iteration
        if self.start_pos:
            self.start = False
            self.start_pos = False
            self.target = False
            self.hidder = random.choice([0,1,2,3,4,5,6,7])

            if np.random.rand() < self.explProb:
                self.strategy = [random.choice([0,1,2,3,4,5,6,7])]
            else:
                self.strategy = [np.argmax(self.qTable[self.all_states.index(self.current_state)])]

            self.og_strategy = self.strategy[:]
            self.boxes= og_boxes[:]
            self.counter -=1
            self.historyQ.append([self.robot_prefix,self.n-self.counter, self.total_Reward,self.qTable])
            self.oldSearch = -1

            # Save information
            with open('/home/coders/MultiQLearning/Exp/'+self.fileName+'QTable.pkl', 'wb') as f:
                pickle.dump(self.qTable, f)
            
            with open('/home/coders/MultiQLearning/Exp/'+self.fileName+'QEntries.pkl', 'wb') as f:
                pickle.dump(self.historyQ, f)



            msgS = Int16()
            msgS.data = 0
            self.start_pub.publish(msgS)

            self.get_logger().info(f'{self.robot_prefix} waiting, New Hidder {self.hidder+1} and strategy {self.strategy}')
            return
        
        # Check if failed
        if rand > targetP:
            # Check if on target
            if self.hidder == indexS:
                self.start_pos = True
                self.hidders_found +=1
                self.get_logger().info(f'{self.robot_prefix} Found at Spot {indexS+1}')
                self.finder=True
                self.historyList.append([self.robot_prefix,self.n-self.counter,self.og_strategy,self.hidder,1,indexS])

                reward = 10
                self.update_Table(reward, True, False)
                self.total_Reward += reward


                with open('/home/coders/MultiQLearning/Exp/'+self.fileName+'.pkl', 'wb') as f:
                    pickle.dump(self.historyList, f)

                msgR = Int16()
                msgR.data = 0
                self.reset_pub.publish(msgR)
                self.target=False
                return 

            msgI = Int16()
            msgI.data = int(indexS)
            self.searched_publisher.publish(msgI)

            reward = 2
            if indexS not in self.current_state:
                reward = -5

            self.update_Table(reward, False, True)
            self.total_Reward += reward

            self.target=False
            self.current_state = self.next_state[:]
            self.oldSearch = indexS
            
        else:
            # Drone failed
            self.target=False
            self.start_pos = True
            self.finder = True
            

            reward = -1
            self.update_Table(reward, True, False)
            self.total_Reward += reward

            self.get_logger().info(f'{self.robot_prefix} Failed at Spot {indexS + 1}')
            self.historyList.append([self.robot_prefix,self.n-self.counter,self.og_strategy,self.hidder,0,indexS])
            
            with open('/home/coders/MultiQLearning/Exp/'+self.fileName+'.pkl', 'wb') as f:
                pickle.dump(self.historyList, f)

    # Final function stopping movement
    def fall(self):
        self.get_logger().info('Done')
        self.timer.cancel()
        msg1 = Twist()
        msg1.linear.x = 0.0
        msg1.linear.y = 0.0
        msg1.linear.z = -0.2
        self.twist_publisher.publish(msg1)
        return
        


    # Main timer function -> nearly all functionality
    def timer_callback(self):

        # Hover if not active
        if not self.start:
            msg = Twist()
            error = 1.5 - self.position[2]
            msg.linear.z = error
            self.twist_publisher.publish(msg)

            return


        # End if all runs have been completed
        if self.counter == 0:
            self.fall()
            self.get_logger().info(f'Average hidders found on {self.n} runs: {self.hidders_found/self.n}')
            with open('/home/coders/MultiQLearning/Exp/'+self.fileName+'.pkl', 'wb') as f:
                pickle.dump(self.historyList, f)
            
            with open('/home/coders/MultiQLearning/Exp/'+self.fileName+'QTable.pkl', 'wb') as f:
                pickle.dump(self.qTable, f)
            return
        

        # wait for the delay to pass and then start Runs
        if self.wait_for_start:
            if self.get_clock().now().nanoseconds * 1e-9 - self.start_clock > self.delay :
                self.get_logger().info('Start cycle')
                self.wait_for_start = False
            else:
                return


        front_range = self.ranges[2]
        msg = Twist()
        time_now = self.get_clock().now().nanoseconds * 1e-9
        
        # obstacle avoidance start
        if front_range < 0.3:
            self.avoiding = True
            self.count = 20
        else:
            self.avoiding = False
        
        # Get target
        targetX = self.boxes[self.strategy[0]][0]
        targetY = self.boxes[self.strategy[0]][1]

        distX = (targetX-self.position[0])
        distY = (targetY-self.position[1])

        z_temp = 0.0
        x_temp = 0.0

        # Angle to target
        angle_to_obj = math.atan2(distX,distY)
        angle_to= angle_to_obj+math.pi*(6/4) if angle_to_obj < - math.pi/2 else angle_to_obj-math.pi/2
        calc = angle_to + self.angles[2]

        if abs(calc) > math.pi:
            sig = 1 if calc < 0 else -1
            res = abs(calc) % math.pi
            calc = res*sig
        

        # adjustment towards target while flying
        if calc > 0.2:
            z_temp = -0.5
        elif calc < -0.2:
            z_temp = 0.5
        else:
            z_temp = 0.0
            self.target=True

        # start flying if in target direction
        if self.target:
            x_temp= 0.8
        else:
            x_temp= 0.0


        # height adjustment
        height = self.height_sensor[0]
        front = self.ranges[2]
        lean = self.lean_sensor[0]

        
        error = 1.5 - self.position[2]
        if lean > height and  height > 0:
            error = 1.5 - height
        elif lean > 0:
            error = 1.5 - lean

        error = error % 0.5 if error > 0 else (abs(error) % 0.5)* -1.0
        if not math.isnan(error):
            msg.linear.z = error
        else:
            msg.linear.z = 1.5 - self.position[2]

        # Avoidance procedure
        if self.avoiding or self.count > 0 or self.ranges[3] < 0.1 or self.ranges[1] < 0.1:
            if self.ranges[3] > self.ranges[1]:
                msg.linear.y = 0.5
            else:
                msg.linear.y = -0.5
            msg.angular.z = 0.0
            msg.linear.x = 0.0
            self.count-=1
        else:
            msg.angular.z = z_temp
            msg.linear.x = x_temp
            msg.linear.y = 0.0


        self.twist_publisher.publish(msg)
        
        # Search if on target
        if abs(distX) < 0.2 and abs(distY) < 0.2:
            self.search()


    # Get odometry scans
    def odom_subscribe_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        euler = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.angles[0] = euler[0]
        self.angles[1] = euler[1]
        self.angles[2] = euler[2]
        self.position_update = True

    # Get lidar sensors
    def scan_subscribe_callback(self, msg):
        self.ranges = msg.ranges

    def scan_subscribe_callback2(self, msg):
        self.lean_sensor= msg.ranges

    def scan_subscribe_callback3(self, msg):
        self.height_sensor= msg.ranges

    # Shared search information
    def searcher_subscribe_callback(self, msg):
        ind = msg.data
        if ind in self.next_state:
            self.next_state.remove(ind)
            self.target = False

        if len(self.next_state) == 0 and self.start:
            
            self.start_pos = True
            self.target = False

            self.strategy = [0]
            self.boxes.insert(0,(self.starting_x, self.starting_y, 1.1))
            if not self.finder:
                self.historyList.append([self.robot_prefix,self.n-self.counter,self.og_strategy,self.hidder,3,ind])
                with open('/home/coders/MultiQLearning/Exp/'+self.fileName+'.pkl', 'wb') as f:
                    pickle.dump(self.historyList, f)

            self.finder = True

    # Reset call
    def reset_subscribe_callback(self,msg):
        if self.start:
            self.start_pos = True
            self.target = False


            if not self.finder:
                self.historyList.append([self.robot_prefix,self.n-self.counter,self.og_strategy,self.hidder,2,self.strategy[0]])

                reward = 10
                self.update_Table(reward, True,False)
                self.total_Reward += reward

                with open('/home/coders/MultiQLearning/Exp/'+self.fileName+'.pkl', 'wb') as f:
                    pickle.dump(self.historyList, f)
                self.finder = True

            self.strategy = [0]
            self.boxes.insert(0,(self.starting_x, self.starting_y, 1.1))

    # Start iteration
    def start_subscribe_callback(self,msg):
        self.start=True
        self.current_state = [0,1,2,3,4,5,6,7]
        self.next_state = self.current_state[:]
        self.hidder = msg.data
        self.boxes= og_boxes[:]
        self.finder = False
            
    # Monte Carlo integration
    def belief_callback(self,msg):
        x = msg.data[0]
        y = msg.data[0]

        self.position[0] = (self.position[0] + x)/2
        self.position[1] = (self.position[1] + y)/2
            

def main(args=None):

    rclpy.init(args=args)
    mover = Moving()
    rclpy.spin(mover)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
