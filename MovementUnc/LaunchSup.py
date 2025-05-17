# Code has been taken and adapted as mentioned in the paper from the bitcraze crazyflie wall following tutorial

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


import itertools
import numpy as np
import math
import random

number_drones = 4
trials = 200

singleExp = [(5.0, 0.3 , 0.5),(-2.0, 7.2  , 0.25),(-2.2, -2.6, 0.33)]
boxes = [(21.0, -5.0 , 0.25),(15.0, 6.0 , 0.5),(5.0,0.3,0.4),(-2.0,7.2,0.2),(-2.2, -2.6, 0.25),(-5.6,-5.9,0.1),(20.2, -0.2,0.4),(11.59,0.67,0.5)]
n=8
S = [i for i in range(n)]

search_strats = list(itertools.permutations(S))

x = [-0.5,0.0,0.5,1.0]
y = [-0.5,0.0,0.5,1.0]

h = random.choice([0,1,2,3,4,5,6,7])

def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    #pkg_project_crazyflie_gazebo = get_package_share_directory('ros_gz_crazyflie_bringup')

    l_start = []
    # Setup to launch a crazyflie gazebo simulation from the ros_gz_crazyflie project
    crazyflie_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join('launch', 'launchTest.py'))
    )
    l_start.append(crazyflie_simulation)

    for i in range(number_drones):
    # start a simple mapper node
        

        mapper = Node(
            package='crazyflie_ros2_multiranger_simple_mapper',
            executable='simple_mapper_multiranger',
            name='simple_mapper'+str(i),
            output='screen',
            parameters=[
                {'robot_prefix': '/crazyflie'+str(i)},
                {'use_sim_time': True}
                ]
            )

        mover = Node(
        package='moving_try',
        executable='moving_try',
        name='moving_try'+str(i),
        output='screen',
        parameters=[
            {'robot_prefix': '/crazyflie'+str(i)},
            {'use_sim_time': True},
            {'delay': 5.0 + i},
            {'max_turn_rate': 0.7},
            {'max_forward_speed': 0.5},
            {'strategy': search_strats[random.randint(0,len(search_strats))]},
            {'starting_x': x[i]},
            {'starting_y': y[i]},
            {'hidder': h},
            {'n': trials}
            ]
            )

        monte = Node(
        package='monte_carlo',
        executable='monte_carlo',
        name='monte'+str(i),
        output='screen',
        parameters=[
            {'robot_prefix': '/crazyflie'+str(i)},
            {'use_sim_time': True},
            {'num_particles': 50},
            {'xmin': -20},
            {'xmax': 20},
            {'ymin': -20},
            {'ymax': 20},
            {'dynamics_translation_noise_std_dev': 0.45},
            {'dynamics_orientation_noise_std_dev': 0.03},
            {'beam_range_measurement_noise_std_dev': 0.3}
            ]
            )

        l_start.append(mapper)
        l_start.append(mover)
        l_start.append(monte)



    globa = Node(
        package='global_Map',
        executable='global_Map',
        name='global',
        output='screen',
        parameters=[
            {'numDrones': number_drones}
        ]
    )

    coor = Node(
        package='multi_coordination',
        executable='multi_coordination',
        name='coord',
        output='screen',
        parameters=[
            {'numDrones': number_drones},
            {'trials': trials}
        ]
    )

    rviz_config_path = os.path.join(
        'config',
        'sim_mapping2.rviz')

    rviz = Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{
                "use_sim_time": True
            }]
            )

    l_start.append(globa)
    l_start.append(coor)
    l_start.append(rviz)

    return LaunchDescription(l_start)
