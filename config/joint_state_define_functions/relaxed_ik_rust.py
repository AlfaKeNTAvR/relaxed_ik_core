#! /usr/bin/env python
from geometry_msgs.msg import PoseStamped, Vector3Stamped, QuaternionStamped, Pose

import csv
import ctypes
import numpy
import os
import rospkg
import rospy
import sys
import utils
import transformations as T
import yaml

from relaxed_ik_ros1.msg import EEPoseGoals, JointAngles
from robot import Robot
from std_msgs.msg import Float64
from timeit import default_timer as timer
from urdf_load import urdf_load
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarkerUpdate

class Opt(ctypes.Structure):
    _fields_ = [("data", ctypes.POINTER(ctypes.c_double)), ("length", ctypes.c_int)]

path_to_src = rospkg.RosPack().get_path('relaxed_ik_ros1')
animation_folder_path = path_to_src + '/animation_files/'
env_settings_file_path = path_to_src + '/relaxed_ik_core/config/settings.yaml'

os.chdir(path_to_src + "/relaxed_ik_core")

lib = ctypes.cdll.LoadLibrary(path_to_src + '/relaxed_ik_core/target/debug/librelaxed_ik_lib.so')
lib.solve.restype = Opt

eepg = None
def eePoseGoals_cb(msg):
    global eepg
    eepg = msg

def main(args=None):
    rospy.init_node('relaxed_ik')

    # Load the infomation
    env_settings_file = open(env_settings_file_path, 'r')
    env_settings = yaml.load(env_settings_file, Loader=yaml.FullLoader)

    if 'loaded_robot' in env_settings:
        robot_info = env_settings['loaded_robot']
    else:
        raise NameError('Please define the relevant information of the robot!')

    info_file_name = robot_info['name']
    robot_name = info_file_name.split('_')[0]
    objective_mode = robot_info['objective_mode']
    print("\nRelaxedIK initialized!\nRobot: {}\nObjective mode: {}\n".format(robot_name, objective_mode))
    
    # Publishers
    angles_pub = rospy.Publisher('/relaxed_ik/joint_angle_solutions', JointAngles, queue_size=10)
    time_pub = rospy.Publisher('/relaxed_ik/current_time', Float64, queue_size=10)
    
    #!
    #ee_pose_goals_pub1 = rospy.Publisher('/relaxed_ik/ee_pose_goals', EEPoseGoals, queue_size=5)

    cur_time = 0.0
    delta_time = 0.01
    step = 1 / 30.0

    # Wait for the start signal
    print("Waiting for ROS param /simulation_time to be set as go...")
    initialized = False
    while not initialized: 
        try: 
            param = rospy.get_param("simulation_time")
            initialized = param == "go"
        except KeyError:
            initialized = False
    print("ROS param /simulation_time is set up!\n")

    global eepg
    rospy.Subscriber('/relaxed_ik/ee_pose_goals', EEPoseGoals, eePoseGoals_cb)

    '''#!
    ee_pose_goals = EEPoseGoals()
    pose_r = Pose()
    pose_r.position.x = 0.493
    pose_r.position.y = 0.320
    pose_r.position.z = 0.201

    pose_r.orientation.w = 0
    pose_r.orientation.x = 0
    pose_r.orientation.y = 0
    pose_r.orientation.z = 0
    ee_pose_goals.ee_poses.append(pose_r)
    ee_pose_goals.header.seq = 1
    ee_pose_goals_pub1.publish()
    #!'''

    while eepg == None: continue

    rate = rospy.Rate(3000)
    speed_list = []
    while not rospy.is_shutdown():
        cur_time_msg = Float64()
        cur_time_msg.data = cur_time
        time_pub.publish(cur_time_msg)
        cur_time += delta_time * step

        pose_goals = eepg.ee_poses
        header = eepg.header
        pos_arr = (ctypes.c_double * (3 * len(pose_goals)))()
        quat_arr = (ctypes.c_double * (4 * len(pose_goals)))()

        for i in range(len(pose_goals)):
            p = pose_goals[i]
            pos_arr[3*i] = p.position.x
            pos_arr[3*i+1] = p.position.y
            pos_arr[3*i+2] = p.position.z

            quat_arr[4*i] = p.orientation.x
            quat_arr[4*i+1] = p.orientation.y
            quat_arr[4*i+2] = p.orientation.z
            quat_arr[4*i+3] = p.orientation.w

        start = timer()
        xopt = lib.solve(pos_arr, len(pos_arr), quat_arr, len(quat_arr))
        end = timer()
        speed = 1.0 / (end - start)
        # print("Speed: {}".format(speed))
        speed_list.append(speed)

        ja = JointAngles()
        ja.header = header
        ja_str = "["
        for i in range(xopt.length):
            ja.angles.data.append(xopt.data[i])
            ja_str += str(xopt.data[i])
            if i == xopt.length - 1:
                ja_str += "]"
            else: 
                ja_str += ", "

        angles_pub.publish(ja)
        # print(ja_str)

        rate.sleep()

    print("Average speed: {} HZ".format(numpy.mean(speed_list)))
    print("Min speed: {} HZ".format(numpy.min(speed_list)))
    print("Max speed: {} HZ".format(numpy.max(speed_list)))
        
if __name__ == '__main__':
    main()
