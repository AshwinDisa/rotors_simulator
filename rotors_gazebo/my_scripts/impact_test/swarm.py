#!/usr/bin/env python3
import rospy
import numpy as np
import std_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Transform, Twist
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
import time
import tf.transformations
import math
import modern_robotics as mr

class controller():

    def __init__(self):

        self.error = 0.7

        rospy.Subscriber('/pelican3/odometry_sensor1/pose', 
                            Pose, self.state_callback)

    def control(self):

        rate = rospy.Rate(100)

        print("trajectory mode")

        while not rospy.is_shutdown():
            
            trajectory_mode.control()
            rate.sleep()

        
    def state_callback(self, data):

        self.current_x = data.position.x
        self.current_y = data.position.y
        self.current_z = data.position.z

class straight_trajectory():

    def __init__(self, trajectory, timegap, Xstart, Xend, samplingTime, Tf):

        self.trajectory = trajectory
        self.timegap = timegap
        self.time = 0.0
        self.Tf = Tf
        self.count = 0
        self.samplingTime = samplingTime

        self.pose_publisher_1 = rospy.Publisher('/pelican1/command/pose', 
                                        PoseStamped, queue_size = 10)

        self.pose_publisher_2 = rospy.Publisher('/pelican2/command/pose', 
                                        PoseStamped, queue_size = 10)

        self.pose_publisher_3 = rospy.Publisher('/pelican3/command/pose', 
                                        PoseStamped, queue_size = 10)

        self.pose_publisher_4 = rospy.Publisher('/pelican4/command/pose', 
                                        PoseStamped, queue_size = 10)
        
        self.pose_publisher_5 = rospy.Publisher('/pelican5/command/pose', 
                                        PoseStamped, queue_size = 10)                                                                                
    def control(self):

        pose = self.trajectory[self.count]
        desired_x = pose[0][3]
        desired_y = pose[1][3]
        desired_z = pose[2][3]

        desired_position = np.mat(np.array([desired_x, desired_y, desired_z]))

        self.publisher(desired_x, desired_y, desired_z)

        self.count += 1

        if(self.count >= len(self.trajectory)):
            
            rospy.signal_shutdown("completed")

    def publisher(self, desired_x, desired_y, desired_z):

        pose_msg_1 = PoseStamped()
        pose_msg_2 = PoseStamped()
        pose_msg_3 = PoseStamped()
        pose_msg_4 = PoseStamped()
        pose_msg_5 = PoseStamped()

        pose_msg_1.pose.position.x = -4
        pose_msg_2.pose.position.x = -2
        pose_msg_3.pose.position.x = 0
        pose_msg_4.pose.position.x = 2
        pose_msg_5.pose.position.x = 4

        pose_msg_1.pose.position.y = desired_y
        pose_msg_2.pose.position.y = desired_y
        pose_msg_3.pose.position.y = desired_y
        pose_msg_4.pose.position.y = desired_y
        pose_msg_5.pose.position.y = desired_y

        pose_msg_1.pose.position.z = 2
        pose_msg_2.pose.position.z = 2
        pose_msg_3.pose.position.z = 2
        pose_msg_4.pose.position.z = 2
        pose_msg_5.pose.position.z = 2

        self.pose_publisher_1.publish(pose_msg_1)
        self.pose_publisher_2.publish(pose_msg_2)
        self.pose_publisher_3.publish(pose_msg_3)
        self.pose_publisher_4.publish(pose_msg_4)
        self.pose_publisher_5.publish(pose_msg_5)
     
if __name__ == '__main__':

    try:

        rospy.init_node('drone_node')

        traj_initial_pos = np.array([0, 0, 2])
        traj_final_pos = np. array([0, 40, 2])

        Xstart = np.zeros((4, 4))
        Xend = np.zeros((4, 4))

        Xstart[:3, 3] = traj_initial_pos
        Xend[:3, 3] = traj_final_pos

        # straight line trajectory
        Tf = 30                                # time to reach from start to end position
        samplingTime = 1/100                    # sampling time in seconds
        N = int(Tf/samplingTime)                # number of samples
        method = 5                              # interpolation method
        trajectory = mr.CartesianTrajectory(
                Xstart, Xend, Tf, N, method)    # get the trajectory
        timegap = Tf / (N - 1.0)


        trajectory_mode = straight_trajectory(
            trajectory, timegap, Xstart, Xend, samplingTime, Tf)

        drone_controller = controller()
        drone_controller.control()
        
        rospy.spin()
    
    except rospy.ROSInterruptException:
        pass