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

    def __init__(self, hover_position):

        self.hover_x = hover_position[0]
        self.hover_y = hover_position[1]
        self.hover_z = hover_position[2]

        self.error = 0.1

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        rospy.Subscriber('/firefly1/ground_truth/pose', 
                            Pose, self.state_callback)

    def control(self):

        rate = rospy.Rate(100)

        while not rospy.is_shutdown():

            displacement = math.sqrt(pow((self.hover_x - self.current_x),2) + 
                                        pow((self.hover_y - self.current_y),2)
                                        + pow((self.hover_z - self.current_z),2))
            
            if (displacement > self.error):
                
                hover_mode.publisher()
                rate.sleep()

            else:

                while not rospy.is_shutdown():    
                    
                    trajectory_mode.control()
                    rate.sleep()

        
    def state_callback(self, data):

        self.current_x = data.position.x
        self.current_y = data.position.y
        self.current_z = data.position.z

class hover():

    def __init__(self, hover_position):

        self.hover_x = hover_position[0]
        self.hover_y = hover_position[1]
        self.hover_z = hover_position[2]

        self.pose_publisher = rospy.Publisher('/firefly1/command/pose',
                                    PoseStamped, queue_size = 10)

    def publisher(self):

        pose_msg = PoseStamped()
        pose_msg.pose.position.x = self.hover_x
        pose_msg.pose.position.y = self.hover_y
        pose_msg.pose.position.z = self.hover_z

        self.pose_publisher.publish(pose_msg)

class straight_trajectory():

    def __init__(self, trajectory, timegap, Xstart, Xend, samplingTime, Tf):

        self.trajectory = trajectory
        self.timegap = timegap
        self.time = 0.0
        self.Tf = Tf
        self.count = 0
        self.samplingTime = samplingTime

        self.vel_publisher = rospy.Publisher('/firefly1/command/trajectory', 
                                        MultiDOFJointTrajectory, queue_size = 10)


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

        traj_msg = MultiDOFJointTrajectory(points=[], joint_names=None, header=None)
        
        transform_msg = Transform()

        transform_msg.translation.x = desired_x
        transform_msg.translation.y = desired_y
        transform_msg.translation.z = desired_z
        
        vel_msg = Twist()
        
        acceleration_msg = Twist()
          
        traj_point = MultiDOFJointTrajectoryPoint(transforms = [transform_msg], velocities = [vel_msg], 
                                        accelerations = [acceleration_msg], time_from_start = rospy.Time())

        traj_msg.points.append(traj_point)

        self.vel_publisher.publish(traj_msg)


if __name__ == '__main__':

    try:

        rospy.init_node('drone_node')

        hover_position = [0, 0, 3]

        traj_initial_pos = np.array(hover_position)
        traj_final_pos = np. array([8, 8, 3])

        Xstart = np.zeros((4, 4))
        Xend = np.zeros((4, 4))

        Xstart[:3, 3] = traj_initial_pos
        Xend[:3, 3] = traj_final_pos

        Tf = 13                                  # time to reach from start to end position
        samplingTime = 1/100                    # sampling time in seconds
        N = int(Tf/samplingTime)                # number of samples
        method = 5                              # interpolation method
        trajectory = mr.CartesianTrajectory(
                Xstart, Xend, Tf, N, method)    # get the trajectory
        timegap = Tf / (N - 1.0)

        # print(len(trajectory))

        hover_mode = hover(hover_position)

        trajectory_mode = straight_trajectory(
            trajectory, timegap, Xstart, Xend, samplingTime, Tf)

        drone_controller = controller(hover_position)
        drone_controller.control()
        
        rospy.spin()
    
    except rospy.ROSInterruptException:
        pass