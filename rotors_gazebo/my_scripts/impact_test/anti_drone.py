#!/usr/bin/env python3
import rospy
import numpy as np
import std_msgs.msg
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Transform, Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
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

        rospy.Subscriber('/firefly2/ground_truth/pose', 
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

                print("Hover completed")
                trajectory_mode.control()
        
    def state_callback(self, data):

        self.current_x = data.position.x
        self.current_y = data.position.y
        self.current_z = data.position.z

class hover():

    def __init__(self, hover_position):

        self.hover_x = hover_position[0]
        self.hover_y = hover_position[1]
        self.hover_z = hover_position[2]

        self.pose_publisher = rospy.Publisher('/firefly2/command/pose',
                                    PoseStamped, queue_size = 10)

    def publisher(self):

        pose_msg = PoseStamped()
        pose_msg.pose.position.x = self.hover_x
        pose_msg.pose.position.y = self.hover_y
        pose_msg.pose.position.z = self.hover_z

        self.pose_publisher.publish(pose_msg)

class trajectory():

    def __init__(self, hover_position):

        self.drone_current_x = 0.0
        self.drone_current_y = 0.0
        self.drone_current_z = 0.0

        self.anti_drone_current_x = 0.0
        self.anti_drone_current_y = 0.0
        self.anti_drone_current_z = 0.0

        self.drone_current_vel_x = 0.0
        self.drone_current_vel_y = 0.0
        self.drone_current_vel_z = 0.0

        self.time_to_impact = 2.0

        self.time_current = 0.0
        self.time_prev = 0.0

        self.desired_z = 2.0

        self.max_speed = 4.0

        self.x_i_new = hover_position[0]
        self.y_i_new = hover_position[1]
        
        
        self.anti_pose_publisher = rospy.Publisher('/firefly2/command/trajectory', 
                                        MultiDOFJointTrajectory, queue_size = 10)

        rospy.Subscriber('/firefly1/ground_truth/odometry', Odometry, self.drone_odom_callback)
        rospy.Subscriber('/firefly2/ground_truth/odometry', Odometry, self.anti_drone_odom_callback)

    def control(self):

        rate = rospy.Rate(100)

        while not rospy.is_shutdown():

            x_j = self.drone_current_x
            x_i = self.anti_drone_current_x
            y_j = self.drone_current_y
            y_i = self.anti_drone_current_y
            v_j = math.sqrt(pow((self.drone_current_vel_x),2) + 
                                            pow((self.drone_current_vel_y),2)
                                            + pow((self.drone_current_vel_z),2))

            phi = math.atan(y_j/x_j)

            v_i_cos_theta = (x_j - x_i + v_j * math.cos(phi) * self.time_to_impact) / self.time_to_impact
            v_i_sin_theta = (y_j - y_i + v_j * math.sin(phi) * self.time_to_impact) / self.time_to_impact

            # print(self.time_to_impact) 

            theta = math.atan(v_i_sin_theta / v_i_cos_theta)

            v_i = v_i_sin_theta / math.sin(theta)

            # print(theta*180/math.pi, v_i)
            # print(v_j)

            self.time_current = rospy.get_time()

            if (v_i < self.max_speed):

                self.v_i_x = v_i * math.cos(theta)
                self.v_i_y = v_i * math.sin(theta)

            else:

                self.v_i_x = self.max_speed * math.cos(theta)
                self.v_i_y = self.max_speed * math.sin(theta)

            # x_i_new = v_i_x / (self.time_current - self.time_prev)
            # y_i_new = v_i_y / (self.time_current - self.time_prev)

            self.x_i_new = self.x_i_new + self.v_i_x * 0.01  
            self.y_i_new = self.y_i_new + self.v_i_y * 0.01
            
            # print(v_i, x_i_new, y_i_new)
            # print("time ", self.time_current - self.time_prev)
            # print("x_i, y_i ", self.x_i_new, self.y_i_new)


            self.publisher(self.x_i_new, self.y_i_new)

            self.time_prev = self.time_current

            rate.sleep()

    def publisher(self, x_i_new, y_i_new):

        traj_msg = MultiDOFJointTrajectory(points=[], joint_names=None, header=None)
        
        transform_msg = Transform()

        transform_msg.translation.x = x_i_new
        transform_msg.translation.y = y_i_new
        transform_msg.translation.z = self.desired_z
        
        vel_msg = Twist()
        
        acceleration_msg = Twist()
          
        traj_point = MultiDOFJointTrajectoryPoint(transforms = [transform_msg], velocities = [vel_msg], 
                                        accelerations = [acceleration_msg], time_from_start = rospy.Time())

        traj_msg.points.append(traj_point)

        self.anti_pose_publisher.publish(traj_msg)

    def drone_odom_callback(self, drone_data):

        self.drone_current_x = drone_data.pose.pose.position.x
        self.drone_current_y = drone_data.pose.pose.position.y
        self.drone_current_z = drone_data.pose.pose.position.z

        self.drone_current_vel_x = drone_data.twist.twist.linear.x
        self.drone_current_vel_y = drone_data.twist.twist.linear.y
        self.drone_current_vel_z = drone_data.twist.twist.linear.z

    def anti_drone_odom_callback(self, anti_drone_data):

        self.anti_drone_current_x = anti_drone_data.pose.pose.position.x
        self.anti_drone_current_y = anti_drone_data.pose.pose.position.y
        self.anti_drone_current_z = anti_drone_data.pose.pose.position.z

if __name__ == '__main__':

    try:

        rospy.init_node('anti_drone_node')

        hover_position = [3, -5, 3]

        hover_mode = hover(hover_position)
        trajectory_mode = trajectory(hover_position)

        drone_controller = controller(hover_position)
        drone_controller.control()
        
        rospy.spin()
    
    except rospy.ROSInterruptException:
        pass