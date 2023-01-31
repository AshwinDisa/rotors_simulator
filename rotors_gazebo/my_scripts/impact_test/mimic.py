#! /usr/bin/env python3

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
import modern_robotics as mr
import tf

class controller(): 

    def __init__(self, hover_position_2, hover_position_3):

        self.error = 0.1

        self.flag = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self.current_state = State()

        rospy.Subscriber('/pelican2/odometry_sensor1/pose',
                            PoseStamped, self.position_callback)

    def control(self):

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            trajectory_mode.control()
            rate.sleep()

    def state_callback(self, state_msg):

        self.current_state = state_msg

    def position_callback(self, data):

        self.current_x = data.position.x
        self.current_y = data.position.y
        self.current_z = data.position.z

class trajectory():

    def __init__(self, hover_position_2, hover_position_3):

        self.drone_current_x = 0.0
        self.drone_current_y = 0.0
        self.drone_current_z = 0.0

        self.drone_2_current_x = 0.0
        self.drone_2_current_y = 0.0
        self.drone_2_current_z = 0.0

        self.drone_3_current_x = 0.0
        self.drone_3_current_y = 0.0
        self.drone_3_current_z = 0.0

        self.drone_previous_x = 0.0
        self.drone_previous_y = 0.0

        self.drone_current_vel_x = 0.0
        self.drone_current_vel_y = 0.0
        self.drone_current_vel_z = 0.0

        self.time_to_impact = 0.7

        self.time_current = 0.0
        self.time_prev = 0.0

        self.desired_z = 6.0

        self.looping_time = 0.01            # 100Hz

        self.displacement = 0.0

        # quaternion x,y,z,w
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 0.0

        self.x_new_2 = 0.0
        self.y_new_2 = 0.0

        self.x_new_3 = 0.0
        self.y_new_3 = 0.0

        self.x_old_2 = hover_position_2[0]
        self.y_old_2 = hover_position_2[1]

        self.x_old_3 = hover_position_3[0]
        self.y_old_3 = hover_position_3[1]    

        self.T_x = 0.0
        self.T_y = 0.0

        self.pose_publisher_2 = rospy.Publisher('/pelican2/command/pose',
                                    PoseStamped, queue_size = 10)

        self.pose_publisher_3 = rospy.Publisher('/pelican3/command/pose',
                                    PoseStamped, queue_size = 10)

        rospy.Subscriber('/pelican1/odometry_sensor1/pose',
                            Pose, self.drone_position_callback)

        rospy.Subscriber('/pelican2/odometry_sensor1/pose',
                            Pose, self.drone_2_position_callback)

        rospy.Subscriber('/pelican3/odometry_sensor1/pose',
                            Pose, self.drone_3_position_callback)

    def control(self):

        # tranformation matrix

        quaternion = [self.x, self.y, self.z, self.w]
        yaw = self.quaternion_to_euler(quaternion)

        # print(yaw*180/math.pi)

        self.x_new_2 = math.cos(yaw) * self.x_old_2 - math.sin(yaw) * self.y_old_2 + self.T_x
        self.y_new_2 = math.sin(yaw) * self.x_old_2 + math.cos(yaw) * self.y_old_2 + self.T_y

        self.x_new_3 = math.cos(yaw) * self.x_old_3 - math.sin(yaw) * self.y_old_3 + self.T_x
        self.y_new_3 = math.sin(yaw) * self.x_old_3 + math.cos(yaw) * self.y_old_3 + self.T_y

        self.T_x = self.drone_current_x
        self.T_y = self.drone_current_y

        self.drone_previous_x = self.drone_current_x
        self.drone_previous_y = self.drone_current_y

        self.publisher_2(self.x_new_2, self.y_new_2)
        self.publisher_3(self.x_new_3, self.y_new_3)

    def quaternion_to_euler(self, quaternion):

        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        return yaw

    def publisher_2(self, x_new, y_new):

        pose_msg = PoseStamped()

        pose_msg.pose.position.x = x_new
        pose_msg.pose.position.y = y_new
        pose_msg.pose.position.z = self.drone_current_z + 0.5
        # pose_msg.pose.position.z = 3

        print(x_new)

        self.pose_publisher_2.publish(pose_msg)

    def publisher_3(self, x_new, y_new):

        pose_msg = PoseStamped()

        pose_msg.pose.position.x = x_new
        pose_msg.pose.position.y = y_new
        pose_msg.pose.position.z = self.drone_current_z + 0.5
        # pose_msg.pose.position.z = 3

        self.pose_publisher_3.publish(pose_msg)

    def drone_position_callback(self, drone_data):

        self.drone_current_x = drone_data.position.x
        self.drone_current_y = drone_data.position.y
        self.drone_current_z = drone_data.position.z

        self.x = drone_data.orientation.x
        self.y = drone_data.orientation.y
        self.z = drone_data.orientation.z
        self.w = drone_data.orientation.w

    def drone_2_position_callback(self, data):

        self.drone_2_current_x = data.position.x
        self.drone_2_current_y = data.position.y
        self.drone_2_current_z = data.position.z

    def drone_3_position_callback(self, data):

        self.drone_3_current_x = data.position.x
        self.drone_3_current_y = data.position.y
        self.drone_3_current_z = data.position.z

if __name__ == "__main__":

    try:

        rospy.init_node("follower_offb_node_py")

        hover_position_2 = np.array([3, 0, 3])
        hover_position_3 = np.array([-3, 0, 3])

        trajectory_mode = trajectory(hover_position_2, hover_position_3)

        drone_controller = controller(hover_position_2, hover_position_3)
        drone_controller.control()

        rospy.spin()

    except rospy.ROSInterruptException:

        pass
