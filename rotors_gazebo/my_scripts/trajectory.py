#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import time
import tf.transformations
import math

class circular_trajectory():

    def __init__(self, centre, radius, timegap, samplingTime, Tf):

        self.xc, self.yc = centre
        self.radius = radius
        self.timegap = timegap
        self.samplingTime = samplingTime
        self.Tf = Tf
        self.time = 0
        self.scale = 5

        self.desired_x = 0.0
        self.desired_y = 0.0
        self.desired_z = 0.0

        self.pose_publisher = rospy.Publisher('/firefly1/command/pose', 
                                        PoseStamped, queue_size = 10)


    def control(self, circle_traj):

        rate = rospy.Rate(100)

        while not rospy.is_shutdown():

            circle_traj.sample()
            rate.sleep()    
    
    def sample(self):

        

        # figure of 8
        t = self.scale * self.time
        self.desired_x = self.scale*math.cos(t)
        self.desired_y = self.scale*math.sin(2*t)/2

        # circle
        # t = 10*self.time % self.Tf
        # self.desired_x = self.xc + self.radius*math.cos(2*math.pi*t/self.Tf)
        # self.desired_y = self.yc + self.radius*math.sin(2*math.pi*t/self.Tf)
        
        self.desired_z = 3.0

        self.time += self.samplingTime
        time.sleep(0.1)

        self.publish(self.desired_x, self.desired_y, self.desired_z)

    def publish(self, desired_x, desired_y, desired_z):

        pose_msg = PoseStamped()
        pose_msg.pose.position.x = self.desired_x
        pose_msg.pose.position.y = self.desired_y
        pose_msg.pose.position.z = self.desired_z

        self.pose_publisher.publish(pose_msg)

if __name__ == '__main__':

    try:

        rospy.init_node('Trajectory_node')

        Tf = 10     #total time
        samplingTime = 1 / 100        #sampling time in secs
        N = int(Tf / samplingTime)        #no. of samples
        timegap = Tf / (N - 1.0)

        circle_traj = circular_trajectory([0,0], 3, timegap, samplingTime, Tf)

        circle_traj.control(circle_traj)
        
        rospy.spin()
    
    except rospy.ROSInterruptException:
        pass


