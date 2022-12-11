#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

class follower():

    def __init__(self):

        self.desired_x = 0.0
        self.desired_y = 0.0
        self.desired_z = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self.displacement = 0.0
        
        # subscriber
        rospy.Subscriber('/firefly1/ground_truth/pose', 
                            Pose, self.state_callback)

        rospy.Subscriber('/firefly2/ground_truth/pose', 
                            Pose, self.state_callback_self)

        # publisher
        self.publisher = rospy.Publisher('/firefly2/command/pose',
        PoseStamped, queue_size = 10)

    
    def state_callback(self, data):

        self.desired_x = data.position.x
        self.desired_y = data.position.y
        self.desired_z = data.position.z

    def state_callback_self(self, data_self):

        self.current_x = data_self.position.x
        self.current_y = data_self.position.y
        self.current_z = data_self.position.z

    def control(self):

        while True:

            self.displacement = math.sqrt(pow((self.desired_x - self.current_x),2) + pow((self.desired_y - self.current_y),2)
                                                                                 + pow((self.desired_z - self.current_z),2))
            print(self.displacement)

            self.publish()

    def publish(self):

        pose_msg = PoseStamped()
        pose_msg.pose.position.x = self.desired_x
        pose_msg.pose.position.y = self.desired_y
        pose_msg.pose.position.z = self.desired_z

        self.publisher.publish(pose_msg)


if __name__ == '__main__':

    try:

        rospy.init_node('follower_node')
        rate = rospy.Rate(10)

        follow = follower()
        follow.control()

        rospy.spin()

    except ROSInterruptException:

        pass
