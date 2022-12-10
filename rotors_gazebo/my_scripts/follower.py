#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

class follower():

    def __init__(self):

        self.desired_x = 0.0
        self.desired_y = 0.0
        self.desired_z = 0.0
        
        # subscriber
        rospy.Subscriber('/firefly1/ground_truth/pose', 
                            Pose, self.state_callback)

        # publisher
        self.publisher = rospy.Publisher('/firefly2/command/pose',
        PoseStamped, queue_size = 10)

    
    def state_callback(self, data):

        self.desired_x = data.position.x
        self.desired_y = data.position.y
        self.desired_z = data.position.z


    def control(self):

        while True:

            follower.publish(self)

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
