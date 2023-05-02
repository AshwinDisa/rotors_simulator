#!/usr/bin/env python3

import rospy
import math

class main():

    def __init__(self):

        self.runway_width = 150.0
        self.drones = 6
        self.fov = 45
        self.overlap = 0.5
        self.pos_array = []

    def func(self):

        patch_width = self.runway_width / self.drones
        patch_overlap = patch_width * self.overlap

        side_width = patch_width / 2

        for x in range(self.drones):

            self.pos_array.append(side_width + patch_width * x)

        height = side_width / math.tan(self.fov/2 * math.pi / 180)
        print("height = ", height)
        print(self.pos_array)

if __name__ == '__main__':

    try:
        rospy.init_node('main_node')

        main_obj = main()
        main_obj.func()
    
    except rospy.ROSInterruptException:
        pass