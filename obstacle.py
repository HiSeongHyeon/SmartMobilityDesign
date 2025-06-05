#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

class Obstacle:
    def __init__(self):  
        self.flag = -1   
        self.pre_flag = 0
        self.drive_time = 0
        self.obstacle_pos = 0



    def state(self):
        # -1: there's no obstacle, 1: obstacle in left side. 2: obstacle in right side 
        if(self.obstacle_pos==-1):  #no obstacle
            self.flag = -1
        elif(self.obstacle_pos<320):
            self.flag = 1
        elif(self.obstacle_pos>320):
            self.flag = 2
            
        if(self.pre_flag != self.flag):
            self.drive_time = rospy.Time.now
            self.pre_flag = self.flag



    def drive(self, obstacle_pos):
        self.obstacle_pos = obstacle_pos
        self.state(self)

        # obstacle in left side
        if (rospy.get_time - self.drive_time >= 0 & self.flag == 1):
            angle = 10
            speed = 5
            return angle, speed
    
        # obstacle in right side
        elif (rospy.get_time - self.drive_time >= 0 & self.flag == 2):
            angle = -10
            speed = 5
            return angle, speed
    