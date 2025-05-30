#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from enum import Enum

class Index(Enum):
    General = 1
    Obstacle = 2
    Tunnel = 3
    Crosswalk = 4
    Stopline = 5

class Perception:    
    #카메라와 라이다 데이터로부터 구간 판단 및 구간에 따른 리턴값 반환
    #구간 별 필요한 리턴값:
    #General needs left, right line pos
    #Obstacle needs obstacle pos
    #Tunnel ?
    # - - - 
    

    def __init__(self):
        pass

