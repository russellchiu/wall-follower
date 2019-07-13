#!/usr/bin/env python

import numpy as np
import sys, math, random, copy
import rospy, copy, time
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class PotentialField:
SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")

def __init__(self):
    self.data = None
    self.cmd = AckermannDriveStamped()

    #write your publishers and subscribers here; they should be the same as the wall follower's
    self.angle = 0
    self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan_callback, queue_size=1)
    self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
    #cartesian points -- to be filled (tuples)
    self.cartPoints = None

    #[speed, angle]
    self.finalVector = [0.5, 0]

def scan_callback(self, data):
    '''Checks LIDAR data'''
    self.data = data.ranges
    self.cartPoints = [None for x in range(len(self.data))]
    self.convertPoints(self.data)
    self.finalVector = self.calcFinalVector()
    self.drive_callback()

def drive_callback(self):
    '''Publishes drive commands'''
    self.cmd.drive.speed = self.finalVector[0]
    self.cmd.drive.steering_angle = self.finalVector[1]
    self.drive_pub.publish(self.cmd)

def convertPoints(self, points):
    '''Convert all current LIDAR data to cartesian coordinates'''
    arr = []
    for i in range(len(points)):
        arr.append(2.0 / (points[i]) ** 2)
    self.cartPoints = list((arr[i] * math.cos(math.radians(i * 270 / (len(arr)-1) - 45)), arr[i] * math.sin(math.radians(i * 270 / (len(arr)-1) - 45))) for i in range(len(arr)))
    #print "[" + str(self.cartPoints[0]) + ", " + str(self.cartPoints[1]) + "]"

def calcFinalVector(self):
    '''Calculate the final driving speed and angle'''
    x , y = 0, 50
    for (i, j) in self.cartPoints:
        x -= i
        y -= j

    angle = (math.atan2(y, x) - math.radians(90)) / math.pi
    #print "x: " + str(x) + "y: " + str(y)
    #return [math.sqrt(x**2 + y**2) / max(self.data), angle]
    #return [min([j for (i, j) in self.cartPoints[int((5.0/12) * (len(self.cartPoints)-1)):int((7.0/12) * (len(self.cartPoints)-1))]]), angle]
    return [1, angle]

if __name__ == "__main__":
rospy.init_node('potential_field')
potential_field = PotentialField()
rospy.spin()
