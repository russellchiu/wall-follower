#!/usr/bin/env python
import numpy as np
import rospy
import math
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
class WallFollower:
     # Import ROS parameters from the "params.yaml" file.
     # Access these variables in class functions with self:
     # i.e. self.CONSTANT
     SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
     DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
     SIDE = rospy.get_param("wall_follower/side")
     VELOCITY = rospy.get_param("wall_follower/velocity")
     DESIRED_DISTANCE = rospy.get_param("wall_follower/desired_distance")
     def __init__(self):
         # Initialize your publishers and
         # subscribers
         self.data = None
         self.angle = 0
         self.cmd = AckermannDriveStamped()
         self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.scan, queue_size=1)
         self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=1)
     def scan(self, data):
     #stores the lidar data so you can work with it
         self.data = data
     #calls function that controls driving
         self.drive()

     def drive(self):
         """controls driving"""
         #Algorithm for driving
         #gets the angle required
         self.angle = self.find_wall()
         #sets speed and driving angle
         self.cmd.drive.speed = self.VELOCITY
         self.cmd.drive.steering_angle = self.angle
         #publishes the command
         self.drive_pub.publish(self.cmd)
     def find_wall(self):
         # if lidar data has not been received, do nothing
         if self.data == None:
             return 0
         ## TO DO: Find Alg for Wall Following ##
         """Lidar data is now stored in self.data, which can be accessed
         using self.data.ranges (in simulation, returns an array).
         Lidar data at an index is the distance to the nearest detected object
         self.data.ranges[0] gives the leftmost lidar point
         self.data.ranges[99] gives the rightmost lidar point
         self.data.ranges[50] gives the forward lidar point
         """
         def error(arr):
             rad_index = math.radians(135) / arr.size
             return arr - self.DESIRED_DISTANCE / (np.cos(arr * rad_index - 45))
         tempAngle = 0
         lidarPoints = np.array([])
         listLen = len(self.data.ranges)
         if self.SIDE is -1:
             lidarPoints = np.array(self.data.ranges[:listLen / 2])
         elif self.SIDE is 1:
             lidarPoints = np.array(self.data.ranges[listLen / 2::-1])
         else:
             lidarPoints = np.array()
             print("ERROR")
         wallAngle = 70
         theta1 = wallAngle
         theta2 = wallAngle / 10
         #e1 is the point in front(approx 45 degrees); e2 is the point behind (approx perpendicular to wall)
         err = error(lidarPoints)
         e1 = np.mean(err[:listLen/4])
         e2 = np.mean(err[listLen/4:listLen/2])

         if e1 > 0 and e2 > 0:
             tempAngle = -theta1
         if e1 > 0 and e2 < 0:
             tempAngle = -theta2
         if e1 < 0 and e2 > 0:
             tempAngle = theta2
         if e1 < 0 and e2 < 0:
             tempAngle = theta1
         #returns the output of your alg, the new angle to drive in
         #tempAngle = 45
         return tempAngle * self.SIDE * -1

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()
