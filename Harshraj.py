#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist 
from turtlesim.msg import Pose
from math import sqrt, pow, atan2

def pose_twist_callback(pose:Pose):
    global dis,angle_diff
    pose.x = round(pose.x,4)
    pose.y = round(pose.y,4)
    dis = sqrt(pow((goal.x-pose.x),2)+pow((goal.y-pose.y),2))
    angle = atan2(goal.y-pose.y,goal.x-pose.y)
    angle_diff = angle-pose.theta  
    
if __name__ == '__main__': 
    rospy.init_node('gtg', anonymous=True) 
    print('started')
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/turtle1/pose', Pose, callback = pose_twist_callback)
    goal  = Pose()
    pose  = Pose()
    goal.x = float(input('Enter x coordinate:'))
    goal.y = float(input('Enter y coordinate:'))
    vel = Twist()
    while dis>0.1:
      vel.linear.x = dis*.5
      vel.angular.z = angle_diff*2
      pub.publish(vel)
      rospy.loginfo(dis)
    #   rospy.loginfo(angle_diff)
    #   rospy.loginfo(vel)
      if(dis<1):
            vel.linear.x = 0
            vel.angular.z = 0
            pub.publish(vel) 
            break
    rospy.spin()