#!/usr/bin/python3
import time
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

distance, distance_current , sub , sub1, x_init , y_init ,x11= 0, 0, 0, 0, 0, 0 ,0


def poseCallback():
    global  distance_current,sub ,distance,x11,x_init
    x11 = x_init    #initial_position
    distance_current=0
    print(distance)
    while(distance-distance_current>0.2):
        x=x_init    #current_postition
        # theta = pose.theta
        distance_current = x - x11
        print("distance:")
        print(distance_current,distance)
        linear_velocity = (distance - distance_current)
        angular_velocity = 0
        pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        z = Twist()
        z.linear.x = linear_velocity
        z.angular.z = angular_velocity
        pub.publish(z)


def poseCallback1(pose):
    global x_init , y_init , distance , distance_current,x11
    x_init = pose.x
    distance_current = 0

def main():
    global distance,  sub , sub1 , x_init , y_init, distance_current
    distance=float((input('Enter the distance you want to move:')))
    while not x_init!=0:
        sub=rospy.Subscriber('/turtle1/pose',Pose,poseCallback1)
    sub=rospy.Subscriber('/turtle1/pose',Pose,poseCallback1)
    print(x_init)
    poseCallback()



if __name__ == "__main__":
    rospy.init_node('turtlesim_motion_pose')
    main()
    rospy.spin()