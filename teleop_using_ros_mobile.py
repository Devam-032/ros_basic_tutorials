#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist
x1,x2=0,0

def callback(data):
    rospy.loginfo("%f %f %f %f %f", data.linear.x, data.linear.y,
                  data.linear.z, data.angular.x, data.angular.y)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    y1 = data.linear.z #front
    y2 = data.linear.y #left
    x1=y1
    x2=y2
    z=Twist()
    z.linear.z=(x1+2*x2)/2
    z.linear.y=(2*x2-x1)/2
    pub.publish(z)
def listener():
    rospy.init_node('listener',anonymous=True)
    rospy.Subscriber('cmd_1',Twist,callback)
    rospy.spin()
if __name__=='__main__':
    listener()
