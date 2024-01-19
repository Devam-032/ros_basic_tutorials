#!/usr/bin/python3
import rospy
import math
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
theta,theta1,sub , a , x_init , y_init,x_goal,y_goal,theta2,yaw,b,flag1,flag2,flag3 = 0,0,0,0,0,0,0,0,0,0,0,0,0,0
# a = 0 jab arrow detect ho and align na hogya ho
# a = 1 jab align hogya ho
def callback(data):
    global  theta , a , theta1 , sub , x_init, y_init, x_goal,y_goal,theta2,yaw,b,flag1,flag2,flag3
    if(b==0):
        yaw = theta
        b = 1
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    theta2 = (math.atan2(y_goal-y_init, x_goal-x_init)) - (theta) # this is angle of deviation of arrow from center line of video feed (ranges from between -pi se pi)
    if(theta2>math.pi):
        theta2= theta2-2*math.pi
    elif(theta2<-math.pi):
        theta2= 2*math.pi+theta2
    print(theta2,a)
    y1 , y2 , center=min_range_index(data.ranges)
    z = Twist()
    if(a == 0):
        if(flag3==0):
            flag2=0
        angular_velocity= (flag3*theta2-(1-flag3)*(theta-yaw))/5 #this is for orienting rover such that arrow is at center line of video feed
        linear_velocity = .1
        if(center>3.0): # when distance between rover and arrow is 3-3.5 =0.5
            angular_velocity = y1-y2 #this is for making rover perpendicular of arrow plan (y1 and y2 are sum of left and right indices(if y1-y2 =0 left and right side have same amount of abstacle)
            linear_velocity = 0
            if(abs(y1-y2)<0.01 ):
                linear_velocity = 0
                angular_velocity = 0
                z.linear.x = linear_velocity
                z.angular.z = angular_velocity
                i=1
                now = rospy.Time.now()
                now_sec=now.to_sec()
                while(i==1):
                    now = rospy.Time.now()
                    nowsec = now.to_sec()
                    delay = rospy.Duration.from_sec(10)  # One minute and one tenth of a second
                    seconds = delay.to_sec() 
                    print(nowsec-now_sec,seconds)
                    if(nowsec-now_sec>seconds):
                        print("bye")
                        i=0
                    pub.publish(z)
                    
                a =1 # after this point rover will start to become parllel to arrow
                theta1 = theta + math.pi/2
                pub.publish(z)
        z.linear.x = linear_velocity
        z.angular.z = angular_velocity
        pub.publish(z)
    if(a==1):
        angular_velocity = (theta1 - theta ) #rover will start to become parllel to arrow
        print(angular_velocity)
        linear_velocity = 0
        if(angular_velocity<0.05):
            angular_velocity= 0
            a=0 #at this point rover is completely parrallel to arrow
            pub.publish(z)
            x_goal = float((input('Enter the goal x:')))
            y_goal = float((input('Enter the goal y:')))

        z.linear.x = linear_velocity
        z.angular.z = angular_velocity
        pub.publish(z)
        
def min_range_index(ranges):
    rangess = [x for x in ranges]
    for i in range(len(rangess)):
        if (math.isnan(rangess[i])):
            rangess[i]=3.5
        elif (rangess[i]>3.5):
            rangess[i]=3.5
    #print(rangess , len(rangess))
    b = 3.5
    y1 = [(b - rangess[i])*i for i in range (1 , 5)]
    y2 = [(b-rangess[len(rangess)-i])*i for i in range (1 , 5)] 
    center = b -rangess[0]
    y11 = sum(y1)
    y22 = sum(y2)
    #print(y,len(y))
    #print(y11, y22 , center )
    return(y11, y22 , center )

def poseCallback1(msg):
    global theta , x_init, y_init
    x_init=msg.pose.pose.position.x
    y_init=msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, theta) = euler_from_quaternion(orientation_list)

def main():
    global  sub, theta,x_goal, y_goal
    x_goal = float((input('Enter the goal x:')))
    y_goal = float((input('Enter the goal y:')))
    sub = rospy.Subscriber('odom', Odometry, poseCallback1)

if __name__ == '__main__':
    rospy.init_node('scan_node', anonymous=True)
    main()
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()