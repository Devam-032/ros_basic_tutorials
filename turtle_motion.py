#!/usr/bin/python3
import rospy,math,time
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
x=0
y=0


def move_sl(velocity_pub,speed,distance,is_forward):
    velocity_msg=Twist()
    global x,y
    x0=x
    y0=y
    rospy.loginfo(f"x:{x} and y:{y} and x0:{x0} and y0:{y0}")
    if (is_forward==1):
        velocity_msg.linear.x=abs(speed)
    else:
        velocity_msg.linear.x=-abs(speed)
    
    distance_moved=0.0
    loop_rate=rospy.Rate(1)
    while not rospy.is_shutdown():
        rospy.loginfo("Turtle moves forward")
        velocity_pub.publish(velocity_msg)
        loop_rate.sleep()
        distance_moved=abs(math.sqrt((x-x0)**2+(y-y0)**2))
        print (distance_moved)
        if (abs(distance_moved-distance)<0.05):
            rospy.loginfo('reached')
            i1=int(input('enter the number for the motion you want to perform:\n1)Straight line\n2)Rotation\n3)G2G\n'))
            selector(i1)
    velocity_msg.linear.x=0
    velocity_pub.publish(velocity_msg)
def posecallback(pose_message):
    global x,y,yaw
    x=pose_message.x
    y=pose_message.y
    yaw=pose_message.theta
def rotate(velocity_pub,angle_speed_degree,relative_angle_degree,clockwise):
    velocity_msg=Twist()
    angular_speed=math.radians(abs(angle_speed_degree))
    if(clockwise==1):
        velocity_msg.angular.z=-abs(angular_speed)
    else:
        velocity_msg.angular.z=abs(angular_speed)
    loop_rate=rospy.Rate(1)
    t0=rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        rospy.loginfo('Turtlesim rotates:')
        velocity_pub.publish(velocity_msg)
        t1=rospy.Time.now().to_sec()
        current_angle_degree=(t1-t0)*angle_speed_degree
        loop_rate.sleep()
        if (current_angle_degree>relative_angle_degree):
            rospy.loginfo('turn completed') 
            i2=int(input('enter the number for the motion you want to perform:\n1)Straight line\n2)Rotation\n3)G2G\n'))
            selector(i2)
    velocity_msg.angular.z=0
    velocity_pub.publish(velocity_msg)
def g2g(velocity_pub,x_coor,y_coor):
    global x,y,yaw
    velocity_msg=Twist()
    loop_rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo('Turtle is moving towards goal')
        k_linear=0.56
        distance=abs(math.sqrt(((x-x_coor)**2)+((y-y_coor)**2)))
        linear_speed=k_linear*distance
        k_angular=3.9
        desired_angle_goal=math.atan2((y_coor-y),(x_coor-x))
        angular_speed=k_angular*(desired_angle_goal-yaw)
        velocity_msg.linear.x=linear_speed
        velocity_msg.angular.z=angular_speed
        velocity_pub.publish(velocity_msg)
        print(f'x={x} and y={y} and current yaw={yaw} and distance to goal={distance}')
        loop_rate.sleep()
        if(distance<0.01):
            i3=int(input('enter the number for the motion you want to perform:\n1)Straight line\n2)Rotation\n3)G2G\n'))
            selector(i3)

def selector(i):
    if(i==1):
        x1=float(input('Enter the distance the turtle must travel:'))
        y1=float(input('Enter the velocity it must have:'))
        z_1=int(input('Direction of motion must be:\n1)Forward\n2)Backward\n'))
        move_sl(velocity_pub,y1,x1,z_1)
    elif(i==2):
        z1=float(input('Enter the rotation angle of the bot:'))
        z11=float(input('Enter the turning speed of the bot:'))
        z_1=int(input('Sense of rotation:\n1)Clockwise\n2)Anti-clockwise\n'))
        rotate(velocity_pub,z11,z1,z_1)
    elif(i==3):
        x11=float(input('Enter the x co-ordinates of the goal:'))
        y11=float(input('Enter the y co-ordinates of the goal:'))
        g2g(velocity_pub,x11,y11)
if __name__=='__main__':    
        try:
            i=int(input('enter the number for the motion you want to perform:\n1)Straight line\n2)Rotation\n3)G2G\n'))
            rospy.init_node('listener', anonymous=True)
            position_topic='/turtle1/pose'
            pose_subscriber=rospy.Subscriber(position_topic,Pose,posecallback)
            cmd_vel_pub='/turtle1/cmd_vel'
            velocity_pub=rospy.Publisher(cmd_vel_pub,Twist,queue_size=10)
            while x == 0 or y == 0 and not rospy.is_shutdown():
                rospy.loginfo("Waiting for initial position update...")
                rospy.sleep(0.1)
            selector(i)
            time.sleep(1)
        except rospy.ROSInterruptException:
            pass
