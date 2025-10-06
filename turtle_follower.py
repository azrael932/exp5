#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

turtle2_pub = None
turtle1_pose = None
turtle2_pose = None

def turtle1_pose_callback(msg):
    global turtle1_pose
    turtle1_pose = msg

def turtle2_pose_callback(msg):
    global turtle2_pose
    turtle2_pose = msg

def follow_leader():
    global turtle1_pose, turtle2_pose, turtle2_pub
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if turtle1_pose and turtle2_pose:
            twist = Twist()
            
            # Compute distance and angle to turtle1
            dx = turtle1_pose.x - turtle2_pose.x
            dy = turtle1_pose.y - turtle2_pose.y
            distance = math.sqrt(dx**2 + dy**2)
            angle_to_goal = math.atan2(dy, dx)
            
            # Simple proportional controller
            twist.linear.x = min(2.0 * distance, 2.0)
            twist.angular.z = 4.0 * (angle_to_goal - turtle2_pose.theta)
            
            turtle2_pub.publish(twist)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node('turtle_follower')
    
    turtle2_pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
    
    rospy.Subscriber('/turtle1/pose', Pose, turtle1_pose_callback)
    rospy.Subscriber('/turtle2/pose', Pose, turtle2_pose_callback)
    
    try:
        follow_leader()
    except rospy.ROSInterruptException:
        pass
