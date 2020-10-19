#!/usr/bin/env python
"""This node moves the turtle inside the turtlesim window in a circle
and stops at initial location"""

#Importing the necessary libraries
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

PI = 3.14159265358979323846


def pose_callback(msg):
    """This is the pose_callback function"""
    #This will print the values of the x,y,theta of the Turtle
    #to the screen as the loginfo also writes to stdout
    rospy.loginfo("x: %.11f, y: %.11f, theta: %.11f ", msg.x, msg.y, msg.theta)


def main():
    """This is the main function"""
    #The linear velocity in x direction and angular velocity in z direction
    #are each 1 in magnitude and are published by the publisher
    #The turtle will move until it covers a circular path of length = circumference
    #This will make the turtle complete one circle

    #Creating a node with name 'turtle_revolve' and making sure it is a unique node
    rospy.init_node('turtle_revolve', anonymous=True)

	#The publisher will publish to the topic '/turtle1/cmd_vel'

	#(The queue_size should not be too large and must be sufficiently big)
	#Queue_size 10 and 1 gave similar results,
	#But for some values of queue_size in between 1 and 10, the results weren't much good
	#And the results were differing sometimes when we had run it on the same system
	#Setting queue_size of 10 is safer than 1 so that messages shouldn't be lost
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    #Create a Twist message and add values
    vel_msg = Twist()

	#Speed = (radius)*(angular velocity)
	#Setting the values of velocities:
    vel_msg.linear.x = 1  #speed (linear speed in x direction in m/s)
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 1  #rotation(angular velocity in z direction in rad/s)


	#Subscriber to the topic '/turtle1/pose'
	#pose_callback is called when a message of type Pose is received
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)

	#Calculate the circumference of the circle
	#(As the speed is 1 and anguar velocity is 1, radius is 1)
    distance = 2*PI*1

	#Initialising the current distance to 0
    current_distance = 0

	#Printing the initial coordinates
    print "The initial coordinates are: "
    print "x: ", 5.544444561, ",y: ", 5.544444561, ",theta: ", 0.0
    print "The turtle has started moving"

    #Setting the rate to 3 Hz
	#We tried to run this at different rates and queue_size combinations.
	#This rate worked the best in our system
	#This can vary for different systems
    rate_rospy = rospy.Rate(3)

	#Get the current time which will be the initial time
    t_0 = rospy.Time.now().to_sec()

	#Using a while loop for the turtle to move until it covers a distance equal to length of 1 circle:
    while current_distance <= distance:

		#Get the current time
        t_1 = rospy.Time.now().to_sec()

		#We know that distance= (speed)*(time)
		#The speed is 1 here and the time is the time difference, (t_1-t_0)
        current_distance = (t_1-t_0)

		#Publishing the vel_msg
        velocity_publisher.publish(vel_msg)

		#For the node to publish at the desired rate:
        rate_rospy.sleep()
		#rate_rospy.sleep() can throw a rospy.ROSInterruptException if sleep is interrupted by shutdown

	#Stopping the turtle after covering 1 round of the circle
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    print"The turtle has stopped moving after 1 round of circle"



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
