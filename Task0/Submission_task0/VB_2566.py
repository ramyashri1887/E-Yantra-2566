#!/usr/bin/env python

#Importing the necessary libraries
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
PI = 3.14159265358979323846


def pose_callback(msg):
	#Print the values of the x,y,theta of the Turtle:
    print "x: ",msg.x, "y: ",msg.y,"theta: ",msg.theta

def main():

	#Creating a node with name 'turtle_revolve' and making sure it is a unique node 
	rospy.init_node('turtle_revolve', anonymous=True)

	#The publisher will publish to the topic '/turtle1/cmd_vel'
	#(The queue_size should not be too large and must be sufficiently big)
	#Queue_size 10 and 1 gave similar results,
	#but for some values of queue_size in between 1 and 10, the results weren't much good
	#And the results were differing sometimes when we had run it on the same system
	#Setting queue_size of 10 is much safer than 1
	velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size= 10)
	vel_msg = Twist()

	#speed = (radius)*(angular velocity)
	#Setting the values of velocities:
	vel_msg.linear.x=1 #speed (linear speed in x direction in m/s)
	vel_msg.linear.y=0
	vel_msg.linear.z=0
	vel_msg.angular.x=0
	vel_msg.angular.y=0
	vel_msg.angular.z=1 #rotation(angular velocity in z direction in rad/s)


	#Subscriber to the topic '/turtle1/pose'
	#pose_callback is called when a message of type Pose is received
	pose_subscriber = rospy.Subscriber("/turtle1/pose", Pose, pose_callback)

	#Calculate the circumference of the circle
	#(As the speed is 1 and anguar velocity is 1, radius is 1) 
	distance = 2*PI*1

	#Initialising the current distance to 0
	current_distance= 0

	#Printing the initial coordinates
	print("The initial coordinates are: ")
	print("x: ",5.544444561,"y: ",5.544444561,"theta: ",0.0)	
	print("The turtle has started moving")

	#Setting the rate to 3 Hz 
	#We tried to run this at different rates and queue_size combinations.
	#This rate worked the best in our system
	#This can vary for different systems
	r = rospy.Rate(3)

	#Get the current time which will be the initial time
	t0= rospy.Time.now().to_sec()
	
	#Using a while loop for the turtle to move until it covers a distance equal to length of 1 circle:
	while(current_distance <= distance):

		#Get the current time
		t1=rospy.Time.now().to_sec()

		#We know that distance= (speed)*(time)
		#The speed is 1 here and the time is the time difference, (t1-t0)
		current_distance= (t1-t0)

		#Publishing the vel_msg
		velocity_publisher.publish(vel_msg)

		#For the node to publish at the desired rate:
		r.sleep()
		#r.sleep() can throw a rospy.ROSInterruptException if the sleep is interrupted by shutdown

	
	#Stopping the turtle after covering 1 round of the circle
	vel_msg.linear.x=0
	vel_msg.angular.z=0
	velocity_publisher.publish(vel_msg)
	print("The turtle has stopped moving after 1 round of circle")



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

