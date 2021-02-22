#!/usr/bin/env python

from __future__ import print_function
import rospy
from std_msgs.msg import String
from turtlesim.srv import ChangeTurtle,ChangeTurtleResponse

turtle_url = "https://raw.githubusercontent.com/ros/ros_tutorials/noetic-devel/turtlesim/images/box-turtle.png"
kinetic_url = 'https://raw.githubusercontent.com/ros/ros_tutorials/melodic-devel/turtlesim/images/kinetic.png'
electric_url = "https://raw.githubusercontent.com/ros/ros_tutorials/melodic-devel/turtlesim/images/electric.png"

def handle_turtle(req):
    pub = rospy.Publisher('turtle_spawn' , String , queue_size=10) #This declares a new publisher named turtle_spawn with the String message type and queue_size.
    rate = rospy.Rate(10) # the Rate instance will attempt to keep the loop at 10hz by accounting for the time used by any operations during the loop.
    while not rospy.is_shutdown():
        if(req.name == "kinetic"):
            rospy.loginfo(kinetic_url) #print to screen req's URL
            rospy.set_param('url',kinetic_url) # set request value of the turtle.
            rate.sleep() # Rate.sleep() can throw a rospy.ROSInterruptException if the sleep is interrupted by shutdown.
            return ChangeTurtleResponse(kinetic_url) # return response
        elif(req.name == "electric"):
            rospy.loginfo(electric_url) #print to screen req's URL
            rospy.set_param('url',electric_url) # set request value of the turtle.
            rate.sleep() # Rate.sleep() can throw a rospy.ROSInterruptException if the sleep is interrupted by shutdown.
            return ChangeTurtleResponse(electric_url) # return response
        else:
            print("error")
        
def change_turtle():
    rospy.init_node('change_turtle_server') #initialize node
    rospy.set_param('url',turtle_url) # set default value of the turtle.
    s = rospy.Service('change_turtle' , ChangeTurtle , handle_turtle) # This declares a new service named change_turtle with the ChangeTurtle service type
                                                                      # All requests are passed to handle_turtle function.
    rospy.spin() # simply keeps python from exiting until this node is stopped

if __name__ == "__main__": #init point
    change_turtle()