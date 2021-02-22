#!/usr/bin/env python

from __future__ import print_function
import rospy
from std_msgs.msg import String
from turtlesim.srv import SetPen,SetPenResponse
from turtlesim.msg import Num


def handle_color(req):
    retuple = (req.r,req.g,req.b,req.width,req.off) # request's value assign to retuple
    rospy.loginfo(retuple) #Print to screen retuple 
    rospy.set_param('color_params',retuple) # retuple set to the color_params parameter.
    return SetPenResponse(retuple) # ros service returns the response with retuple

def change_color():
    rospy.init_node('change_color_server') #initialize node
    retuple = (255,0,0,2,0) # set default value of the color of the turtle.
    rospy.set_param('color_params',retuple) # retuple set to the color_params parameter.

    s = rospy.Service('change_color' , SetPen , handle_color) # This declares a new service named change_color with the SetPen service type. 
                                                              # All requests are passed to handle_color function.
    rospy.spin() # simply keeps python from exiting until this node is stopped

if __name__ == "__main__": #init point
    change_color()