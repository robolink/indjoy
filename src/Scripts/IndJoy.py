#!/usr/bin/env python
import roslib; 
import ros
import rospy

from sensor_msgs.msg import Joy

from std_msgs.msg import String

from xboxcontroller.src.scripts import XboxControl



IndJoyPublisher = rospy.Publisher('IndJoy',String)#, RobolinkControl)
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
CALLBACKS
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""

def joyCallback(Joy):
    
    ij_x = Joy.axes[0]
    ij_y = Joy.axes[1]
    
    if ij_x == ij_y == 1.0:
        IndJoyPublisher.publish('centered')
    #jointVelocities = array('l', [0, 0, 0, 0, 0])
    
    #if Joy.buttons[5] or Joy.buttons[4]:
        
        ##There seems to be a bug in the joy_node that causes the trigger values to show up as zero when they haven't been pressed yet.
        ##This causes a problem where joints would move without triggers being pressed.
        ##We'll ignore them if they're zero since they should be 1 by default.
        #if(Joy.axes[2] != 0 and Joy.axes[5] != 0):
            #leftTrigger = 0.5*Joy.axes[2]
            #rightTrigger = -0.5*Joy.axes[5]
            #jointVelocities[0] = int((leftTrigger+rightTrigger)*robolinkJoint.maxAbsVelocity)
            ##print jointVelocities[0]
        #else:
            #jointVelocities[0] = 0
            
        #jointVelocities[1] = int(Joy.axes[1]*robolinkJoint.maxAbsVelocity)
        #jointVelocities[2] = int(Joy.axes[0]*robolinkJoint.maxAbsVelocity)
        #jointVelocities[3] = int(Joy.axes[4]*robolinkJoint.maxAbsVelocity)
        #jointVelocities[4] = int(Joy.axes[3]*robolinkJoint.maxAbsVelocity)
        
    #else:
        #jointVelocities = [0, 0, 0, 0, 0]
    
    #robolinkControlMsg = buildControlMsg(jointVelocities)
    #IndJoyPublisher.publish(robolinkControlMsg)

"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
MAIN
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""    
# The main function
if __name__ == '__main__':
    
    #initialize()
    try:
        rospy.init_node('IndJoy')  
        rospy.Subscriber("joy", Joy, joyCallback)            
        print "Convert the CTI Joystick output to something more useful"
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
    finally:
        ij_axes = [1.0,1.0]
        #robolinkControlMsg = buildControlMsg(jointVelocities)
        #IndJoyPublisher.publish(robolinkControlMsg)
