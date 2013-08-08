#!/usr/bin/env python
import roslib,ros,rospy
roslib.load_manifest('xbox_controller')

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from std_msgs.msg import String

roslib.load_manifest('roboteq_mc_nxtgen_driver')
from roboteq_mc_nxtgen_driver.msg import RPM

from control import XboxController

from threading import Lock, Thread

from numpy import interp,pi,floor,average,power

# for fun
BEEPS = True
BEEP_CMD = 'echo -en "\007"'
if BEEPS: import os

#math stuff
twopi = 2*pi

#ow
global x_map
global y_map
x_map = 0
y_map = 0
#if the wheelchair takes ints or something
MIN_OUT = -1
MAX_OUT = 1

def convert_joy_value_to_int(joy_value):
    return interp(joy_value,[-1.0,1.0],[MIN_OUT,MAX_OUT])
    
def return_max_speed(value):
    if value < 0:
        return MIN_OUT
    elif value > 0:
        return MAX_OUT
            
        

extremes = [0.0,-0.0,-1.0,]
POS_MAX_VAR = 0.99
POS_MIN_VAR = 0.0
NEG_MAX_VAR = POS_MAX_VAR * -1
NEG_MIN_VAR = POS_MIN_VAR * -1
#rolling average values for extremes
ROLLING_FLOOR = 0.35
ROLLING_MAP = 0.95

ARRAY_TIMELINE_LEN = 20
POWER_WEIGHTING = 1.5
WEIGHTS = power([i for i in reversed(range(1,ARRAY_TIMELINE_LEN+1))],POWER_WEIGHTING)

#          _               _      _           _      
#__      _| |__   ___  ___| | ___| |__   __ _(_)_ __ 
#\ \ /\ / / '_ \ / _ \/ _ \ |/ __| '_ \ / _` | | '__|
# \ V  V /| | | |  __/  __/ | (__| | | | (_| | | |   
#  \_/\_/ |_| |_|\___|\___|_|\___|_| |_|\__,_|_|_|   
#                                                   
# wheelchair/rosws/sandbox/xboxcontroller/src/
r = .0508 #Encoder Wheel radius in meters
b = .508 #Wheelbase in meters
class IndJoy(XboxController):

    _oncliff = False
    lock = Lock()
    maxrpm = 400
    rpm = 300
    twistpub = None
    cmdpub = None
    rpmpub = None
    decelLock = Lock()
    _decellAssist = 0
    def __init__(self):
        super(IndJoy,self).__init__()
        self.x_timeline = [0.0 for i in range(ARRAY_TIMELINE_LEN)]
        self.y_timeline = [0.0 for i in range(ARRAY_TIMELINE_LEN)]
        
    def _ij_x_avg(self):
        return average(self.x_timeline,weights=WEIGHTS)    
    def _ij_y_avg(self):
        return average(self.y_timeline,weights=WEIGHTS)
        
    def _ij_x_update(self,val):
        self.x_timeline.pop()
        self.x_timeline.insert(0,val)
    
    def _ij_y_update(self,val):
        self.y_timeline.pop()
        self.y_timeline.insert(0,val)

    def callback(self,joymsg):
        self.lock.acquire()
        oncliffdup = self._oncliff
        self.lock.release()
        if oncliffdup:
            self.cmdpub.publish("!EX\r\n")
            print "At a cliff!"
        else:
            #estoprealse
            self.cmdpub.publish("!MG\r\n")
            x_map,y_map = indJoyCallback(joymsg,self)
            # do rpm stuff
            #if x_map > .95 * MAX_OUT:
            #    self.rpm = self.rpm + 20
            #if x_map < .95 * MIN_OUT:
            #    self.rpm = self.rpm -20
            #    
            #if self.rpm > self.maxrpm:
            #    self.rpm = self.maxrpm
            #elif self.rpm < 0:
            #    self.rpm = 0
            
            print "x: %s" % x_map
            fwd = self.calcfwd(x_map,self.rpm)
            #forward = joymsg.axes[1]*self.rpm*(2*3.141592653*r/60) #Desired forward velocity in meters/second
            #fwd = x_map*self.rpm*(2*3.141592653*r/60) #Desired forward velocity in meters/second 
            print "fwd: %s rpm: %s" % (fwd,self.rpm)

            turn = self.calcturn(y_map,x_map)
            phi1,phi2 = self.calcphis(fwd,turn)
            
            twist = Twist()
            twist.linear.x = fwd
            twist.angular.z = -turn
            self.twistpub.publish(twist)
            
            
            
    def calcfwd(self,x_map,rpm):
	if x_map <0:
	    fwd = x_map * self.rpm * (twopi*r/60) * 1
            if BEEPS: os.popen(BEEP_CMD)
        if x_map >=0:
	    fwd = x_map * self.rpm * (twopi*r/60) * 4.3 #4.3 MAX. 4 for non-jerky time @ 300RPM
        return fwd
        
    def calcturn(self,y_map,x_map):
        #fixes the drift
	if -0.8 < x_map > 0.8 and -0.2 < y_map < 0.2:
            y_map = 0 
        turn = y_map * pi * -1
        return turn
        
    def calcphis(self,fwd,turn):
        phi2 = (2 * fwd - turn * b)/(2 * r)
        phi2 = floor(60 * phi2 / twopi)
        
        phi1 = turn * b / r + phi2
        phi1 = floor(60 * phi1 / twopi)
        
        return phi1,phi2
    def listener(self):
        rospy.init_node('indjoy', anonymous=False)
        rospy.Subscriber("joy", Joy, self.callback)
        #rospy.Subscriber("CliffData", String, self.stop)
        rospy.Subscriber("primative_decell",Int16, self.assistedDecell)
        self.rpmpub = rospy.Publisher("cmd_rpm", RPM)
        self.twistpub = rospy.Publisher("behavior_twist", Twist)
        self.cmdpub = rospy.Publisher("roboteq_custom_command", String)
	print "hello world"
        rospy.spin()
        

IndJoyPublisher = rospy.Publisher('IndJoy',String)#, RobolinkControl)
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
CALLBACKS
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""



    
def indJoyCallback(Joy,ijO):
    
    ij_x = Joy.axes[0]
    ij_y = Joy.axes[1]
    x_map = 0.0
    y_map = 0.0
    
    if ij_x == ij_y == 1.0:
        #IndJoyPublisher.publish('centered')
        x_map = 0.0
        y_map = 0.0
        ijO._ij_x_update(x_map)
        ijO._ij_y_update(y_map)
        
        #out = "X: %s Y: %s" % (x_map,y_map)
        print x_map,y_map  
        #IndJoyPublisher.publish(out)
    else:
        if ij_x > POS_MIN_VAR and ij_x < POS_MAX_VAR:
            if ij_x not in extremes:
                x_map = convert_joy_value_to_int(1.0-ij_x)
                ijO._ij_x_update(x_map)
                
        elif ij_x < NEG_MIN_VAR and ij_x > NEG_MAX_VAR:
            if ij_x not in extremes:
                x_map = convert_joy_value_to_int(-1.0-ij_x)
                ijO._ij_x_update(x_map)
                
        else:
            a = ijO._ij_x_avg()
            print a
            if a > ROLLING_FLOOR:
                x_map = ROLLING_MAP
            elif a < -ROLLING_FLOOR:
                x_map = -ROLLING_MAP/2
                
        if ij_y > POS_MIN_VAR and ij_y < POS_MAX_VAR:
            if ij_y not in extremes:
                y_map = convert_joy_value_to_int(1.0-ij_y)
                ijO._ij_y_update(y_map)
                
        if ij_y < NEG_MIN_VAR and ij_y > NEG_MAX_VAR:
            if ij_y not in extremes:
                y_map = convert_joy_value_to_int(-1.0-ij_y)
                ijO._ij_y_update(y_map)
                
        else:
            b = ijO._ij_y_avg()
            print b
            if b > ROLLING_FLOOR:
                y_map = ROLLING_MAP
            elif b < -ROLLING_FLOOR:
                y_map = -ROLLING_MAP

    if x_map or y_map:
        print x_map,y_map    
        return(x_map,y_map)
    else:
	return 0,0
            
            
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""
MAIN
"""""""""""""""""""""""""""""""""""""""""""""""""""""""""""    
# The main function
if __name__ == '__main__':
    ij = IndJoy()
    ij.listener()
    ##initialize()
    #try:
        #rospy.init_node('IndJoy')  
        #rospy.Subscriber("joy", Joy, indJoyCallback)            
        #print "Convert the CTI Joystick output to something more useful"
        #rospy.spin()
    #except rospy.ROSInterruptException:
        #pass
    
    #finally:
        #ij_axes = [1.0,1.0]
        ##robolinkControlMsg = buildControlMsg(jointVelocities)
        ##IndJoyPublisher.publish(robolinkControlMsg)
