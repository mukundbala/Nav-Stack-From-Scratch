#!/usr/bin/env python3

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
# Modified by Lai Yan Kai for EE4308
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
from __future__ import print_function
import rospy
from geometry_msgs.msg import Twist
import sys, select, os

import threading

if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

BURGER_MAX_LIN_VEL = 0.4 #0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

topic = "/turtle/cmd_vel"
msg = """
      TELEOP TURTLE
=========================
Moving around:
        w
   a    s    d
        x
=========================
w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)
=========================
space key, s : force stop
CTRL-C to quit
"""


exitFlag = False

class threadCmdVel (threading.Thread):
    def __init__(self, pub):
        threading.Thread.__init__(self)
        self.pub = pub
    def run(self):
        global twist
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not exitFlag:
            pub.publish(twist)
            rate.sleep()


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    sys.stdin.flush() 
    return key

def vels(target_linear_vel, target_angular_vel, key):
    return "speed {:7.3f}\tturn {:7.3f}\tkey: {:s}".format(target_linear_vel,target_angular_vel,key)

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high

    return input

def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_turtle')
    pub = rospy.Publisher(topic, Twist, queue_size=1)

    turtlebot3_model = rospy.get_param("model", "burger")
    
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0

    try:
        print(msg)
        print(vels(0.0, 0.0, ''), end='\r')
        
        global twist
        twist = Twist()
        
        tcv = threadCmdVel(pub)
        tcv.start()
        
        while(1):
            key = getKey()
            if key == 'w' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
            elif key == 'x' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
            elif key == 'a' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
            elif key == 'd' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
            elif key == ' ' or key == 's' :
                target_linear_vel   = 0.0
                target_angular_vel  = 0.0
            else:
                if (key == '\x03'):
                    print(vels(target_linear_vel,target_angular_vel, key))
                    exitFlag = True
                    break
                    

            twist.linear.x = target_linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            
            if abs(target_angular_vel) < 1e-5:
                target_angular_vel = 0.0
                
            if abs(target_linear_vel) < 1e-5:
                target_linear_vel = 0.0
                
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = target_angular_vel
            
            print(vels(target_linear_vel,target_angular_vel, key), end='\r')
        
        tcv.join()

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        exitFlag = True
        tcv.join()
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)