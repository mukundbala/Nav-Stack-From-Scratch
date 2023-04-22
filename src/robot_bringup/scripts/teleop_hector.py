#!/usr/bin/env python3
#  <version>0.6.2</version>
#  <description>Generic keyboard teleop for twist robots.</description>
#  <maintainer email="namniart@gmail.com">Austin Hendrix</maintainer>
#  <license>BSD</license>
#  <url type="website">http://wiki.ros.org/teleop_twist_keyboard</url>
#  <author>Graylin Trevor Jay</author>
#  modified by Lai Yan Kai for use for EE4308

from __future__ import print_function

import roslib
import rospy

from geometry_msgs.msg import Twist
from hector_uav_msgs.srv import EnableMotors
import sys, select, termios, tty, threading

    
    
topic = "/hector/cmd_vel"
msg = """
    TELEOP HECTOR
====== Strafe =======
   q    w    e
   a    s    d
   z    x    c
==== Yaw / Climb ====
        i    
   j    k    l
        ,
==== Increments =====
r/v : inc/dec max speeds by 10%
t/b : inc/dec linear speed by 10%
y/n : inc/dec angular speed by 10%
=====================
Anything else : stop
CTRL-C to quit
=====================
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

moveBindings = {
        'j':(0,0,0,1),
        'l':(0,0,0,-1),
        'e':(1,-1,0,0),
        'w':(1,0,0,0),
        'a':(0,1,0,0),
        'd':(0,-1,0,0),
        'q':(1,1,0,0),
        'x':(-1,0,0,0),
        'c':(-1,-1,0,0),
        'z':(-1,1,0,0),
        'i':(0,0,1,0),
        ',':(0,0,-1,0)
    }

speedBindings={
        'r':(1.1,1.1),
        'v':(.9,.9),
        't':(1.1,1),
        'b':(.9,1),
        'y':(1,1.1),
        'n':(1,.9),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    sys.stdin.flush()
    return key


def feedback(speed,turn,key):
    return "speed {:7.3f}\tturn {:7.3f}\tkey: {:s}".format(speed,turn,key)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher(topic, Twist, queue_size = 1)
    rospy.init_node('teleop_hector')

    speed = rospy.get_param("~speed", 0.3)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0

    global twist
    twist = Twist()
    tcv = threadCmdVel(pub)
    tcv.start()

        
    # Services (Enable motors)
    enable_motors = rospy.ServiceProxy('/hector/enable_motors', EnableMotors)
    enable_motors(True)
    print('Motors Enabled')
    
    print(msg)
    print(feedback(speed,turn,''), end='\r')
    
    try:
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    print(feedback(speed, turn, key))
                    exitFlag = True
                    break
            print(feedback(speed,turn,key), end='\r')
            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            pub.publish(twist)

        tcv.join()

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)
        exitFlag = True
        tcv.join()

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
