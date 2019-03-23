#!/usr/bin/env python
import roslib
roslib.load_manifest('bebop_keyboard')
import rospy

from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Empty

import sys
import select
import termios
import tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   q    w    e            
   a    s    d     
Move vertically:
  -   up
  +   down

Basic commands:
  1 Take off
  2 Land

Camera control:
   i Up
   k Down
   j Left
   l Right
   0 Default

Speed linear:
n/m -> +/-
Speed angular:
N/M -> +/-
 
CTRL-C to quit
"""

# UAV moving values
moveBindings = {
    'w': (0.2, 0, 0, 0),
    'd': (0.2, 0, 0, -0.2),
    'q': (0, 0, 0, 0.2),
    'e': (0, 0, 0, -0.2),
    'a': (0.2, 0, 0, 0.2),
    's': (-0.2, 0, 0, 0),
    '-': (0, 0, 0.2, 0),
    '+': (0, 0, -0.2, 0),
}

# UAV speed increase/decrease values
speedBindings = {
    'n': (0.1, 0),
    'm': (-0.1, 0),
    'N': (0, 0.1),
    'M': (0, -0.1),
}

# UAV camera position change values
cameraBindings = {
    'i': (5, 0),
    'k': (-5, 0),
    'l': (0, 5),
    'j': (0, -5),
}

# Reading key from keyboard
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

# Print speed and turn when is changed
def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

#                  < Publishers > 
    pub = rospy.Publisher('bebop_follower/cmd_vel', Twist, queue_size=1)
    pub_takeoff = rospy.Publisher('bebop_follower/takeoff', Empty, queue_size=10)
    pub_land = rospy.Publisher('bebop_follower/land', Empty, queue_size=10)
    pub_camera=rospy.Publisher('bebop_follower/camera_control', Twist, queue_size=1)

    rospy.init_node('bebop_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    x = y = z = 0
    th = 0
    status = 0
    gd = lp = 0

    try:
        print msg
        print vels(speed, turn)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
            	speed = speed + speedBindings[key][0]
            	turn = turn + speedBindings[key][1]
            	print vels(speed,turn)
                x=0
		y=0
		z=0
		th=0
            elif key in cameraBindings.keys():
               	gd=gd+cameraBindings[key][0]
                lp=lp+cameraBindings[key][1]
                x=0
		y=0
		z=0
		th=0
            elif key == '1':
                print('takeoff')
                empty = Empty()
                pub_takeoff.publish(empty)

            elif key == '2':
                print('landing')
                empty = Empty()
                pub_land.publish(empty)
	    elif key == '0':
                print('camera default')
                gd = 0
		lp = 0
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th * turn
            pub.publish(twist)

            twistCam=Twist()
            twistCam.linear.x = 0
            twistCam.linear.y = 0
            twistCam.linear.z = 0
            twistCam.angular.x=0
	    twistCam.angular.y=gd
            twistCam.angular.z=lp
            pub_camera.publish(twistCam)

	

    except e:
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

        twistCam=Twist()
	twistCam.angular.y = gd;
	twistCam.angular.z=lp
        pub_camera.publish(twistCam)



termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
