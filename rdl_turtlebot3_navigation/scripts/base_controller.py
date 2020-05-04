#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class baseController(object):
    def __init__(self):
        # parameters
        self.distance_between_wheels = rospy.get_param('~distance_between_wheels', None)
        if not self.distance_between_wheels:
            rospy.logerr('Missing required parameter: distance_between_wheels')
            raise Exception('Missing required parameter')
        self.wheel_radius = rospy.get_param('~wheel_diameter', None)
        if not self.wheel_radius:
            rospy.logerr('Missing required parameter: wheel_diameter')
            raise Exception('Missing required parameter')
        self.wheel_radius = self.wheel_radius / 2.0
        # subscriptions
        rospy.Subscriber("/cmd_vel", Twist, self.baseCmdVel, queue_size=1)
        # publications
        self.left_wheel_cmd_pub = rospy.Publisher('/turtlebot3_burger/wheel_left_joint_controller/command', Float64, queue_size=1)
        self.right_wheel_cmd_pub = rospy.Publisher('/turtlebot3_burger/wheel_right_joint_controller/command', Float64, queue_size=1)

    def baseCmdVel(self, msg):
        '''
        This callback gets triggered every time a /cmd_vel msg is received
        '''
        # ============= YOUR CODE GOES HERE! =====
        # hint: watch https://youtu.be/aE7RQNhwnPQ


        # ============= YOUR CODE ENDS HERE! =====
        # prepare msgs
        left_wheel_msg = Float64()
        right_wheel_msg = Float64()
        left_wheel_msg.data = vl
        right_wheel_msg.data = vr
        # publish
        self.left_wheel_cmd_pub.publish(left_wheel_msg)
        self.right_wheel_cmd_pub.publish(right_wheel_msg)

    def start_base_controller(self):
        # wait for ctrl + c, prevent the node from dying (to allow callbacks to be received)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('turtlebot3_base_controller', anonymous=False)
    tb_base_controller = baseController()
    tb_base_controller.start_base_controller()
