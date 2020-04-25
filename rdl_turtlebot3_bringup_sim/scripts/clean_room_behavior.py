#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class robotSimpleBehavior(object):
    '''
    Exposes a behavior for the robot to move forward until it sees an ostacle ahead
    (e.g. a wall), if so, rotate for a while and then loop.
    '''
    def __init__(self):
        # register node in ROS network
        rospy.init_node('turtlebot3_cleaning', anonymous=False)
        # print message in terminal
        rospy.loginfo('Turtlebot3 simple behavior started !')
        # subscribe to robot laser scanner topic
        rospy.Subscriber("/scan", LaserScan, self.laserCallback)
        # setup publisher to later on move the robot base
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # it will store the distance from the robot to the walls
        self.distance = 10.0

        # defines the range threshold bellow which the robot should stop moving foward and rotate instead
        if rospy.has_param('distance_threshold'):
            # retrieves the threshold from the parameter server in the case where the parameter exists
            self.distance_threshold = rospy.get_param('distance_threshold')
        else:
            self.distance_threshold = 0.5
        

    def rotate_right(self):
        twist_msg = Twist()
        # liner speed
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        # angular speed
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = -0.6

        # publish Twist message to /cmd_vel to move the robot
        self.pub_cmd_vel.publish(twist_msg)


    def move_forward(self):
        twist_msg = Twist()
        # linear speed
        twist_msg.linear.x = 0.5
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        # angular speed
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0

        # publish Twist message to /cmd_vel to move the robot
        self.pub_cmd_vel.publish(twist_msg)

    def laserCallback(self, msg):
        '''
        This function gets executed everytime a laser scanner msg is received on the
        topic: /scan
        '''
        # ============= YOUR CODE GOES HERE! =====
        # hint: msg contains the laser scanner msg
        # hint: check http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html


        # ============= YOUR CODE ENDS HERE! =====


    def run_behavior(self):
        while not rospy.is_shutdown():
            if self.distance > self.distance_threshold:
                self.move_forward()
            else:
                self.rotate_right()
            rospy.sleep(0.1)

if __name__ == '__main__':
    my_object = robotSimpleBehavior()
    my_object.run_behavior()
