#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseArray

class ObstacleLocalisationMockup:
    def __init__(self):
        self.pub_left_goals = rospy.Publisher('/left_goals', PoseArray, queue_size=1, latch=True)
        self.pub_right_goals = rospy.Publisher('/right_goals', PoseArray, queue_size=1, latch=True)
        self.pub_obstacles = rospy.Publisher('/obstacles', PoseArray, queue_size=1, latch=True)

    def gen_pose(self, x, y):
        pose_msg = Pose()
        pose_msg.position.x = x
        pose_msg.position.y = y
        pose_msg.position.z = 0.0
        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = 0.0
        pose_msg.orientation.w = 1.0
        return pose_msg

    def gen_pose_array(self, frame_id, list_of_x_y):
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = rospy.Time.now()
        pose_array_msg.header.frame_id = frame_id

        for pos in list_of_x_y:
            pose_array_msg.poses.append(self.gen_pose(pos[0], pos[1]))

        return pose_array_msg

    def start_object_localisation_mockup(self):
        obstacles = [[0.0, 0.0], [1.0, 1.0]]
        goal_left = [[0.0, 1.0]]
        goal_right = [[0.0, 2.0]]
        self.pub_obstacles.publish(self.gen_pose_array('map', obstacles))
        self.pub_left_goals.publish(self.gen_pose_array('map', goal_left))
        self.pub_right_goals.publish(self.gen_pose_array('map', goal_right))
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('turtlebot3_odometry', anonymous=False)
    obs_loc_mock = ObstacleLocalisationMockup()
    obs_loc_mock.start_object_localisation_mockup()
