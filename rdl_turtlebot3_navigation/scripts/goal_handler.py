#!/usr/bin/env python

import rospy
import tf

from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan

'''
Subscribe to /move_base_simple/goal
query current robot pose using tf
make service call to request a global plan using previous information
'''

class GoalHandler(object):
    def __init__(self):
        # subscriptions
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.NavGoalCallBack, queue_size=1)
        srv_name = rospy.get_param('~path_planning_srv_name', '/move_base/make_plan') # '/robot_design_lab/make_plan' also possible
        rospy.loginfo('waiting for service : {}'.format(srv_name))
        rospy.wait_for_service(srv_name)
        rospy.loginfo('service available, proceed')
        self.make_plan_service = rospy.ServiceProxy(srv_name, GetPlan)
        self.tf_listener = tf.TransformListener()
        rospy.loginfo('goal handler node initialized')

    def NavGoalCallBack(self, msg):
        '''
        This callback gets triggered every time a /move_base_simple/goal msg is received
        NOTE: msg = goal
        '''
        rospy.loginfo('navigation goal received')
        try:
            trans, rot = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
        except:
            rospy.logwarn('could not get robot pose (tf)')
            return
        start = PoseStamped()
        start.header.frame_id = 'map'
        start.pose.position.x = trans[0]
        start.pose.position.y = trans[1]
        start.pose.position.z = 0.0
        # 0 degree orientation
        start.pose.orientation.x = 0.0
        start.pose.orientation.y = 0.0
        start.pose.orientation.z = 0.0
        start.pose.orientation.w = 1.0
        tolerance = 0.0
        rospy.loginfo('calling service to make plan')
        resp1 = self.make_plan_service(start, msg, tolerance)

    def start_goal_handler(self):
        # wait for ctrl + c, prevent the node from dying (to allow callbacks to be received)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('rdl_goal_handler', anonymous=False)
    tb_goal_handler = GoalHandler()
    tb_goal_handler.start_goal_handler()
