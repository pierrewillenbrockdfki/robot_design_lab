#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan

from geometry_msgs.msg import PoseStamped

"""
navigation - occupancy grid path planning

A simpler version of http://wiki.ros.org/global_planner
made for learning purposes
"""

class GlobalPlanner(object):
    def __init__(self):
        # subscribe to costmap
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.globalCostmapCB, queue_size=1)
        self.global_plan_pub = rospy.Publisher('/move_base/NavfnROS/plan', Path, queue_size=1)
        # flag to indicate if a costmap msg was received
        self.costmap_received = False

    def gen_dummy_plan(self):
        '''
        this is a hand made plan, use a pathfinder istead!
        '''
        waypoints = [
            [0.0, 0.0],
            [0.6, 0.0],
            [0.6, 1.0],
            [1.7, 1.0],
            [1.7, 1.7]
        ]
        return waypoints

    def findPlan(self, costmap_as_array, map_resolution, width, height, start, goal):
        '''
        receive all necessary elements (grid map + properties, etc), find a path from start to goal
        '''
        # ============= YOUR CODE GOES HERE! =====

        # hint: replace the following line below with your implementation
        plan_as_array_of_poses = self.gen_dummy_plan()

        # hint: see : http://wiki.ros.org/global_planner
        # hint: assist yourself with information from the pdf worksheet

        # ============= YOUR CODE ENDS HERE! =====
        
        return plan_as_array_of_poses

    def handleMakePlanSrv(self, req):
        '''
        this is a callback that gets triggered upon receiving a service request on /robot_design_lab/make_plan
        - assumes all data comes in map frame
        - make plan and return the resulting path (navigation path planning)
        '''
        # create emtpy path msg
        plan = Path()
        if not self.costmap_received:
            rospy.logwarn("costmap not received, can't make plan")
            return plan
        rospy.loginfo('make plan service request received')
        # extract start pose and goal pose from request service message
        start_pos = [req.start.pose.position.x, req.start.pose.position.y]
        goal_pos = [req.goal.pose.position.x, req.goal.pose.position.y]
        # find plan
        plan_as_array_of_poses = self.findPlan(self.costmap.data, self.costmap.info.resolution,\
            self.costmap.info.width, self.costmap.info.height, start_pos, goal_pos)
        # fill "plan" message with data
        plan.header.frame_id = 'map'
        plan.header.stamp = rospy.Time.now()
        for pos in plan_as_array_of_poses:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = pos[0]
            pose_msg.pose.position.y = pos[1]
            pose_msg.pose.position.z = 0.0 # robot can't fly
            # set 0 degree orientation (array of positions)
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            plan.poses.append(pose_msg)
        # publish plan for visualisation purposes
        self.global_plan_pub.publish(plan)
        # return planned path
        return plan

    def globalCostmapCB(self, msg):
        '''
        This callback gets triggered every time a /move_base/global_costmap/costmap msg is received
        '''
        # save received costmap in member variable
        self.costmap = msg
        # raise flag
        self.costmap_received = True

    def start_global_planner(self):
        rospy.loginfo('path planning node started')
        # setup this node so that it can offer the make plan service
        s = rospy.Service('/robot_design_lab/make_plan', GetPlan, self.handleMakePlanSrv)
        # wait for ctrl + c, prevent the node from dying (to allow callbacks to be received)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('global_planner', anonymous=False)
    tb_global_planner = GlobalPlanner()
    tb_global_planner.start_global_planner()
