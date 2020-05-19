#!/usr/bin/env python

import math, random, copy

import rospy
import tf

from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, PoseStamped, Twist

class DWALocalPlanner(object):
    def __init__(self):
        # get required params
        self.get_params()
        # publications
        self.pub_trajectory_cloud = rospy.Publisher('trajectory_cloud', PointCloud, queue_size=1)
        self.pruned_global_plan_pub = rospy.Publisher('/move_base/DWAPlannerROS/global_plan', Path, queue_size=1)
        self.pub_winner_traj = rospy.Publisher('winner_traj', PointCloud, queue_size=1) # Remove and make it path instead
        self.pub_pose_to_follow = rospy.Publisher('pose_to_follow', PoseStamped, queue_size=1) # remove, debug only
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        # subscriptions
        rospy.Subscriber('/move_base/NavfnROS/plan', Path, self.globalPlanCB, queue_size=1)
        # tf
        self.tf_listener = tf.TransformListener()
        rospy.loginfo('rdl dwa local planner initialized...')
        # member variables
        self.pose_to_follow = None # dynamic pointer to the current pose from the global plan to follow
        self.plan = None # to store the plan (array of poses) as list of 2d array lists [[x1, y1], [x2, y2] ...]
        self.finished_goal = True

    def get_params(self):
        '''
        query required parameters from param server
        '''
        # robot properties
        self.distance_between_wheels = rospy.get_param('~distance_between_wheels', None)
        if not self.distance_between_wheels:
            rospy.logerr('Missing required parameter: distance_between_wheels')
            raise Exception('Missing required parameter')
        self.wheel_radius = rospy.get_param('~wheel_diameter', None)
        if not self.wheel_radius:
            rospy.logerr('Missing required parameter: wheel_diameter')
            raise Exception('Missing required parameter')
        self.wheel_radius = self.wheel_radius / 2.0

        # ============= YOUR TUNING GOES HERE! =====

        # hint: only perform parameter tuning here, no code changes are needed
        # hint: this parameters are good for development but they are actually bad for a competition
        # hint: note that this params are missing from the launch file, feel free to add them there if needed
        #       however when a parameter is missing you can see that we are providing default values

        # DWA params
        # ---
        # control the frequency at which this node will run
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))
        # control how long do you want to forward simulate a sample (vx (v), and vtheta (w))
        self.t = rospy.get_param('~sim_time', 0.25) # in seconds
        # (x=vx_samples) x random numbers (samples) are generated between min_vel_trans and max_vel_trans
        self.min_vel_trans = rospy.get_param('~min_vel_trans', 0.0)
        self.max_vel_trans = rospy.get_param('~max_vel_trans', 0.2)
        # (y=vth_samples) y random numbers (samples) are generated between min_vel_theta and max_vel_theta
        self.min_vel_theta = rospy.get_param('~min_vel_theta', -0.1)
        self.max_vel_theta = rospy.get_param('~max_vel_theta', 0.1)
        # the number of random samples to generate in vx (unicycle model - v)
        self.vx_samples = rospy.get_param('~vx_samples', 20)
        # the number of random samples to generate in vtheta (unicycle model - w)
        self.vth_samples = rospy.get_param('~vth_samples', 40)
        
        # Global planner params
        # ---
        # max amount of poses that will be considered from the global plan (prune the plan and follow on sections)
        self.look_ahead_global_plan = rospy.get_param('~look_ahead_global_plan', 30)
        
        # ============= YOUR TUNING ENDS HERE! =====

    def transform_pcl(self, pcl):
        '''
        transform a pointcloud from one reference frame to another
        '''
        self.tf_listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(0.1))
        return self.tf_listener.transformPointCloud('map', pcl)

    def extract_plan(self, array_of_poses):
        '''
        receive an array of geometry_msgs/PoseStamped
        convert to list
        '''
        self.plan = []
        for pose in array_of_poses:
            self.plan.append([pose.pose.position.x, pose.pose.position.y])

    def globalPlanCB(self, msg):
        '''
        this is a callback that gets triggered upon receiving a msg on /move_base/NavfnROS/plan
        it receives a global plan that needs to be followed
        '''
        rospy.loginfo('global plan received')
        # raise flag to start dwa
        self.finished_goal = False
        # extract plan from msg, convert to list and save in member variable
        self.extract_plan(msg.poses)
        # cut plan to follow some percentage of it
        pruned_path_msg = Path()
        pruned_path_msg.header.frame_id = msg.header.frame_id
        pruned_path_msg.header.stamp = rospy.Time.now()
        pruned_path_msg.poses = msg.poses[:self.look_ahead_global_plan]
        # publish pruned path for visualisation purposes
        self.pruned_global_plan_pub.publish(pruned_path_msg)

    def gen_pcl_msg(self, trajectories, reference_frame):
        '''
        publish sampled trajectories for visualisation purposes
        trajectories format -> trajectories = [[0.5, 0.0, 0.0],[1.0, 0.0, 0.0]]
        x, y, phi (phi always zero, no orientation)
        '''
        pcl_msg = PointCloud()
        # filling pointcloud header
        pcl_msg.header.frame_id = reference_frame
        pcl_msg.header.stamp = rospy.Time.now()
        for traj in trajectories:
            # filling some points
            pcl_msg.points.append(Point32(traj[2], traj[3], 0.0))
        return pcl_msg

    def generate_random_samples(self, min_vel_trans, max_vel_trans, min_vel_theta, max_vel_theta, vx_samples, vth_samples):
        random_samples = []
        vx_gen_samples = []
        vth_gen_samples = []
        # ============= YOUR CODE GOES HERE! =====
        # hint: generate random v, w samples uniformly distributed
        # hint: vx_samples - the max amount of v samples required
        # hint: vth_samples - the max amount of w samples required
        # hint: combine the samples [[v1, w1], [v1, w2], [v2, w3] ...]

        # ============= YOUR CODE ENDS HERE! =====
        return random_samples

    def motion_model(self, v, w, t):
        # ============= YOUR CODE GOES HERE! =====
        # hint: input v (linear forward speed), w (angular speed) and t (time)
        # hint: simulate for time t [seconds]
        # find out where the robot would be if that v, w, t where applied


        # ============= YOUR CODE ENDS HERE! =====
        return [v, w, x, y, 0.0] # z = 0

    def euclidean_distance(self, y2, y1, x2, x1):
        return math.sqrt(math.pow(y2 - y1, 2) + math.pow(x2 - x1, 2))

    def score_trajectories(self, trajectories, pose_to_follow):
        # ============= YOUR CODE GOES HERE! =====
        # find out which is the best trajectory to follow based on euclidean distance
        # hint: use euclidean distance function (self.euclidean_distance)
        # hint: winner trajectory has the format [v, w, x, y, z] , (z is always equal to 0)


        # ============= YOUR CODE ENDS HERE! =====
        return winner_traj

    def publish_winner_traj(self, winner_traj):
        pcl_msg = PointCloud()
        # filling pointcloud header
        pcl_msg.header.frame_id = 'map'
        pcl_msg.header.stamp = rospy.Time.now()
        # filling some points
        pcl_msg.points.append(Point32(winner_traj[2], winner_traj[3], 0.0))
        self.pub_winner_traj.publish(pcl_msg)

    def publish_pose_to_follow(self, pose_to_follow):
        pose_to_follow_msg = PoseStamped()
        pose_to_follow_msg.header.frame_id = 'map'
        pose_to_follow_msg.header.stamp = rospy.Time.now()
        pose_to_follow_msg.pose.position.x = pose_to_follow[0]
        pose_to_follow_msg.pose.position.y = pose_to_follow[1]
        pose_to_follow_msg.pose.position.z = 0.0
        pose_to_follow_msg.pose.orientation.x = 0.0
        pose_to_follow_msg.pose.orientation.y = 0.0
        pose_to_follow_msg.pose.orientation.z = 0.0
        pose_to_follow_msg.pose.orientation.w = 1.0
        self.pub_pose_to_follow.publish(pose_to_follow_msg)

    def extract_traj_from_pcl(self, trajectories, pcl_msg):
        '''
        input a pointcloud and output its points as a 2D array
        '''
        assert(len(pcl_msg.points) == len(trajectories))
        traj = []
        for i, point in enumerate(pcl_msg.points):
            # extract original v and w from trajectories
            traj.append([trajectories[i][0], trajectories[i][1], point.x, point.y, 0.0])
        return traj

    def execute_traj(self, v, w):
        '''
        send v and w to the robot via cmd_vel
        '''
        twist_msg = Twist()
        twist_msg.linear.x = v
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = w
        self.pub_cmd_vel.publish(twist_msg)

    def get_robot_pose(self):
        try:
            # get latest available transform (robot pose)
            trans, rot = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
        except:
            rospy.logwarn('could not get robot pose (tf)')
            return None
        return [trans[0], trans[1]]

    def find_robot_in_plan(self, robot_position, plan):
        '''
        iterate over plan, compute euclidean distance between robot pose
        and each step in the plan, return index of plan that corresponds
        to the position of the robot in the plan array
        '''
        distance = 1000.0
        index = None
        for i, step in enumerate(plan):
            dis = self.euclidean_distance(step[1], robot_position[1], step[0], robot_position[0])
            if dis < distance:
                distance = dis
                index = i
        return index

    def update(self):
        if self.finished_goal:
            return
        # track the progress of the robot, how much from the global plan has accomplished?
        robot_pose = self.get_robot_pose()
        if robot_pose:
            # index : the index in the plan array where the robot is located
            # e.g. [p1, p2, p3, p4, p5, p6]
            #                R
            # index = 2
            index = self.find_robot_in_plan(robot_pose, self.plan)
            # update pose to follow next
            if (index + self.look_ahead_global_plan) >= len(self.plan):
                self.pose_to_follow = self.plan[-1] # take last step of the plan
            else:
                self.pose_to_follow = self.plan[index + self.look_ahead_global_plan] # take pose from n steps ahead of the plan
        # check if robot has finished
        tolerance = 3
        if (len(self.plan) - index) < tolerance: # consider the robot finished if it gets close to any of the last 3 poses in the plan
            # publish 0 speed (to stop the robot)
            self.execute_traj(0.0, 0.0)
            self.finished_goal = True
            rospy.loginfo('goal achieved')
            return
        # generate uniformly distributed trajectory samples
        samples = self.generate_random_samples(self.min_vel_trans, self.max_vel_trans, \
            self.min_vel_theta, self.max_vel_theta, self.vx_samples, self.vth_samples)
        # ============= YOUR CODE GOES HERE! =====
        # hint: use any of the methods above that you want to assist you
        # hint: finish DWA local planner algorithm, get main idea from : http://wiki.ros.org/dwa_local_planner
        #
        # tf part
        # ---
        # hint: some transformations are needed, watch https://youtu.be/2gVo06HR2Tc to understand tf
        # hint: it is recommended that you query transformations always inside a try catch block, something like:
        # try:
        #    ?
        # except: # a better way to show exactly what is the error is recommended to implement here
        #    rospy.logwarn('could not transform traj from base_link to map frame')
        #    return
        # NOTE: be aware that a couple of this msgs might appear (and is fine, we recommend to drop the loop <return> and catch the next one):
        # [WARN] [/rdl_dwa_local_planner]: could not transform traj from base_link to map frame

        # hint: publish trajectories for visualisation purposes using:
        # self.pub_trajectory_cloud.publish(pontcloud_msg)

        # ============= YOUR CODE ENDS HERE! =====

    def start_dwa_local_planner(self):
        while not rospy.is_shutdown():
            self.update()
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('rdl_dwa_local_planner', anonymous=False)
    tb_dwa = DWALocalPlanner()
    tb_dwa.start_dwa_local_planner()
