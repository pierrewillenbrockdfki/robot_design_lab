#!/usr/bin/env python

import inverse_range_sensor_model
import rospy
import math
import tf

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

class UpdateMap:
    '''
    Subscribes to laser scanner data and updates an Occupancy grid map
    '''
    def __init__(self):
        rospy.loginfo('Mapping node started, getting parameters')
        # subscribe to laser scanner reading
        rospy.Subscriber('~scan', LaserScan, self.laserCallback, queue_size=1)
        # to publish the map
        self.pub_map = rospy.Publisher('~map', OccupancyGrid, queue_size=1)
        # flag to indicate that a topic msg was received
        self.scan_received = False
        # to control the frequency at which the node will run
        self.loop_rate = rospy.Rate(rospy.get_param('~loop_rate', 10.0))
        # the msg to publish the updated map
        self.updated_map_msg = OccupancyGrid()
        # to store incomming laser scaner data
        self.scan_msg = None
        # flag used to enable/disable laser scanner updates
        self.lock_scan = False
        # to store object of inverse range sensor model
        self.irsm_object = None
        # map parameters
        self.map_resolution = None
        self.map_width = None
        # to store the sensor pose(query from tf)
        self.sensor_pose = None
        # to get and store the pose of the sensor(query from tf)
        self.listener = tf.TransformListener()
        self.wait_for_transform = rospy.get_param('~sensor_pose_transform_tolerance', 0.1)
        # inverse range sensor model params
        self.obstacle_thickness = rospy.get_param('~obstacle_thickness', 0.08)
        # nan laser scanner ranges will be replaced with this number
        self.replace_nan_with = rospy.get_param('~replace_laser_nan_with', 1000.0)
        # perform map setup one time only, get params, etc.(check function documentation)
        self.update_map_setup()
        # compute the length of the map in array
        self.map_length = self.map_width * self.map_height
        # sleep to give some time for publishers ans subscribers to register into the network
        rospy.sleep(0.5)

    def laserCallback(self, msg):
        '''
        This function get executed every time a laser scan msg is received
        '''
        if not self.lock_scan:
            self.scan_msg = msg
            self.scan_received = True

    def remove_laser_nans(self):
        '''
        Replace nan values with INF(a big number)
        '''
        for n, i in enumerate(self.scan_msg.ranges):
            if i=='nan':
                self.scan_msg.ranges [n] = self.replace_nan_with

    def get_sensor_pose_and_laser(self):
        '''
        Query tf to get the sensor pose
        '''
        try:
            # wait for transform to become available
            self.listener.waitForTransform(self.global_reference_frame, self.sensor_reference_frame, rospy.Time(0), rospy.Duration(self.wait_for_transform))
            # get sensor pose transform
            translation, rotation = self.listener.lookupTransform(self.global_reference_frame, self.sensor_reference_frame, rospy.Time(0))
        except tf.Exception, error:
            # an error ocurred, print in terminal the error
            rospy.logwarn('Exception occurred: {0}'.format(error))

        # lock to ignore further laser scanner msgs
        self.lock_scan = True

        # replace laser scanner nan's with big numbers
        self.remove_laser_nans()

        # transform quaternion into euler using tf library
        quaternion =(rotation[0], rotation[1], rotation[2], rotation[3])
        euler = tf.transformations.euler_from_quaternion(quaternion)

        # return sensor pose(x, y, yaw)
        return [translation[0], translation[1], euler[2]]

    def update_prob(self, prior, p_observation):
        '''
        receives a prior and a new value and updates according to the
        occupancy grid mapping algorithm
        '''
        # ============= YOUR CODE GOES HERE! =====
        # hint : the following code generates a binary non probabilistic map
        #        which might be a good starting point to test your code
        # hint : after you are done with this version, make the shift toward a probabilistic formulation
        #        of the mapping problem, using the log odds ratio as explained here:
        #        http://ais.informatik.uni-freiburg.de/teaching/ss18/robotics/recordings/12-grid-maps-and-mapping-with-known-poses-part1.mp4
        # hint : replace this snippet with a probabilistic formulation of the problem
        # hint : both prior and p_observation are probabilities in the range 0 - 100 (except when its -1, which means unknown)
        if p_observation == -1:
            return prior
        else:
            return p_observation
        # hint: instead of returning p_observation, return a prior merged with the observed new probability
        # hint: it requires multiple times having the same value to trust it, to be robust toward sensor noise
        # ============= YOUR CODE ENDS HERE! =====

    def process_scan(self):
        '''
        process laser scanner reading to update the map
        '''
        # set z_t one time only(array of measurements)
        self.irsm_object.set_z_t(self.scan_msg.ranges)

        # iterate over each cell of the map and compute its value: unknown, free or occupied
        for cell_index in range(0, self.map_length):
            # run inverse_range_sensor_model - update cell function and save its return value in the map at the right index
            self.updated_map_msg.data[cell_index] = self.update_prob( \
                self.updated_map_msg.data[cell_index], \
                self.irsm_object.update_cell(cell_index, self.sensor_pose))

        self.pub_map.publish(self.updated_map_msg)
        # enable new laser scanner updates
        self.lock_scan = False

    def update_map_setup(self):
        '''
        1. Subscribe to laser scanner msg and read parameters from it
        2. Subscribe to an existing map if required
        3. Either get params from param server or inherit from existing map
        4. Create object of inverser range sensor model
        '''
        rospy.loginfo('Waiting for first laser scanner msg scanner to get parameters')

        # wait for first laser scanner reading to get laser params from there
        while self.scan_received == False and not rospy.is_shutdown():
            self.loop_rate.sleep()

        # create a new empty map
        self.global_reference_frame = rospy.get_param('~global_reference_frame', 'map')
        self.map_resolution = rospy.get_param('~map_resolution', 0.1)
        self.map_width = rospy.get_param('~map_width', 80)
        self.map_height = rospy.get_param('~map_height', 80)
        self.map_x_offset = rospy.get_param('~map_x_offset', -4.0)
        self.map_y_offset  = rospy.get_param('~map_y_offset', -4.0)
        # fill updated map msg
        self.updated_map_msg.header.frame_id = self.global_reference_frame
        self.updated_map_msg.info.resolution = self.map_resolution
        self.updated_map_msg.info.width = self.map_width
        self.updated_map_msg.info.height = self.map_height
        # set updated map origin
        map_origin_pose = Pose()
        map_origin_pose.position.x = self.map_x_offset
        map_origin_pose.position.y = self.map_y_offset
        self.updated_map_msg.info.origin = map_origin_pose
        # init updated map to unknown
        for i in range(self.map_width * self.map_height):
            self.updated_map_msg.data.append(-1)

        # publish updated map for the first time
        rospy.loginfo('Publishing updated map for the first time')
        self.pub_map.publish(self.updated_map_msg)

        # get laser reference frame from first received laser scanner msg
        self.sensor_reference_frame = self.scan_msg.header.frame_id

        # create object of inverse range sensor model
        self.irsm_object = inverse_range_sensor_model.InverseRangeSensorModel(\
            self.scan_msg.angle_min, self.scan_msg.angle_max,\
            self.scan_msg.angle_increment, self.scan_msg.range_max, \
            self.map_x_offset, self.map_y_offset, self.map_resolution, \
            self.map_width, self.obstacle_thickness)

    def start_mapping(self):
        '''
        Perform a map update if a new laser scanner msg is received
        '''
        while not rospy.is_shutdown():
            if self.scan_received == True:
                # lower flag
                self.scan_received = False
                # get robot pose and lock laser scanner data to ignore future msgs
                self.sensor_pose = self.get_sensor_pose_and_laser()
                # update map with received laser scanner reading
                self.process_scan()
            self.loop_rate.sleep()

if __name__ == '__main__':
    # register the node in the network
    rospy.init_node('mapping_node', anonymous=False)
    # create object of the class(constructor get executed)
    update_map_instance = UpdateMap()
    # call class method start_update_map
    update_map_instance.start_mapping()
