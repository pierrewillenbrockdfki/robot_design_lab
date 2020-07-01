#!/usr/bin/python

import rospy
import cv2
import sys
import numpy as np

from sensor_msgs.msg import Image, CompressedImage
from rdl_rgb_obstacle_detection.msg import Object, Obstacles, Goals
from cv_bridge import CvBridge, CvBridgeError
from dynamic_reconfigure.server import Server
from rdl_rgb_obstacle_detection.cfg import rdl_rgb_obstacle_detectionConfig

class ColorLimit:
    def __init__(self):
        self.low = (0, 0, 0)
        self.high = (0, 0, 0)

class ObjectDetector:
    """
    A very naive object detector that detects obstacles and goal posts based on their color.

    Detection Algorithm
    -------------------
    A simple filtering pipeline is used to detect the objects:
    1.) Remove parts of the image based on color limits for the hsv channels. After removal only the objects should remain
    2.) Convert Image to black & white
    3.) Find contours on the image to detect object borders
    4.) Calculate center, width and height based on the contours


    Parameters
    ----------
    The color limits can be configured using the ROS parameter server.
    See cfg/image_processing.cfg for details on all available parameters.

    The color limits depend on the lighting conditions in the room. I.e. they have to be adapted before the competition

    The debug parameter enables/disables the output of additional images that can be used for parameter tuning.


    Subscribed Topics
    -----------------
    input_obstacles_image - Camera image (raw or compressed - user needs to ensure XOR!)

    Published Topics
    ----------------
    /obstacles               - A list of currently detected obstacles
    /goals                   - A list of currently detected goal posts
    /debug_image_result      - Visualization of the detected contours
    /debug_image_goals_left  - Visualization of the color segmentation for the left goal post
    /debug_image_goals_right - Visualization of the color segmentation for the right goal post
    /debug_image_obstacles   - Visualization of the color segmentation for the obstacle detection

    """

    def __init__(self):

        # bridge converts ros images <-> opencv images
        self.bridge = CvBridge()

        # parameters
        self.obstacle_color_limits = ColorLimit()
        self.left_goal_limits = ColorLimit()
        self.right_goal_limits = ColorLimit()
        self.binarization_threshold = 30
        self.debug_mode = False

        # pubs/subs
        self.image_sub = rospy.Subscriber("input_obstacles_image", Image, self.callback)
        self.comp_image_sub = rospy.Subscriber("input_obstacles_image_compressed", CompressedImage, self.callback_compressed)
        self.obstacle_publisher = rospy.Publisher("/obstacles", Obstacles, queue_size=5)
        self.goal_publisher = rospy.Publisher("/goals", Goals, queue_size=5)

        self.debug_result_image_pub = rospy.Publisher("/debug_image_result", Image, queue_size=5)
        self.debug_image_goals_left_pub = rospy.Publisher("/debug_image_goals_left", Image, queue_size=5)
        self.debug_image_goals_right_pub = rospy.Publisher("/debug_image_goals_right", Image, queue_size=5)
        self.debug_image_obstacles_pub = rospy.Publisher("/debug_image_obstacles", Image, queue_size=5)
        rospy.sleep(0.2)

    def detect_contours(self, bgr_image, hsv_image, low_limit, high_limit, debug_image_publisher=None):
        # ============= YOUR CODE GOES HERE! =====
        # hint: use opencv to find contours
        # use other cv2.? methods
        # use cv2.findContours(...) - method

        # ============= YOUR CODE ENDS HERE! =====
        return contours

    def detect_obstacles(self, bgr_image, hsv_image):
        """
        :return: contours
        """
        low_col = self.obstacle_color_limits.low
        high_col = self.obstacle_color_limits.high

        debug_pub = None
        if self.debug_mode:
            debug_pub = self.debug_image_obstacles_pub

        return self.detect_contours(bgr_image, hsv_image, low_col, high_col, debug_pub)

    def detect_goals(self, bgr_image, hsv_image):

        debug_pub_left = None
        debug_pub_right = None
        if self.debug_mode:
            debug_pub_left = self.debug_image_goals_left_pub
            debug_pub_right = self.debug_image_goals_right_pub

        low_col = self.left_goal_limits.low
        high_col = self.left_goal_limits.high

        left_contours = self.detect_contours(bgr_image, hsv_image, self.left_goal_limits.low, self.left_goal_limits.high, debug_pub_left)
        right_contours =  self.detect_contours(bgr_image, hsv_image, self.right_goal_limits.low, self.right_goal_limits.high, debug_pub_right)

        return left_contours, right_contours

    def contour_to_object(self, contour):
        """
        :return: Obstacle from contour
        """
        ob = Object()

        moment = cv2.moments(contour)

        if moment['m00'] != 0:
            ob.x = int(moment['m10'] / moment['m00'])
            ob.y = int(moment['m01'] / moment['m00'])
        else:
            raise RuntimeError("Contour Moment Zero")

        min_x = sys.maxsize
        max_x = 0
        min_y = sys.maxsize
        max_y = 0

        for p in contour:
            x, y = p[0]
            min_x = min(min_x, x)
            max_x = max(max_x, x)
            min_y = min(min_y, y)
            max_y = max(max_y, y)

        ob.width = max_x - min_x
        ob.height = max_y - min_y

        return ob

    def callback_compressed(self, compressed_image):
        '''
        callback for compressed images
        NOTE: do not use both at same time : (callback_compressed & callback)
        '''
        try:
            bgr_image = self.bridge.compressed_imgmsg_to_cv2(compressed_image, desired_encoding="bgr8")
            self.process_bgr_image(bgr_image)
        except CvBridgeError as e:
            rospy.logerr(e)

    def callback(self, raw_image):
        '''
        callback for raw images
        NOTE: do not use both at same time : (callback_compressed & callback)
        '''
        try:
            bgr_image = self.bridge.imgmsg_to_cv2(raw_image, desired_encoding="bgr8")
            self.process_bgr_image(bgr_image)
        except CvBridgeError as e:
            rospy.logerr(e)

    def process_bgr_image(self, bgr_image):
        try:

            hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

            obstacle_contours = self.detect_obstacles(bgr_image, hsv_image)
            left_goal_contours, right_goal_contours = self.detect_goals(bgr_image, hsv_image)

            obstacles = Obstacles()
            for c in obstacle_contours:
                try:
                    obstacle = self.contour_to_object(c)
                    obstacles.Obstacles.append(obstacle)
                except RuntimeError as e:
                    pass

            goals = Goals()
            for c in left_goal_contours:
                try:
                    goals.left_goals.append(self.contour_to_object(c))
                except RuntimeError:
                    pass

            for c in right_goal_contours:
                try:
                    goals.right_goals.append(self.contour_to_object(c))
                except RuntimeError:
                    pass

            self.obstacle_publisher.publish(obstacles)

            self.goal_publisher.publish(goals)

            if self.debug_mode:
                result_img = bgr_image.copy()

                cv2.drawContours(result_img, obstacle_contours, -1, (0, 255, 0), 3)
                cv2.drawContours(result_img, left_goal_contours, -1, (255, 0, 0), 3)
                cv2.drawContours(result_img, right_goal_contours, -1, (0, 0, 255), 3)
                self.debug_result_image_pub.publish(self.bridge.cv2_to_imgmsg(result_img, "bgr8"))

        except CvBridgeError as e:
            rospy.loginfo(e)

    def reconfigure_callback(self, pdic, level):
        '''
        every time a parameter changes through dynamic reconfigure this callback gets called
        and all parameters are provided as a dictionary (pdic)
        '''
        self.obstacle_color_limits.low = (pdic['obs_h_lo'], pdic['obs_s_lo'], pdic['obs_v_lo'])
        self.obstacle_color_limits.high = (pdic['obs_h_hi'], pdic['obs_s_hi'], pdic['obs_v_hi'])
        self.left_goal_limits.low = (pdic['goal_left_h_lo'], pdic['goal_left_s_lo'], pdic['goal_left_v_lo'])
        self.left_goal_limits.high = (pdic['goal_left_h_hi'], pdic['goal_left_s_hi'], pdic['goal_left_v_hi'])
        self.right_goal_limits.low = (pdic['goal_right_h_lo'], pdic['goal_right_s_lo'], pdic['goal_right_v_lo'])
        self.right_goal_limits.high = (pdic['goal_right_h_hi'], pdic['goal_right_s_hi'], pdic['goal_right_v_hi'])
        self.binarization_threshold = pdic['binarization_threshold']
        self.debug_mode = pdic['debug']
        # return a possibly updated configuration (mandatory)
        return pdic

    def start(self):
        # Enable dynamic reconfigure parameters
        srv = Server(rdl_rgb_obstacle_detectionConfig, callback=self.reconfigure_callback)
        # wait for ctrl + c to be pressed (avoid killing process so that callbacks can do their work)
        rospy.spin()
        # user has pressed ctrl + c, shut down safely
        rospy.loginfo("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node("rdl_rgb_obstacle_detection")
    obj_detector = ObjectDetector()
    obj_detector.start()
