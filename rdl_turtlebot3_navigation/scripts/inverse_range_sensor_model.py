#!/usr/bin/python

import math
import numpy as np

class InverseRangeSensorModel(object):
    '''
    Identify if a cell is occupied, free or unknown based upon
    laser scanner readings and sensor pose
    '''
    def __init__(self, angle_min, angle_max, angle_increment, \
            z_max, map_origin_x, map_origin_y, resolution, \
            map_width, obstacle_thickness):
        '''
        Constructor
        Input:
            angle_min, angle_max - the min/max angle of the sensor (cone)
            angle_increment - angle between beams
            z_max - maximum range of the laser in meters
            map_origin_x, map_origin_y - offset between cell world and real world
            resolution - map resolution in cells per pixel
            map_width - the width of the map in cells
            obstacle_thickness - size of the smallest obstacle in the room in meters
        '''
        # laser scanner params, save to member variables
        # ---
        # laser scaner minimum and maximum angles
        self.angle_min = angle_min
        self.angle_max = angle_max
        # angle increment in radians between laser beams
        self.angle_increment = angle_increment
        # maximum beam lenght
        self.z_max = z_max
        # allow to set one time z_t to speed up
        self.z_t = None

        # map parameters, save to member variables
        # ---
        # map resolution in meters per cell
        self.resolution = resolution
        # offset between world and map origin
        self.world_offset_x = map_origin_x
        self.world_offset_y = map_origin_y
        # obstacle thickness
        self.alpha = obstacle_thickness
        # saving map width to member variable
        self.map_width = map_width
        # ============= YOUR CODE GOES HERE! =====
        # hint : feel free to add initialization code here if needed
        # ============= YOUR CODE ENDS HERE! =====

    def set_z_t (self, z_t):
        '''
        allows to set array of ranges
        '''
        self.z_t = z_t

    def update_cell(self, cell_index, x_t):
        '''
        # ============= YOUR CODE GOES HERE! =====
        hint : compute distance between sensor and cell
        hint : check if cell is outside the cone
        hint : find the index of the beam that best hits the cell
        hint : check if cell is behind an occupied cell (alpha is obstacle thickness) if true then return unknown
        hint : for ROS, 0  means cell is free, 100 is occupied and -1 unknown
        hint : feel free to add as many functions above as required
        # ============= YOUR CODE ENDS HERE! =====
        '''
