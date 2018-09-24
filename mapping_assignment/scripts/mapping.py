#!/usr/bin/env python3

"""
    # Rizwan Asif
    # 921008-8556
    # rasif@kth.se
"""

# Python standard library
from math import cos, sin, atan2, fabs
import math

# Numpy
import numpy as np

# "Local version" of ROS messages
from local.geometry_msgs import PoseStamped, Quaternion
from local.sensor_msgs import LaserScan
from local.map_msgs import OccupancyGridUpdate

from grid_map import GridMap
from copy import deepcopy


class Mapping:
    def __init__(self, unknown_space, free_space, c_space, occupied_space,
                 radius, optional=None):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {"self.unknown_space": self.unknown_space,
                                      "self.free_space": self.free_space,
                                      "self.c_space": self.c_space,
                                      "self.occupied_space": self.occupied_space}
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception("{0} is not an allowed value to be added to the map. "
                            .format(value) + "Allowed values are: {0}. "
                            .format(self.allowed_values_in_map.keys()) +
                            "Which can be found in the '__init__' function.")

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def update_map(self, grid_map, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.
        
        For E: 
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """
        grid_map0 = deepcopy(grid_map)

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()


        """
        Fill in your solution here
        """
        robPos = [pose.pose.position.x, pose.pose.position.y, robot_yaw]
        traces = []
        
        min_x = 100000
        min_y = 100000
        max_x = -1
        max_y = -1
        
        for i in range(len(scan.ranges)):
            lv = scan.ranges[i]
            if lv > scan.range_min and lv < scan.range_max:
                x = robPos[0]-origin.position.x + lv*math.cos(scan.angle_min + robPos[2] + i*scan.angle_increment)
                y = robPos[1]-origin.position.y + lv*math.sin(scan.angle_min + robPos[2] + i*scan.angle_increment)

                x = int(x/resolution)
                y = int(y/resolution)

                #Add to trace
                t = self.raytrace( ((robPos[0]-origin.position.x)/resolution,(robPos[1]-origin.position.y)/resolution) ,(x,y) )
                for v in t:
                    if v[0] < min_x:
                         min_x = v[0]
                    if v[1] < min_y:
                         min_y = v[1]
                    if v[0] > max_x:
                         max_x = v[0]
                    if v[1] > max_y:
                         max_y = v[1]

                    if not v[0]==x or not v[1]==y:
                        if not grid_map[v[0],v[1]]==self.free_space:
                            self.add_to_map(grid_map,v[0],v[1],self.free_space)
                            pass

                self.add_to_map(grid_map,x,y,self.occupied_space)

     
        """
        For C only!
        Fill in the update correctly below.
        """
        trace_data = [self.free_space for _ in range(len(traces))]
 
        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = min_x
        # The minimum y index in 'grid_map' that has been updated
        update.y = min_y
        # Maximum x index - minimum x index + 1
        update.width = max_x-min_x+1
        # Maximum y index - minimum y index + 1
        update.height = max_y-min_y+1
        # The map data inside the rectangle, in row-major order.
        data = []
        for j in range(update.height):
            for i in range(update.width):
                if self.is_in_bounds(grid_map,min_x+i,min_y+j):
                    data.append(grid_map0[min_x+i,min_y+j])
        update.data = data
        #print("Updatd: {}".format(np.shape(update.data)))

        # Return the updated map together with only the
        # part of the map that has been updated
        #return grid_map, update
        
        #grid_map0 = GridMap()
        return grid_map, update

    def getSquareCoords(self, center, res):
        data = []
        for i in range(1,res):
            data.append([center[0]+i,center[1]])
            data.append([center[0],center[1]+i])
            data.append([center[0]+i,center[1]+i])
            data.append([center[0]-i,center[1]])
            data.append([center[0],center[1]-i])
            data.append([center[0]-i,center[1]-i])
        return data

    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.
        
        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """


        """
        Fill in your solution here
        """
        origin = grid_map.get_origin()
        resolution = grid_map.get_resolution()
        width = grid_map.get_width()
        height = grid_map.get_height()
        radius_res = int(self.radius)
        #print("{}*{} = {}".format(self.radius, resolution, radius_res))

        for i in range(width):
            for j in range(height):
                x = int(origin.position.x + i)
                y = int(origin.position.y + j)
                if self.is_in_bounds(grid_map,x,y):
                    if grid_map[x,y] == self.occupied_space:
                        points = self.getSquareCoords([x,y], radius_res)
                        for p in points:
                            if self.is_in_bounds(grid_map, p[0], p[1]) and \
                            not grid_map[p[0],p[1]]==self.occupied_space:
                                self.add_to_map(grid_map, p[0],p[1],self.c_space)
        # Return the inflated map
        return grid_map
