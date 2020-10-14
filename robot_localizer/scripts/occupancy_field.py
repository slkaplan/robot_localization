""" An implementation of an occupancy field that you can use to implement
    your particle filter """

import rospy

from nav_msgs.srv import GetMap
import numpy as np
from queue import Queue

class OccupancyField(object):
    """ Stores an occupancy field for an input map.  An occupancy field returns
        the Manhattan distance to the closest obstacle for any coordinate in the map
        Attributes:
            map: the map to localize against (nav_msgs/OccupancyGrid)
            closest_occ: the distance for each entry in the OccupancyGrid to
            the closest obstacle stored as a numpy array
    """
    def get_closest_obstacle_matrix(self):
        q = np.hstack((self.occupied, np.zeros((self.occupied.shape[0],1)))).astype(np.int)
        closest_occ = -1*np.ones((self.map.info.width, self.map.info.height), dtype=np.int)
        while q.shape[0] > 0:
            print("Proportion occupancy field computed:", (closest_occ != -1).mean())
            closest_occ[q[:, 0], q[:, 1]] = q[:, -1]
            # check if the cost is already in there (not done yet)
            new_queue = np.vstack((np.vstack((q[:,0]+1, q[:,1], q[:,2]+1)).T,
                                   np.vstack((q[:,0]-1, q[:,1], q[:,2]+1)).T,
                                   np.vstack((q[:,0], q[:,1]+1, q[:,2]+1)).T,
                                   np.vstack((q[:,0], q[:,1]-1, q[:,2]+1)).T))
            in_bounds = (new_queue[:, 0] >= 0) & (new_queue[:, 0] < self.map.info.width) & (new_queue[:, 1] >= 0) & (new_queue[:, 1] < self.map.info.height)
            new_queue = new_queue[in_bounds, :]
            new_queue = np.unique(new_queue, axis=0)
            new_queue = new_queue[closest_occ[new_queue[:, 0], new_queue[:, 1]] == -1, :]
            q = new_queue
        return closest_occ

    def __init__(self):
        # grab the map from the map server
        rospy.wait_for_service("static_map")
        static_map = rospy.ServiceProxy("static_map", GetMap)
        self.map = static_map().map
        self.res = self.map.info.resolution
        self.distance_map = {}
        # The coordinates of each grid cell in the map
        # while we're at it let's count the number of occupied cells
        total_occupied = 0
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order
                ind = i + j*self.map.info.width
                if self.map.data[ind] > 0:
                    total_occupied += 1
                curr += 1

        # The coordinates of each occupied grid cell in the map
        occupied = np.zeros((total_occupied, 2))
        curr = 0
        for i in range(self.map.info.width):
            for j in range(self.map.info.height):
                # occupancy grids are stored in row major order
                ind = i + j*self.map.info.width
                if self.map.data[ind] > 0:
                    occupied[curr, 0] = float(i)
                    occupied[curr, 1] = float(j)
                    curr += 1
        self.occupied = occupied
        self.closest_occ = self.get_closest_obstacle_matrix()

    def get_obstacle_bounding_box(self):
        """
        Returns: the upper and lower bounds of x and y such that the resultant
        bounding box contains all of the obstacles in the map.  The format of
        the return value is ((x_lower, x_upper), (y_lower, y_upper))
        """
        lower_bounds = self.occupied.min(axis=0)
        upper_bounds = self.occupied.max(axis=0)
        return ((lower_bounds[0]*self.res + self.map.info.origin.position.x,
                 upper_bounds[0]*self.res + self.map.info.origin.position.x),
                (lower_bounds[1]*self.res + self.map.info.origin.position.y,
                 upper_bounds[1]*self.res + self.map.info.origin.position.y))

    def get_closest_obstacle_distance(self, x, y):
        """ Compute the closest obstacle to the specified (x,y) coordinate in
            the map.  If the (x,y) coordinate is out of the map boundaries, nan
            will be returned. """
        x_coord = \
            int((x - self.map.info.origin.position.x)/self.res)
        y_coord = \
            int((y - self.map.info.origin.position.y)/self.res)

        # check if we are in bounds
        if x_coord >= self.map.info.width or x_coord < 0:
            return float('nan')
        if y_coord >= self.map.info.height or y_coord < 0:
            return float('nan')
        return self.closest_occ[x_coord, y_coord]*self.res
