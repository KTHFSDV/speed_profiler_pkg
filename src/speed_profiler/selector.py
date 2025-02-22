from speed_profiler.optimizer import SpeedProfileOptimizer
from speed_profiler.ros_interface import Logger as logger

import rospy
import numpy as np

class SpeedProfileSelector(object):
    """
    This class provides methods that add a speed profile based on factors like
    the car's mission, lap number, etc. to the given path.
    """

    def __init__(self, parameters, speed_profiler, mission):

        ## If the MSE to the last paths position is below this value, the old profile will be used.
        self._path_similar_mse = parameters['path_similar_mse']
        
        ## Speed in Lap 1 or if speed optimization fails
        self._safe_speed = parameters['safe_speed']
        
        ## Eventspecific speed limits
        self.real_speed_limit = parameters['speed_limit']
        self.acceleration_speed_limit = parameters['acceleration_speed_limit']
        self.ebs_speed_limit = parameters['EBS_speed_limit']

        ##Selected mission
        self.mission = mission

        self._acceleration_event = False
        if self.mission == 1:
            self._acceleration_event = True 
            self.real_speed_limit = self.acceleration_speed_limit
        elif self.mission == 4:
            self.real_speed_limit = self.ebs_speed_limit
        
        ## Instance of the SpeedProfiler class
        self._speed_profiler = speed_profiler
        
        ## Previous path for which speed profile was computed including the speed profile
        self._previous_path = None

        self.time = 0  

        ## Lap 1 complete flag
        self._in_first_lap = parameters['safe_lap']
        self.lap = 1

    def update_lap(self, lap):
        """ Check if car is in first lap """
        if lap > 1:
            self._in_first_lap = False
            self.time = rospy.Time.now().to_sec() 
            self.lap = lap
    
    def update_mission(self, mission):
        """ Check if mission is the same """
        self.mission = mission
        if self.mission == 1:
            self._acceleration_event = True 
            self.real_speed_limit = self.acceleration_speed_limit
        elif self.mission == 4:
            self.real_speed_limit = self.ebs_speed_limit

    def get_distances_from_points(self, x, y):
        """ Computes distances between points on path.

        @param x: List of N x coordinates on path
        @param y: List of N y coordinates on path
        @return List of N-1 euclidean distances between the (x,y) points.
        """
        return np.linalg.norm(np.diff(np.array([x, y]), axis=1), axis=0).tolist()

    def check_paths_similar(self, path1, path2, mse_threshold):
        """ Checks if two paths are similar (by computing mse between points).

        @param path1 (fs_msgs/PlannedPath)
        @param path2 (fs_msgs/PlannedPath)
        @param mse_threshold
        @return True or False
        """
        # Not similar if one path is None
        if path1 is None or path2 is None:
            return False
        # Not similar if different number of points
        if len(path1.x) != len(path2.x):
            return False
        errors = np.square(np.subtract(path2.x, path1.x)) + \
            np.square(np.subtract(path2.y, path1.y))
        return errors.mean() < mse_threshold

    def select_speed_profile(self, path, pose):
        """ 
        Add an appropriate speed profile to the given path.

        @param path fs_msgs/PlannedPath without speed profile
        @return fs_msgs/PlannedPath with speed profile
        """

        # Check if the car is in the first lap and not in the acceleration event
        if self._in_first_lap and not self._acceleration_event:
            logger.debug("In first lap. Returning constant speed.")
            return self._use_safe_speed(path)

        # Check if the current path is too similar to the previous one
        if self.check_paths_similar(path, self._previous_path, self._path_similar_mse):
            logger.debug("Path too similar. Using previous speed profile.")
            return self._use_previous_speed_profile(path)

        # Use the last known velocity or a safe speed as the initial velocity
        v_init = self._safe_speed
        if self._previous_path:
            v_init = self._previous_path.speed_profile[-1]
        
        # Attempt to compute the optimal speed profile
        path = self._use_optimal_speed_profile(path, speed_limit=self.real_speed_limit, v_init=v_init)

        # Update and return the path if an optimal speed profile is available
        if path.speed_profile is not None:
            logger.debug("Using last known velocity optimal speed profile.")
            self._previous_path = path
            return path

        # Attempt to compute the speed profile
        path = self._use_optimal_speed_profile(path, speed_limit=self.real_speed_limit, v_init=self._safe_speed)
        # Update and return the path if an optimal speed profile is available
        if path.speed_profile is not None:
            logger.debug("Using safe speed optimal speed profile.")
            self._previous_path = path
            return path

        # Fallback to a safe speed if optimization fails
        logger.warning("Optimization failed. Falling back to safe speed.")
        return self._use_safe_speed(path)

    def _use_safe_speed(self, path):
        """ Add profile with constant speed to path.

        @param path fs_msgs/PlannedPath without speed profile
        @return fs_msgs/PlannedPath with speed profile
        """
        path.speed_profile = len(path.x) * [self._safe_speed]
        return path

    def _use_previous_speed_profile(self, path):
        """ Add previously used speed profile to path.

        @param path fs_msgs/PlannedPath without speed profile
        @return fs_msgs/PlannedPath with speed profile
        """
        path.speed_profile = self._previous_path.speed_profile
        return path

    def _use_optimal_speed_profile(self, path, speed_limit, v_init=None, v_final=None):
        """ Compute optimal speed profile and add it to path.

        @param path fs_msgs/PlannedPath without speed profile
        @param v_init (optional): Initial speed on the path (in m/s)
        @param v_final (optional): Final speed on the path (in m/s)
        @return fs_msgs/PlannedPath with speed profile
        """
        path.speed_profile = self._speed_profiler.compute_speed_profile(
            curvatures=path.curvatures,
            distances=self.get_distances_from_points(path.x, path.y),
            v_init=v_init, v_final=v_final,
            speed_limit=speed_limit)
        return path

    def get_closest_waypoint(waypoints, car_pose):
        """
        Finds the closest waypoint that is in front of the car.
        
        Args:
            waypoints (list of np.ndarray): List of 2D waypoints (x, y).
            car_pose (np.ndarray): The car's pose as [x, y, theta].

        Returns:
            int: Index of the closest waypoint in front of the car.
        """
        min_distance = float('inf')
        closest_waypoint = -1

        for i in range(len(waypoints)):
            waypoint_dir = waypoints[i] - car_pose[:2]
            waypoint_angle = np.arctan2(waypoint_dir[1], waypoint_dir[0])

            # Check if the waypoint is in front of the car
            angle_dif = angle_diff(car_pose[2], waypoint_angle)
            if -np.pi / 2 <= angle_dif <= np.pi / 2:
                # This waypoint is in front of the car
                return min(i, len(waypoints) - 1)

        return len(waypoints) - 1

    def angle_diff(theta1, theta2):
        """
        Computes the smallest angular difference between two angles.
        
        Args:
            theta1 (float): First angle (in radians).
            theta2 (float): Second angle (in radians).

        Returns:
            float: Angular difference in radians, normalized between -pi and pi.
        """
        return (theta2 - theta1 + np.pi) % (2 * np.pi) - np.pi

    def _use_skidpad_speed_profile(self, path, pose):
        """ Add profile with constant speed to path.

        @param path fs_msgs/PlannedPath without speed profile
        @return fs_msgs/PlannedPath with speed profile
        """

        if self._previous_path:
            return self._previous_path.speed_profile
    
        return self._use_optimal_speed_profile(path, speed_limit=self.real_speed_limit, v_init=self._safe_speed)

