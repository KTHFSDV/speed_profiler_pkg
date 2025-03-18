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
        self.update_mission(mission)
        
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
        # return np.linalg.norm(np.diff(np.array([x, y]), axis=1), axis=0).tolist()
        diffs = np.diff(np.array([x, y]), axis=1)
        distances = np.linalg.norm(diffs, axis=0)
        return distances.tolist()

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

        if self.mission == 2:
            path = self._use_skidpad_speed_profile(path)
            
        else:
            path = self._use_speed_profile(path)
            
        return path

    def _use_speed_profile(self, path):
      
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
          
        logger.warning("Optimization failed. Falling back to safe speed.")
        return self._use_safe_speed(path)

    # Define a function to calculate the Exponential Moving Average (EMA)
    def calculate_ema(self, data, alpha=0.1):
        ema_values = np.zeros_like(data)
        ema_values[0] = data[0]  # Initial value is the first data point
        for i in range(1, len(data)):
            ema_values[i] = alpha * data[i] + (1 - alpha) * ema_values[i - 1]
        return ema_values

    def _use_skidpad_speed_profile(self, path):
        """
        Adjusts the speed profile for the given skidpad path.
        """
        logger.debug("Using skidpad speed profile")
        
        if not self._previous_path:
            x = np.array(path.x)
            y = np.array(path.y)
            curvatures = np.array(path.curvatures)

            # Smooth curvature due to curvature data bug
            smoothed_curvatures_ema = self.calculate_ema(curvatures, 0.1)

            # Initialize the speed profile with zeros
            speed_profile = np.zeros_like(path.x)

            tolerance = 1e-5  
            # Calculate the squared distance between consecutive points to check proximity
            diffs_x = np.diff(x)
            diffs_y = np.diff(y)

            # Check if the squared distance is less than or equal to the tolerance
            distances_squared = diffs_x**2 + diffs_y**2
            duplicates = distances_squared <= tolerance**2  # Consider points duplicates if distance is less than tolerance

            # Replace the curvatures of duplicates with the average of the previous and next curvature
            new_curvatures = np.copy(smoothed_curvatures_ema)

            # Filter out the duplicates by keeping only the first occurrence
            new_x = x[np.concatenate(([True], ~duplicates))]
            new_y = y[np.concatenate(([True], ~duplicates))]
            new_curvatures = new_curvatures[np.concatenate(([True], ~duplicates))]

            new_distances = np.array(self.get_distances_from_points(new_x, new_y))
            # Find indices where new_distances exist in the original distances
            new_speed_profile = self._speed_profiler.compute_speed_profile(
                curvatures=new_curvatures,
                distances=new_distances,
                v_init=self._safe_speed, v_final=None,
                speed_limit=self.real_speed_limit
            )

            for i in range(len(new_x)):
                index = np.where((x == new_x[i]) & (y == new_y[i]))[0][0]
                speed_profile[index] = new_speed_profile[i]

            for i in range(len(x)):
                if speed_profile[i] == 0:
                    if i == len(path.x) - 1:  # Last element, average with previous
                        speed_profile[i] = speed_profile[i-1]
                    else:  # Middle elements, average with both neighbors
                        speed_profile[i] = (speed_profile[i-1] + speed_profile[i+1]) / 2

            path.speed_profile = speed_profile

            
        else:
            prev_speed_profile = self._previous_path.speed_profile
            path.speed_profile = prev_speed_profile

        
        
        self._previous_path = path
        return path 

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

    def _use_skidpad_speed_profile(self, path, pose):
        """ Add profile with constant speed to path.

        @param path fs_msgs/PlannedPath without speed profile
        @return fs_msgs/PlannedPath with speed profile
        """

        if self._previous_path:
            return self._previous_path.speed_profile
    
        return self._use_optimal_speed_profile(path, speed_limit=self.real_speed_limit, v_init=self._safe_speed)
