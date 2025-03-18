#!/usr/bin/env python
import rospy
from speed_profiler.optimizer import SpeedProfileOptimizer
from speed_profiler.selector import SpeedProfileSelector
from speed_profiler.ros_interface import SpeedProfilerROSInterface
from speed_profiler.ros_interface import Logger as logger


if __name__ == '__main__':

    ## ROS Interface for speed profiler
    ros_interface = SpeedProfilerROSInterface(rate=30, log_level=logger.INFO)

    ## Speed profile optimizer instance
    speed_profiler = SpeedProfileOptimizer(ros_interface.parameters)

    ## Speed profile selector instance
    ## Note: later, SpeedProfileSelector can also be used for selecting the
    ## speed profile based on different missions (trackdrive, ...)
    speed_profile_selector = SpeedProfileSelector(
        ros_interface.parameters, speed_profiler, ros_interface.mission)

    while not ros_interface.is_shutdown():

        path_from_planner = ros_interface.check_path_received()
        if path_from_planner is not None:
            # Update the speed profile selector with all information
            # that it needs to select the speed profile.
            # Later, a subscriber to a state / lap number could be added
            # in ROSInterface and be forwarded to the SpeedProfileSelector here.
            speed_profile_selector.update_lap(ros_interface._lap_counter)
            speed_profile_selector.update_mission(ros_interface.mission)
            
            try:
                path = speed_profile_selector.select_speed_profile(
                    path_from_planner, ros_interface.vehicle_pose)

                ros_interface.publish_path(path)
                ros_interface.publish_visualisation_marker(path)

            except Exception as e:
                # If some unexpected error occurs,
                # just publish path as it comes from path planner.
                logger.error("Unexpected error. No speed profile inserted. \
                    Error was " + str(e))
                ros_interface.publish_path(path_from_planner)
                
                ros_interface.publish_visualisation_marker(path_from_planner)

        ros_interface.sleep()
