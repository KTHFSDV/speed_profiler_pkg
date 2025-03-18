import rospy

from std_msgs.msg import ColorRGBA, Int32
from visualization_msgs.msg import Marker
from fs_msgs.msg import PlannedPath
from ros2can_msgs.msg import dbu_status_1
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
import numpy as np

class SpeedProfilerROSInterface(object):

    def __init__(self, rate, log_level=rospy.INFO):

        # Initialize node
        rospy.init_node('navigation_speed_profiler', log_level=log_level)

        self._rate = rospy.Rate(rate)

        self._track_name = rospy.get_param('~track_name')
        
        #commented out for debugging
        # Set the mission based on track_name, with trackdrive as default
        self.mission = rospy.get_param('~mission')   

        self.load_parameters_for_mission(self.mission)  # Load initial mission params

        self.parameters['acceleration_event'] = self._track_name == 'acceleration'
        ## Path Frame
        # self._frame = rospy.get_param('~frame')
        self._frame = "odom"
        ## Latest path from path constructor
        self._latest_path = None
        self._lap_counter = 1
        self.parameters['safe_lap'] = rospy.get_param('/navigation/safe_lap')

        self.vehicle_pose = None

        # Subscribe to mission
        rospy.Subscriber('~dbu1', dbu_status_1, self._cb_dbu1)

        # Subscribe to path from path constructor
        rospy.Subscriber('~path/without_speeds', PlannedPath, self._cb_path)
        rospy.Subscriber('~skidpad/path/without_speeds', PlannedPath, self._cb_skidpath)
        rospy.Subscriber('~lap_counter', Int32, self._cb_lap_count)
        rospy.Subscriber('~odometry', Odometry, self._cb_odometry)

        ## Publisher to vizualize path and speed profile in rviz
        self._path_visualisation_publisher = rospy.Publisher(
            '~path/visualization', Marker, queue_size=1)

        ## Publisher to publish combined path and speed profile
        self._path_with_speeds_publisher = rospy.Publisher(
            '~path/with_speeds', PlannedPath, queue_size=1)

    def load_parameters_for_mission(self, mission):
        mission_param_prefix = 'mission_{}'.format(mission)
        self.parameters = {}
        for parameter_name in [
            'speed_limit',
            'lateral_acceleration_limit',
            'alpha_smooth',
            'accel_min',
            'accel_max',
            'safe_speed',
            'path_similar_mse',
            'EBS_speed_limit',
            'acceleration_speed_limit',
        ]:
            param_path = '/navigation/{}/{}'.format(mission_param_prefix, parameter_name)
            try:
                self.parameters[parameter_name] = float(rospy.get_param(param_path))
                rospy.loginfo("Parameter '{}' found with value '{}'.".format(parameter_name, self.parameters[parameter_name]))
            except KeyError:
                self.parameters[parameter_name] = None  # Assign a placeholder or default value


    def _cb_path(self, path):
        """ Callback for path from path constructor """
        if self.mission == 2:
            pass
        else:
            self._latest_path = path

    def _cb_skidpath(self, path):
        """ Callback for path from path constructor """
        if self.mission == 2:
            self._latest_path = path
        else:
            pass

    def _cb_lap_count(self, lap):
        """ Callback for lap"""
        self._lap_counter = lap.data

    def _cb_odometry(self, odometry):
        """ Receives vehicle pose (position and orientation)

        @param data: ROS message of typ 'Odometry', see /nav_msgs
        """
        x = odometry.pose.pose.position.x
        y = odometry.pose.pose.position.y

        self.vehicle_pose = np.array([x, y])

    def _cb_dbu1(self, msg):
        """Callback for the DBU status containing the mission selected."""
        if msg is None:
            return
        
        if msg.selected_mission != self.mission:
            self.mission = msg.selected_mission
            self.load_parameters_for_mission(self.mission)

    def check_path_received(self):
        """ Returns the newest path or None if no new path was received.
        Reset latest_path to None to indicate that the path was used. """
        path = self._latest_path
        self._latest_path = None
        return path

    def publish_path(self, path):
        """Publishes path.

        @param path: Path to publish (fs_msgs/PlannedPath)
        """
        self._path_with_speeds_publisher.publish(path)

    def publish_visualisation_marker(self, path):
        """Publishes ROS Marker for visualization of path in rviz.

        @param path: Path to visualize (fs_msgs/PlannedPath)
        """
        x, y = path.x, path.y
        speeds = path.speed_profile

        # Speeds that are used as minimum and maximum of color gradient
        minimum_speed = 0.0
        if self.mission == 4:
            maximum_speed = float(self.parameters['EBS_speed_limit'])
        else:
            maximum_speed = float(self.parameters['speed_limit'])

        def get_rgb_heat(value):
            ratio = (value - minimum_speed) / (maximum_speed - minimum_speed)
            color = ColorRGBA()
            if ratio < 0.5:
                color.r = 1
                color.g = 2.0 * ratio
            else:
                color.r = 1.0 - 2.0 * ratio
                color.g = 1
            color.b = 0.2
            color.a = 1
            return color

        line_marker = Marker()
        line_marker.header.frame_id = self._frame
        line_marker.type = line_marker.LINE_STRIP
        line_marker.action = line_marker.MODIFY
        line_marker.scale.x = 0.1
        line_marker.lifetime = rospy.Duration(0.2)
        line_marker.colors = [get_rgb_heat(value) for value in speeds]
        line_marker.points = [Point(x=x[i], y=y[i]) for i in range(len(x))]
        line_marker.pose.orientation.w = 1.0
        self._path_visualisation_publisher.publish(line_marker)

    def sleep(self):
        try:
            self._rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            # Shutdown occured during sleep
            pass

    @staticmethod
    def is_shutdown():
        return rospy.is_shutdown()


class Logger(object):

    DEBUG = rospy.DEBUG
    INFO = rospy.INFO
    WARN = rospy.WARN
    ERROR = rospy.ERROR

    @staticmethod
    def debug(text):
        rospy.logdebug('[SpeedProfiler] ' + text)

    @staticmethod
    def info(text):
        rospy.loginfo('[SpeedProfiler] ' + text)

    @staticmethod
    def warning(text):
        rospy.logwarn('[SpeedProfiler] ' + text)

    @staticmethod
    def error(text):
        rospy.logerr('[SpeedProfiler] ' + text)
