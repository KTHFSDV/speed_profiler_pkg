import sys

import numpy as np
from numpy import random
import pytest

from speed_profiler.planner import Planner
from speed_profiler.utils import calculate_min_distance_to_point


@pytest.fixture
def planner(planner_parameters):
    return Planner(planner_parameters)


@pytest.fixture
def linear_track(planner_parameters):
    initial_vehicle_pose = np.array([0, 0, 0])

    distance_between_waypoints = 3
    total_waypoints = 10
    total_points_in_path = (total_waypoints + 1) * planner_parameters['points_per_waypoint']

    x_coordinates = range(
        distance_between_waypoints,
        total_waypoints * distance_between_waypoints,
        distance_between_waypoints
    )

    orange_cones = [np.array([0, 1]), np.array([0, -1])]
    blue_cones = [np.array([x, -1]) for x in x_coordinates]
    yellow_cones = [np.array([x, 1]) for x in x_coordinates]

    waypoints = compute_waypoints_as_cone_averages(orange_cones, blue_cones, yellow_cones)

    path_x = np.linspace(
        initial_vehicle_pose[0] - planner_parameters['path_gen_margin'],
        waypoints[-1][0] + planner_parameters['path_gen_margin'],
        total_points_in_path
    )
    path_y = [0] * total_points_in_path

    path = np.column_stack((path_x, path_y))

    return {
        'orange_cones': orange_cones,
        'blue_cones': blue_cones,
        'yellow_cones': yellow_cones,
        'waypoints': waypoints,
        'path': path,
        'initial_vehicle_pose': initial_vehicle_pose,
        'track_name': 'acceleration'
    }


@pytest.fixture
def circular_track_open(planner_parameters):
    return create_circular_track(planner_parameters, open_track=True, max_angle_degrees=300)


@pytest.fixture
def circular_track_closed(planner_parameters):
    return create_circular_track(planner_parameters)


@pytest.mark.parametrize('track', ['linear_track', 'circular_track_open', 'circular_track_closed'])
def test_generate_waypoints(planner, track, request):
    track = request.getfixturevalue(track)
    expected_waypoints = track['waypoints']

    waypoints = planner._generate_waypoints(
        track['orange_cones'], track['blue_cones'], track['yellow_cones']
    )

    assert len(waypoints) == 2 * len(expected_waypoints)

    for waypoint in waypoints:
        assert calculate_min_distance_to_point(waypoint, expected_waypoints) < sys.float_info.epsilon


@pytest.mark.parametrize('track', ['linear_track', 'circular_track_open', 'circular_track_closed'])
def test_filter_waypoints(planner, track, request):
    track = request.getfixturevalue(track)
    expected_waypoints = track['waypoints']

    # Random deviation [..., [xk, yk], ...] , where xk and yk are between -0.5 and 0.5
    random_deviation = random.random_sample((len(expected_waypoints), 2)) - 0.5

    unfiltered_waypoints = np.vstack((expected_waypoints, expected_waypoints + random_deviation))
    filtered_waypoints = planner._filter_waypoints(unfiltered_waypoints)

    assert len(filtered_waypoints) == len(expected_waypoints)
    assert np.linalg.norm(np.array(filtered_waypoints) - np.array(expected_waypoints)) < sys.float_info.epsilon


@pytest.mark.parametrize('track', ['linear_track', 'circular_track_open', 'circular_track_closed'])
def test_construct_path(planner, track, request):
    track = request.getfixturevalue(track)
    expected_path = track['path']

    planner.initial_vehicle_pose = track['initial_vehicle_pose']
    planner.track_name = track['track_name']
    path, _ = planner.construct_path(
        track['orange_cones'], track['blue_cones'], track['yellow_cones']
    )

    path_xy = path[:, 0:2]
    assert len(path_xy) == len(expected_path)
    assert np.linalg.norm(path_xy - expected_path) < 0.1


def create_circular_track(planner_parameters, open_track=False, max_angle_degrees=None):
    inner_radius = 14.0
    outer_radius = 18.0
    center_radius = (inner_radius + outer_radius) / 2

    step = 10
    if open_track:
        max_interpolated_angle_degrees = max_angle_degrees - step
    else:
        max_angle_degrees = 360
        max_interpolated_angle_degrees = 360

    angles = [np.deg2rad(theta) for theta in range(10, max_angle_degrees, step)]

    orange_cones = [np.array([inner_radius, 0]), np.array([outer_radius, 0])]
    blue_cones = [
        np.array([outer_radius * np.cos(theta), outer_radius * np.sin(theta)])
        for theta in angles
    ]
    yellow_cones = [
        np.array([inner_radius * np.cos(theta), inner_radius * np.sin(theta)])
        for theta in angles
    ]

    waypoints = compute_waypoints_as_cone_averages(orange_cones, blue_cones, yellow_cones)

    total_interpolated_angles = (
        len(waypoints) * planner_parameters['points_per_waypoint']
        if open_track
        else (len(waypoints) + 1) * planner_parameters['points_per_waypoint']
    )

    interpolated_angles = np.deg2rad(
        np.linspace(0, max_interpolated_angle_degrees, total_interpolated_angles)
    )
    path = np.array([
        [center_radius * np.cos(theta), center_radius * np.sin(theta)]
        for theta in interpolated_angles
    ])

    return {
        'orange_cones': orange_cones,
        'blue_cones': blue_cones,
        'yellow_cones': yellow_cones,
        'waypoints': waypoints,
        'path': path,
        'initial_vehicle_pose': np.array([center_radius, 0, np.pi/2]),
        'track_name': 'trackdrive'
    }


def compute_waypoints_as_cone_averages(orange_cones, blue_cones, yellow_cones):
    yellow_blue_waypoints = [
        (yellow_cone + blue_cone) / 2
        for yellow_cone, blue_cone in zip(yellow_cones, blue_cones)
    ]
    orange_waypoint = (orange_cones[0] + orange_cones[1]) / 2

    return [orange_waypoint] + yellow_blue_waypoints
