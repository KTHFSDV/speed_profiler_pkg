import numpy as np

from speed_profiler.interpolate import interpolate_with_least_squares


def test_interpolate_with_least_squares(planner_parameters):
    points_per_waypoint = planner_parameters['points_per_waypoint']
    path_gen_margin = planner_parameters['path_gen_margin']
    total_waypoints = 10

    total_points_in_path = total_waypoints * points_per_waypoint
    expected_positive = np.linspace(
        1 - path_gen_margin,
        (total_waypoints - 1) + path_gen_margin,
        total_points_in_path
    )
    expected_negative = np.negative(expected_positive)
    expected_zero = [0] * total_points_in_path

    test_cases = [
        {
            'initial_vehicle_position': np.array([0, 0]),
            'waypoints': np.array([[x, 0] for x in range(total_waypoints)]),
            'expected_path': np.column_stack((expected_positive, expected_zero))
        },
        {
            'initial_vehicle_position': np.array([0, 0]),
            'waypoints': np.array([[x, 0] for x in range(0, -total_waypoints, -1)]),
            'expected_path': np.column_stack((expected_negative, expected_zero))
        },
        {
            'initial_vehicle_position': np.array([0, 0]),
            'waypoints': np.array([[0, y] for y in range(total_waypoints)]),
            'expected_path': np.column_stack((expected_zero, expected_positive))
        },
        {
            'initial_vehicle_position': np.array([0, 0]),
            'waypoints': np.array([[0, y] for y in range(0, -total_waypoints, -1)]),
            'expected_path': np.column_stack((expected_zero, expected_negative))
        },
    ]

    for test_case in test_cases:
        initial_vehicle_position = test_case['initial_vehicle_position']
        waypoints = test_case['waypoints']
        expected_path = test_case['expected_path']
        path = interpolate_with_least_squares(
            waypoints, initial_vehicle_position, points_per_waypoint, path_gen_margin
        )

        assert len(path) == len(expected_path)
        assert np.linalg.norm(path - expected_path) < 0.1
