# Speed Profiler

The Speed Profiler uses OSQP quadratic problem solvers to compute an optimal speed profile for a given path. The custom configurations for various types of races (e.g., Acceleration, Skidpad) are specified in the `config/speed_profile.yaml` file.

## Configuration

The `config/speed_profile.yaml` file is organized by race type, each containing specific parameters:

- **alpha_smooth**: Smoothening parameter used as a weight in the cost function to penalize large accelerations.
- **safe_speed**: The speed for the first lap or fallback speed if the solver fails.
- **EBS_speed_limit**: Speed limit for the Emergency Brakes System (EBS).
- **acceleration_speed_limit**: Maximum allowable speed limit during acceleration.
- **speed_limit**: General speed limit for the race.
- **lateral_acceleration_limit**: Maximum lateral acceleration limit.
- **accel_min**: Minimum acceleration.
- **accel_max**: Maximum acceleration.
- **path_similar_mse**: If the Mean Squared Error (MSE) of current and previous path positions is below this threshold, speed profile computation is skipped, and the previous solution is reused.

## Features

- Optimal speed profile computation for different race missions.
- Customizable configuration for various race types.
- Safe fallback mechanisms for handling solver failures.
- Efficient reuse of previous speed profiles when current paths are similar.

## Usage

1. Run `docker compose up`.
2. Run `docker ps` to find the docker image id for the speed profiler.
3. Run `exec -it <id> bash` to get into the speed profiler docker image.
4. When inside the docker image run `Source Devel/Setup.bash`.
5. Configure your race type parameters in `config/speed_profile.yaml`.
6. Run `Catkin Build` to build the program.
7. Run the Speed Profiler: `roslaunch speed_profiler run_speed_profiler.launch`.
8. Play rosbag with topic `/navigation/speed_profiler/path` that has type `fs_msgs/PlannedPath`
9. For visualization: RVIZ or other similar program. Add topic `/navigation/speed_profiler/path`

## Coming soon (Hopefully!)

- MKL pardiso solver for parallization.
- Maybe hardcoded skidpad speed profiler if needed.




