# pure_pursuit_controller

## Description
The `pure_pursuit_controller` package is responsible for subscribing to `/planned_path` and `/odom`, and publishing `/cmd_vel` to follow the path using the Pure Pursuit controller.

## Dependencies
- ROS 2
- A robot with odometry data

## Installation Instructions
1. First, clone this repository inside the `src` folder of your ROS 2 workspace (replace `ros2_ws` with the name of your ROS 2 workspace):
    ```sh
    cd ~/ros2_ws/src
    git clone https://github.com/dhaval-lad/Pure-Pursuit-Controller.git
    ```
2. Next, build your ROS 2 workspace to install the package (replace `ros2_ws` with the name of your ROS 2 workspace):
    ```sh
    cd ~/ros2_ws
    colcon build --packages-select pure_pursuit_controller
    ```

## Usage Instructions
To verify that everything is working:
1. Run the Gazebo world with the robot in it.
2. Change the topic name of `/odom` in `pure_pursuit_controller.py` to match the topic name in your environment.
3. Adjust the `self.coordinates` in `path_publisher_node.py` according to your robot's initial position in the world.
4. Rebuild the package:
    ```sh
    cd ~/ros2_ws
    colcon build --packages-select pure_pursuit_controller
    ```
5. Run the following command in the terminal to publish the path every 2 seconds:
    ```sh
    ros2 run pure_pursuit_controller path_publisher_node
    ```
6. Now run the `pure_pursuit_node` to see that the robot is following the path:
    ```sh
    ros2 run pure_pursuit_controller pure_pursuit_node
    ```

## Configuration
You can change the `lookahead_distance` and `speed` in `pure_pursuit_controller.py`.

## Nodes
- **path_publisher_node**: Responsible for publishing the path on `/planned_path` for `pure_pursuit_node` to follow.
- **pure_pursuit_node**: Responsible for subscribing to `/planned_path` and `/odom`, and publishing `/cmd_vel` to follow the specified path.

## Topics
- **path_publisher_node**:
  - **Published Topics**:
    - `/planned_path`
- **pure_pursuit_node**:
  - **Published Topics**:
    - `/cmd_vel`
  - **Subscribed Topics**:
    - `/planned_path`
    - `/odom`

## License
This project is licensed under the Apache License, Version 2.0, January 2004. See [http://www.apache.org/licenses/](http://www.apache.org/licenses/) for more details.
