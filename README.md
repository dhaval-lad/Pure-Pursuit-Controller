# pure_pursuit_controller

## Description
The `pure_pursuit_controller` package is responsible for subscribing to `/planned_path` and `/odom`, and publishing `/cmd_vel` to follow the path using the Pure Pursuit controller.

## Dependencies
- [Ubuntu 22.04](https://releases.ubuntu.com/22.04/)
- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
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
This can be varified either in simulation or in real GrassHopper robot. 
### Simulation Setup
To verify that everything is working in simulation:
1. Run the Gazebo world with the robot in it. You can use GrassHopper simulation model available at: [GrassHopper Gazebo](https://github.com/dhaval-lad/grasshopper_gazebo.git)
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
### Real World Setup
To verify that everything is working onGrassHopper:
1. Start the GrassHopper robot with GPS, IMU and dual-EKF node to get odometry information (x,y,yaw). 
2. Change the topic name if needed, for now the node is configured specificly for it. No change require. 
3. Adjust the `self.coordinates` in `path_publisher_node.py` according to the robot's initial position in the world and the path you want the robot to be follow. 
4. Rebuild the package:
    ```sh
    cd ~/ros2_ws
    colcon build --packages-select pure_pursuit_controller
    ```
5. Run the following command in the terminal to publish the path every 2 seconds:
    ```sh
    ros2 run pure_pursuit_controller path_publisher_node
    ```
6. Now run the `pure_pursuit_node_real` to see that the robot is following the path:
    ```sh
    ros2 run pure_pursuit_controller pure_pursuit_node_real
    ```

## Configuration
You can change the `lookahead_distance` and `speed` in `pure_pursuit_controller.py`. For `pure_pursuit_controller_real.py`, these parameter are tuned and tested do not need to change them. 

## Nodes
- **path_publisher_node**: Responsible for publishing the path on `/planned_path` for `pure_pursuit_node` to follow.
- **pure_pursuit_node**: Responsible for subscribing to `/planned_path` and `/odom`, and publishing `/cmd_vel` to follow the specified path.
- **pure_pursuit_node_real**: Responsible for subscribing to `/planned_path`, `/odometry/global` and `/bno055/imu/corrected`, and publishing `/cmd_vel` to follow the specified path. This node is configured specifically for real GrassHopper robot. 

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
- **pure_pursuit_node_real**:
  - **Published Topics**:
    - `/cmd_vel`
  - **Subscribed Topics**:
    - `/planned_path`
    - `/odometry/global`
    - `/bno055/imu/correcte`

## License
This project is licensed under the Apache License, Version 2.0, January 2004. See [http://www.apache.org/licenses/](http://www.apache.org/licenses/) for more details.
