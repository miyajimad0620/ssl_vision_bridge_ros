<!-- Copyright 2024 TRAPS

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. -->
# ssl_vision_bridge_ros

ROS 2 package for bridging [SSL-Vision](https://github.com/RoboCup-SSL/ssl-vision) outputs into ROS 2 messages for use in RoboCup Small Size League environments.

## Features

- Receives detection and geometry data from SSL-Vision via UDP
- Converts data into ROS 2 messages
- Publishes tracked and untracked detection messages
- Provides RViz visualization support
- ROS 2 launch support with `.launch.xml` files

## Dependencies

- ROS 2 (tested with Humble or newer)
- Protobuf
- Colcon build system

## Build Instructions

```bash
# Clone the repository into your ROS 2 workspace
cd ~/ros2_ws/src
git clone https://github.com/your-org/ssl_vision_bridge_ros.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --packages-select ssl_vision_bridge_ros

# Source the environment
source install/setup.bash
```

## Usage

```bash
# Launch the bridge node
ros2 launch ssl_vision_bridge_ros bridge.launch.xml

# (Optional) Launch RViz with config
ros2 launch ssl_vision_bridge_ros rviz.launch.xml
```
