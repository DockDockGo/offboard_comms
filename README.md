# Build and Run Commands

In `mission_client.py`, populate TESTBED_EMULATOR_APP_SERVER_IP with the IP address of the machine running the AMR offboard infra backend server.

### In your ROS workspace inside the AMR docker container, run:
```
colcon build --symlink-install --packages-select-regex robot_action
colcon build --symlink-install --packages-select-regex offboard_comms
source install/setup.bash 
```
**Note**: The mission client should only be launched after the mission control action server is live on the fleet infrastructure server.
```
ros2 run offboard_comms mission_client 
```
