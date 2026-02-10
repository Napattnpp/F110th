# F110th

# First Setup
```
sudo apt update && rosdep update
sudo apt install ros-$ROS_DISTRO-diagnostic-updater
rosdep install --from-paths src --ignore-src -y
colcon build
```

## Micro ROS
- **Download micro-ROS agent packages:** `ros2 run micro_ros_setup create_agent_ws.sh`
- **Build step:**
    ```
    ros2 run micro_ros_setup build_agent.sh
    source install/local_setup.bash
    ```

# Run micro ros
`ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0`
