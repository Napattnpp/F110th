# F110th

# First Setup
```
sudo apt update && rosdep update
sudo apt install ros-$ROS_DISTRO-diagnostic-updater
rosdep install --from-paths src --ignore-src -y
colcon build
```

## udev Rules Setup
- **Get vensor id and product id:** `lsusb`
- **Crate a rules file:** `sudo nano /etc/udev/rules.d/99-f1tenth-usb.rules`

```
# Arduino MKR Zero for micro-ROS
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="YYYY", MODE="0666", SYMLINK+="sensors/micro_ros"

# VESC Motor Controller
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="YYYY", MODE="0666", SYMLINK+="sensors/vesc"
```

`sudo udevadm control --reload-rules && sudo udevadm trigger`

---

### Foxglove bridge / ROS bridge
- `sudo apt install ros-$ROS_DISTRO-foxglove-bridge`
> [!NOTE]
> Distributions older than Humble: `sudo apt install ros-foxy-rosbridge-server`


### Micro ROS
- **Download micro-ROS agent packages:** `ros2 run micro_ros_setup create_agent_ws.sh`
- **Build step:**
    ```
    ros2 run micro_ros_setup build_agent.sh
    source install/local_setup.bash
    ```


# Execute
- ## Run F1 tenth stack
    - `ros2 launch f1tenth_stack bringup_launch.py`
- ## Run micro ros
    - `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/sensors/micro_ros`

- ## Run Foxglove bridge / ROS bridge
    - ***Foxglove bridge:*** `ros2 launch foxglove_bridge foxglove_bridge_launch.xml`
    - ***ROS bridge:*** `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`