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

- ## Recording data
    - **MCAP:** `ros2 bag record -s mcap -o <file_name> -a`
    - **.db3:** `ros2 bag record -o <file_name> -a`

---

### TODO
 - ## Calibrating the Odometry
    ### 1. Tuning the steering
    - Adjust `steering_angle_to_servo_gain` \
        in `/F110th/src/f1tenth_system/f1tenth_stack/config/vesc.yaml`
    ### 2. Linear calibration
    - #### 2.1. Set max/min speed
        - Adjust `speed_min` and `speed_max` \
            in `/F110th/src/f1tenth_system/f1tenth_stack/config/vesc.yaml`
    - #### 2.2. Direction in odom
        - Change the sign of this expression: `(-state->state.speed - speed_to_erpm_offset_)` \
            in `/F110th/src/vesc/vesc_ackermann/src/vesc_to_odom.cpp` 
    - #### 2.3. Determine ERPM gain
        Now we have to determine the `speed_to_erpm_gain` and **Joy teleop scale** \
        in `/F110th/src/f1tenth_system/f1tenth_stack/config/vesc.yaml`
        and `/F110th/src/f1tenth_system/f1tenth_stack/config/joy_teleop.yaml` **(Line 33)**, respectively.

        By using this equation:  `(speed_to_erpm_gain + x) * (scale + y) = MAX ERPM`
> [!NOTE]
> `MAX ERPM` is the maximum ERPM value that you want the car can goes. \
    It can be any number that you want or even `MAX ERPM` = `speed_max`, but should not higher than `speed_max` \
> [Use this website to help you plot the graph and pick the right value](https://www.desmos.com/calculator)
>
> [For more information on calibrating the odometry](https://f1tenth.readthedocs.io/en/foxy_test/getting_started/driving/drive_calib_odom.html)
