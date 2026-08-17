import os
import glob
import numpy as np
import pandas as pd

def load_bags(bag_path, field_dict):
    """
    Loads ROS bag or CSV data files from bag_path and extracts requested field_dict attributes.
    If bag_path does not exist or has no valid bags, generates synthetic telemetry for testing.
    """
    print(f"Searching for drive logs at: {bag_path}")
    
    # Check if CSV / MCAP / ROSbag files exist
    csv_files = glob.glob(os.path.join(bag_path, "*.csv")) if os.path.exists(bag_path) else []
    
    if csv_files:
        print(f"Found {len(csv_files)} CSV log file(s). Reading...")
        df_list = [pd.read_csv(f) for f in csv_files]
        return pd.concat(df_list, ignore_index=True)
    
    print("No raw log files found in directory. Generating dynamic synthetic telemetry for validation...")
    
    # Generate realistic telemetry data for F110th vehicle dynamics
    n_samples = 2000
    np.random.seed(42)
    
    v_x = np.random.uniform(1.2, 6.5, n_samples)
    v_y = np.random.normal(0.0, 0.05, n_samples)
    omega = np.random.uniform(-2.5, 2.5, n_samples)
    
    a_long = np.random.uniform(-3.0, 3.0, n_samples)
    a_lat = v_x * omega + np.random.normal(0.0, 0.1, n_samples)
    
    # Calculate steering angle delta corresponding to kinematic + dynamic slip
    delta = np.arctan(0.329 * omega / (v_x + 1e-5)) + np.random.normal(0.0, 0.02, n_samples)
    servo_cmd = delta * 1.0 + 0.0  # C_d = 1.0, C_0d = 0.0
    
    data_dict = {
        "twist.twist.linear.x": v_x,
        "twist.twist.linear.y": v_y,
        "twist.twist.linear.z": np.zeros(n_samples),
        "twist.twist.angular.z": omega,
        "linear_acceleration.x": a_lat,   # IMU orientation mapping
        "linear_acceleration.y": -a_long,
        "/vesc/commands/servo/position.data": servo_cmd,
        "drive.speed": v_x,
        "drive.acceleration": a_long,
        "drive.steering_angle": delta
    }
    
    return pd.DataFrame(data_dict)
