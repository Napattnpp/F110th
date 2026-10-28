"""
F1TENTH Vehicle Profile Loader
Parses and adapts unified vehicle profiles (profiles/<car_name>.yaml) for ROS 2 launch files and nodes.
"""

import os
from pathlib import Path
from typing import Dict, Any, Optional
import yaml


def find_repo_root() -> Path:
    """Finds repository root directory containing profiles/."""
    curr = Path(__file__).resolve().parent
    for _ in range(6):
        if (curr / "profiles").exists() or (curr / ".git").exists():
            return curr
        if curr.parent == curr:
            break
        curr = curr.parent

    cwd = Path.cwd()
    for _ in range(6):
        if (cwd / "profiles").exists() or (cwd / ".git").exists():
            return cwd
        if cwd.parent == cwd:
            break
        cwd = cwd.parent

    # Default fallback
    return Path("/home/napat/F110th") if Path("/home/napat/F110th").exists() else Path.cwd()


def get_active_car_name() -> str:
    """Resolves the currently active vehicle name."""
    # 1. Explicit runtime environment variable
    if os.environ.get("ACTIVE_CAR"):
        return os.environ.get("ACTIVE_CAR")

    repo_root = find_repo_root()

    # 2. Check active symlink (e.g. profiles/active.yaml -> sim.yaml)
    active_symlink = repo_root / "profiles" / "active.yaml"
    if active_symlink.is_symlink():
        try:
            return Path(os.readlink(active_symlink)).stem
        except Exception:
            pass

    # 3. Check .env file
    dotenv = repo_root / ".env"
    if dotenv.exists():
        try:
            with open(dotenv, "r") as f:
                for line in f:
                    if line.startswith("ACTIVE_CAR="):
                        return line.strip().split("=", 1)[1].strip("'\"")
        except Exception:
            pass

    # 4. Fallback to legacy RACECAR_VERSION or default to sim
    return os.environ.get("RACECAR_VERSION", "sim")


def load_profile(car_name: Optional[str] = None) -> Dict[str, Any]:
    """Loads and returns parsed vehicle profile dictionary."""
    repo_root = find_repo_root()
    target_car = car_name if car_name else get_active_car_name()

    profile_candidates = [
        repo_root / "profiles" / f"{target_car}.yaml",
        repo_root / "profiles" / f"{target_car.lower()}.yaml",
        repo_root / "profiles" / "active.yaml",
        repo_root / "profiles" / "sim.yaml",
    ]

    for cand in profile_candidates:
        if cand.exists():
            try:
                with open(cand, "r") as f:
                    data = yaml.safe_load(f)
                    if isinstance(data, dict) and "geometry_mass" in data:
                        return data
            except Exception:
                pass

    raise FileNotFoundError(f"[Profile Loader] Could not resolve vehicle profile for '{target_car}'.")


def get_vesc_params(profile: Dict[str, Any]) -> Dict[str, Any]:
    """Adapts profile steering & powertrain settings for vesc_driver / vesc_to_odom_node."""
    sp = profile.get("steering_powertrain", {})
    gm = profile.get("geometry_mass", {})
    se = profile.get("sensor_extrinsics", {})

    return {
        "speed_to_erpm_gain": float(sp.get("speed_to_erpm_gain", -3440.0)),
        "speed_to_erpm_offset": float(sp.get("speed_to_erpm_offset", 0.0)),
        "steering_angle_to_servo_gain": float(sp.get("steering_to_servo_gain", 1.2135)),
        "steering_angle_to_servo_offset": float(sp.get("steering_to_servo_offset", 0.50)),
        "port": str(se.get("vesc_port", "/dev/sensors/vesc")),
        "current_min": -float(sp.get("max_current_a", 91.53)),
        "current_max": float(sp.get("max_current_a", 91.53)),
        "servo_min": float(sp.get("servo_min", 0.15)),
        "servo_max": float(sp.get("servo_max", 0.85)),
        "wheelbase": float(gm.get("wheelbase", 0.33)),
        "max_acceleration": float(sp.get("max_acceleration", 2.0)),
    }


def get_controller_params(profile: Dict[str, Any]) -> Dict[str, Any]:
    """Adapts profile geometry and tuning defaults for controller node."""
    gm = profile.get("geometry_mass", {})
    td = profile.get("tuning_defaults", {})
    sp = profile.get("steering_powertrain", {})

    return {
        "wheelbase": float(gm.get("wheelbase", 0.33)),
        "track_width": float(gm.get("track_width", 0.28)),
        "vehicle_mass": float(gm.get("mass", 3.47)),
        "max_steer_angle": float(sp.get("max_steering_angle_rad", 0.4189)),
        "lookahead_min": float(td.get("lookahead_min", 0.50)),
        "lookahead_max": float(td.get("lookahead_max", 3.00)),
        "max_speed_cap": float(td.get("max_speed_mps", 8.00)),
        "aeb_ttc_threshold": float(td.get("aeb_ttc_threshold", 0.45)),
    }


def get_lidar_params(profile: Dict[str, Any]) -> Dict[str, Any]:
    """Adapts sensor extrinsics for LiDAR node and static transforms."""
    se = profile.get("sensor_extrinsics", {})
    tf = se.get("lidar_tf", [0.275, 0.0, 0.10, 0.0, 0.0, 0.0])

    return {
        "serial_port": str(se.get("lidar_port", "/dev/sensors/rplidar")),
        "tf_x": float(tf[0]),
        "tf_y": float(tf[1]),
        "tf_z": float(tf[2]),
        "tf_roll": float(tf[3]),
        "tf_pitch": float(tf[4]),
        "tf_yaw": float(tf[5]),
    }
