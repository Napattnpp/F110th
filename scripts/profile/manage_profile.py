#!/usr/bin/env python3
"""
F1TENTH Vehicle Profile Manager
Manages, validates, and activates vehicle configuration profiles.
"""

import sys
import os
import argparse
import json
import yaml
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
PROFILES_DIR = REPO_ROOT / "profiles"
SCHEMA_PATH = REPO_ROOT / "schemas" / "car_profile.schema.json"
DOTENV_PATH = REPO_ROOT / ".env"


def load_schema():
    if not SCHEMA_PATH.exists():
        print(f"[!] Warning: Schema not found at {SCHEMA_PATH}")
        return None
    with open(SCHEMA_PATH, "r") as f:
        return json.load(f)


def validate_profile(profile_dict, schema=None):
    """Validates profile dict against JSON schema or fallback rules."""
    errors = []
    
    # Try jsonschema if available
    if schema:
        try:
            import jsonschema
            jsonschema.validate(instance=profile_dict, schema=schema)
            return True, []
        except ImportError:
            pass  # Fall back to manual rule verification
        except Exception as e:
            errors.append(str(e))
            return False, errors

    # Manual structural fallback validation
    required_sections = ["metadata", "geometry_mass", "steering_powertrain", "sensor_extrinsics", "tuning_defaults"]
    for sec in required_sections:
        if sec not in profile_dict:
            errors.append(f"Missing required section: '{sec}'")

    if "metadata" in profile_dict:
        m = profile_dict["metadata"]
        if "name" not in m:
            errors.append("metadata.name is required")
        if "compute_platform" not in m:
            errors.append("metadata.compute_platform is required")

    if "geometry_mass" in profile_dict:
        gm = profile_dict["geometry_mass"]
        for k in ["wheelbase", "track_width", "mass", "cog_front", "cog_rear"]:
            if k not in gm or not isinstance(gm[k], (int, float)) or gm[k] <= 0:
                errors.append(f"geometry_mass.{k} must be a positive number")

    if "steering_powertrain" in profile_dict:
        sp = profile_dict["steering_powertrain"]
        for k in ["steering_to_servo_gain", "steering_to_servo_offset", "speed_to_erpm_gain", "max_current_a"]:
            if k not in sp or not isinstance(sp[k], (int, float)):
                errors.append(f"steering_powertrain.{k} must be a valid number")

    return (len(errors) == 0), errors


def list_profiles():
    if not PROFILES_DIR.exists():
        print("[!] No profiles directory found.")
        return []
    
    profiles = []
    for f in sorted(PROFILES_DIR.glob("*.yaml")):
        if f.name == "active.yaml":
            continue
        try:
            with open(f, "r") as pf:
                data = yaml.safe_load(pf)
                desc = data.get("metadata", {}).get("description", "No description")
                platform = data.get("metadata", {}).get("compute_platform", "unknown")
                profiles.append((f.stem, desc, platform))
        except Exception:
            profiles.append((f.stem, "Invalid YAML", "unknown"))
    return profiles


def get_active_profile():
    active_symlink = PROFILES_DIR / "active.yaml"
    if active_symlink.is_symlink():
        target = os.readlink(active_symlink)
        return Path(target).stem
    
    # Check .env
    if DOTENV_PATH.exists():
        with open(DOTENV_PATH, "r") as f:
            for line in f:
                if line.startswith("ACTIVE_CAR="):
                    return line.strip().split("=", 1)[1].strip("'\"")
    return "sim"


def set_active_profile(car_name):
    profile_path = PROFILES_DIR / f"{car_name}.yaml"
    if not profile_path.exists():
        print(f"[!] Error: Profile '{car_name}' not found at {profile_path}")
        print("    Available profiles:", ", ".join([p[0] for p in list_profiles()]))
        return False

    with open(profile_path, "r") as f:
        data = yaml.safe_load(f)

    schema = load_schema()
    is_valid, errors = validate_profile(data, schema)
    if not is_valid:
        print(f"[!] Validation failed for {profile_path.name}:")
        for err in errors:
            print(f"    - {err}")
        return False

    # 1. Update symlink
    active_symlink = PROFILES_DIR / "active.yaml"
    if active_symlink.exists() or active_symlink.is_symlink():
        active_symlink.unlink()
    os.symlink(f"{car_name}.yaml", active_symlink)

    # 2. Update .env
    env_lines = []
    found = False
    if DOTENV_PATH.exists():
        with open(DOTENV_PATH, "r") as f:
            for line in f:
                if line.startswith("ACTIVE_CAR="):
                    env_lines.append(f"ACTIVE_CAR={car_name}\n")
                    found = True
                else:
                    env_lines.append(line)
    if not found:
        env_lines.append(f"ACTIVE_CAR={car_name}\n")

    with open(DOTENV_PATH, "w") as f:
        f.writelines(env_lines)

    meta = data.get("metadata", {})
    gm = data.get("geometry_mass", {})
    sp = data.get("steering_powertrain", {})

    print(f"\n\033[32m[✓] Active vehicle profile set to: {car_name}\033[0m")
    print(f"    Description: {meta.get('description')}")
    print(f"    Platform:    {meta.get('compute_platform')}")
    print(f"    Wheelbase:   {gm.get('wheelbase')} m | Mass: {gm.get('mass')} kg")
    print(f"    Servo Trim:  {sp.get('steering_to_servo_offset')} | Max Steer: {sp.get('max_steering_angle_rad')} rad")
    print(f"    Symlink:     profiles/active.yaml -> {car_name}.yaml")
    print(f"    Persisted:   .env (ACTIVE_CAR={car_name})\n")
    return True


def main():
    parser = argparse.ArgumentParser(description="F1TENTH Vehicle Profile Manager")
    parser.add_argument("--list", action="store_true", help="List all available vehicle profiles")
    parser.add_argument("--status", action="store_true", help="Show currently active profile")
    parser.add_argument("--set", type=str, metavar="CAR", help="Set and activate vehicle profile")
    parser.add_argument("--validate", type=str, metavar="CAR", help="Validate a profile against schema")
    
    args = parser.parse_args()

    if args.list:
        active = get_active_profile()
        profiles = list_profiles()
        print("\n\033[36mAvailable Vehicle Profiles:\033[0m")
        for name, desc, platform in profiles:
            marker = "\033[32m* (active)\033[0m" if name == active else "  "
            print(f"  {marker} \033[1m{name:<12}\033[0m [{platform}] — {desc}")
        print()
    elif args.status:
        active = get_active_profile()
        print(f"\nActive profile: \033[32m{active}\033[0m (profiles/{active}.yaml)\n")
    elif args.set:
        success = set_active_profile(args.set)
        sys.exit(0 if success else 1)
    elif args.validate:
        profile_path = PROFILES_DIR / f"{args.validate}.yaml"
        if not profile_path.exists():
            print(f"[!] Profile '{args.validate}' not found.")
            sys.exit(1)
        with open(profile_path, "r") as f:
            data = yaml.safe_load(f)
        schema = load_schema()
        valid, errors = validate_profile(data, schema)
        if valid:
            print(f"[✓] {profile_path.name} is valid against schema.")
        else:
            print(f"[!] {profile_path.name} validation failed:")
            for e in errors:
                print(f"    - {e}")
            sys.exit(1)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()

