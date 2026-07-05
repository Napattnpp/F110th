import os
import yaml
from ament_index_python.packages import get_package_share_directory

def load_active_profile():
    """
    Finds and parses configs/active.yaml.
    Returns a dictionary of parameters. If not found or fails, returns an empty dict.
    """
    profile_data = {}
    
    # Try various relative paths to find configs/active.yaml
    pkg_bringup = get_package_share_directory('race_bringup')
    
    paths_to_try = [
        # Colcon install structure: install/race_bringup/share/race_bringup/../../../../profiles/active_profile.yaml
        os.path.abspath(os.path.join(pkg_bringup, '..', '..', '..', '..', 'profiles', 'active_profile.yaml')),
        # Fallback to standard host/container mount points
        '/workspaces/F110th/profiles/active_profile.yaml',
        os.path.expanduser('~/F110th/profiles/active_profile.yaml'),
        os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', 'profiles', 'active_profile.yaml')),
    ]
    
    for path in paths_to_try:
        if os.path.exists(path):
            try:
                with open(path, 'r') as f:
                    profile_data = yaml.safe_load(f) or {}
                # Success
                break
            except Exception as e:
                print(f"[Profile Loader] Error loading active config from {path}: {e}")
                
    return profile_data
