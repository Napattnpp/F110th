import json
import os

class DotDict(dict):
    """Dot notation access to dictionary attributes"""
    __getattr__ = dict.get
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__

def get_dotdict(model_name):
    """
    Returns dotdict model parameters for the given model_name.
    Example model_name: 'NUC1_pacejka', 'NUC3_pacejka', 'SIM_linear'
    """
    parts = model_name.split("_")
    model_id = parts[0]
    tire_model = parts[1] if len(parts) > 1 else "pacejka"

    # Standard F110th vehicle physical parameters
    model_params = DotDict({
        "name": model_id,
        "tire_model": tire_model,
        "m": 3.47,           # total car mass [kg]
        "h_cg": 0.074,       # center of gravity height [m]
        "l_f": 0.158,        # CG to front axle [m]
        "l_r": 0.171,        # CG to rear axle [m]
        "l_wb": 0.329,       # wheelbase [m]
        "mu": 1.0,           # friction coefficient
        "C_0d": 0.0,         # servo offset
        "C_d": 1.0,          # servo gain
        "C_Pf": [10.0, 1.9, 1.0, 0.97],  # initial Pacejka front [B, C, D, E]
        "C_Pr": [10.0, 1.0, 1.0, 0.97],  # initial Pacejka rear [B, C, D, E]
        "C_Sf": 15.0,        # initial linear front cornering stiffness
        "C_Sr": 15.0         # initial linear rear cornering stiffness
    })

    # Optional load from local JSON config if present
    config_file = os.path.join(os.path.dirname(__file__), f"../{model_id}_config.json")
    if os.path.exists(config_file):
        try:
            with open(config_file, 'r') as f:
                data = json.load(f)
                model_params.update(data)
        except Exception as e:
            print(f"Warning: Could not read config file {config_file}: {e}")

    return model_params
