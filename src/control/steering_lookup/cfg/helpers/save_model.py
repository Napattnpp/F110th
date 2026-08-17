import json
import os

def save(model):
    """
    Saves vehicle model parameters (Pacejka C_Pf, C_Pr or linear cornering stiffness)
    to a JSON file and prints fitting results.
    """
    output_dir = os.path.join(os.path.dirname(__file__), "..")
    filename = os.path.join(output_dir, f"{model.name}_fitted_params.json")
    
    out_dict = dict(model)
    with open(filename, 'w') as f:
        json.dump(out_dict, f, indent=2)
        
    print(f"\n=======================================================")
    print(f" Saved fitted tire parameters for model: {model.name}")
    if model.tire_model == "pacejka":
        print(f" Front Pacejka C_Pf (B, C, D, E): {model.C_Pf}")
        print(f" Rear Pacejka C_Pr  (B, C, D, E): {model.C_Pr}")
    elif model.tire_model == "linear":
        print(f" Front Linear C_Sf: {model.C_Sf:.4f}")
        print(f" Rear Linear C_Sr:  {model.C_Sr:.4f}")
    print(f" Output File: {filename}")
    print(f"=======================================================\n")
