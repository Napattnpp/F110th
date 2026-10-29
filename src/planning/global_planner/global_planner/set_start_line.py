#!/usr/bin/env python3
"""Proxy redirect to canonical scripts/map/set_start_line.py"""
import sys
from pathlib import Path

# Add repo root to path
repo_root = Path(__file__).resolve().parents[4]
script_path = repo_root / "scripts" / "map" / "set_start_line.py"

if script_path.exists():
    import importlib.util
    spec = importlib.util.spec_from_file_location("set_start_line", str(script_path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    main = mod.main
else:
    def main():
        print(f"Error: Could not locate canonical script at {script_path}")
        sys.exit(1)

if __name__ == "__main__":
    main()

