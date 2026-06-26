# Map Cleaner

Lightweight Python tool for cleaning SLAM/Cartographer/similar map images. The main entry point is `map_cleaner_matplotlib.py`, which supports interactive tuning with Matplotlib and non-interactive batch export.

## Files

- `map_cleaner_matplotlib.py` - main app for interactive and batch map cleaning
- `map_cleaner_core.py` - shared cleaning logic used by the main app
- `requirements.txt` - Python dependencies
- `setup_jetson.sh` - optional setup helper for Jetson/Linux systems
- `map_evaluate/` - sample maps for testing

## Install

```bash
pip install -r requirements.txt
```

On Jetson/Linux, you can also run:

```bash
bash setup_jetson.sh
```

## Run

Interactive mode:

```bash
python3 map_cleaner_matplotlib.py map_evaluate/cb4_7th_floor.yaml
```

Batch mode:

```bash
python3 map_cleaner_matplotlib.py map_evaluate/cb4_7th_floor.yaml --batch -o clean.pgm
```

Demo mode:

```bash
python3 map_cleaner_matplotlib.py --demo
```

Useful options:

```bash
python3 map_cleaner_matplotlib.py --help
```

The cleaner accepts ROS map YAML, PGM, PNG, and other image formats OpenCV can read. For ROS YAML files, it follows the `image:` field and reads `resolution:` for measurement.

## Interactive Controls

- `S` saves the cleaned map
- `Q` quits
- `R` resets cleaning parameters from auto-tuning
- Click two points on the cleaned image to measure distance

## Output

PGM output uses:

- `0` for walls
- `255` for free space
- `128` for unknown space
