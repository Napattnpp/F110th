# Quick Start

## Install

```bash
pip install -r requirements.txt
```

Or on Jetson/Linux:

```bash
bash setup_jetson.sh
```

## Interactive Cleaning

```bash
python3 map_cleaner_matplotlib.py map_evaluate/cb4_7th_floor.yaml
```

Use the sliders to tune the cleaned map. Press `S` to save, `Q` to quit, and `R` to reset.

## Batch Cleaning

```bash
python3 map_cleaner_matplotlib.py map_evaluate/cb4_7th_floor.yaml --batch -o clean.pgm
```

## Test Without a Map

```bash
python3 map_cleaner_matplotlib.py --demo
```
