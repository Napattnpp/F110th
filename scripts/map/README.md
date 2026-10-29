# F1TENTH Map Cleaning & Binary Mask Utilities

Core map cleaning library and algorithms conforming to **[ADR-003](file:///Users/napat/Developer/Babel/docs/DECISIONS.md#adr-003--web-based-tooling-over-x11-double-forwarding)**, **[ADR-004](file:///Users/napat/Developer/Babel/docs/DECISIONS.md#adr-004--standardized-4-subfolder-map-architecture--testing-isolation)**, and **[ADR-010](file:///Users/napat/Developer/Babel/docs/DECISIONS.md#adr-010--single-canonical-tooling-mandate--tooling-bloat-elimination)**.

---

## Architecture & Canonical Tools

Under **ADR-010 (Single Canonical Tooling Mandate)**, all map cleaning operations share the identical underlying engine ([`map_cleaner_core.py`](map_cleaner_core.py)) with zero display or X11 dependencies:

1. **Interactive Mode (Canonical Web Studio)**:
   ```bash
   make clean_web MAP=<map_name>
   # or:
   python3 scripts/map/map_cleaner_web.py --map <map_name> --port 8051
   ```
   - Synchronized side-by-side view (Raw SLAM vs. Cleaned Track).
   - Real-time parameter sliders (white threshold, wall thickness, smooth radius, min blob).
   - Canvas tools: ✏️ Wall Brush, 🧽 Track Eraser, 📏 Metric Track Ruler.
   - Bakes 100% ADR-004 compliant `clean/<map>.pgm` (3-value grid) and `waypoint_gen/<map>_waypoint_gen.pgm` (strict 2-color binary mask).

2. **Automated Batch Mode (CI / Scripting)**:
   ```bash
   make clean_map MAP=<map_name>
   # or:
   python3 scripts/map/manage_map.py clean <map_name>
   ```

---

## Files

- `map_cleaner_core.py`: Core algorithmic engine (`MapCleaner`). Morphological filtering, auto-tuning, and cubic B-spline contour smoothing.
- `map_cleaner_web.py`: Interactive Web Studio frontend (`make clean_web MAP=<map_name>`).
- `manage_map.py`: Unified map lifecycle manager & headless batch cleaner (`make clean_map MAP=<map_name>`).
- `init_map.py`: Standardizes raw SLAM outputs into ADR-004 4-subfolder contract (`make init_map MAP=<map_name>`).
- `set_start_line.py`: Start line visual interactive setter (`make start_line MAP=<map_name>`).
- `finish_map.sh`: Automated post-SLAM map packaging script.
