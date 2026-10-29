# Map Cleaner Quickstart

Under **[ADR-010](file:///Users/napat/Developer/Babel/docs/DECISIONS.md#adr-010--single-canonical-tooling-mandate--tooling-bloat-elimination)**, all map cleaning is executed via the canonical tools:

### Interactive Web Studio (Zero X11)
```bash
make clean_web MAP=rectangle1
```
Open `http://localhost:8051` in any browser.

### Headless Batch Cleaning
```bash
make clean_map MAP=rectangle1
```
Automatically generates `clean/rectangle1.pgm` and `waypoint_gen/rectangle1_waypoint_gen.pgm` adhering strictly to **[ADR-004](file:///Users/napat/Developer/Babel/docs/DECISIONS.md#adr-004--standardized-4-subfolder-map-architecture--testing-isolation)**.
