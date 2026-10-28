# F1TENTH Autonomous Racing — Agent Protocol & Knowledge Portal

> **File Role**: This file is the primary entry point for any AI agent operating within the `F110th` codebase. It links this implementation repository directly to **Babel**, the central institutional brain and architecture registry.

---

## 1. Central Brain Connection (Babel)

Before implementing features, changing controller parameters, modifying Docker configurations, or debugging hardware/simulation failures, **you must consult Babel**:

- **Babel Gate**: [Babel/AGENTS.md](file:///Users/napat/Developer/Babel/AGENTS.md)
- **Architecture Decisions (ADRs)**: [Babel/docs/DECISIONS.md](file:///Users/napat/Developer/Babel/docs/DECISIONS.md)
- **Hardware & Driver Errata**: [Babel/docs/ERRATA.md](file:///Users/napat/Developer/Babel/docs/ERRATA.md)
- **Approved R&D Workflows**: [Babel/docs/WORKFLOWS.md](file:///Users/napat/Developer/Babel/docs/WORKFLOWS.md)
- **Concepts & Mathematics**: [Babel/docs/CONCEPTS.md](file:///Users/napat/Developer/Babel/docs/CONCEPTS.md)
- **Source Registry (180+ Papers & Specs)**: [Babel/docs/SOURCE_REGISTRY.md](file:///Users/napat/Developer/Babel/docs/SOURCE_REGISTRY.md)
- **Unapproved Ideas & Backlog**: [Babel/notes/inbox.md](file:///Users/napat/Developer/Babel/notes/inbox.md)

---

## 2. Core Architectural Invariants (Non-Negotiable)

When writing or modifying code in this repository, strictly adhere to these accepted decisions:

1. **Web-Based Tooling over X11** ([ADR-003](file:///Users/napat/Developer/Babel/docs/DECISIONS.md#adr-003-web-based-visualization--tooling-over-x11-double-forwarding)):
   - Never rely on X11 forwarding for remote vehicle debugging.
   - Use Foxglove Studio / native websocket bridges for telemetry, and Flask/matplotlib web UIs for tuning.
2. **Standard 4-Subfolder Map Standard** ([ADR-004](file:///Users/napat/Developer/Babel/docs/DECISIONS.md#adr-004-standardized-4-subfolder-map-architecture--testing-isolation)):
   - Every map in `assets/maps/<map_name>/` must maintain:
     - `raw/`: Untouched scan outputs (`.pgm`, `.yaml`)
     - `clean/`: Binary mask (`.png`, `.yaml`) with noise removed
     - `centerline/`: Filtered track boundary and centerline CSVs
     - `raceline/`: Optimized racelines generated via TUM IQP optimizer
3. **Hierarchical Overriding & Fail-Fast Safety** ([ADR-005](file:///Users/napat/Developer/Babel/docs/DECISIONS.md#adr-005-hierarchical-overriding--fail-fast-safety-policy)):
   - Safety emergency braking (AEB) and manual joystick override have absolute priority over autonomous planners.
   - If LiDAR or odometry fails or drops below frequency thresholds, the vehicle must trigger emergency stop (`ackermann_cmd` speed = 0) immediately.

---

## 3. Troubleshooting & Errata Lookup

If you encounter unexpected errors during compilation, execution, or hardware bring-up:
1. Search [Babel/docs/ERRATA.md](file:///Users/napat/Developer/Babel/docs/ERRATA.md) first:
   - **Hokuyo LiDAR timeouts**: Check `ERR-LIDAR-01` (Static IP `192.168.0.10`).
   - **VESC disconnects under acceleration**: Check `ERR-VESC-01` (USB autosuspend & udev rules).
   - **ROS 2 discovery issues over Wi-Fi**: Check `ERR-NET-01` (CycloneDDS unicast config).
   - **Foxglove message drops**: Check `ERR-VIZ-01` (binary MCAP `foxglove_bridge`).
   - **Raceline wall collisions**: Check `ERR-MAP-01` (PGM mask polarity & YAML origin).
   - **Simulated car snaps heading or cannot reverse on crash**: Check `ERR-SIM-01` (state vector yaw preservation & body-frame collision check).
2. If you solve a **new, unrecorded issue**, append the verified solution directly to [Babel/docs/ERRATA.md](file:///Users/napat/Developer/Babel/docs/ERRATA.md).

---

## 4. Staging New Concepts & Ideas

If you or the user conceive a new experimental algorithm (e.g. adaptive lookahead, online friction estimation) that has **not yet been benchmarked**:
- **Do not** pollute production configs or master docs directly.
- Stage the idea in [Babel/notes/inbox.md](file:///Users/napat/Developer/Babel/notes/inbox.md) under `[HYPOTHESIS]` or `[PROPOSAL]`.
- Promote to [Babel/docs/DECISIONS.md](file:///Users/napat/Developer/Babel/docs/DECISIONS.md) only after empirical validation.
