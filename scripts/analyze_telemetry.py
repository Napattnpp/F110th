#!/usr/bin/env python3
import json
import time

def analyze():
    path = "/home/jetson/F110th/out/obstacle_telemetry_live.jsonl"
    with open(path) as f:
        records = [json.loads(line) for line in f]

    print(f"Total Telemetry Frames: {len(records)} (spanning {records[-1]['timestamp'] - records[0]['timestamp']:.1f}s)")
    
    # State transitions
    last_state = None
    transitions = []
    obs_sightings = []
    
    t0 = records[0]['timestamp']
    for r in records:
        t_rel = r['timestamp'] - t0
        state = r['state']
        frenet = r['frenet'] or {}
        s = frenet.get('s', 0.0)
        vs = frenet.get('vs', 0.0)
        drive = r['drive'] or {}
        cmd_v = drive.get('speed', 0.0)
        cmd_steer = drive.get('steering', 0.0)
        obstacles = r['obstacles'] or []
        
        if state != last_state:
            transitions.append((t_rel, last_state, state, s, vs, cmd_v, cmd_steer, len(obstacles)))
            last_state = state
            
        if obstacles:
            for o in obstacles:
                gap = (o['s_center'] - s) % 18.82
                obs_sightings.append((t_rel, o['id'], gap, s, vs, state, cmd_v, cmd_steer, o['vs'], o['is_static']))

    print("\n--- STATE TRANSITIONS TIMELINE ---")
    for t_rel, prev_s, new_s, s, vs, cmd_v, cmd_steer, n_obs in transitions:
        s_val = s if s is not None else 0.0
        vs_val = vs if vs is not None else 0.0
        cmd_v_val = cmd_v if cmd_v is not None else 0.0
        cmd_steer_val = cmd_steer if cmd_steer is not None else 0.0
        print(f"T+{t_rel:6.2f}s | {str(prev_s):>16} -> {str(new_s):<16} | s={s_val:5.2f}m | vs={vs_val:5.2f}m/s | cmd_v={cmd_v_val:5.2f}m/s steer={cmd_steer_val:+6.2f}rad | obs_in_fov={n_obs}")

    print(f"\n--- OBSTACLE SIGHTINGS ({len(obs_sightings)} frames) ---")
    if obs_sightings:
        # Group into contiguous encounter clusters
        clusters = []
        cur_cluster = [obs_sightings[0]]
        for o in obs_sightings[1:]:
            if o[0] - cur_cluster[-1][0] < 1.0 and o[1] == cur_cluster[-1][1]:
                cur_cluster.append(o)
            else:
                clusters.append(cur_cluster)
                cur_cluster = [o]
        clusters.append(cur_cluster)
        
        for idx, cl in enumerate(clusters):
            first = cl[0]
            last = cl[-1]
            dur = last[0] - first[0]
            min_gap = min(x[2] for x in cl)
            max_gap = max(x[2] for x in cl)
            print(f"\nCluster #{idx+1}: Obs ID={first[1]} | Duration: {dur:.2f}s (T+{first[0]:.2f}s to T+{last[0]:.2f}s)")
            print(f"  Gap: {first[2]:.2f}m -> {min_gap:.2f}m (min) -> {last[2]:.2f}m")
            print(f"  Ego Car: s={first[3]:.2f}m -> {last[3]:.2f}m | Speed: {first[4]:.2f}m/s -> {last[4]:.2f}m/s")
            print(f"  Obs Speed: {first[8]:.2f}m/s | is_static: {first[9]}")
            states_seen = sorted(list(set(x[5] for x in cl)))
            print(f"  States Encountered: {', '.join(states_seen)}")

if __name__ == '__main__':
    analyze()
