#!/usr/bin/env python3
"""
Matplotlib-based Map Cleaner with interactive sliders
Minimal dependencies, works well over X11 forwarding (even double forwarding)
"""

import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
import cv2
import numpy as np
import os
import sys
import yaml
import signal
from pathlib import Path
from map_cleaner_core import MapCleaner

def read_yaml_resolution(map_path, default=0.05):
    """Read map resolution from an adjacent ROS map YAML, without modifying it."""
    if not map_path:
        return default
    return MapCleaner.read_ros_map_resolution(map_path, default=default)

def compute_stats(img):
    """Compute statistics."""
    total = img.size
    return {
        'walls': int(np.sum(img == 0) / total * 100),
        'free': int(np.sum(img == 255) / total * 100),
        'unknown': int(np.sum(img == 128) / total * 100),
    }

def load_map(file_path):
    """Load PGM, PNG, or ROS map YAML."""
    return MapCleaner.load_image(file_path)

def load_pgm(file_path):
    """Parse PGM file."""
    with open(file_path, 'rb') as f:
        lines = []
        for _ in range(3):
            line = f.readline().decode('latin-1').strip()
            while line.startswith('#'):
                line = f.readline().decode('latin-1').strip()
            lines.append(line)
        
        magic, (w, h), maxval = lines[0], tuple(map(int, lines[1].split())), int(lines[2])
        
        if magic == 'P5':
            data = np.frombuffer(f.read(w * h), dtype=np.uint8)
        else:
            data = np.array(list(map(int, f.read().decode('latin-1').split())), dtype=np.uint16)
        
        if maxval != 255:
            data = (data * 255 // maxval).astype(np.uint8)
        else:
            data = data.astype(np.uint8)
        
        return data.reshape((h, w))

def save_pgm(file_path, img):
    """Save PGM file."""
    h, w = img.shape
    header = f"P5\n# Cleaned by map_cleaner_matplotlib\n{w} {h}\n255\n".encode('latin-1')
    with open(file_path, 'wb') as f:
        f.write(header)
        f.write(img.astype(np.uint8).tobytes())

def load_demo_map():
    """Generate a demo/test map."""
    img = np.full((400, 400), 128, dtype=np.uint8)
    y, x = np.ogrid[:400, :400]
    cy, cx = 200, 200
    
    dist = np.sqrt((x - cx)**2 + (y - cy)**2)
    track = (dist > 60) & (dist < 120)
    img[track] = 255
    img[170:230, 170:230] = 128

    noise = np.random.randint(-3, 4, img.shape)
    img = np.clip(img.astype(int) + noise, 0, 255).astype(np.uint8)
    
    return img

# ============================================================================
# INTERACTIVE MATPLOTLIB VIEWER
# ============================================================================

class MapCleanerApp:
    def __init__(self):
        self.raw_img = None
        self.map_path = None
        self.clean_img = None
        self.fig = None
        self.ax_clean = None
        self.text_stats = None
        self.measure_points = []
        self.measure_text = None
        
        # Parameter storage
        self.params = {
            'free_th': 222,
            'wall_thickness': 3,                # Optimized default based on manual edits
            'smooth_radius': 5,                 # Optimized default for smooth curves
            'min_blob': 80,
            'cover_track_edge': False,          # Optimized default: walls sit outside the track
            'keep_largest_track': True,
            'edge_smooth_method': 'spline',     # 'bilateral', 'distance', 'spline', 'none'
            'edge_smooth_iterations': 1,        # 0-3: smoothing iterations / strength
            'fill_unknown_as_wall': False,
        }
        self.map_resolution = 0.05
        
        # Setup signal handlers for safe exit (Ctrl+C, Ctrl+Z)
        signal.signal(signal.SIGINT, self._signal_handler)   # Ctrl+C
        signal.signal(signal.SIGTSTP, self._signal_handler)  # Ctrl+Z
    
    def _signal_handler(self, signum, frame):
        """Handle Ctrl+C and Ctrl+Z gracefully."""
        signal_name = "Ctrl+C" if signum == signal.SIGINT else "Ctrl+Z"
        print(f"\n\n→ {signal_name} detected")
        print("✓ Closing interactive GUI safely...")
        if self.fig is not None:
            plt.close('all')
        print("✓ Goodbye!")
        sys.exit(0)

    def resolve_filepath(self, filepath):
        """Flexible map path resolution (prefers _raw files for map_cleaner)."""
        if not filepath:
            return None
        if filepath.startswith("map_name="):
            filepath = filepath.split("=", 1)[1]
        
        path_obj = Path(filepath)
        if path_obj.exists():
            return str(path_obj)

        # Normalize base map name (e.g. track7th_raw -> track7th)
        raw_name = path_obj.stem
        for suffix in ['_waypoint_gen', '_waypoint', '_raceline', '_raw']:
            if raw_name.endswith(suffix):
                raw_name = raw_name[:-len(suffix)]

        # Check assets/maps/<map_name>/<map_name>_raw.yaml first
        raw_yaml = Path("assets/maps") / raw_name / f"{raw_name}_raw.yaml"
        if raw_yaml.exists():
            return str(raw_yaml)
            
        raw_pgm = Path("assets/maps") / raw_name / f"{raw_name}_raw.pgm"
        if raw_pgm.exists():
            return str(raw_pgm)

        base_yaml = Path("assets/maps") / raw_name / f"{raw_name}.yaml"
        if base_yaml.exists():
            return str(base_yaml)
            
        base_pgm = Path("assets/maps") / raw_name / f"{raw_name}.pgm"
        if base_pgm.exists():
            return str(base_pgm)

        return filepath

    def load_file(self, filepath=None):
        """Load map file interactively."""
        filepath = self.resolve_filepath(filepath)
        if filepath is None or not os.path.exists(filepath):
            print(f"\n✗ Map file not found: {filepath}")
            print("  Example usage: python3 map_cleaner_matplotlib.py track7th")
            print("                 python3 map_cleaner_matplotlib.py assets/maps/track7th/track7th.yaml")
            sys.exit(1)

        self.raw_img = load_map(filepath)
        self.map_path = filepath
        
        if self.raw_img is None:
            print("✗ Failed to load map")
            sys.exit(1)
        
        h, w = self.raw_img.shape
        self.map_resolution = read_yaml_resolution(self.map_path)
        print(f"✓ Loaded: {w}×{h} px | Range: {self.raw_img.min()}–{self.raw_img.max()}")
        print(f"✓ Map resolution: {self.map_resolution:g} m/px")
    
    def update_clean(self):
        """Recompute cleaned map based on current parameters."""
        self.clean_img = MapCleaner(verbose=False).clean_f1tenth(
            self.raw_img,
            free_threshold=self.params['free_th'],
            wall_thickness=self.params['wall_thickness'],
            smooth_radius=self.params['smooth_radius'],
            min_track_blob=self.params['min_blob'],
            cover_track_edge=self.params['cover_track_edge'],
            keep_largest_track=self.params['keep_largest_track'],
            edge_smooth_method=self.params['edge_smooth_method'],
            edge_smooth_iterations=self.params['edge_smooth_iterations'],
            fill_unknown_as_wall=self.params['fill_unknown_as_wall'],
        )
        if self.fig is not None and self.ax_clean is not None:
            self.redraw()

    def measurement_summary(self):
        """Return the active measurement in pixels and meters."""
        if len(self.measure_points) < 2:
            return "Measure: click two points on the image"

        (x1, y1), (x2, y2) = self.measure_points[:2]
        px = float(np.hypot(x2 - x1, y2 - y1))
        meters = px * self.map_resolution
        return f"Measure: {px:.1f} px   {meters:.3f} m"

    def draw_measurement(self):
        """Draw point-to-point measurement on the cleaned map."""
        if not self.measure_points:
            return

        xs, ys = zip(*self.measure_points)
        self.ax_clean.plot(xs, ys, 'o', color='#ffcc00', markersize=6)
        if len(self.measure_points) >= 2:
            self.ax_clean.plot(xs[:2], ys[:2], '-', color='#ffcc00', linewidth=2)
            mid_x = (xs[0] + xs[1]) / 2
            mid_y = (ys[0] + ys[1]) / 2
            label = self.measurement_summary().replace('Measure: ', '')
            self.ax_clean.text(
                mid_x, mid_y, label,
                color='black',
                fontsize=9,
                ha='center',
                va='bottom',
                bbox={'facecolor': '#ffcc00', 'edgecolor': 'none', 'alpha': 0.85, 'pad': 3},
            )

    def update_measure_text(self):
        """Refresh the measurement readout below the image."""
        if self.measure_text is not None:
            self.measure_text.set_text(self.measurement_summary())

    def on_image_click(self, event):
        """Handle RViz-style two-click measurement."""
        if event.inaxes != self.ax_clean or event.xdata is None or event.ydata is None:
            return

        if len(self.measure_points) >= 2:
            self.measure_points = []

        self.measure_points.append((float(event.xdata), float(event.ydata)))
        self.redraw()
    
    def redraw(self):
        """Redraw the cleaned image."""
        if self.clean_img is None:
            return
        
        self.ax_clean.clear()
        
        self.ax_clean.imshow(self.clean_img, cmap='gray', vmin=0, vmax=255)
        self.ax_clean.set_title('Cleaned Output', fontsize=12, fontweight='bold')
        self.ax_clean.axis('off')
        self.draw_measurement()
        
        # Stats
        stats = compute_stats(self.clean_img)
        stat_text = f"Walls: {stats['walls']}% | Free: {stats['free']}% | Unknown: {stats['unknown']}%"
        self.ax_clean.text(0.5, -0.05, stat_text, transform=self.ax_clean.transAxes,
                          ha='center', fontsize=9, color='gray')
        
        # Params display
        param_text = (
            f"free_th={self.params['free_th']} wall={self.params['wall_thickness']}px "
            f"smooth_radius={self.params['smooth_radius']} cover_edge={self.params['cover_track_edge']} "
            f"method={self.params['edge_smooth_method']} ({self.params['edge_smooth_iterations']} iter) "
            f"fill_wall={1 if self.params['fill_unknown_as_wall'] else 0} resolution={self.map_resolution:.3f} m/px"
        )
        for text in list(self.fig.texts):
            if getattr(text, '_map_cleaner_param_text', False):
                text.remove()
        text = self.fig.text(0.5, 0.02, param_text, ha='center', fontsize=8,
                             color='gray', family='monospace')
        text._map_cleaner_param_text = True
        self.update_measure_text()
        
        self.fig.canvas.draw_idle()
    
    def run_interactive(self):
        """Launch interactive mode with sliders."""
        if self.raw_img is None:
            print("✗ No map loaded")
            return
        
        self.fig = plt.figure(figsize=(11, 8))
        self.fig.suptitle('F1TENTH Track Map Cleaner', fontsize=14, fontweight='bold')

        self.ax_clean = self.fig.add_axes([0.06, 0.39, 0.88, 0.53])
        
        # Initial display
        self.update_clean()
        
        self.measure_text = self.fig.text(
            0.06, 0.34, self.measurement_summary(),
            fontsize=10,
            family='monospace',
            color='#202020',
        )

        sliders_height = 0.020
        slider_y_start = 0.29
        slider_spacing = 0.042
        slider_width = 0.54
        slider_x = 0.38

        sliders = {}
        slider_labels = [
            ('free_th', 'White threshold', 150, 254, 1, self.params['free_th']),
            ('wall_thickness', 'Wall thickness', 1, 24, 1, self.params['wall_thickness']),
            ('smooth_radius', 'Track smooth radius', 0, 10, 1, self.params['smooth_radius']),
            ('cover_track_edge', 'Cover track edge (0/1)', 0, 1, 1, 1 if self.params['cover_track_edge'] else 0),
            ('edge_smooth_iterations', 'Edge straighten (0=none)', 0, 3, 1, self.params['edge_smooth_iterations']),
            ('fill_unknown_as_wall', 'Fill unknown as wall (0/1)', 0, 1, 1, 1 if self.params['fill_unknown_as_wall'] else 0),
        ]
        
        for i, (key, label, vmin, vmax, step, init) in enumerate(slider_labels):
            y = slider_y_start - i * slider_spacing
            ax_slider = plt.axes([slider_x, y, slider_width, sliders_height])
            slider = Slider(ax_slider, label, vmin, vmax, valinit=init, valstep=step, 
                          color='#185FA5', alpha=0.7)
            sliders[key] = slider
            
            def make_callback(k):
                def on_change(val):
                    if k == 'cover_track_edge' or k == 'fill_unknown_as_wall':
                        self.params[k] = bool(val)
                    else:
                        self.params[k] = int(val)
                    self.update_clean()
                return on_change
            
            slider.on_changed(make_callback(key))

        clear_ax = self.fig.add_axes([0.74, 0.315, 0.16, 0.055])
        clear_button = Button(clear_ax, 'Clear measure')

        def clear_measure(_event):
            self.measure_points = []
            self.redraw()

        clear_button.on_clicked(clear_measure)
        self.clear_button = clear_button

        # Radio buttons for selecting smoothing method
        from matplotlib.widgets import RadioButtons
        rax = self.fig.add_axes([0.05, 0.09, 0.12, 0.16])
        methods = ('none', 'bilateral', 'distance', 'spline')
        active_idx = methods.index(self.params['edge_smooth_method'])
        radio = RadioButtons(rax, methods, active=active_idx, activecolor='#185FA5')
        for label in radio.labels:
            label.set_fontsize(8)
        self.radio = radio  # Prevent garbage collection
        
        def set_method(label):
            self.params['edge_smooth_method'] = label
            self.update_clean()
            
        radio.on_clicked(set_method)

        instr = "Click two points to measure   [S] save (cleaned image)   [Q] quit   [R] reset   Ctrl+C/Z safe exit"
        plt.figtext(0.06, 0.055, instr, fontsize=9, family='monospace')
        
        # Disable matplotlib's default key handlers completely
        # This prevents matplotlib from intercepting 's' to save the figure
        if getattr(self.fig.canvas.manager, 'toolbar', None) is not None:
            if hasattr(self.fig.canvas.manager.toolbar, 'hide'):
                self.fig.canvas.manager.toolbar.hide()  # Hide toolbar (Qt backend)
            elif hasattr(self.fig.canvas.manager.toolbar, 'pack_forget'):
                self.fig.canvas.manager.toolbar.pack_forget()  # Hide toolbar (Tkinter backend)
        
        # Remove ALL default matplotlib key press handlers
        for cid in list(self.fig.canvas.callbacks.callbacks['key_press_event'].keys()):
            self.fig.canvas.mpl_disconnect(cid)
        
        # Key handlers
        def on_key(event):
            if event.key == 's':
                # Our custom save handler for the cleaned image
                self.save_interactive()
            elif event.key == 'q':
                print("\n→ Quitting...")
                plt.close('all')
                print("✓ Goodbye!")
                sys.exit(0)
            elif event.key == 'r':
                tuned = MapCleaner(verbose=False).auto_tune_f1tenth(self.raw_img)
                defaults = {
                    'free_th': tuned['free_threshold'],
                    'wall_thickness': tuned['wall_thickness'],
                    'smooth_radius': tuned['smooth_radius'],
                    'cover_track_edge': 1 if tuned['cover_track_edge'] else 0,
                    'edge_smooth_iterations': 1,
                    'fill_unknown_as_wall': 0,
                }
                self.params.update({
                    'keep_largest_track': tuned['keep_largest_track'],
                    'edge_smooth_method': 'spline',
                })
                # Update radio buttons to 'spline' index 3
                self.radio.set_active(3)
                for key, value in defaults.items():
                    sliders[key].set_val(value)
                    if key == 'cover_track_edge' or key == 'fill_unknown_as_wall':
                        self.params[key] = bool(value)
                    else:
                        self.params[key] = int(value)
                self.update_clean()
        
        # Connect our handler AFTER removing defaults
        self.fig.canvas.mpl_connect('key_press_event', on_key)
        self.fig.canvas.mpl_connect('button_press_event', self.on_image_click)
        
        plt.show()
    
    def save_map_files(self, clean_img):
        """Save clean map as <base_name>.pgm/yaml AND <base_name>_waypoint_gen.pgm/yaml, preserving _raw files."""
        if not self.map_path:
            save_pgm('map.pgm', clean_img)
            cv2.imwrite('map.png', clean_img)
            print("✓ Saved demo map to map.pgm and map.png")
            return

        import shutil
        input_path = Path(self.map_path)
        input_dir = input_path.parent
        
        # Normalize clean base map name (e.g. track7th_raw -> track7th)
        base_name = input_path.stem
        for suffix in ['_waypoint_gen', '_waypoint', '_raceline', '_raw']:
            if base_name.endswith(suffix):
                base_name = base_name[:-len(suffix)]

        raw_yaml_path = input_dir / f"{base_name}_raw.yaml"
        raw_pgm_dest = input_dir / f"{base_name}_raw.pgm"
        
        yaml_path = input_dir / f"{base_name}.yaml"
        src_yaml = raw_yaml_path if raw_yaml_path.exists() else yaml_path
        yaml_data = {}
        has_yaml = False
        if src_yaml.exists():
            has_yaml = True
            try:
                with open(src_yaml, 'r') as f:
                    yaml_data = yaml.safe_load(f)
            except Exception as e:
                print(f"⚠ Failed to read yaml configuration: {e}")

        # 1. Preserve original raw PGM & YAML
        if not raw_pgm_dest.exists() and input_path.exists():
            try:
                shutil.copy(str(input_path), str(raw_pgm_dest))
                print(f"✓ Preserved raw PGM: {raw_pgm_dest.name}")
            except Exception as e:
                print(f"⚠ Could not preserve raw PGM: {e}")

        if has_yaml and not raw_yaml_path.exists():
            try:
                raw_yaml_data = dict(yaml_data)
                raw_yaml_data['image'] = raw_pgm_dest.name
                with open(raw_yaml_path, 'w') as f:
                    yaml.safe_dump(raw_yaml_data, f, default_flow_style=False)
                print(f"✓ Created raw YAML: {raw_yaml_path.name}")
            except Exception as e:
                print(f"⚠ Could not create raw YAML: {e}")

        # 2. Save clean PGM & YAML to base name (<map_name>.pgm / <map_name>.yaml)
        clean_pgm_path = input_dir / f"{base_name}.pgm"
        clean_yaml_path = input_dir / f"{base_name}.yaml"
        try:
            save_pgm(str(clean_pgm_path), clean_img)
            print(f"✓ Saved cleaned base PGM: {clean_pgm_path.name}")
        except Exception as e:
            print(f"✗ Failed to save cleaned base PGM: {e}")

        if has_yaml:
            try:
                clean_yaml_data = dict(yaml_data)
                clean_yaml_data['image'] = clean_pgm_path.name
                with open(clean_yaml_path, 'w') as f:
                    yaml.safe_dump(clean_yaml_data, f, default_flow_style=False)
                print(f"✓ Created cleaned base YAML: {clean_yaml_path.name}")
            except Exception as e:
                print(f"✗ Failed to save cleaned base YAML: {e}")

        # 3. Save clean PGM, PNG & YAML to waypoint_gen name (<map_name>_waypoint_gen.pgm / .png / .yaml)
        # Convert clean_img to 100% pure 2-colour binary image (0 = wall, 255 = track) with no grey (128)
        binary_2color = np.where(clean_img < 210, 0, 255).astype(np.uint8)
        wp_pgm_path = input_dir / f"{base_name}_waypoint_gen.pgm"
        wp_png_path = input_dir / f"{base_name}_waypoint_gen.png"
        wp_yaml_path = input_dir / f"{base_name}_waypoint_gen.yaml"
        
        try:
            save_pgm(str(wp_pgm_path), binary_2color)
            cv2.imwrite(str(wp_png_path), binary_2color)
            print(f"✓ Saved 2-colour waypoint generator PGM & PNG: {wp_pgm_path.name}, {wp_png_path.name}")
        except Exception as e:
            print(f"✗ Failed to save waypoint generator PGM/PNG: {e}")

        if has_yaml:
            try:
                wp_yaml_data = dict(yaml_data)
                wp_yaml_data['image'] = wp_png_path.name
                with open(wp_yaml_path, 'w') as f:
                    yaml.safe_dump(wp_yaml_data, f, default_flow_style=False)
                print(f"✓ Created waypoint generator YAML: {wp_yaml_path.name}")
            except Exception as e:
                print(f"✗ Failed to save waypoint generator YAML: {e}")

        # 4. Save base preview PNG (<map_name>.png)
        clean_png_path = input_dir / f"{base_name}.png"
        try:
            cv2.imwrite(str(clean_png_path), clean_img)
            print(f"✓ Saved base preview PNG: {clean_png_path.name}")
        except Exception as e:
            print(f"⚠ Failed to save preview PNG: {e}")

    def save_interactive(self):
        """Save cleaned image when user presses 'S' - auto-saves to file and closes window."""
        if self.clean_img is None:
            print("✗ No cleaned image to save")
            return
        
        try:
            self.save_map_files(self.clean_img)
            print("→ Closing window...")
            # Auto-close window after successful save
            if self.fig is not None:
                plt.close('all')
            print("✓ Goodbye!")
            sys.exit(0)
        except Exception as e:
            print(f"✗ Save failed: {e}")

    def run_batch(self, output_path=None):
        """Run processing and save without GUI."""
        if self.raw_img is None:
            print("✗ No map loaded")
            return
        
        self.update_clean()
        
        if output_path is not None:
            try:
                if output_path.endswith('.pgm'):
                    save_pgm(output_path, self.clean_img)
                else:
                    cv2.imwrite(output_path, self.clean_img)
                print(f"✓ Saved: {output_path}")
            except Exception as e:
                print(f"✗ Save failed: {e}")
        else:
            self.save_map_files(self.clean_img)

# ============================================================================
# CLI ENTRY POINT
# ============================================================================

def main():
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Lightweight map cleaner for Cartographer/SLAM',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  Interactive mode:
    python map_cleaner_matplotlib.py scan.pgm
    python map_cleaner_matplotlib.py --demo
  
  Batch processing:
    python map_cleaner_matplotlib.py scan.pgm --batch -o output.pgm
  
  Custom parameters:
    python map_cleaner_matplotlib.py scan.pgm --batch \\
      --free-th 230 --wall-thickness 6 --smooth-radius 2 \\
      -o output.pgm
        """
    )
    
    parser.add_argument('map', nargs='?', help='Input ROS YAML, PGM, or PNG map (omit for --demo)')
    parser.add_argument('--demo', action='store_true', help='Load demo map')
    parser.add_argument('--batch', action='store_true', help='Batch mode (no GUI)')
    parser.add_argument('-o', '--output', help='Output PGM path (batch mode)')
    
    parser.add_argument('--free-th', type=int, help='White track threshold override')
    parser.add_argument('--wall-thickness', type=int, help='Boundary wall thickness in pixels')
    parser.add_argument('--smooth-radius', type=int, help='Track mask smoothing radius')
    parser.add_argument('--min-blob', type=int, help='Minimum white track blob size')
    parser.add_argument('--outside-only-wall', action='store_true', help='Draw wall outside the track instead of covering the edge')
    parser.add_argument('--keep-all-track', action='store_true', help='Keep all large white blobs instead of only the largest one')
    
    args = parser.parse_args()
    
    # Determine input
    if args.demo:
        mapfile = None
    elif args.map:
        mapfile = args.map
    else:
        parser.print_help()
        sys.exit(1)
    
    # Create app
    app = MapCleanerApp()
    
    # Load map
    if args.demo:
        print("→ Generating demo map...")
        app.raw_img = load_demo_map()
    elif mapfile:
        app.load_file(mapfile)
    
    if app.raw_img is None:
        sys.exit(1)
    
    tuned_f1 = MapCleaner(verbose=False).auto_tune_f1tenth(app.raw_img)
    app.params.update({
        'free_th': tuned_f1['free_threshold'],
        'wall_thickness': tuned_f1['wall_thickness'],
        'smooth_radius': tuned_f1['smooth_radius'],
        'min_blob': tuned_f1['min_track_blob'],
        'cover_track_edge': tuned_f1['cover_track_edge'],
        'keep_largest_track': tuned_f1['keep_largest_track'],
    })

    overrides = {
        'free_th': args.free_th,
        'wall_thickness': args.wall_thickness,
        'smooth_radius': args.smooth_radius,
        'min_blob': args.min_blob,
    }
    app.params.update({key: value for key, value in overrides.items() if value is not None})
    if args.outside_only_wall:
        app.params['cover_track_edge'] = False
    if args.keep_all_track:
        app.params['keep_largest_track'] = False
    
    # Run
    if args.batch:
        app.run_batch(args.output)
    else:
        print("\n→ Launching interactive GUI...")
        print("   [S] to save cleaned image | [Q] to quit | [R] to reset")
        print("   Ctrl+C or Ctrl+Z for safe exit")
        app.run_interactive()

if __name__ == '__main__':
    main()
