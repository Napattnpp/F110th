"""
map_cleaner_core.py - Core map cleaning functions as a reusable module

Import and use the cleaning pipeline in your own code:

    from map_cleaner_core import MapCleaner
    
    cleaner = MapCleaner()
    cleaned = cleaner.clean_from_file('input.pgm', 
                                      med_kernel=7, 
                                      free_threshold=222)
    cleaner.save_pgm('output.pgm', cleaned)
"""

import cv2
import numpy as np
from scipy.ndimage import label, binary_dilation, binary_erosion
from pathlib import Path


class MapCleaner:
    """Reusable map cleaning pipeline for SLAM/Cartographer outputs."""
    
    def __init__(self, verbose=True):
        self.verbose = verbose
    
    def log(self, msg):
        """Print verbose output."""
        if self.verbose:
            print(f"[MapCleaner] {msg}")
    
    @staticmethod
    def load_pgm(file_path):
        """Load PGM file (P5 binary or P2 ASCII)."""
        with open(file_path, 'rb') as f:
            lines = []
            for _ in range(3):
                line = f.readline().decode('latin-1').strip()
                while line.startswith('#'):
                    line = f.readline().decode('latin-1').strip()
                lines.append(line)
            
            magic = lines[0]
            w, h = map(int, lines[1].split())
            maxval = int(lines[2])
            
            if magic == 'P5':
                data = np.frombuffer(f.read(w * h), dtype=np.uint8)
            else:  # P2 ASCII
                data = np.array(list(map(int, f.read().decode('latin-1').split())), 
                              dtype=np.uint16)
            
            if maxval != 255:
                data = (data * 255 // maxval).astype(np.uint8)
            else:
                data = data.astype(np.uint8)
            
            return data.reshape((h, w))

    @staticmethod
    def read_ros_map_yaml(yaml_path):
        """Read the simple ROS map YAML used by map_server and slam_toolbox."""
        metadata = {}
        yaml_path = Path(yaml_path)

        with open(yaml_path, 'r', encoding='utf-8') as f:
            for line in f:
                line = line.split('#', 1)[0].strip()
                if not line or ':' not in line:
                    continue
                key, value = line.split(':', 1)
                metadata[key.strip()] = value.strip().strip('"').strip("'")

        return metadata

    @staticmethod
    def resolve_ros_map_image(yaml_path):
        """Resolve the image path referenced by a ROS map YAML file."""
        yaml_path = Path(yaml_path)
        metadata = MapCleaner.read_ros_map_yaml(yaml_path)
        image_value = metadata.get('image')

        if not image_value:
            raise ValueError(f"No image field found in map YAML: {yaml_path}")

        image_path = Path(image_value)
        if image_path.is_absolute() and image_path.exists():
            return image_path

        relative_path = yaml_path.parent / image_path
        if relative_path.exists():
            return relative_path

        basename_fallback = yaml_path.parent / image_path.name
        if basename_fallback.exists():
            return basename_fallback

        raise FileNotFoundError(f"Map image from YAML not found: {image_value}")

    @staticmethod
    def read_ros_map_resolution(map_path, default=0.05):
        """Read map resolution from a YAML file or an adjacent YAML file."""
        path = Path(map_path)
        yaml_path = path if path.suffix.lower() in ('.yaml', '.yml') else path.with_suffix('.yaml')

        if not yaml_path.exists():
            return default

        try:
            metadata = MapCleaner.read_ros_map_yaml(yaml_path)
            return float(metadata.get('resolution', default))
        except Exception:
            return default
    
    @staticmethod
    def load_image(file_path):
        """Load PGM, common image formats, or ROS map YAML."""
        suffix = Path(file_path).suffix.lower()
        if suffix in ('.yaml', '.yml'):
            file_path = MapCleaner.resolve_ros_map_image(file_path)
            suffix = Path(file_path).suffix.lower()
        if suffix == '.pgm':
            return MapCleaner.load_pgm(file_path)
        return cv2.imread(str(file_path), 0)
    
    @staticmethod
    def save_pgm(file_path, img):
        """Save as PGM file."""
        h, w = img.shape
        header = f"P5\n# Cleaned by map_cleaner_core\n{w} {h}\n255\n".encode('latin-1')
        with open(file_path, 'wb') as f:
            f.write(header)
            f.write(img.astype(np.uint8).tobytes())
    
    @staticmethod
    def median_filter(img, kernel_size):
        """Apply median filter."""
        return cv2.medianBlur(img, kernel_size)
    
    @staticmethod
    def sobel_edges(img):
        """Compute Sobel edge magnitude."""
        sobelx = cv2.Sobel(img, cv2.CV_32F, 1, 0, ksize=3)
        sobely = cv2.Sobel(img, cv2.CV_32F, 0, 1, ksize=3)
        mag = np.hypot(sobelx, sobely)
        return (mag / mag.max() * 255).astype(np.uint8) if mag.max() > 0 else mag.astype(np.uint8)
    
    @staticmethod
    def label_blobs(binary_img):
        """Label connected components."""
        labeled, count = label(binary_img)
        return labeled, count

    @staticmethod
    def _clip_int(value, low, high):
        return int(np.clip(round(float(value)), low, high))

    @staticmethod
    def _upper_valley_threshold(img):
        """
        Pick a free-space threshold from the bright end of the histogram.
        This handles common SLAM maps where unknown is light gray and free is white.
        """
        hist = np.bincount(img.ravel(), minlength=256).astype(np.float32)
        hist = np.convolve(hist, np.ones(7, dtype=np.float32) / 7, mode='same')
        peak_candidates = []

        for value in range(151, 255):
            if hist[value] >= hist[value - 1] and hist[value] >= hist[value + 1]:
                peak_candidates.append((hist[value], value))

        bright_pixels = img[img >= 150]
        if len(peak_candidates) >= 2:
            peaks = sorted(peak_candidates, reverse=True)[:4]
            peaks = sorted({value for _, value in peaks})
            if len(peaks) >= 2:
                low_peak, high_peak = peaks[-2], peaks[-1]
                if high_peak - low_peak >= 12:
                    valley = low_peak + int(np.argmin(hist[low_peak:high_peak + 1]))
                    return MapCleaner._clip_int(valley, 180, 254)

        if bright_pixels.size:
            return MapCleaner._clip_int(np.percentile(bright_pixels, 75), 180, 254)
        return 222

    @staticmethod
    def _otsu_threshold(values, fallback, low, high):
        values = np.asarray(values, dtype=np.uint8)
        if values.size < 64 or values.min() == values.max():
            return fallback

        threshold, _ = cv2.threshold(values.reshape(-1, 1), 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        return MapCleaner._clip_int(threshold, low, high)

    def auto_tune(self, raw_img):
        """
        Estimate good starting parameters from the image histogram and edge strength.

        The returned values are intentionally conservative; they should make a
        useful first cleaned map while still leaving room for manual adjustment.
        """
        h, w = raw_img.shape
        total_px = raw_img.size
        img = raw_img.astype(np.uint8)

        sample = img
        if total_px > 1_000_000:
            step = int(np.ceil(np.sqrt(total_px / 1_000_000)))
            sample = img[::step, ::step]

        noise = np.median(np.abs(sample.astype(np.float32) - np.median(sample)))
        if noise > 12 or min(h, w) > 1800:
            med_kernel = 9
        elif noise < 5 and min(h, w) < 700:
            med_kernel = 5
        else:
            med_kernel = 7

        filtered = self.median_filter(img, med_kernel)
        free_threshold = self._upper_valley_threshold(filtered)

        non_free = filtered[filtered <= free_threshold]
        dark_threshold = self._otsu_threshold(non_free, fallback=120, low=50, high=200)

        bright_mask = (filtered > free_threshold).astype(np.uint8)
        labeled_bright, n_bright = self.label_blobs(bright_mask)
        if n_bright > 0:
            sizes = np.bincount(labeled_bright.ravel())
            track_label = np.argmax(sizes[1:]) + 1 if len(sizes) > 1 else 1
            track_mask = labeled_bright == track_label
        else:
            track_mask = np.zeros_like(filtered, dtype=bool)

        mag = self.sobel_edges(filtered)
        radius_seed = self._clip_int(np.sqrt(total_px) / 180, 4, 20)
        edge_zone = binary_dilation(track_mask, iterations=radius_seed) & ~track_mask
        edge_values = mag[edge_zone & (mag > 0)]
        if edge_values.size < 64:
            edge_values = mag[mag > 0]

        if edge_values.size:
            grad_threshold = max(
                self._otsu_threshold(edge_values, fallback=60, low=20, high=200),
                self._clip_int(np.percentile(edge_values, 65), 20, 200),
            )
        else:
            grad_threshold = 60

        return {
            'med_kernel': med_kernel,
            'free_threshold': free_threshold,
            'grad_threshold': grad_threshold,
            'dark_threshold': dark_threshold,
            'dilate': 1,
            'erode': 1,
            'search_radius': radius_seed,
            'min_blob': self._clip_int(total_px * 0.00005, 10, 250),
            'fill_gaps': True,
            'smooth_walls': True,
        }

    def auto_clean(self, raw_img, overrides=None):
        """Tune parameters from the image, apply optional overrides, then clean."""
        params = self.auto_tune(raw_img)
        if overrides:
            params.update({k: v for k, v in overrides.items() if v is not None})
        return self.clean(raw_img, **params), params

    def auto_tune_f1tenth(self, raw_img):
        """Estimate controls for a white-track/gray-background F1TENTH map."""
        img = raw_img.astype(np.uint8)
        free_threshold = self._upper_valley_threshold(img)
        total_px = img.size

        return {
            'free_threshold': free_threshold,
            'wall_thickness': self._clip_int(np.sqrt(total_px) / 220, 3, 14),
            'smooth_radius': 2,
            'min_track_blob': self._clip_int(total_px * 0.0002, 30, 1000),
            'cover_track_edge': True,
            'keep_largest_track': True,
        }

    def clean_f1tenth(self, raw_img, free_threshold=None, wall_thickness=5,
                     smooth_radius=2, min_track_blob=80,
                     cover_track_edge=True, keep_largest_track=True):
        """
        Clean an F1TENTH racetrack map by segmenting the white track first.

        This is better for maps where drivable track is white and non-track
        space is gray: the wall is a fixed-thickness band around the track
        boundary, so both inner and outer boundaries stay consistent.
        """
        img = raw_img.astype(np.uint8)
        h, w = img.shape

        if free_threshold is None:
            free_threshold = self.auto_tune_f1tenth(img)['free_threshold']

        track_mask = img >= int(free_threshold)

        if smooth_radius > 0:
            kernel_size = int(smooth_radius) * 2 + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kernel_size, kernel_size))
            track_mask = cv2.morphologyEx(track_mask.astype(np.uint8), cv2.MORPH_CLOSE, kernel) > 0
            track_mask = cv2.morphologyEx(track_mask.astype(np.uint8), cv2.MORPH_OPEN, kernel) > 0

        labeled_track, n_track = self.label_blobs(track_mask)
        cleaned_track = np.zeros_like(track_mask, dtype=bool)

        if n_track > 0:
            sizes = np.bincount(labeled_track.ravel())
            if keep_largest_track:
                track_label = np.argmax(sizes[1:]) + 1 if len(sizes) > 1 else 1
                cleaned_track = labeled_track == track_label
            else:
                for blob_id in range(1, len(sizes)):
                    if sizes[blob_id] >= min_track_blob:
                        cleaned_track[labeled_track == blob_id] = True

        track_mask = cleaned_track
        wall_thickness = max(1, int(wall_thickness))
        if cover_track_edge:
            inside_px = max(1, int(np.ceil(wall_thickness / 2)))
            outside_px = max(1, wall_thickness - inside_px)
        else:
            inside_px = 0
            outside_px = wall_thickness

        track_u8 = track_mask.astype(np.uint8)
        outside_u8 = (~track_mask).astype(np.uint8)
        dist_inside = cv2.distanceTransform(track_u8, cv2.DIST_L2, 3)
        dist_outside = cv2.distanceTransform(outside_u8, cv2.DIST_L2, 3)

        wall_mask = (~track_mask) & (dist_outside <= outside_px)
        if inside_px > 0:
            wall_mask |= track_mask & (dist_inside <= inside_px)

        result = np.full((h, w), 128, dtype=np.uint8)
        result[track_mask] = 255
        result[wall_mask] = 0

        stats = self.compute_stats(result)
        self.log(
            f"F1TENTH clean: {stats['walls']}% wall, "
            f"{stats['free']}% free, {stats['unknown']}% unknown"
        )

        return result

    def auto_clean_f1tenth(self, raw_img, overrides=None):
        """Tune and clean using the F1TENTH racetrack algorithm."""
        params = self.auto_tune_f1tenth(raw_img)
        if overrides:
            params.update({k: v for k, v in overrides.items() if v is not None})
        return self.clean_f1tenth(raw_img, **params), params
    
    def clean(self, raw_img, med_kernel=7, free_threshold=222, grad_threshold=60,
              dark_threshold=120, dilate=1, erode=1, search_radius=8, min_blob=20,
              fill_gaps=True, smooth_walls=True):
        """
        Clean a map image.
        
        Args:
            raw_img: Input grayscale numpy array (0-255)
            med_kernel: Median filter kernel size (odd number, 3-11)
            free_threshold: Brightness cutoff for free space (180-254)
            grad_threshold: Edge detection sensitivity (20-200)
            dark_threshold: Dark pixel threshold (50-200)
            dilate: Wall dilation iterations (0-8)
            erode: Wall erosion iterations (0-8)
            search_radius: Search distance from track interior (2-20)
            min_blob: Minimum blob size to keep (5-200)
            fill_gaps: Fill dark regions inside track
            smooth_walls: Smooth edges before morphology
        
        Returns:
            Cleaned binary image (0=wall, 255=free, 128=unknown)
        """
        h, w = raw_img.shape
        self.log(f"Processing {w}×{h} image...")
        
        # Step 1: Median filter
        med_kernel = int(med_kernel) | 1  # Ensure odd
        filtered = self.median_filter(raw_img, med_kernel)
        self.log(f"  ✓ Median filtered (k={med_kernel})")
        
        # Step 2: Detect free space
        bright_mask = (filtered > free_threshold).astype(np.uint8)
        labeled_bright, n_bright = self.label_blobs(bright_mask)
        
        if n_bright > 0:
            sizes = np.bincount(labeled_bright.ravel())
            track_label = np.argmax(sizes[1:]) + 1 if len(sizes) > 1 else 1
            track_mask = (labeled_bright == track_label).astype(np.uint8)
            track_size = np.sum(track_mask)
            self.log(f"  ✓ Free space detected ({track_size} px)")
        else:
            track_mask = np.zeros_like(filtered, dtype=np.uint8)
            self.log(f"  ⚠ No free space detected (all unknown)")
        
        # Step 3: Gradient detection
        mag = self.sobel_edges(filtered)
        self.log(f"  ✓ Edge detection complete")
        
        # Search zone
        search_zone = binary_dilation(track_mask > 0, iterations=search_radius).astype(np.uint8)
        
        # Wall candidates
        wall_cand = np.zeros_like(filtered, dtype=np.uint8)
        wall_cand[(search_zone > 0) & (track_mask == 0) & 
                  ((mag > grad_threshold) | (filtered < dark_threshold))] = 1
        
        self.log(f"  ✓ Wall candidates identified ({np.sum(wall_cand)} px)")
        
        # Step 4: Morphology
        wall_mask = wall_cand.copy()
        
        if smooth_walls:
            wall_mask = binary_dilation(wall_mask > 0, iterations=1).astype(np.uint8)
            wall_mask = binary_erosion(wall_mask > 0, iterations=1).astype(np.uint8)
        
        if dilate > 0:
            wall_mask = binary_dilation(wall_mask > 0, iterations=dilate).astype(np.uint8)
        if erode > 0:
            wall_mask = binary_erosion(wall_mask > 0, iterations=erode).astype(np.uint8)
        
        wall_mask[track_mask > 0] = 0
        self.log(f"  ✓ Morphology applied (d={dilate} e={erode})")
        
        # Step 5: Fill inner
        if fill_gaps:
            inner_dark = (track_mask > 0) & (filtered < dark_threshold)
            wall_mask[inner_dark] = 1
            self.log(f"  ✓ Inner gaps filled")
        
        # Step 6: Remove small blobs
        labeled_wall, n_wall = self.label_blobs(wall_mask > 0)
        if n_wall > 0:
            sizes = np.bincount(labeled_wall.ravel())
            removed = 0
            for blob_id in range(1, len(sizes)):
                if sizes[blob_id] < min_blob:
                    wall_mask[labeled_wall == blob_id] = 0
                    removed += 1
            self.log(f"  ✓ Removed {removed} noise blobs")
        
        # Step 7: Output
        result = np.full((h, w), 128, dtype=np.uint8)
        result[track_mask > 0] = 255
        result[wall_mask > 0] = 0
        
        # Statistics
        stats = self.compute_stats(result)
        self.log(f"  ✓ Complete: {stats['walls']}% wall, {stats['free']}% free, {stats['unknown']}% unknown")
        
        return result
    
    def clean_from_file(self, input_path, **kwargs):
        """
        Load a map file and clean it.
        
        Args:
            input_path: Path to PGM or PNG file
            **kwargs: Parameters for clean() method
        
        Returns:
            Cleaned image array
        """
        self.log(f"Loading: {input_path}")
        raw_img = self.load_image(input_path)
        
        if raw_img is None:
            raise FileNotFoundError(f"Could not load: {input_path}")
        
        return self.clean(raw_img, **kwargs)

    def auto_clean_from_file(self, input_path, overrides=None):
        """Load a map and clean it with automatically tuned parameters."""
        self.log(f"Loading: {input_path}")
        raw_img = self.load_image(input_path)

        if raw_img is None:
            raise FileNotFoundError(f"Could not load: {input_path}")

        return self.auto_clean(raw_img, overrides=overrides)
    
    @staticmethod
    def compute_stats(img):
        """Compute image statistics."""
        total = img.size
        return {
            'walls': int(np.sum(img == 0) / total * 100),
            'free': int(np.sum(img == 255) / total * 100),
            'unknown': int(np.sum(img == 128) / total * 100),
            'wall_px': int(np.sum(img == 0)),
            'free_px': int(np.sum(img == 255)),
        }
    
    def process_and_save(self, input_path, output_path, **kwargs):
        """Load, clean, and save in one step."""
        cleaned = self.clean_from_file(input_path, **kwargs)
        self.save_pgm(output_path, cleaned)
        self.log(f"Saved: {output_path}")


# ============================================================================
# USAGE EXAMPLES
# ============================================================================

if __name__ == '__main__':
    import sys
    
    print("=" * 60)
    print("Map Cleaner Core Module — Usage Examples")
    print("=" * 60)
    print()
    
    # Example 1: Simple cleaning
    print("Example 1: Simple one-liner cleaning")
    print("-" * 60)
    print("""
cleaner = MapCleaner()
cleaned = cleaner.clean_from_file('input.pgm')
cleaner.save_pgm('output.pgm', cleaned)
    """)
    print()
    
    # Example 2: Custom parameters
    print("Example 2: Custom parameters")
    print("-" * 60)
    print("""
cleaner = MapCleaner()
cleaned = cleaner.clean_from_file('input.pgm',
    med_kernel=9,           # More blur
    free_threshold=220,     # Lower = more free space
    grad_threshold=40,      # Lower = more edges
    dilate=2,               # Thicker walls
    erode=1,                # Thin them back slightly
    min_blob=30             # Remove smaller noise
)
cleaner.save_pgm('output.pgm', cleaned)
    """)
    print()
    
    # Example 3: Silent mode (no logs)
    print("Example 3: Silent mode")
    print("-" * 60)
    print("""
cleaner = MapCleaner(verbose=False)
cleaned = cleaner.clean_from_file('input.pgm')
stats = cleaner.compute_stats(cleaned)
print(f"Walls: {stats['walls']}%, Free: {stats['free']}%")
    """)
    print()
    
    # Example 4: Batch processing
    print("Example 4: Batch processing multiple files")
    print("-" * 60)
    print("""
from pathlib import Path
cleaner = MapCleaner()

for pgm_file in Path('maps/').glob('*.pgm'):
    output = Path('clean') / f"{pgm_file.stem}_clean.pgm"
    output.parent.mkdir(exist_ok=True)
    cleaner.process_and_save(str(pgm_file), str(output))
    """)
    print()
    
    # Example 5: Programmatic use in ROS node
    print("Example 5: In a ROS node or custom pipeline")
    print("-" * 60)
    print("""
import rospy
from nav_msgs.msg import OccupancyGrid
from map_cleaner_core import MapCleaner
import numpy as np

def map_callback(msg):
    # Convert ROS OccupancyGrid to numpy
    data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
    
    # Clean
    cleaner = MapCleaner(verbose=False)
    cleaned = cleaner.clean(data, med_kernel=7, free_threshold=222)
    
    # Convert back to ROS message
    clean_msg = OccupancyGrid()
    clean_msg.data = cleaned.flatten().tolist()
    clean_msg.info = msg.info
    pub.publish(clean_msg)

rospy.Subscriber('/map', OccupancyGrid, map_callback)
    """)
    print()
    
    # Example 6: Tuning workflow
    print("Example 6: Find optimal parameters")
    print("-" * 60)
    print("""
cleaner = MapCleaner()
raw = cleaner.load_image('input.pgm')

best_params = None
best_score = 0

for med_k in [5, 7, 9]:
    for grad_th in [40, 60, 80]:
        cleaned = cleaner.clean(raw, med_kernel=med_k, grad_threshold=grad_th)
        stats = cleaner.compute_stats(cleaned)
        
        # Simple heuristic: prefer higher free space, lower unknown
        score = stats['free'] - stats['unknown']
        if score > best_score:
            best_score = score
            best_params = {'med_kernel': med_k, 'grad_threshold': grad_th}

print(f"Best params: {best_params}")
    """)
    print()
    
    print("=" * 60)
    print("Import this module in your code:")
    print("  from map_cleaner_core import MapCleaner")
    print("=" * 60)
