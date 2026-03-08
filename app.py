import webview
from flask import Flask, send_from_directory, Response, request, jsonify
from flask_cors import CORS
import os
import threading
import sys
import cv2
import torch
import numpy as np
import time
import requests
from deepface import DeepFace

# Define the folder containing your HTML and assets
static_folder = os.path.dirname(os.path.abspath(__file__))
app = Flask(__name__, static_folder=static_folder)
CORS(app) # Enable CORS for all routes

# --- SUPABASE CONFIG ---
SUPABASE_URL = 'https://gnetjhhufrwdgpoyhuph.supabase.co'
SUPABASE_KEY = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImduZXRqaGh1ZnJ3ZGdwb3lodXBoIiwicm9sZSI6ImFub24iLCJpYXQiOjE3NzExNTEzNzgsImV4cCI6MjA4NjcyNzM3OH0.-tiSm5QzsEYCN-0noQimGnOAcbUF499rqlFw3lbI7YI'

def fetch_settings_from_supabase():
    """Fetch settings from Supabase at startup"""
    global MOBILE_IP, VIDEO_URL, VIDEO_ROTATION
    try:
        headers = {
            'apikey': SUPABASE_KEY,
            'Authorization': f'Bearer {SUPABASE_KEY}'
        }
        response = requests.get(
            f'{SUPABASE_URL}/rest/v1/settings?id=eq.1&select=*',
            headers=headers,
            timeout=5
        )
        if response.status_code == 200:
            data = response.json()
            if data and len(data) > 0:
                settings = data[0]
                if settings.get('mobile_ip'):
                    MOBILE_IP = settings['mobile_ip']
                    VIDEO_URL = f"http://{MOBILE_IP}:8888/video?feed=true"
                if settings.get('video_rotation') is not None:
                    VIDEO_ROTATION = int(settings['video_rotation'])
                print(f"✅ Settings loaded from Supabase:")
                print(f"   Mobile IP: {MOBILE_IP}")
                print(f"   Video URL: {VIDEO_URL}")
                print(f"   Rotation: {VIDEO_ROTATION}°")
                return True
    except Exception as e:
        print(f"⚠️ Could not fetch settings from Supabase: {e}")
    return False

# Mobile Sensor App IP (loaded from Supabase or default)
MOBILE_IP = '10.49.229.251'
VIDEO_URL = f"http://{MOBILE_IP}:8888/video?feed=true"
mobile_ip_lock = threading.Lock()

# Video rotation for AI processing (0, 90, 180, 270)
VIDEO_ROTATION = 0

# Fetch settings from Supabase at startup
print("🚀 Loading settings from Supabase...")
fetch_settings_from_supabase()

# Precompute rotation constants for speed
ROTATE_MAP = {
    90: cv2.ROTATE_90_CLOCKWISE,
    180: cv2.ROTATE_180,
    270: cv2.ROTATE_90_COUNTERCLOCKWISE
}

# --- Face Recognition Globals ---
target_dir = os.path.join(static_folder, "targets")
if not os.path.exists(target_dir):
    os.makedirs(target_dir)

alert_active = False
detected_target = None
alert_last_seen = 0  # Timestamp of last detection
ALERT_PERSIST_SECONDS = 8  # Keep alert active for 8 seconds after last detection
target_lock = threading.Lock()

# --- MiDaS Model Setup ---
class MidasSmall:
    def __init__(self):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"Using device: {self.device}")
        
        # Load MiDaS Small model
        self.midas = torch.hub.load("isl-org/MiDaS", "MiDaS_small")
        self.midas.to(self.device)
        self.midas.eval()
        
        # Load transforms
        midas_transforms = torch.hub.load("isl-org/MiDaS", "transforms")
        self.transform = midas_transforms.small_transform

    def predict(self, frame):
        # Resize small for faster inference
        small_frame = cv2.resize(frame, (320, 240)) 
        img = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        input_batch = self.transform(img).to(self.device)

        with torch.no_grad():
            prediction = self.midas(input_batch)
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=img.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()

        depth_map = prediction.cpu().numpy()
        # Normalize to 0-255
        depth_map = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        # Apply color map
        depth_map = cv2.applyColorMap(depth_map, cv2.COLORMAP_INFERNO)
        
        return depth_map
    
    def predict_both(self, frame):
        """Run model once, return (colored_depth, raw_gray_depth)"""
        small_frame = cv2.resize(frame, (320, 240))
        img = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)
        input_batch = self.transform(img).to(self.device)

        with torch.no_grad():
            prediction = self.midas(input_batch)
            prediction = torch.nn.functional.interpolate(
                prediction.unsqueeze(1),
                size=img.shape[:2],
                mode="bicubic",
                align_corners=False,
            ).squeeze()

        depth_map = prediction.cpu().numpy()
        depth_gray = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        depth_colored = cv2.applyColorMap(depth_gray, cv2.COLORMAP_INFERNO)
        return depth_colored, depth_gray

    def predict_raw(self, frame):
        """Return raw normalized depth values (0-255) without colormap"""
        _, depth_gray = self.predict_both(frame)
        return depth_gray

# --- Obstacle Detection & Path Planning ---
class ObstacleDetector:
    def __init__(self):
        # Threshold for obstacle detection (higher = closer = obstacle)
        self.obstacle_threshold = 180  # Objects with depth > 180 are "close"
        self.safe_threshold = 100      # Objects with depth < 100 are "far/safe"
        
        # Grid divisions for path analysis
        self.grid_cols = 5  # Left, Left-Center, Center, Right-Center, Right
        self.grid_rows = 3  # Top, Middle, Bottom (we care about bottom 2 rows)
        
        # Direction names
        self.directions = ['HARD_LEFT', 'LEFT', 'FORWARD', 'RIGHT', 'HARD_RIGHT']
        
        # Last known rover heading
        self.rover_heading = 0
        
    def set_heading(self, heading):
        """Update rover heading from compass"""
        self.rover_heading = heading
        
    def analyze_depth(self, depth_gray):
        """
        Analyze depth map to find obstacles and safe paths
        Returns: obstacle_map, path_scores, best_direction, obstacles_info
        """
        h, w = depth_gray.shape
        
        # Focus on lower 2/3 of image (ground level)
        roi_top = h // 3
        roi = depth_gray[roi_top:, :]
        roi_h, roi_w = roi.shape
        
        # Divide into grid cells
        cell_w = roi_w // self.grid_cols
        cell_h = roi_h // 2  # 2 rows
        
        # Calculate obstacle score for each column (0=clear, 100=blocked)
        path_scores = []
        obstacles_info = []
        
        for col in range(self.grid_cols):
            x1 = col * cell_w
            x2 = (col + 1) * cell_w if col < self.grid_cols - 1 else roi_w
            
            # Analyze both rows in this column
            total_obstacle_pixels = 0
            total_pixels = 0
            
            for row in range(2):
                y1 = row * cell_h
                y2 = (row + 1) * cell_h if row < 1 else roi_h
                
                cell = roi[y1:y2, x1:x2]
                
                # Count obstacle pixels (high depth = close = obstacle)
                obstacle_pixels = np.sum(cell > self.obstacle_threshold)
                total_pixels += cell.size
                
                # Weight bottom row more (closer to rover)
                weight = 2.0 if row == 1 else 1.0
                total_obstacle_pixels += obstacle_pixels * weight
            
            # Calculate obstruction percentage (0-100)
            obstruction = min(100, int((total_obstacle_pixels / (total_pixels * 1.5)) * 100))
            path_scores.append(100 - obstruction)  # Higher = safer
            
            # Track obstacle details
            if obstruction > 30:
                obstacles_info.append({
                    'direction': self.directions[col],
                    'severity': 'HIGH' if obstruction > 70 else 'MEDIUM' if obstruction > 50 else 'LOW',
                    'percentage': obstruction
                })
        
        # Find best direction
        best_idx = np.argmax(path_scores)
        best_direction = self.directions[best_idx]
        
        # If center is reasonably clear, prefer it
        center_score = path_scores[2]
        if center_score > 60:
            best_direction = 'FORWARD'
            best_idx = 2
            
        # Calculate suggested heading change
        heading_adjustment = (best_idx - 2) * 30  # -60 to +60 degrees
        suggested_heading = (self.rover_heading + heading_adjustment) % 360
        
        return {
            'path_scores': path_scores,
            'best_direction': best_direction,
            'best_score': path_scores[best_idx],
            'obstacles': obstacles_info,
            'heading_adjustment': heading_adjustment,
            'suggested_heading': suggested_heading,
            'current_heading': self.rover_heading,
            'center_clear': center_score > 60,
            'path_blocked': all(score < 30 for score in path_scores)
        }
    
    def draw_path_overlay(self, frame, depth_gray, analysis):
        """Draw path visualization overlay on frame"""
        h, w = frame.shape[:2]
        overlay = frame.copy()
        
        # Draw path corridor visualization
        path_scores = analysis['path_scores']
        num_cols = len(path_scores)
        col_width = w // num_cols
        
        # Draw vertical path strips
        for i, score in enumerate(path_scores):
            x1 = i * col_width
            x2 = (i + 1) * col_width if i < num_cols - 1 else w
            
            # Color based on safety (green=safe, yellow=caution, red=blocked)
            if score > 70:
                color = (0, 255, 0)  # Green - safe
                alpha = 0.15
            elif score > 40:
                color = (0, 255, 255)  # Yellow - caution
                alpha = 0.2
            else:
                color = (0, 0, 255)  # Red - blocked
                alpha = 0.25
            
            # Draw semi-transparent strip on lower 2/3
            y_start = h // 3
            cv2.rectangle(overlay, (x1, y_start), (x2, h), color, -1)
        
        # Blend overlay
        cv2.addWeighted(overlay, 0.3, frame, 0.7, 0, frame)
        
        # Draw center path guide line
        best_idx = self.directions.index(analysis['best_direction'])
        center_x = (best_idx * col_width) + (col_width // 2)
        
        # Draw arrow pointing in best direction
        arrow_start = (w // 2, h - 50)
        arrow_end = (center_x, h // 2)
        
        arrow_color = (0, 255, 0) if analysis['best_score'] > 60 else (0, 255, 255)
        cv2.arrowedLine(frame, arrow_start, arrow_end, arrow_color, 3, tipLength=0.3)
        
        # Draw path boundary lines
        for i in range(1, num_cols):
            x = i * col_width
            cv2.line(frame, (x, h // 3), (x, h), (255, 255, 255), 1)
        
        # Draw direction indicator at top
        direction_text = f"GO: {analysis['best_direction']}"
        if analysis['path_blocked']:
            direction_text = "STOP - PATH BLOCKED"
            text_color = (0, 0, 255)
        elif analysis['center_clear']:
            direction_text = "CLEAR - GO FORWARD"
            text_color = (0, 255, 0)
        else:
            text_color = (0, 255, 255)
        
        # Draw HUD box
        cv2.rectangle(frame, (10, 10), (280, 100), (0, 0, 0), -1)
        cv2.rectangle(frame, (10, 10), (280, 100), (255, 255, 255), 1)
        
        cv2.putText(frame, direction_text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
        cv2.putText(frame, f"Heading: {analysis['current_heading']:.0f} deg", (20, 55), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(frame, f"Suggest: {analysis['suggested_heading']:.0f} deg ({analysis['heading_adjustment']:+.0f})", 
                    (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 255, 100), 1)
        cv2.putText(frame, f"Path Score: {analysis['best_score']}%", (20, 95), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Draw obstacle warnings
        y_offset = 120
        for obs in analysis['obstacles'][:3]:  # Show top 3 obstacles
            severity_color = (0, 0, 255) if obs['severity'] == 'HIGH' else (0, 165, 255) if obs['severity'] == 'MEDIUM' else (0, 255, 255)
            cv2.putText(frame, f"! {obs['direction']}: {obs['severity']}", (20, y_offset), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, severity_color, 1)
            y_offset += 18
        
        return frame

# --- Threaded Video Stream ---
class VideoStream:
    def __init__(self):
        self.current_frame = None
        self.current_depth = None
        self.current_depth_gray = None  # Raw depth for obstacle detection
        self.running = False
        self.lock = threading.Lock()
        self.model = None
        self.obstacle_detector = ObstacleDetector()
        self.obstacle_analysis = None
        self.face_locations = [] # [(x, y, w, h, name)]
        self.face_names = []
        self.rover_heading = 0  # Updated from mobile sensors

    def start(self):
        if self.running:
            return
        self.running = True
        self.thread_read = threading.Thread(target=self.update_frames)
        self.thread_read.daemon = True
        self.thread_read.start()
        
        self.thread_process = threading.Thread(target=self.process_depth)
        self.thread_process.daemon = True
        self.thread_process.start()

        self.thread_face = threading.Thread(target=self.process_faces)
        self.thread_face.daemon = True
        self.thread_face.start()

    def reconnect_stream(self, new_url):
        """Force reconnection to a new video stream URL"""
        self.video_url = new_url
        print(f"Reconnecting to new video stream: {new_url}")
    
    def update_frames(self):
        global VIDEO_URL
        current_url = VIDEO_URL
        print(f"Connecting to video stream: {current_url}")
        cap = cv2.VideoCapture(current_url)
        if not cap.isOpened():
             print("Network stream failed. Using local webcam.")
             cap = cv2.VideoCapture(0)

        while self.running:
            # Check if URL changed
            if current_url != VIDEO_URL:
                print(f"Video URL changed from {current_url} to {VIDEO_URL}")
                cap.release()
                current_url = VIDEO_URL
                cap = cv2.VideoCapture(current_url)
                if not cap.isOpened():
                    print("Network stream failed. Using local webcam.")
                    cap = cv2.VideoCapture(0)
            
            success, frame = cap.read()
            if success:
                # Apply rotation for AI models (cv2.rotate is C++ optimized, <1ms)
                if VIDEO_ROTATION in ROTATE_MAP:
                    frame = cv2.rotate(frame, ROTATE_MAP[VIDEO_ROTATION])
                with self.lock:
                    self.current_frame = frame
            else:
                print("Failed to read frame, reconnecting...")
                cap.release()
                time.sleep(2)
                cap = cv2.VideoCapture(VIDEO_URL)
                if not cap.isOpened():
                     cap = cv2.VideoCapture(0)
            time.sleep(0.01)

    def process_depth(self):
        if self.model is None:
             self.model = MidasSmall()

        while self.running:
            frame_to_process = None
            with self.lock:
                if self.current_frame is not None:
                    frame_to_process = self.current_frame.copy()
            
            if frame_to_process is not None:
                depth_map, depth_gray = self.model.predict_both(frame_to_process)
                
                self.obstacle_detector.set_heading(self.rover_heading)
                obstacle_analysis = self.obstacle_detector.analyze_depth(depth_gray)

                with self.lock:
                    self.current_depth = depth_map
                    self.current_depth_gray = depth_gray
                    self.obstacle_analysis = obstacle_analysis
            else:
                time.sleep(0.1)
            time.sleep(0.01)

    def process_faces(self):
        global alert_active, detected_target, alert_last_seen

        while self.running:
            frame_to_process = None
            with self.lock:
                if self.current_frame is not None:
                    frame_to_process = self.current_frame.copy()

            if frame_to_process is not None:
                try:
                    target_files = [f for f in os.listdir(target_dir) if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
                    if len(target_files) > 0:
                        small_frame = cv2.resize(frame_to_process, (0, 0), fx=0.5, fy=0.5)
                        
                        dfs = DeepFace.find(
                            img_path=small_frame, 
                            db_path=target_dir, 
                            model_name="VGG-Face",
                            enforce_detection=False,
                            detector_backend="opencv",
                            silent=True
                        )
                        
                        found_targets = []
                        current_locations = []
                        
                        for df in dfs:
                            if not df.empty:
                                for index, row in df.iterrows():
                                    identity = row['identity']
                                    name = os.path.basename(identity).split('.')[0]
                                    found_targets.append(name)
                                    
                                    x = int(row['source_x'] * 2)
                                    y = int(row['source_y'] * 2)
                                    w = int(row['source_w'] * 2)
                                    h = int(row['source_h'] * 2)
                                    current_locations.append((x, y, w, h, name))
                        
                        with self.lock:
                            if found_targets:
                                alert_active = True
                                detected_target = found_targets[0]
                                alert_last_seen = time.time()
                                self.face_locations = current_locations
                                self.face_names = found_targets
                                print(f"🎯 TARGET DETECTED: {found_targets}")
                            else:
                                if time.time() - alert_last_seen > ALERT_PERSIST_SECONDS:
                                    alert_active = False
                                    detected_target = None
                                    self.face_locations = []
                                    self.face_names = []
                    else:
                        if time.time() - alert_last_seen > ALERT_PERSIST_SECONDS:
                            with self.lock:
                                alert_active = False

                except Exception as e:
                    if time.time() - alert_last_seen > ALERT_PERSIST_SECONDS:
                        with self.lock:
                            alert_active = False
                            detected_target = None
                            self.face_locations = []
                            self.face_names = []
            else:
                time.sleep(0.1)
            time.sleep(0.05)

    def get_frame(self, mode):
        with self.lock:
            if self.current_frame is None:
                return None
            
            display_frame = self.current_frame.copy()

            if mode == 'rgb':
                # Draw Faces on RGB mode
                if self.face_locations:
                    for (x, y, w, h, name) in self.face_locations:
                        cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.rectangle(display_frame, (x, y - 35), (x + w, y), (0, 255, 0), cv2.FILLED)
                        font = cv2.FONT_HERSHEY_DUPLEX
                        cv2.putText(display_frame, name, (x + 6, y - 6), font, 1.0, (255, 255, 255), 1)
                return display_frame
            
            elif mode == 'depth':
                if self.current_depth is None:
                    return display_frame
                return self.current_depth
            
            elif mode == 'nav' or mode == 'navigation':
                # Navigation mode with path overlay
                if self.obstacle_analysis and self.current_depth_gray is not None:
                    nav_frame = self.obstacle_detector.draw_path_overlay(
                        display_frame, self.current_depth_gray, self.obstacle_analysis
                    )
                    return nav_frame
                return display_frame
            
            elif mode == 'both':
                # Draw faces on the RGB side
                if self.face_locations:
                    for (x, y, w, h, name) in self.face_locations:
                        cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.rectangle(display_frame, (x, y - 35), (x + w, y), (0, 255, 0), cv2.FILLED)
                        font = cv2.FONT_HERSHEY_DUPLEX
                        cv2.putText(display_frame, name, (x + 6, y - 6), font, 1.0, (255, 255, 255), 1)
                if self.current_depth is None:
                    depth_display = display_frame
                else:
                    h, w = display_frame.shape[:2]
                    depth_display = cv2.resize(self.current_depth, (w, h))
                combined = np.hstack((display_frame, depth_display))
                return combined
        return None
    
    def set_rover_heading(self, heading):
        """Update rover heading for navigation calculations"""
        with self.lock:
            self.rover_heading = heading
        return None

    def get_ai_data(self):
        with self.lock:
            faces = []
            if self.face_names:
                for name in self.face_names:
                    faces.append({"name": name, "confidence": 95})
            
            # Navigation data
            nav_data = {
                "path_clear": True,
                "best_direction": "FORWARD",
                "path_scores": [100, 100, 100, 100, 100],
                "obstacles": [],
                "heading_adjustment": 0,
                "suggested_heading": 0
            }
            
            if self.obstacle_analysis:
                nav_data = {
                    "path_clear": self.obstacle_analysis.get('center_clear', True),
                    "path_blocked": self.obstacle_analysis.get('path_blocked', False),
                    "best_direction": self.obstacle_analysis.get('best_direction', 'FORWARD'),
                    "best_score": self.obstacle_analysis.get('best_score', 100),
                    "path_scores": self.obstacle_analysis.get('path_scores', [100, 100, 100, 100, 100]),
                    "obstacles": self.obstacle_analysis.get('obstacles', []),
                    "heading_adjustment": self.obstacle_analysis.get('heading_adjustment', 0),
                    "suggested_heading": self.obstacle_analysis.get('suggested_heading', 0),
                    "current_heading": self.obstacle_analysis.get('current_heading', 0)
                }
            
            return {
                "objects": [], 
                "faces": faces, 
                "confidence": 0,
                "depth": {"min": 0, "max": 0, "avg": 0}, 
                "alert": {
                    "active": alert_active,
                    "target": detected_target
                },
                "navigation": nav_data
            }

# Global Video Stream (starts AFTER settings are loaded from Supabase)
print(f"🎬 Starting video stream with URL: {VIDEO_URL}")
video_stream = VideoStream()
video_stream.start()

def generate_frames(display_mode='both'):
    while True:
        frame = video_stream.get_frame(display_mode)
        if frame is not None:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        else:
            time.sleep(0.1)
        time.sleep(0.03)

@app.route('/')
def index():
    return send_from_directory(static_folder, 'index.html')

@app.route('/<path:filename>')
def serve_static(filename):
    return send_from_directory(static_folder, filename)

@app.route('/video_feed')
def video_feed():
    mode = request.args.get('mode', 'both')
    return Response(generate_frames(mode), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/upload_target', methods=['POST'])
def upload_target():
    if 'file' not in request.files:
        return jsonify({"error": "No file part"}), 400
    file = request.files['file']
    name = request.form.get('name', 'Unknown Target')
    
    if file.filename == '':
        return jsonify({"error": "No selected file"}), 400

    if file:
        try:
            # Read image bytes and decode with OpenCV (handles PNG, WebP, JPEG, etc.)
            file_bytes = np.frombuffer(file.read(), np.uint8)
            img = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)
            
            if img is None:
                return jsonify({"error": "Invalid image file. Could not decode."}), 400
            
            # Clean name to be safe filename
            safe_name = "".join([c for c in name if c.isalpha() or c.isdigit() or c==' ']).rstrip()
            if not safe_name:
                safe_name = "target"
            filename = f"{safe_name}.jpg"
            filepath = os.path.join(target_dir, filename)
            
            # Save as proper JPEG (ensures DeepFace can always read it)
            cv2.imwrite(filepath, img, [cv2.IMWRITE_JPEG_QUALITY, 95])
            
            # Remove old pickle files if DeepFace generated them, to force re-indexing
            for f in os.listdir(target_dir):
                if f.endswith(".pkl"):
                    try: os.remove(os.path.join(target_dir, f))
                    except: pass

            print(f"✅ Target '{name}' saved as {filename} ({img.shape[1]}x{img.shape[0]})")
            return jsonify({"status": "success", "message": f"Target '{name}' added."}), 200
        except Exception as e:
            print(f"❌ Upload error: {e}")
            return jsonify({"error": str(e)}), 500

@app.route('/list_targets')
def list_targets():
    """List all target face images"""
    try:
        targets = []
        for f in os.listdir(target_dir):
            if f.lower().endswith(('.jpg', '.jpeg', '.png', '.webp')):
                name = os.path.splitext(f)[0]
                targets.append({
                    "name": name,
                    "filename": f,
                    "url": f"/targets/{f}"
                })
        return jsonify({"targets": targets})
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/delete_target/<filename>', methods=['DELETE'])
def delete_target(filename):
    """Delete a target face image"""
    try:
        filepath = os.path.join(target_dir, filename)
        if os.path.exists(filepath):
            os.remove(filepath)
            # Remove pickle files to force re-indexing
            for f in os.listdir(target_dir):
                if f.endswith(".pkl"):
                    try: os.remove(os.path.join(target_dir, f))
                    except: pass
            return jsonify({"status": "success", "message": f"Target '{filename}' deleted."})
        else:
            return jsonify({"error": "Target not found"}), 404
    except Exception as e:
        return jsonify({"error": str(e)}), 500

@app.route('/targets/<filename>')
def serve_target(filename):
    """Serve target face images"""
    return send_from_directory(target_dir, filename)

@app.route('/ai/data')
def ai_data():
    try:
        return jsonify(video_stream.get_ai_data())
    except Exception as e:
        print(f"Error in /ai/data: {e}")
        return jsonify({
            "objects": [],
            "faces": [],
            "confidence": 0,
            "depth": {"min": 0, "max": 0, "avg": 0},
            "alert": {"active": False, "target": None},
            "navigation": {"path_clear": True, "best_direction": "FORWARD"}
        })

@app.route('/ai/navigation')
def ai_navigation():
    """Get current navigation/obstacle data"""
    data = video_stream.get_ai_data()
    return jsonify(data.get('navigation', {}))

@app.route('/ai/set_heading', methods=['POST'])
def set_heading():
    """Update rover heading from compass for navigation calculations"""
    data = request.get_json()
    heading = data.get('heading', 0)
    video_stream.set_rover_heading(heading)
    return jsonify({"status": "success", "heading": heading})

@app.route('/set_rotation', methods=['POST'])
def set_rotation():
    """Set video rotation for AI processing (synced from frontend)"""
    global VIDEO_ROTATION
    data = request.get_json()
    angle = data.get('angle', 0)
    if angle in [0, 90, 180, 270]:
        VIDEO_ROTATION = angle
        print(f"AI video rotation set to: {angle}°")
        return jsonify({"status": "success", "rotation": angle})
    return jsonify({"error": "Invalid angle. Use 0, 90, 180, or 270"}), 400

@app.route('/update_mobile_ip', methods=['POST'])
def update_mobile_ip():
    global MOBILE_IP, VIDEO_URL
    try:
        data = request.get_json()
        new_ip = data.get('mobile_ip')
        
        if not new_ip:
            return jsonify({"error": "No mobile_ip provided"}), 400
        
        with mobile_ip_lock:
            MOBILE_IP = new_ip
            VIDEO_URL = f"http://{MOBILE_IP}:8888/video?feed=true"
        
        print(f"Mobile IP updated to: {MOBILE_IP}")
        print(f"Video URL updated to: {VIDEO_URL}")
        
        return jsonify({
            "status": "success", 
            "mobile_ip": MOBILE_IP,
            "video_url": VIDEO_URL
        }), 200
    except Exception as e:
        return jsonify({"error": str(e)}), 500

def start_server():
    app.run(host='127.0.0.1', port=5000, threaded=True)

if __name__ == '__main__':
    t = threading.Thread(target=start_server)
    t.daemon = True
    t.start()

    webview.create_window(
        title='ATLAS // MISSION CONTROL', 
        url='http://127.0.0.1:5000',
        width=1200,
        height=800,
        background_color='#080808',
        confirm_close=True
    )
    
    webview.start()