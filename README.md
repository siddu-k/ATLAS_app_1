# 🎯 ATLAS – Autonomous Target Location & Autonomous System
## Mission Control & AI-Powered Rover Navigation Platform

![Language Composition](https://img.shields.io/badge/HTML-81.6%25-orange) ![Python](https://img.shields.io/badge/Python-8.4%25-blue) ![C++](https://img.shields.io/badge/C%2B%2B-7.4%25-brightgreen) ![PLpgSQL](https://img.shields.io/badge/PLpgSQL-2.6%25-purple)

**ATLAS** is an advanced autonomous rover control system combining real-time AI-powered navigation, facial recognition, depth perception, and obstacle avoidance. The system seamlessly integrates hardware control (ESP32-CAM), computer vision (OpenCV + PyTorch), and a modern web-based mission control interface.

---

## 🎬 Project Showcase

**See the ATLAS system in action:**

📹 **Project Overview & Robotics Integration**:  
[LinkedIn Post - Robotics, AI & Computer Vision](https://www.linkedin.com/posts/ksridatta_robotics-ai-computervision-activity-7439146997367906304-qVLO?utm_source=share&utm_medium=member_desktop&rcm=ACoAAE0Vj1wBU7XTIOR-iSf-SIF2RDrUgycpylQ)

🎥 **Full System Demonstration & Project Expo**:  
[LinkedIn Project Video - Mission Control in Action](https://www.linkedin.com/posts/ksridatta_gratitude-projectexpo-learningexperience-activity-7439270633449951232-Q2lT?utm_source=share&utm_medium=member_desktop&rcm=ACoAAE0Vj1wBU7XTIOR-iSf-SIF2RDrUgycpylQ)

---

## 🌟 Key Features

### 🤖 AI-Powered Vision System
- **Real-Time Face Recognition**: Uses DeepFace (VGG-Face model) for instant target detection and tracking
- **Depth Perception**: MiDaS Small model for monocular depth estimation
- **Obstacle Detection**: Intelligent path analysis with directional threat assessment
- **Multi-Mode Display**:
  - **RGB Mode**: Live camera feed with facial recognition overlays
  - **Depth Mode**: Colorized depth maps (Inferno colormap)
  - **Navigation Mode**: Obstacle-aware path guidance with directional arrows
  - **Dual Mode**: Split-screen RGB + depth visualization

### 🧭 Autonomous Navigation
- **Real-Time Path Planning**: 5-column grid analysis (HARD_LEFT, LEFT, CENTER, RIGHT, HARD_RIGHT)
- **Obstacle Avoidance**: Threshold-based detection with severity classification (LOW, MEDIUM, HIGH)
- **Compass Integration**: Rover heading synchronization with suggested heading adjustments
- **Dynamic Path Scoring**: Real-time safety assessment (0-100 scale)

### 🎥 Remote Control Interface
- **Web-Based Mission Control**: Modern, responsive HTML/CSS/JavaScript dashboard
- **Live Video Streaming**: Multi-format support with frame rate optimization
- **Target Management**: Upload, track, and delete facial recognition targets
- **Settings Sync**: Supabase integration for persistent configuration

### 🔧 Hardware Integration
- **ESP32-CAM Rover Control**: Motor speed/direction control via L298N driver
- **MOSFET Weapon Trigger**: GPIO-based firing mechanism (weapon system)
- **Mobile Sensor Integration**: Real-time video feed from smartphone camera app
- **Dynamic IP Configuration**: Network-agnostic operation with fallback to local webcam

---

## 📋 Technical Architecture

### Backend Stack
```
Flask (Web Framework)
├── CORS Support
├── Threading (Multi-threaded video processing)
├── REST API Endpoints
└── Real-time Data Streaming
```

### AI/ML Components
```
Computer Vision:
├── OpenCV (cv2) – Frame processing, depth visualization
├── DeepFace – Facial recognition & matching
├── PyTorch – Model inference backbone
└── MiDaS – Monocular depth estimation

Detection Models:
├── MiDaS Small (224x224) – Fast depth estimation
├── VGG-Face – Face embedding & recognition
└── OpenCV Haar Cascades – Face detection
```

### Frontend Stack
```
index.html (304 KB)
├── Real-time video streaming canvas
├── Navigation HUD (Heads-Up Display)
├── Target management panel
├── Settings configuration
├── Live telemetry display
└── Responsive design
```

### Database & Cloud
```
Supabase PostgreSQL
├── Settings Table (mobile IP, rotation angle)
├── Real-time configuration sync
└── Cloud-based persistence
```

---

## 🚀 Installation & Setup

### Prerequisites
```bash
# Python 3.8+
# CUDA Toolkit (optional, for GPU acceleration)
# Compatible Mobile Device with Camera (for stream)
```

### 1. Clone Repository
```bash
git clone https://github.com/siddu-k/ATLAS_app_1.git
cd ATLAS_app_1
```

### 2. Install Dependencies
```bash
pip install flask flask-cors opencv-python torch torchvision deepface numpy requests pywebview
```

### 3. Hardware Setup

**See detailed wiring guide in [CONNECTIONS.md](CONNECTIONS.md)**

**Quick Connections:**
```
ESP32-CAM → L298N Motor Driver:
  GPIO 12 → ENA (Motor A Speed - PWM)
  GPIO 13 → IN1 (Motor A Direction)
  GPIO 15 → IN2 (Motor A Direction)
  GPIO 14 → IN3 (Motor B Direction)
  GPIO 2  → IN4 (Motor B Direction)
  GND     → GND (Common Ground - CRITICAL)

ESP32-CAM → MOSFET Weapon:
  GPIO 4  → SIG/Gate (Trigger Signal)
  GND     → GND
  5V      → VCC (if required)

⚠️ CRITICAL: DO NOT use GPIO 16 (PSRAM conflict)
```

### 4. Configuration

#### Set Mobile IP (Supabase)
Edit `SUPABASE_URL` and `SUPABASE_KEY` in `app.py`:
```python
SUPABASE_URL = 'your-supabase-url'
SUPABASE_KEY = 'your-supabase-anon-key'
```

#### Or Update at Runtime
POST request to `/update_mobile_ip`:
```json
{
  "mobile_ip": "192.168.x.x"
}
```

### 5. Launch ATLAS
```bash
python app.py
```

The application will:
1. Load settings from Supabase
2. Initialize AI models (MiDaS, DeepFace)
3. Connect to mobile video stream (with fallback to local webcam)
4. Launch Mission Control at `http://127.0.0.1:5000`

---

## 📡 API Endpoints

### Video & Stream
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/video_feed?mode=rgb\|depth\|nav\|both` | GET | Real-time video stream (MJPEG) |
| `/` | GET | Mission Control dashboard |

### AI Data & Navigation
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/ai/data` | GET | Current AI detection data (faces, obstacles, depth) |
| `/ai/navigation` | GET | Real-time navigation/obstacle analysis |
| `/ai/set_heading` | POST | Update rover heading (compass integration) |

### Target Management
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/upload_target` | POST | Upload facial recognition target image |
| `/list_targets` | GET | List all stored target faces |
| `/delete_target/<filename>` | DELETE | Remove target from database |
| `/targets/<filename>` | GET | Serve target image |

### Configuration
| Endpoint | Method | Description |
|----------|--------|-------------|
| `/update_mobile_ip` | POST | Update mobile camera stream IP |
| `/set_rotation` | POST | Set video rotation (0°/90°/180°/270°) |

---

## 🎮 Usage Guide

### 1. Mission Control Dashboard
- **Live Feed**: Central video stream with real-time overlays
- **Navigation HUD**: Current heading, suggested path, obstacle warnings
- **Target Panel**: Upload and manage facial recognition targets
- **Settings Panel**: Configure mobile IP, rotation, and stream parameters

### 2. Autonomous Navigation
1. Place rover in environment
2. System automatically:
   - Analyzes depth map for obstacles
   - Calculates safest path (5-direction analysis)
   - Suggests heading adjustment
   - Displays confidence score

### 3. Target Recognition
1. Upload target face image via Mission Control
2. System continuously monitors feed
3. On detection:
   - Face highlighted with bounding box
   - Alert activated (8-second persistence)
   - Target name displayed
   - Rover can auto-engage weapon

### 4. Video Rotation
- Adjust rotation via `/set_rotation` endpoint
- Supports: 0°, 90°, 180°, 270°
- Useful for different camera mount angles

---

## 📊 Data Structures

### AI Data Response
```json
{
  "objects": [],
  "faces": [
    {
      "name": "target_name",
      "confidence": 95
    }
  ],
  "alert": {
    "active": true,
    "target": "target_name"
  },
  "navigation": {
    "path_clear": true,
    "path_blocked": false,
    "best_direction": "FORWARD",
    "best_score": 85,
    "path_scores": [70, 95, 85, 90, 75],
    "obstacles": [
      {
        "direction": "LEFT",
        "severity": "LOW",
        "percentage": 25
      }
    ],
    "heading_adjustment": 0,
    "suggested_heading": 45,
    "current_heading": 45
  }
}
```

### Target Response
```json
{
  "targets": [
    {
      "name": "John Doe",
      "filename": "JohnDoe.jpg",
      "url": "/targets/JohnDoe.jpg"
    }
  ]
}
```

---

## ⚙️ Performance Optimization

### Threading Model
- **update_frames**: Continuously reads from video stream (MJPEG or webcam)
- **process_depth**: MiDaS inference & obstacle detection
- **process_faces**: DeepFace recognition on downsampled frames (0.5x)
- **Main Thread**: Flask server for API requests

### Model Optimization
- **MiDaS Small**: 224x224 input resolution (faster than full model)
- **Face Detection**: 0.5x downsampled frames (reduces computation by 4x)
- **CUDA Support**: Automatic GPU acceleration if available
- **Frame Caching**: Thread-safe shared memory model

### Target Detection
- Downsample frames by 50% before face recognition
- Cache DeepFace pickle files for fast indexing
- Automatic pickle regeneration on target updates

---

## 🛠️ Troubleshooting

### Issue: Video Stream Not Connecting
**Solution**: 
1. Check mobile IP address and port 8888 accessibility
2. Ensure mobile app is running and streaming
3. System falls back to local webcam automatically

### Issue: Face Recognition Not Working
**Solution**:
1. Verify target images are in `targets/` folder
2. Check image format (PNG, JPG, WebP supported)
3. Delete `.pkl` files in `targets/` to force re-indexing
4. Ensure face is clearly visible in target image

### Issue: Slow Navigation Response
**Solution**:
1. Enable GPU acceleration (CUDA)
2. Reduce frame resolution via settings
3. Adjust MiDaS inference resolution in code
4. Check system CPU/memory usage

### Issue: ESP32-CAM Boot Loop
**Solution**:
- **DO NOT use GPIO 16** (PSRAM conflict)
- Weapon trigger moved to **GPIO 4**
- See [CONNECTIONS.md](CONNECTIONS.md) for updated pinout

---

## 📦 Project Structure

```
ATLAS_app_1/
├── README.md                 # This file
├── CONNECTIONS.md            # ESP32-CAM wiring guide
├── app.py                    # Flask backend + AI processing
├── index.html                # Mission Control dashboard
├── supabase_schema.sql       # Database schema
├── targets/                  # Facial recognition database
│   └── [target_faces.jpg]
└── ATLAS_FIRMWARE/           # ESP32-CAM firmware
    └── [arduino sketches]
```

---

## 🔐 Security Notes

⚠️ **Important**: This is a research/educational project. Before production deployment:

1. **Rotate Supabase Credentials**: Never commit API keys
2. **Enable HTTPS**: Use proper SSL certificates
3. **Authenticate Endpoints**: Add API key/JWT validation
4. **Sanitize File Uploads**: Validate all image inputs
5. **Rate Limiting**: Implement request throttling
6. **Access Control**: Restrict to trusted networks

---

## 🎓 Learning Resources

### Computer Vision
- **OpenCV**: https://docs.opencv.org/
- **PyTorch**: https://pytorch.org/docs/
- **MiDaS Depth**: https://github.com/isl-org/MiDaS

### Facial Recognition
- **DeepFace**: https://github.com/serengp/deepface
- **Face Detection**: https://docs.opencv.org/master/d7/d8b/tutorial_py_face_detection_in_an_image.html

### ESP32-CAM & Robotics
- **Official Docs**: https://github.com/espressif/arduino-esp32
- **L298N Motor Driver**: Component datasheet
- **Robotics Best Practices**: Industry standards for autonomous systems

### Web Technologies
- **Flask**: https://flask.palletsprojects.com/
- **Supabase**: https://supabase.com/docs

---

## 📝 License

This project is provided as-is for educational and research purposes.

---

## 🤝 Contributors & Credits

**Project Lead**: K. Sridatta  

**Related Project Posts**:
- 🤖 [Robotics, AI & Computer Vision Integration](https://www.linkedin.com/posts/ksridatta_robotics-ai-computervision-activity-7439146997367906304-qVLO?utm_source=share&utm_medium=member_desktop&rcm=ACoAAE0Vj1wBU7XTIOR-iSf-SIF2RDrUgycpylQ)
- 📺 [Project Expo Presentation & Full System Demo](https://www.linkedin.com/posts/ksridatta_gratitude-projectexpo-learningexperience-activity-7439270633449951232-Q2lT?utm_source=share&utm_medium=member_desktop&rcm=ACoAAE0Vj1wBU7XTIOR-iSf-SIF2RDrUgycpylQ)

**Technologies Used**:
- Python 3, Flask, PyTorch, OpenCV, DeepFace
- HTML5, CSS3, JavaScript
- Supabase, ESP32-CAM, Arduino
- L298N Motor Driver, MOSFET Trigger
- Robotics & Autonomous Systems

---

## 📞 Support & Feedback

For issues, feature requests, or contributions:
1. Open a GitHub issue with detailed description
2. Include logs and system information
3. Attach relevant screenshots/videos from your implementation

---

## 🎯 Roadmap

- [ ] Advanced SLAM integration (Simultaneous Localization and Mapping)
- [ ] Multi-target tracking (simultaneous target engagement)
- [ ] Voice command integration & speech recognition
- [ ] Real-time mission logging & replay functionality
- [ ] Mobile app companion (iOS/Android native apps)
- [ ] Telemetry dashboard with historical data analytics
- [ ] Autonomous patrol mode with predefined routes
- [ ] Machine learning model optimization for edge devices
- [ ] ROS (Robot Operating System) integration
- [ ] Computer vision model pruning & quantization

---

**Last Updated**: June 2026  
**Status**: Active Development  
**Project Type**: Autonomous Robotics + AI Vision System  
🚀 **Next Generation of Autonomous Systems**

---

## 🔬 Research & Development

This project integrates cutting-edge technologies in:
- **Computer Vision**: Real-time object/face detection & tracking
- **Deep Learning**: AI models for autonomous decision-making
- **Robotics**: Hardware control & sensor integration
- **IoT**: ESP32 microcontroller programming
- **Full-Stack Development**: Backend API design & frontend UI

Perfect for students and developers exploring robotics, AI, and autonomous systems!
