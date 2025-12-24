# Visual Servoing CNC Tracker

A **vision-based control system** that uses a single USB camera to guide a 2-axis CNC machine (CoreXY / H-Bot).  
The machine automatically tracks an object, keeps it centered in the camera frame, and maintains a fixed physical distance using **PD (Proportional–Derivative) control**.

This project focuses on **robustness, safety, and explainability**, rather than raw speed or black-box AI.

---

## 🚀 Features

- **Object Tracking**
    - Uses OpenCV's **CSRT tracker** for stable object locking
- **Monocular Depth Control**
    - Maintains distance using bounding-box size (no LiDAR or stereo camera)
- **PD Controller**
    - Proportional term for responsiveness
    - Derivative term for damping and oscillation suppression
- **CoreXY / H-Bot Kinematics**
    - Correct diagonal motor mixing using vector-based control
- **Soft Safety Limits**
    - Prevents collisions with physical frame boundaries
    - Supports "wall-sliding" instead of full stop
- **Auto-Homing**
    - Sets home position on startup (`G92`)
    - Returns to home on exit (`G0 X0 Y0`)
- **Noise Filtering**
    - Moving-average smoothing (temporal filtering)
    - Minimum-size threshold to ignore background noise
- **Live Control**
    - Keyboard shortcuts for re-locking and re-centering
- **GRBL Compatible**
    - Sends standard G-code over USB serial

---

## 🛠️ Hardware Requirements

1. **CNC Machine**
     - 2-Axis setup (CoreXY, H-Bot, or Cartesian)
     - Running **GRBL firmware**
2. **Controller**
     - Arduino UNO
     - CNC Shield (v3)
     - A4988 / DRV8825 stepper drivers
3. **Camera**
     - USB webcam mounted on the moving carriage
4. **Computer**
     - Python 3.x
     - Windows / Linux

---

## 📦 Software Dependencies

Install the required Python packages:

```bash
pip install opencv-python opencv-contrib-python pyserial
```

> **Note:**
> `opencv-contrib-python` is required for the CSRT tracker.

---

## ⚙️ Configuration

Open `main.py` and adjust the **CONFIGURATION** section at the top:

### Hardware

- `SERIAL_PORT` – COM port of the Arduino (e.g., `COM3` or `/dev/ttyUSB0`)
- `CAMERA_INDEX` – Camera device index (`0` or `1`)

### Safety

- `MAX_TRAVEL_LIMIT` – Maximum allowed distance from center (soft limit)
- `MAX_SINGLE_STEP` – Maximum motion per control loop
- `MIN_TRACKING_SIZE` – Minimum bounding-box size before tracking is ignored

### Control (PD Gains)

- `KP_X`, `KD_X` – Lateral control gains
- `KP_DEPTH`, `KD_DEPTH` – Depth (distance) control gains
- `DEAD_ZONE_X`, `DEAD_ZONE_DEPTH` – Error tolerance zones
- `INVERT_X`, `INVERT_Y` – Axis direction correction flags

---

## 🎮 Usage Guide

### 1️⃣ Power & Setup

1. Power the CNC machine
2. Connect Arduino via USB
3. Physically move the machine to the **center of its workspace**

### 2️⃣ Run the Program

```bash
python main.py
```

### 3️⃣ Select Target

- A camera window opens
- Draw a bounding box around the object
- Press **SPACE** or **ENTER** to lock

### 4️⃣ Tracking Behavior

- The CNC automatically:
    - Centers the object in the camera frame
    - Maintains the initial distance
- On-screen overlays show:
    - Machine position
    - Controller state
    - Safety warnings

---

## ⌨️ Keyboard Controls

| Key | Action                           |
| --- | -------------------------------- |
| `r` | Reset target distance to current |
| `x` | Re-select object (new ROI)       |
| `q` | Quit and return to Home (0,0)    |

---

## 🔧 Tuning the PD Controller

If motion feels unstable or sluggish, tune the gains:

| Parameter             | Effect                                    |
| --------------------- | ----------------------------------------- |
| **KP (Proportional)** | Reaction speed. Too high → overshoot      |
| **KD (Derivative)**   | Damping. Too high → sluggish              |
| **DEAD_ZONE**         | Sensitivity band. Increase to stop jitter |

> **Note:**
> Integral control (I-term) is intentionally omitted to prevent wind-up in a bounded workspace.

---

## ⚠️ Safety Notes

- The program sends `G92 X0 Y0` on startup
    **Always start with the machine at the physical center**
- `MAX_TRAVEL_LIMIT` is a **software (soft) limit**
- Physical limit switches are still recommended
- The derivative term assumes approximately constant loop timing (~50 Hz)

---

## 🧠 System Architecture (Conceptual)

```
Camera
    ↓
Object Tracker (CSRT)
    ↓
Error Calculation (X + Depth)
    ↓
PD Controller
    ↓
CoreXY Kinematics
    ↓
GRBL (G-code)
    ↓
Stepper Motors
```

---

## 📌 Known Limitations

- Depth estimation is **relative**, not metric
- Tracking quality depends on lighting and object texture
- Sudden occlusions may temporarily disrupt control
- Soft limits do not replace hardware safety mechanisms

---

## 📈 Project Status

**Version:** v1.0 – Final
**State:** Stable, demo-ready, and report-ready

---

## 📄 License

This project is intended for **academic and educational use**.
Feel free to adapt and extend with proper attribution.

