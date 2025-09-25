# Task-4-Build-Your-Own-Mobile-Robot
(ROS2 Integrated)

## 📁 File Structure
```markdown
📁 /DesignFiles
│   ├── Project Robotic Concept Design.pdf         # Complete design in PDF format
│   └── TB3_FullAssembly.step      # Optional STEP file for 3D viewing or editing
└── README.md
```
------
### 🤖 Overview

This repository contains the CAD design and documentation for a custom-built ROS2-integrated mobile robot platform, modeled in SolidWorks 2024. The robot is designed for:

~ Autonomous navigation
~ Mapping (SLAM)
~ Object detection

It features a modular multi-tier architecture to accommodate sensors, computing units, motor drivers, and power systems—ideal for research, prototyping, and education.

### 🔧 Features

~ 🧱 Modular Frame – Stackable plate design for easy component organization and expansion
~ 🧠 Compute Ready – Supports SBCs (e.g., Raspberry Pi, Jetson Nano) or microcontrollers
~ 🔋 Power Management – Space for onboard battery and power distribution

### 🎯 Sensor Suite:

~ 🔲 2D LiDAR – For SLAM and obstacle detection
~ 🎥 Camera Module – Front-mounted for vision-based tasks

### ⚙️ Drive System:

~ ⚙️ Differential Drive – Using geared DC motors
~ 🛞 Caster Wheel – For passive balancing

### 🛠️ Tools Used

~ SolidWorks 2024 – For mechanical CAD modeling
~ Fusion 360 / Cura (optional) – For 3D printing components
~ PlatformIO / Arduino IDE – For firmware development
~ ROS2 – For real-time robot control and navigation

### 📷 Components Breakdown

~ 🔲 LiDAR Sensor – Mounted on the top tier for 360° scanning and SLAM.
~ 🎥 Camera Module – Positioned front-facing for vision-based tasks such as object detection.
~ 🧠 PCB / Controller – Installed in the middle layers; handles computing and I/O operations.
~ ⚙️ DC Motors – Attached to the rear wheels for differential drive control.
~ 🛞 Caster Wheel – Located at the front for passive balancing and support.
~ 🔌 Power Module – Houses the battery and manages power distribution to all components.

