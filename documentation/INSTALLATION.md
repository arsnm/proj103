# Installation Guide

This guide will help you set up the robot control system on your hardware.

## Prerequisites

### Hardware Requirements
- Raspberry Pi 4B or higher
- USB Camera
- I2C Motor Controller Card
- Two DC Motors with encoders
- Power supply
- ArUco markers (printed)



### Software Requirements
- Python 3.8+
- OpenCV 4.7+
- Web browser (Chrome/Firefox recommended)



## Step-by-Step Installation

1. **System Setup**

```bash
# Update system
sudo apt-get update
sudo apt-get upgrade

# Install required system packages
sudo apt-get install -y \
    python3-pip \
    python3-opencv \
    python3-smbus \
    i2c-tools

# Enable I2C
sudo raspi-config
# Select: Interface Options -> I2C -> Yes
```

2. **Python Dependencies**

```bash
# Create virtual environment
python -m venv .venv
source .venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

3. **Camera Calibration**

```bash
# Run calibration script
python src/vision/camera_calibration.py
```

4. **ArUco Marker Setup**
- Print the provided ArUco markers
- Corner markers (10x10cm): IDs 1-4
- Flag markers (2x2cm): IDs 5-16

5. **Server Setup**

```bash
# Start WebSocket/HTTP server
python src/server/server.py
```

6. **Robot Control System**

```bash
# Start robot control
python src/main.py
```



## Configuration

Key configuration files:
- `src/config.py`: System parameters
- `src/vision/aruco_detector.py`: Marker configurations
- `src/controllers/position_controller.py`: Control parameters


## Troubleshooting

### Debug Mode

Enable debug mode in config.py:
```python
DEBUG = True
TEST_MODE = True  # For testing without hardware
```

## Updates

Keep your system updated:
```bash
git pull origin main
pip install -r requirements.txt --upgrade
```

## Support

For issues and questions:
- Create an issue on GitLab
- Check existing documentation
- Contact system administrators
