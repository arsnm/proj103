# Projet ARTEFACT (proj103) - Robot Control System
> educative project at Telecom Paris

A complete robot control system with vision-based localization, autonomous navigation, and web interface control.

![System Overview](documentation/images/architecture.png)

## Features

- Real-time robot control and monitoring via web interface
- Vision-based localization using ArUco markers
- Dual-mode operation (Manual/Automatic)
- Two-level cascaded control system
- Real-time video streaming
- Position tracking and visualization
- Flag detection and capture point system

## System Requirements

- Python 3.8+
- OpenCV 4.7+
- Raspberry Pi 2B or higher
- Web browser with WebSocket support
- I2C-enabled motor controller card

## Quick Start

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Start the server:
```bash
python server/server.py
```

3. Start the robot control system:
```bash
python robot_control/main.py
```

4. Access the web interface at: http://localhost:8000

## Documentation

- [Installation Guide](documentation/INSTALLATION.md)
- [System Architecture](documentation/ARCHITECTURE.md)
- [WebSocket API](documentation/API.md)
- [Component Documentation](documentation/components/)

## Project Structure

```
robot_control/
├── main.py              # Main robot control system
├── config.py            # Configuration parameters
├── controllers/         # Control system components
├── vision/              # Vision and localization
├── models/              # Data models
├── communication/       # WebSocket communication
├── server/              # Websocket and HTTP server - Web Interface
├── utils/               # Utility functions
└── library_motor/       # Motor controller library
```

## License

MIT License. See [LICENSE](LICENSE) for details.
```
