# Vision System Documentation

## Overview

The vision system handles real-time position estimation using ArUco markers:
- Large markers (10x10cm) for robot localization
- Small markers (2x2cm) for capture points


## Components

### ArucoDetector
```python
class ArucoDetector:
    def __init__(self, calibration_file: str, test_mode: bool = False)
    def estimate_position(self, frame) -> Optional[Position]
    def get_visible_flags(self, frame) -> List[int]
```


### Camera Manager
```python
class CameraManager:
    def __init__(self, config: VisionConfig, test_mode: bool = False)
    def read_frame(self) -> Tuple[bool, Optional[np.ndarray]]
```


## Marker Setup

- **Corner Markers (10x10cm)**
  - IDs: 1-4
  - Positions:
    - 4: (0.0, 0.0) - Origin
    - 3: (0.0, 3.0) - X-axis
    - 2: (3.0, 3.0) - Opposite
    - 1: (3.0, 0.0) - Y-axis

- **Flag Markers (2x2cm)**
  - IDs: 5-16
  - Maximum detection range: 70.7cm
