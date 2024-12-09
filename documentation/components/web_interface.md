# Web Interface Documentation

## Components

### Grid View
- 2D visualization of robot position
- Interactive target setting
- Flag visualization
- Real-time updates


### Control Panel
- Mode selection (Manual/Automatic)
- Manual control interface
- Speed control
- Status display


### Video Feed
- Real-time camera feed
- Position overlay
- Flag detection visualization


## Race status

- Select team id
- Race status
- Grid informations (whole size, case size)
- Position of teams in grid
- Detected markers


## JavaScript Architecture

```javascript
// Core classes
class WebSocketConnection
class RobotState
class GridView
class VideoFeed
class RobotControls
