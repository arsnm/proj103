# WebSocket API Documentation

## Connection

- WebSocket URL: `ws://localhost:8765`
- HTTP Interface: `http://localhost:8000`

## Message Format

All messages follow the JSON format:
```json
{
    "type": "message_type",
    "data": {},
    "timestamp": 1234567890.123
}
```

## Message Types

### Robot → Server Messages

1. **Robot Status**
```json
{
    "type": "robot_status",
    "data": {
        "mode": "automatic",
        "pose": {
            "x": 1.23,
            "y": 4.56,
            "theta": 0.789,
            "camera_angle": 0.1,
            "confidence": 0.95
        },
        "visible_flags": [10, 11, 12]
    }
}
```

2. **Video Frame**
```json
{
    "type": "video_frame",
    "data": {
        "frame": "base64_encoded_jpeg"
    }
}
```

### Server → Robot Messages

1. **Mode Change**
```json
{
    "type": "mode_change",
    "data": {
        "mode": "manual"  // or "automatic"
    }
}
```

2. **Manual Control**
```json
{
    "type": "manual_control",
    "data": {
        "left_speed": 0.5,
        "right_speed": 0.7
    }
}
```

3. **Target Position**
```json
{
    "type": "target_position",
    "data": {
        "x": 2.0,
        "y": 1.5,
        "theta": 1.57
    }
}
```

## Error Handling

Error messages follow the format:
```json
{
    "type": "error",
    "data": {
        "code": "ERROR_CODE",
        "message": "Error description"
    }
}
```
