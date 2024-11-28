# Control System Documentation

## Cascaded Control Architecture

### Position Controller
- Update rate: 10Hz
- Input: Vision-based position
- Output: Target velocities
- PID control for position and orientation


### Motor Controller
- Update rate: 50Hz
- Input: Target velocities
- Output: Motor PWM
- Encoder feedback


## Controller Parameters

```python
class ControlConfig:
    # Position Control
    POSITION_RATE: int = 10
    POSITION_Kp: float = 0.5
    POSITION_Ki: float = 0.1
    POSITION_Kd: float = 0.1
    
    # Motor Control
    MOTOR_RATE: int = 50
    MOTOR_Kp: float = 0.8
    MOTOR_Ki: float = 0.3
    MOTOR_Kd: float = 0.1
