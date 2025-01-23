document.addEventListener('DOMContentLoaded', () => {
    // Initialize WebSocket connection
    const wsConnection = new WebSocketConnection('ws://localhost:8765');

    // Initialize robot state
    const robotState = new RobotState();

    const videoPlayer = new VideoFeed(
        document.getElementById('videoPlayer')
    );

    const controls = new RobotControls(wsConnection);

    // Setup WebSocket message handlers
    wsConnection.on('robot_status', (data) => {
        robotState.update(data);
        updatePositionDisplay(data.pose);
    });

    function updatePositionDisplay(pose) {
        document.getElementById('posX').textContent = pose.x.toFixed(2);
        document.getElementById('posY').textContent = pose.y.toFixed(2);
        document.getElementById('posTheta').textContent = (pose.theta * 180 / Math.PI).toFixed(1);
    }

    function updateSpeedDisplay(speed) {
        document.getElementById('speed').textContent = `${speed.left.toFixed(2)}, ${speed.right.toFixed(2)}`
    }
});
