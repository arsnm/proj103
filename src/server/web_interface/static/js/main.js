document.addEventListener('DOMContentLoaded', () => {
    // Initialize WebSocket connection
    const wsConnection = new WebSocketConnection('ws://localhost:8765');

    // Initialize robot state
    const robotState = new RobotState();

    // Initialize components
    const gridView = new GridView(
        document.getElementById('gridCanvas'),
        robotState
    );

    const videoFeed = new VideoFeed(
        document.getElementById('videoFeed')
    );
    
    const controls = new RobotControls(wsConnection);
    
    // Setup WebSocket message handlers
    wsConnection.on('robot_status', (data) => {
        robotState.update(data);
        updatePositionDisplay(data.pose);
    });
    
    wsConnection.on('video_frame', (data) => {
        videoFeed.updateFrame(data.frame);
    });
    
    wsConnection.on('race_status', (data) => {
        raceState.updateStatus(data);
    })

    function updatePositionDisplay(pose) {
        document.getElementById('posX').textContent = pose.x.toFixed(2);
        document.getElementById('posY').textContent = pose.y.toFixed(2);
        document.getElementById('posTheta').textContent = (pose.theta * 180 / Math.PI).toFixed(1);
    }
});
