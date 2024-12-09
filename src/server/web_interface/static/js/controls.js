class RobotControls {
    constructor(websocket) {
        this.websocket = websocket;
        this.mode = 'manual';
        this.speed = 0.5;
        this.setupControls();
    }

    setupControls() {
        // Mode controls
        document.getElementById('manualModeBtn').onclick = () => this.setMode('manual');
        document.getElementById('autoModeBtn').onclick = () => this.setMode('automatic');

        // Speed control
        const speedSlider = document.getElementById('speedSlider');
        speedSlider.oninput = (e) => {
            this.speed = e.target.value / 100;
            document.getElementById('speedValue').textContent = e.target.value;
        };

        // Direction controls
        const controls = {
            'forwardBtn': { left: 1, right: 1 },
            'backBtn': { left: -1, right: -1 },
            'leftBtn': { left: -0.5, right: 0.5 },
            'rightBtn': { left: 0.5, right: -0.5 },
            'stopBtn': { left: 0, right: 0 }
        };

        Object.entries(controls).forEach(([id, speeds]) => {
            const button = document.getElementById(id);
            button.onmousedown = () => this.sendControl(speeds);
            button.onmouseup = () => this.sendControl({ left: 0, right: 0 });
            button.onmouseleave = () => this.sendControl({ left: 0, right: 0 });
        });
    }

    setMode(mode) {
        this.mode = mode;
        this.websocket.send('mode_change', { mode: mode });
        
        document.getElementById('manualModeBtn').classList.toggle('active', mode === 'manual');
        document.getElementById('autoModeBtn').classList.toggle('active', mode === 'automatic');
        document.querySelector('.robot-mode').textContent = `Mode: ${mode}`;
    }

    sendControl(speeds) {
        if (this.mode !== 'manual') return;
        
        this.websocket.send('manual_control', {
            left: speeds.left * this.speed,
            right: speeds.right * this.speed
        });
    }
}
