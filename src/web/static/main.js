class RobotController {
    constructor() {
        this.socket = new WebSocket(`ws://${window.location.hostname}:8765`);
        this.selectedRobotId = null;
        this.currentMode = 'manual';
        this.pressedKeys = new Set();
        this.keyMap = {
            'w': 'forward',
            'a': 'left',
            's': 'backward',
            'd': 'right'
        };

        this.bindMethods();
        this.setupWebSocket();
        this.setupEventListeners();
        this.updateControlsVisibility('manual');
    }

    bindMethods() {
        const methods = [
            'sendMovement', 'handleKeyPress', 'handleMouseDown', 'handleMouseUp',
            'handleMouseLeave', 'disconnectClient', 'disconnectRobot', 'updateUI',
            'setMode', 'setTargetType', 'sendTarget', 'validateNumberInput',
            'validateCaseInput', 'updateControlsVisibility'
        ];
        methods.forEach(method => this[method] = this[method].bind(this));
    }

    setupWebSocket() {
        this.socket.onopen = () => {
            this.socket.send(JSON.stringify({
                type: 'connect',
                data: { client_type: 'client' }
            }));
        };

        this.socket.onmessage = (event) => {
            const message = JSON.parse(event.data);
            this.handleServerMessage(message);
        };

        window.onbeforeunload = () => {
            this.socket.send(JSON.stringify({
                type: 'disconnect',
                data: { client_type: 'client' }
            }));
        };
    }

    setupEventListeners() {
        document.getElementById('robotIdSelect').onchange = (e) => {
            this.selectedRobotId = e.target.value;
            this.updateUI('robot-selected');
        };

        document.getElementById('speedSlider').oninput = (e) => {
            document.getElementById('speedSliderValue').textContent = e.target.value;
        };

        this.setupControlButtons();
        this.setupKeyboardControls();
        this.setupModeControls();
        this.setupTargetControls();
        this.setupInputValidation();

        document.getElementById('disconnectClient').onclick = this.disconnectClient;
        document.getElementById('disconnectRobot').onclick = this.disconnectRobot;
    }

    setupControlButtons() {
        const controlButtons = {
            'forwardBtn': 'forward',
            'leftBtn': 'left',
            'rightBtn': 'right',
            'backwardBtn': 'backward'
        };

        Object.entries(controlButtons).forEach(([btnId, direction]) => {
            const button = document.getElementById(btnId);
            if (button) {
                ['mousedown', 'touchstart'].forEach(event => {
                    button.addEventListener(event, (e) => {
                        e.preventDefault();
                        this.handleMouseDown(direction);
                    });
                });

                ['mouseup', 'mouseleave', 'touchend', 'touchcancel'].forEach(event => {
                    button.addEventListener(event, (e) => {
                        e.preventDefault();
                        this.handleMouseUp();
                    });
                });
            }
        });
    }

    setupKeyboardControls() {
        document.addEventListener('keydown', (e) => this.handleKeyPress(e, true));
        document.addEventListener('keyup', (e) => this.handleKeyPress(e, false));
    }

    setupModeControls() {
        document.querySelectorAll('.mode-toggle button').forEach(btn => {
            btn.onclick = () => {
                const newMode = btn.textContent.toLowerCase();
                if (this.currentMode !== newMode) {
                    this.setMode(newMode);
                }
            };
        });
    }

    setupTargetControls() {
        document.querySelectorAll('.target-toggle button').forEach(btn => {
            btn.onclick = () => this.setTargetType(btn.textContent.toLowerCase());
        });

        document.getElementById('sendTarget').onclick = this.sendTarget;
    }

    setupInputValidation() {
        const coordX = document.getElementById('coordX');
        const coordY = document.getElementById('coordY');
        const caseInput = document.getElementById('case');

        if (coordX) coordX.addEventListener('input', this.validateNumberInput);
        if (coordY) coordY.addEventListener('input', this.validateNumberInput);
        if (caseInput) caseInput.addEventListener('input', this.validateCaseInput);
    }

    handleServerMessage(message) {
        switch (message.type) {
            case 'connected':
                console.log('Connected with ID:', message.data.id);
                break;
            case 'robot_list':
                this.updateRobotList(message.data.robots);
                break;
            case 'robot_status':
                this.updateRobotStatus(message.data);
                break;
            case 'camera_frame':
                this.updateCameraFrame(message.data);
                break;
            case 'log':
                this.addLog(message.data);
                break;
        }
    }

    handleMouseDown(direction) {
        if (this.currentMode === 'manual') {
            this.isButtonPressed = true;
            this.sendMovement(direction);
        }
    }

    handleMouseUp() {
        if (this.isButtonPressed && this.currentMode === 'manual') {
            this.isButtonPressed = false;
            this.sendMovement('stop');
        }
    }

    handleMouseLeave() {
        if (this.isButtonPressed && this.currentMode === 'manual') {
            this.isButtonPressed = false;
            this.sendMovement('stop');
        }
    }

    handleKeyPress(event, isKeyDown) {
        const direction = this.keyMap[event.key.toLowerCase()];
        if (!direction || this.currentMode !== 'manual') return;

        if (isKeyDown) {
            if (!this.pressedKeys.has(event.key)) {
                this.pressedKeys.add(event.key);
                this.sendMovement(direction);
            }
        } else {
            this.pressedKeys.delete(event.key);
            if (this.pressedKeys.size === 0) {
                this.sendMovement('stop');
            } else {
                const lastKey = Array.from(this.pressedKeys)[this.pressedKeys.size - 1];
                this.sendMovement(this.keyMap[lastKey]);
            }
        }
    }

    updateControlsVisibility(mode) {
        const controlSections = {
            'manual': ['.speed-control', '.control-buttons'],
            'automatic': [],
            'target': ['.target-controls']
        };

        ['speed-control', 'control-buttons', 'target-controls'].forEach(section => {
            const element = document.querySelector(`.${section}`);
            if (element) {
                element.classList.add('hidden');
            }
        });

        controlSections[mode]?.forEach(selector => {
            const element = document.querySelector(selector);
            if (element) {
                element.classList.remove('hidden');
            }
        });
    }

    validateNumberInput(e) {
        e.target.value = e.target.value.replace(/[^\d.-]/g, '');
        const parts = e.target.value.split('.');
        if (parts.length > 2) e.target.value = parts[0] + '.' + parts.slice(1).join('');
    }

    validateCaseInput(e) {
        e.target.value = e.target.value.toUpperCase().replace(/[^A-G1-9]/g, '');
        if (e.target.value.length > 2) {
            e.target.value = e.target.value.slice(0, 2);
        }
    }

    sendMovement(direction) {
        if (!this.selectedRobotId) return;

        this.socket.send(JSON.stringify({
            type: 'command',
            data: {
                robot_id: this.selectedRobotId,
                command: 'move',
                direction: direction,
                speed: document.getElementById('speedSlider').value
            }
        }));
    }

    setMode(mode) {
        this.currentMode = mode;
        document.querySelectorAll('.mode-toggle button').forEach(btn => {
            btn.classList.toggle('active', btn.textContent.toLowerCase() === mode);
        });

        this.updateControlsVisibility(mode);

        if ((mode === 'automatic' || mode === 'manual') && this.selectedRobotId) {
            this.socket.send(JSON.stringify({
                type: 'command',
                data: {
                    robot_id: this.selectedRobotId,
                    command: 'mode',
                    mode_type: mode
                }
            }));
        }
    }

    setTargetType(type) {
        document.querySelectorAll('.target-toggle button').forEach(btn => {
            btn.classList.toggle('active', btn.textContent.toLowerCase() === type);
        });
        document.querySelector('.coordinates-container').classList.toggle('active', type === 'coordinates');
        document.querySelector('.case-container').classList.toggle('active', type === 'case');
    }

    sendTarget() {
        if (!this.selectedRobotId || this.currentMode !== 'target') return;

        const targetType = document.querySelector('.target-toggle .active')?.textContent.toLowerCase();
        if (!targetType) return;

        let targetData = {};
        let isValid = true;

        if (targetType === 'coordinates') {
            const x = parseFloat(document.getElementById('coordX').value);
            const y = parseFloat(document.getElementById('coordY').value);
            if (isNaN(x) || isNaN(y)) {
                isValid = false;
            } else {
                targetData = { "x":x, "y":y };
            }
        } else if (targetType === 'case') {
            const caseValue = document.getElementById('case').value.toUpperCase();
            if (!caseValue || !/^[A-G][1-9]$/.test(caseValue)) {
                isValid = false;
            } else {
                targetData = { case: caseValue };
            }
        }

        if (!isValid) {
            this.addLog({
                timestamp: new Date().toISOString(),
                log_type: 'error',
                message: 'Invalid target input'
            });
            return;
        }

        this.socket.send(JSON.stringify({
            type: 'command',
            data: {
                robot_id: this.selectedRobotId,
                command: 'target',
                ...targetData
            }
        }));
    }

    updateUI(type) {
        if (type === 'robot-selected' && this.selectedRobotId) {
            this.socket.send(JSON.stringify({
                type: 'get_status',
                data: {
                    robot_id: this.selectedRobotId
                }
            }));
        }
    }

    updateRobotList(robots) {
        const dropdown = document.getElementById('robotIdSelect');
        dropdown.innerHTML = '<option value="">--Select a Robot--</option>';
        robots.forEach(id => {
            const option = document.createElement('option');
            option.value = id;
            option.textContent = `Robot ${id}`;
            dropdown.appendChild(option);
        });
    }

    updateRobotStatus(status) {
        if (!status) return;

        const elements = {
            'position': `Position: (${status.position?.[0]?.toFixed(2) ?? 'N/A'}, ${status.position?.[1]?.toFixed(2) ?? 'N/A'}, ${status.position?.[2]?.toFixed(2) ?? 'N/A'})`,
            'speed': `Speed: (${status.speed?.[0] ?? 'N/A'}, ${status.speed?.[1] ?? 'N/A'})`,
            'mode': `Mode: ${status.mode ?? 'N/A'}`,
            'target': `Target: (${status.target?.[0]?.toFixed(2) ?? 'N/A'}, ${status.target?.[1]?.toFixed(2) ?? 'N/A'})`
        };

        Object.entries(elements).forEach(([id, text]) => {
            const element = document.getElementById(id);
            if (element) element.textContent = text;
        });
    }

    updateCameraFrame(data) {
        if (!data.frame) return;

        const videoContainer = document.getElementById('videoContainer');
        let img = videoContainer.querySelector('img');
        if (!img) {
            img = document.createElement('img');
            videoContainer.appendChild(img);
        }
        img.src = `data:image/jpeg;base64,${data.frame.image}`;
    }

    addLog(logData) {
        const logsContainer = document.getElementById('logs');
        const logEntry = document.createElement('p');
        
        const timestamp = new Date(logData.timestamp).toLocaleString();
        const typeClass = {
            'error': 'log-error',
            'command': 'log-command',
            'status': 'log-status',
            'connection': 'log-connection',
            'movement': 'log-movement',
            'robot': 'log-robot'
        }[logData.log_type] || 'log-default';

        logEntry.className = typeClass;
        logEntry.textContent = `${timestamp} - ${logData.log_type} - ${logData.message}`;

        logsContainer.appendChild(logEntry);
        logsContainer.scrollTop = logsContainer.scrollHeight;
    }

    disconnectClient() {
        this.socket.send(JSON.stringify({
            type: 'disconnect',
            data: { client_type: 'client' }
        }));
        this.socket.close();
    }

    disconnectRobot() {
        if (this.selectedRobotId) {
            this.socket.send(JSON.stringify({
                type: 'disconnect',
                data: {
                    client_type: 'robot',
                    robot_id: this.selectedRobotId
                }
            }));
        }
    }
}

document.addEventListener('DOMContentLoaded', () => new RobotController());
