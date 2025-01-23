class RobotControls {
    constructor(websocket) {
        this.websocket = websocket;
        this.mode = 'manual';
        this.speed = 50;
        this.pressedKeys = new Set();
        this.keyMap = {
            'w': 'forward',
            'a': 'left',
            's': 'backward',
            'd': 'right'
        };
        this.setupControls();

    }

    setupKeyboardControls() {
        document.addEventListener('keydown', (e) => this.handleKeyPress(e, true));
        document.addEventListener('keyup', (e) => this.handleKeyPress(e, false));
    }


    setupControls() {
        // Mode controls
        document.getElementById('manualModeBtn').onclick = () => this.setMode('manual');
        document.getElementById('autoModeBtn').onclick = () => this.setMode('automatic');
        document.getElementById('targetModeBtn').onclick = () => this.setMode('target');
        document.getElementById('groupModeBtn').onclick = () => this.setMode('group');

        // Speed control
        const speedSlider = document.getElementById('speedSlider');
        speedSlider.oninput = (e) => {
            this.speed = e.target.value;
            document.getElementById('speedValue').textContent = e.target.value;
        };

        // Direction controls
        const controls = {
            'forwardBtn': "forward",
            'backBtn': "backward",
            'leftBtn': "left",
            'rightBtn': "right",
            'stopBtn': "stop"
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

    handleMouseDown(direction) {
        if (this.currentMode === 'manual') {
            this.isButtonPressed = true;
            this.sendControl(direction);
        }
    }

    handleMouseUp() {
        if (this.isButtonPressed && this.currentMode === 'manual') {
            this.isButtonPressed = false;
            this.sendControl('stop');
        }
    }

    handleMouseLeave() {
        if (this.isButtonPressed && this.currentMode === 'manual') {
            this.isButtonPressed = false;
            this.sendControl('stop');
        }
    }

    handleKeyPress(event, isKeyDown) {
        const direction = this.keyMap[event.key.toLowerCase()];
        if (!direction || this.currentMode !== 'manual') return;

        if (isKeyDown) {
            if (!this.pressedKeys.has(event.key)) {
                this.pressedKeys.add(event.key);
                this.sendControl(direction);
            }
        } else {
            this.pressedKeys.delete(event.key);
            if (this.pressedKeys.size === 0) {
                this.sendControl('stop');
            } else {
                const lastKey = Array.from(this.pressedKeys)[this.pressedKeys.size - 1];
                this.sendControl(this.keyMap[lastKey]);
            }
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
        if (this.currentMode !== 'target') return;

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
            console.log("Invalid input for target")
            return;
        }

        this.websocket.send('mode_change', {
                id : this.id,
                command: 'target',
                ...targetData
            });
    }

    setMode(mode) {
        this.mode = mode;
        this.websocket.send('mode_change', { mode: mode });

        document.getElementById('manualModeBtn').classList.toggle('active', mode === 'manual');
        document.getElementById('autoModeBtn').classList.toggle('active', mode === 'automatic');
        document.getElementById('targetModeBtn').classList.toogle('active', mode === 'target'); document.getElementById('groupModeBtn').classList.toogle('active', mode === 'group')
        document.querySelector('.robot-mode').textContent = `Mode: ${mode}`;
    }

    sendControl(movement) {
        if (this.mode !== 'manual') return;

        this.websocket.send('manual_control', {
            movement : movement
            speed: this.speed
        });
    }
}
