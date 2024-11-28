// static/js/gridView.js
class GridView {
    constructor(canvas, robotState) {
        this.canvas = canvas;
        this.ctx = canvas.getContext('2d');
        this.robotState = robotState;
        this.gridSize = { width: 2, height: 2 }; // 2m x 2m
        this.pixelsPerMeter = 0;
        this.robotIcon = new Image();
        this.robotIcon.src = 'static/img/robot-icon.svg';
        
        this.setupCanvas();
        this.robotState.addObserver(() => this.draw());
        this.setupInteraction();
    }

    setupCanvas() {
        // Make canvas full size of container with proper resolution
        const resizeObserver = new ResizeObserver(entries => {
            for (const entry of entries) {
                const width = entry.contentRect.width;
                this.canvas.width = width;
                this.canvas.height = width;
                this.pixelsPerMeter = width / this.gridSize.width;
                this.draw();
            }
        });
        resizeObserver.observe(this.canvas);
    }

    setupInteraction() {
        this.canvas.addEventListener('click', (event) => {
            if (this.robotState.mode !== 'automatic') return;
            
            const rect = this.canvas.getBoundingClientRect();
            const x = (event.clientX - rect.left) / this.pixelsPerMeter;
            const y = (event.clientY - rect.top) / this.pixelsPerMeter;
            
            this.robotState.setTarget({ x, y, theta: 0 });
        });
    }

    draw() {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        this.drawGrid();
        this.drawTarget();
        this.drawRobot();
        this.drawFlags();
    }

    drawGrid() {
        // Draw grid lines
        this.ctx.strokeStyle = '#ddd';
        this.ctx.lineWidth = 1;

        for (let i = 0; i <= this.gridSize.width; i++) {
            const x = i * this.pixelsPerMeter;
            this.ctx.beginPath();
            this.ctx.moveTo(x, 0);
            this.ctx.lineTo(x, this.canvas.height);
            this.ctx.stroke();
        }

        for (let i = 0; i <= this.gridSize.height; i++) {
            const y = i * this.pixelsPerMeter;
            this.ctx.beginPath();
            this.ctx.moveTo(0, y);
            this.ctx.lineTo(this.canvas.width, y);
            this.ctx.stroke();
        }
    }

    drawRobot() {
        const { x, y, theta } = this.robotState.position;
        const robotSize = 0.3 * this.pixelsPerMeter; // 30cm robot

        this.ctx.save();
        this.ctx.translate(x * this.pixelsPerMeter, y * this.pixelsPerMeter);
        this.ctx.rotate(theta);
        
        // Draw robot
        if (this.robotIcon.complete) {
            this.ctx.drawImage(this.robotIcon, -robotSize/2, -robotSize/2, robotSize, robotSize);
        } else {
            // Fallback if image not loaded
            this.ctx.fillStyle = '#2196F3';
            this.ctx.beginPath();
            this.ctx.arc(0, 0, robotSize/2, 0, Math.PI * 2);
            this.ctx.fill();
            // Direction indicator
            this.ctx.beginPath();
            this.ctx.moveTo(0, 0);
            this.ctx.lineTo(robotSize/2, 0);
            this.ctx.strokeStyle = 'white';
            this.ctx.stroke();
        }
        
        this.ctx.restore();
    }

    drawTarget() {
        if (!this.robotState.target) return;

        const { x, y } = this.robotState.target;
        this.ctx.fillStyle = '#FF4081';
        this.ctx.beginPath();
        this.ctx.arc(
            x * this.pixelsPerMeter,
            y * this.pixelsPerMeter,
            5,
            0,
            Math.PI * 2
        );
        this.ctx.fill();
    }

    drawFlags() {
        this.robotState.visibleFlags.forEach(flagId => {
            // Draw detected flags
            // Position should come from a flag position mapping
            // This is just a placeholder
            this.ctx.fillStyle = '#4CAF50';
            this.ctx.beginPath();
            this.ctx.arc(
                flagId * 0.5 * this.pixelsPerMeter,
                flagId * 0.5 * this.pixelsPerMeter,
                3,
                0,
                Math.PI * 2
            );
            this.ctx.fill();
        });
    }
}
