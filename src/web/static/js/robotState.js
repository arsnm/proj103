class RobotState {
    constructor() {
        this.position = { x: 0, y: 0, theta: 0};
        this.speed = {left: 0, right: 0}
        this.id = null;
        this.mode = 'manual';
        this.visibleFlags = [];
    }

    update(data) {
        Object.assign(this.position, data.pose);
        if (data.mode) this.mode = data.mode;
        if (data.visible_flags) this.visibleFlags = data.visible_flags;
        if (data.id) this.id = data.id;
    }

    setTarget(target) {
        this.target = target;
    }
}
