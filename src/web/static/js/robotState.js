class RobotState {
    constructor() {
        this.position = { x: 0, y: 0, theta: 0, confidence: 0 };
        this.mode = 'manual';
        this.target = null;
        this.visibleFlags = [];
        this.observers = [];
    }

    update(data) {
        Object.assign(this.position, data.pose);
        if (data.mode) this.mode = data.mode;
        if (data.visible_flags) this.visibleFlags = data.visible_flags;
        this.notifyObservers();
    }

    setTarget(target) {
        this.target = target;
        this.notifyObservers();
    }

    addObserver(observer) {
        this.observers.push(observer);
    }

    notifyObservers() {
        this.observers.forEach(observer => observer(this));
    }
}
