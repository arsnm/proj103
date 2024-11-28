// static/js/websocket.js
class WebSocketConnection {
    constructor(url) {
        this.url = url;
        this.ws = null;
        this.reconnectTimeout = 1000; // 1 second
        this.handlers = new Map();
        this.connect();
    }

    connect() {
        this.ws = new WebSocket(this.url);
        
        this.ws.onopen = () => {
            console.log('Connected to robot');
            this.send('client_type', { client_type : 'interface'})
            document.querySelector('.connection-status').textContent = 'Connected';
            document.querySelector('.connection-status').classList.add('connected');
        };

        this.ws.onclose = () => {
            console.log('Disconnected from robot');
            document.querySelector('.connection-status').textContent = 'Disconnected';
            document.querySelector('.connection-status').classList.remove('connected');
            setTimeout(() => this.connect(), this.reconnectTimeout);
        };

        this.ws.onmessage = (event) => {
            try {
                const message = JSON.parse(event.data);
                if (this.handlers.has(message.type)) {
                    this.handlers.get(message.type)(message.data);
                }
            } catch (error) {
                console.error('Error processing message:', error);
            }
        };
    }

    on(messageType, handler) {
        this.handlers.set(messageType, handler);
    }

    send(type, data) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            const message = {
                type: type,
                data: data,
                timestamp: Date.now()
            };
            this.ws.send(JSON.stringify(message));
        }
    }
}
