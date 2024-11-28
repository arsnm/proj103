// static/js/videoFeed.js
class VideoFeed {
    constructor(imageElement) {
        this.imageElement = imageElement;
    }

    updateFrame(frameData) {
        this.imageElement.src = `data:image/jpeg;base64,${frameData}`;
    }
}
