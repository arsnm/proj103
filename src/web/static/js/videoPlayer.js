document.addEventListener('DOMContentLoaded', function() {
    const video = document.getElementById('videoPlayer');
    const status = document.getElementById('status');
    const videoSrc = '/static/stream.m3u8';

    if (Hls.isSupported()) {
        const hls = new Hls({
            debug: false,
            enableWorker: true,
            lowLatencyMode: true,
            backBufferLength: 90
        });
        
        hls.loadSource(videoSrc);
        hls.attachMedia(video);
        
        hls.on(Hls.Events.MANIFEST_PARSED, function() {
            status.textContent = 'Stream connected';
            video.play().catch(function(error) {
                console.log("Video autoplay failed:", error);
            });
        });

        hls.on(Hls.Events.ERROR, function(event, data) {
            if (data.fatal) {
                status.textContent = 'Stream connection error. Retrying...';
                switch(data.type) {
                    case Hls.ErrorTypes.NETWORK_ERROR:
                        hls.startLoad();
                        break;
                    case Hls.ErrorTypes.MEDIA_ERROR:
                        hls.recoverMediaError();
                        break;
                    default:
                        hls.destroy();
                        break;
                }
            }
        });
    }
    // For browsers with native HLS support (Safari)
    else if (video.canPlayType('application/vnd.apple.mpegurl')) {
        video.src = videoSrc;
        video.addEventListener('loadedmetadata', function() {
            status.textContent = 'Stream connected';
            video.play().catch(function(error) {
                console.log("Video autoplay failed:", error);
            });
        });
    }
    else {
        status.textContent = 'HLS streaming not supported in this browser';
    }
});
