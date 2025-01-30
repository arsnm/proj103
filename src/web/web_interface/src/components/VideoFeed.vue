<template>
  <div class="panel video-feed">
    <video
      ref="videoPlayer"
      id="videoPlayer"
      class="video-js vjs-default-skin"
      controls
      preload="auto"
      width="100%"
      height="auto"
    >
      <source :src="streamUrl" type="application/x-mpegURL" />
    </video>
  </div>
</template>

<script>
import { onMounted, ref } from 'vue';
import videojs from 'video.js';
import 'video.js/dist/video-js.css';

export default {
  name: 'VideoFeed',
  props: {
    streamUrl: {
      type: String,
      required: true,
    },
  },
  setup(props) {
    const videoPlayer = ref(null);

    onMounted(() => {
      const player = videojs(videoPlayer.value, {
        autoplay: true,
        controls: true,
        fluid: true,
        preload: 'auto',
      });

      // You can listen for events from the player if needed
      player.on('ready', () => {
        console.log('Video player is ready!');
      });
    });

    return { videoPlayer };
  },
};
</script>

<style scoped>
.video-feed {
  width: 600px;
  max-width: 50%;
  margin: 0 auto;
}

#videoPlayer {
  width: 100%;
  height: auto;
  align-items: center;
}
</style>
