<template>
  <div id="app">
    <header>
      <h1>Robot Interface</h1>
      <h2>{{ subtitle }}</h2>
    </header>

    <status-panel :status="robotStatus" />

    <div class="main-content">
      <video-feed :streamUrl="videoStreamUrl"/>

      <div class="panel controls-panel">
        <div class="mode-toggle">
          <button
            v-for="mode in modes"
            :key="mode.name"
            @click="setLocalMode(mode.name)"
            :class="{ active: currentMode === mode.name }"
          >
            {{ mode.label }}
          </button>
        </div>

        <div v-if="currentMode === 'manual'">
          <manual-controls 
            :speed="speed"
            @update-speed="updateSpeed"
            @send-command="sendCommand"
            @set-mode="setMode"
          />
        </div>

        <div v-else-if="currentMode === 'automatic'">
          <automatic-controls 
            @set-mode="setMode"
          />
        </div>

        <div v-else-if="currentMode === 'target'">
          <target-controls
            @set-mode="setMode"
          />
        </div>

        <div v-else-if="currentMode === 'group'">
          <group-controls
            @set-mode="setMode"
          />
        </div>
      </div>
    </div>

    <div class="panel center">
      <button id="disconnectButton" @click="disconnect">Disconnect</button>
    </div>


  </div>
</template>

<script>
import { ref, reactive, computed } from 'vue';
import StatusPanel from './components/StatusPanel.vue';
import VideoFeed from './components/VideoFeed.vue';
import ManualControls from './components/ManualControls.vue';
import AutomaticControls from './components/AutomaticControls.vue';
import TargetControls from './components/TargetControls.vue';
import GroupControls from './components/GroupControls.vue';

export default {
  name: 'RobotInterface',
  components: {
    StatusPanel,
    VideoFeed,
    ManualControls,
    AutomaticControls,
    TargetControls,
    GroupControls,
  },
  setup() {
    // Reactive state
    const socket = ref(null);
    const connected = ref(false);
    const robotId = ref(null);
    const currentMode = ref('manual');
    const speed = ref(50);
    const robotStatus = reactive({ position: [0, 0, 0], speed: 0, mode: 'manual'});
    const videoStreamUrl = "http://localhost:8080/video";
    const websocketUrl = "ws://localhost:8765";

    if (!videoStreamUrl) {
      console.log(import.meta.env.VITE_WEBSOCKET_URL);
      console.error("VITE_VIDEOSTREAM_URL is not defined in .env file");
      return;
    };

    const subtitle = computed(() => {
      if (connected.value) {
        return `Connected - Robot: ${robotId.value}`;
      } else {
        return 'Disconnected';
      }
    });

    const modes = [
      { name: 'manual', label: 'Manual' },
      { name: 'automatic', label: 'Automatic' },
      { name: 'target', label: 'Target' },
      { name: 'group', label: 'Group' },
    ];

    const connectToSocket = () => {
      if (!websocketUrl) {
        console.error('WEBSOCKET_URL is not defined in .env file');
        return;
      }

      socket.value = new WebSocket(websocketUrl);

      socket.value.onopen = () => {
        console.log('Connected to WebSocket');
        connected.value = true;
      };

      socket.value.onevent = (event) => {
        const message = JSON.parse(event.data);
        handleServerMessage(message)
      };

      socket.value.onclose = () => {
        connected.value = false;
        robotStatus.value = 'Robot is offline';
        console.log('Disconnected from WebSocket');
        connected.value = false
      };

      socket.value.onerror = (error) => {
        console.error('WebSocket error: ', error);
        connected.value = false
      };
    };

    function handleServerMessage(message) {
      switch (message.type) {
        case 'connected':
          connnected.value = true;
          robotId.value = message.data.id;
          break;
        case 'robot_status':
          Object.assign(robotStatus, message.data);
          break;
        default:
          console.warn('Unknown message type:', message.type);
          break;
      }
    }

    const setLocalMode = (mode) => {
      currentMode.value = mode;
    }

    const setMode = (mode, modeData) => {
      currentMode.value = mode;
      if (connected.value) {
        socket.value.send(
          JSON.stringify({ type: 'change_mode', data: { mode: mode, ...modeData} })
        );
      } else {
        console.log("Websocket is not connected.");
        return;
      }
    };

    const updateSpeed = (newSpeed) => {
      speed.value = newSpeed;
    };

    const sendCommand = (direction) => {
      if (connected.value && currentMode.value === "manual") {
        socket.value.send(
          JSON.stringify({ type: 'manual_control', data: { direction: direction, speed: speed.value } })
        );
      } else {
        console.log("Websocket is not connected");
        return;
      }
    };

    const sendTarget = (targetData) => {
      if (currentMode.value !== 'target') return;
      if (connected.value) {
      socket.value.send(
        JSON.stringify({ type: 'mode_change', data: { mode: 'target', ...targetData } })
      );
      } else {
        console.log("Websocket is not connected");
        return;
      }
    };

    const disconnect = () => {
      if (connected.value) {
        socket.value.send(
          JSON.stringify({ type: 'disconnect', data: { client_type: 'robot' } })
        );
        socket.value.close();
      } else {
        console.log("Websocket is already disconnected")
        return
      }
    };

    return {
      socket,
      subtitle,
      videoStreamUrl,
      connected,
      robotId,
      currentMode,
      modes,
      speed,
      robotStatus,
      setLocalMode,
      setMode,
      updateSpeed,
      sendCommand,
    };
  },
};
</script>

<style>
@import './styles/styles.css';
</style>
