<template>
  <div class="controller">
    <!-- Slider for Speed -->
    <div class="speed-slider">
      <label for="speed">Speed: {{ localSpeed }}</label>
      <input
        id="speed"
        type="range"
        v-model.number="localSpeed"
        :min="minSpeed"
        :max="maxSpeed"
        @input="updateSpeed"
      />
    </div>

    <!-- Arrow Buttons -->
    <button
      class="btn up"
      @mousedown="handlePress('forward')"
      @mouseup="handleRelease"
      @touchstart.prevent="handlePress('forward')"
      @touchend.prevent="handleRelease"
    >
      ↑
    </button>
    <button
      class="btn left"
      @mousedown="handlePress('left')"
      @mouseup="handleRelease"
      @touchstart.prevent="handlePress('left')"
      @touchend.prevent="handleRelease"
    >
      ←
    </button>
    <button
      class="btn stop"
      @mousedown="handlePress('stop')"
      @mouseup="handleRelease"
      @touchstart.prevent="handlePress('stop')"
      @touchend.prevent="handleRelease"
    >
      Stop
    </button>
    <button
      class="btn right"
      @mousedown="handlePress('right')"
      @mouseup="handleRelease"
      @touchstart.prevent="handlePress('right')"
      @touchend.prevent="handleRelease"
    >
      →
    </button>
    <button
      class="btn down"
      @mousedown="handlePress('backward')"
      @mouseup="handleRelease"
      @touchstart.prevent="handlePress('backward')"
      @touchend.prevent="handleRelease"
    >
      ↓
    </button>
  </div>
</template>

<script>
export default {
  props: {
    speed: {
      type: Number,
      default: 50,
    },
    minSpeed: {
      type: Number,
      default: 0,
    },
    maxSpeed: {
      type: Number,
      default: 100,
    },
  },
  data() {
    return {
      localSpeed: this.speed, // Sync local speed with parent-provided speed
      keyMappings: {
        w: 'forward',
        a: 'left',
        s: 'backward',
        d: 'right',
      },
      pressedKeys: new Set()
    };
  },
  methods: {
    updateSpeed() {
      // Emit the updated speed to the parent component
      this.$emit('update:speed', this.localSpeed);
    },

    debounce(func, delay) {
      let timeoutId;
      return function() {
        const context = this;
        const args = arguments;
        clearTimeout(timeoutId);
        timeoutId = setTimeout(() => func.apply(context, args), delay);
      };
    },

    debouncedHandleRelease: null,

    handlePress(direction) {
      // Emit the command with direction and current speed
      if (direction === 'stop') {
        this.$emit('send-command', direction);
      } else {
        this.$emit('send-command', direction);
      }
    },

    handleRelease() {
      // Emit a stop command when the button is released
      this.$emit('send-command', 'stop');
    },

    handleKeyDown(event) {
      const direction = this.keyMappings[event.key.toLowerCase()];
      // Only handle if the key isn't already pressed
      if (direction && !this.pressedKeys.has(event.key)) {
        this.pressedKeys.add(event.key);
        this.handlePress(direction);
      }
    },

    handleKeyUp(event) {
      const direction = this.keyMappings[event.key.toLowerCase()];
      if (direction) {
        this.pressedKeys.delete(event.key);
        if (this.pressedKeys.size === 0) {
          this.debouncedHandleRelease();
        }
        else if (this.pressedKeys.size === 1) {
          const lastDirection = [...this.pressedKeys][0];
          this.handlePress(lastDirection);
        }
      }
    },
  },

  watch: {
    speed(newSpeed) {
      this.localSpeed = newSpeed;
    },
  },

  mounted() {
    // Add event listeners for keyboard controls
    this.debouncedHandleRelease = this.debounce(this.handleRelease, 200)
    window.addEventListener('keydown', this.handleKeyDown);
    window.addEventListener('keyup', this.handleKeyUp);
  },

  beforeDestroy() {
    // Remove event listeners for keyboard controls
    window.removeEventListener('keydown', this.handleKeyDown);
    window.removeEventListener('keyup', this.handleKeyUp);
  },
};
</script>

<style>
.controller {
  display: grid;
  grid-template-columns: repeat(3, 100px);
  grid-gap: 10px;
  justify-content: center;
  align-items: center;
  margin: 20px;
}
.speed-slider {
  grid-column: 1 / 4;
  text-align: center;
  margin-bottom: 20px;
}
.speed-slider input {
  width: 80%;
}
.btn {
  width: 100px;
  height: 100px;
  font-size: 20px;
  text-align: center;
  border: 1px solid #ccc;
  border-radius: 8px;
  background-color: #f9f9f9;
  cursor: pointer;
}
.btn:active {
  background-color: #ddd;
}
.up {
  grid-column: 2;
  grid-row: 1;
}
.left {
  grid-column: 1;
  grid-row: 2;
}
.stop {
  grid-column: 2;
  grid-row: 2;
  background-color: #ff4d4d;
  color: white;
}
.right {
  grid-column: 3;
  grid-row: 2;
}
.down {
  grid-column: 2;
  grid-row: 3;
}
</style>
