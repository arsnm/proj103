<template>
  <div class="start-button">
    <button
      @click="startGroupControl"
    >
      Start Group Control
    </button>
  </div>
</template>

<script>
export default {
  props: {
    setMode: {
      type: Function,
      required: true,
    },
  },
  methods: {
    startGroupControl() {
      this.setMode("group", {});
    },
  },
};
</script>

<style scoped>

.start-button {
  flex-grow: 1;
  padding-top: 10px;
  align-items: center;
}
button {
  font-size: 2rem;
  font-weight: bold;
}
</style>
