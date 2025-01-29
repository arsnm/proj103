<template>
  <div class="p-4 border rounded shadow">
    <button
      @click="startGroupControl"
      class="bg-green-500 text-white py-2 px-4 rounded hover:bg-green-600"
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
button {
  font-size: 1rem;
  font-weight: bold;
}
</style>
