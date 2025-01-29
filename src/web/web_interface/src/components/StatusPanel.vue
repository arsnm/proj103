<template>
  <div class="panel p-4 border rounded shadow space-y-4">
    <div class="text-lg font-bold">Robot Status</div>

    <div class="flex items-center space-x-2">
      <span class="font-medium">X:</span>
      <span>{{ status.position[0] }} cm</span>
    </div>

    <div class="flex items-center space-x-2">
      <span class="font-medium">Y:</span>
      <span>{{ status.position[1] }} cm</span>
    </div>

    <div class="flex items-center space-x-2">
      <span class="font-medium">Theta:</span>
      <span>{{ status.position[2] }}°</span>
    </div>

    <div class="flex items-center space-x-2">
      <span class="font-medium">Speed:</span>
      <span>{{ status.speed }}</span>
    </div>

    <div class="flex items-center space-x-2">
      <span class="font-medium">Mode:</span>
      <span>{{ status.mode }}</span>
    </div>
  </div>
</template>

<script>
export default {
  props: {
    status: {
      type: Object,
      required: true,
      default: () => ({
        position: (0, 0, 0),
        speed: 0,
        mode: "manual",
      }),
    },
  },
};
</script>

<style scoped>
span {
  font-size: 1rem;
}
</style>
