<template>
  <div class="panel status-container">
    <div class="status-text">Robot Status</div>

    <div class="status-information">
      <span>X:</span>
      <span>{{ status.position[0] }} cm</span>
    </div>

    <div class="status-information">
      <span>Y:</span>
      <span>{{ status.position[1] }} cm</span>
    </div>

    <div class="status-information">
      <span>Theta:</span>
      <span>{{ status.position[2] }}°</span>
    </div>

    <div class="status-information">
      <span>Speed:</span>
      <span>{{ status.speed }}</span>
    </div>

    <div class="status-information">
      <span>Mode:</span>
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
.status-container {
  display: flex;
  align-items: center;
  justify-content: center;
  flex-wrap: wrap;  /* Allows wrapping */
  gap: 15px;  /* Optional spacing */
  padding: 10px;
}

.status-text {
  font-size: 2rem;
}

.status-information {
  width: 150px; /* Adjust size */
  height: 100px;
  background-color: lightblue;
  text-align: center;
  line-height: 100px;
  font-weight: bold;
}

span {
  font-size: 1rem;
}
</style>
