<template>
  <div class="p-4 border rounded shadow">
    <div class="mb-4">
      <label class="flex items-center space-x-2">
        <input type="checkbox" v-model="setStart" />
        <span>Set Start</span>
      </label>
      <div v-if="setStart" class="mt-2">
        <input
          v-model="startInput"
          type="text"
          placeholder="Enter start value"
          class="border p-2 rounded w-full"
        />
      </div>
    </div>

    <div class="mb-4">
      <label class="flex items-center space-x-2">
        <input type="checkbox" v-model="isCoordMode" />
        <span>{{ isCoordMode ? "Coord" : "Case" }}</span>
      </label>

      <div v-if="isCoordMode" class="mt-2 space-y-2">
        <input
          v-model.number="coord.x"
          type="number"
          placeholder="X (0 - 300 cm)"
          min="0"
          max="300"
          class="border p-2 rounded w-full"
        />
        <input
          v-model.number="coord.y"
          type="number"
          placeholder="Y (0 - 300 cm)"
          min="0"
          max="300"
          class="border p-2 rounded w-full"
        />
        <input
          v-model.number="coord.theta"
          type="number"
          placeholder="Theta (0 - 360°)"
          min="0"
          max="360"
          class="border p-2 rounded w-full"
        />
      </div>

      <div v-else class="mt-2">
        <input
          v-model="caseInput"
          type="text"
          placeholder="Case (e.g., A3)"
          class="border p-2 rounded w-full"
          @input="validateCase"
        />
        <p v-if="caseError" class="text-red-500 text-sm">{{ caseError }}</p>
      </div>
    </div>

    <button
      @click="sendTarget"
      class="bg-blue-500 text-white py-2 px-4 rounded hover:bg-blue-600"
    >
      Send Target
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
  data() {
    return {
      setStart: false,
      startInput: "",
      isCoordMode: true,
      coord: {
        x: null,
        y: null,
        theta: null,
      },
      caseInput: "",
      caseError: "",
    };
  },
  methods: {
    validateCase() {
      const regex = /^[A-G][1-6]$/;
      if (!regex.test(this.caseInput)) {
        this.caseError = "Case must be a letter A-G followed by a number 1-6.";
      } else {
        this.caseError = "";
      }
    },

    sendTarget() {
      let targetData = {};

      if (this.isCoordMode) {
        const { x, y, theta } = this.coord;
        if (x === null || y === null || theta === null) {
          alert("Please fill in all coordinate fields.");
          return;
        }
        targetData = { type: "coord", x, y, theta };
      } else {
        if (this.caseError || !this.caseInput) {
          alert("Please provide a valid case.");
          return;
        }
        targetData = { type: "case", value: this.caseInput };
      }

      if (this.setStart && this.startInput) {
        targetData.start = this.startInput;
      }

      this.setMode("target", targetData);
    },
  },
};
</script>

<style scoped>
input {
  outline: none;
}
</style>
