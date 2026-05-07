<template>
  <el-card shadow="never" class="panel">
    <template #header>
      <div class="panel-header">
        <span>Manual Actions</span>
        <el-tag>{{ actions.length }} actions</el-tag>
      </div>
    </template>

    <div class="action-grid">
      <el-button
        v-for="action in actions"
        :key="action"
        :type="action === currentAction ? 'primary' : 'default'"
        :disabled="emergencyStopped && action !== 'stop'"
        :loading="busy === action"
        @click="$emit('run-action', action)"
      >
        {{ formatAction(action) }}
      </el-button>
    </div>
  </el-card>
</template>

<script setup>
defineProps({
  actions: {
    type: Array,
    required: true,
  },
  currentAction: {
    type: String,
    required: true,
  },
  emergencyStopped: {
    type: Boolean,
    required: true,
  },
  busy: {
    type: String,
    required: true,
  },
  formatAction: {
    type: Function,
    required: true,
  },
})

defineEmits(['run-action'])
</script>

<style scoped>
.panel {
  border-radius: 8px;
  border: 1px solid #dfe5ef;
}

.panel-header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 12px;
  font-weight: 700;
}

.action-grid {
  display: grid;
  grid-template-columns: repeat(3, minmax(0, 1fr));
  gap: 10px;
}

.action-grid :deep(.el-button) {
  margin-left: 0;
  min-height: 40px;
}

@media (max-width: 640px) {
  .action-grid {
    grid-template-columns: 1fr;
  }
}
</style>
