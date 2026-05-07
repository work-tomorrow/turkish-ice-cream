<template>
  <el-card shadow="never" class="panel danger-panel">
    <template #header>
      <div class="panel-header">
        <span>Emergency Stop</span>
        <el-tag :type="emergencyStopped ? 'danger' : 'success'">
          {{ emergencyStopped ? 'Active' : 'Ready' }}
        </el-tag>
      </div>
    </template>

    <div class="emergency-actions">
      <el-button
        type="danger"
        size="large"
        :icon="WarningFilled"
        :loading="busy === 'emergency'"
        @click="$emit('stop')"
      >
        Emergency Stop
      </el-button>
      <el-button
        size="large"
        :icon="CircleCheck"
        :loading="busy === 'clear-emergency'"
        @click="$emit('clear')"
      >
        Clear Stop
      </el-button>
    </div>
  </el-card>
</template>

<script setup>
import { CircleCheck, WarningFilled } from '@element-plus/icons-vue'

defineProps({
  emergencyStopped: {
    type: Boolean,
    required: true,
  },
  busy: {
    type: String,
    required: true,
  },
})

defineEmits(['stop', 'clear'])
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

.danger-panel {
  background: #fffafa;
}

.emergency-actions {
  display: flex;
  flex-direction: column;
  flex-wrap: wrap;
  gap: 10px;
}

.emergency-actions :deep(.el-button) {
  width: 100%;
  margin-left: 0;
}
</style>
