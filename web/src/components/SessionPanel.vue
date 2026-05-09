<template>
  <el-card shadow="never" class="panel">
    <template #header>
      <div class="panel-header">
        <span>Session</span>
        <el-tag :type="session.active ? 'success' : 'info'">
          {{ session.active ? 'Active' : 'Inactive' }}
        </el-tag>
      </div>
    </template>

    <div class="button-row">
      <el-button
        type="primary"
        :icon="VideoPlay"
        :loading="busy === 'start'"
        @click="$emit('start')"
      >
        Start
      </el-button>
      <el-button type="danger" :icon="Close" :loading="busy === 'stop'" @click="$emit('stop')">
        Stop
      </el-button>
    </div>
  </el-card>
</template>

<script setup>
import { Close, VideoPlay } from '@element-plus/icons-vue'

defineProps({
  session: {
    type: Object,
    required: true,
  },
  busy: {
    type: String,
    required: true,
  },
})

defineEmits(['start', 'stop'])
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

.button-row {
  display: flex;
  flex-wrap: wrap;
  gap: 10px;
}
</style>
