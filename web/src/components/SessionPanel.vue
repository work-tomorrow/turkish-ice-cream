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

    <el-form label-position="top" class="session-form">
      <el-form-item label="Max Tease">
        <el-input-number
          :model-value="maxTease"
          :min="1"
          :max="20"
          @update:model-value="$emit('update:maxTease', $event)"
          @focus="$emit('edit-max-tease', true)"
          @blur="$emit('edit-max-tease', false)"
        />
      </el-form-item>
    </el-form>

    <div class="button-row">
      <el-button type="primary" :icon="VideoPlay" :loading="busy === 'start'" @click="$emit('start')">
        Start
      </el-button>
      <el-button :icon="SwitchButton" :loading="busy === 'pause'" @click="$emit('pause')">
        Pause
      </el-button>
      <el-button :icon="RefreshLeft" :loading="busy === 'reset'" @click="$emit('reset')">
        Reset
      </el-button>
      <el-button type="warning" :icon="Close" :loading="busy === 'stop'" @click="$emit('stop')">
        Stop
      </el-button>
    </div>
  </el-card>
</template>

<script setup>
import { Close, RefreshLeft, SwitchButton, VideoPlay } from '@element-plus/icons-vue'

defineProps({
  session: {
    type: Object,
    required: true,
  },
  maxTease: {
    type: Number,
    required: true,
  },
  busy: {
    type: String,
    required: true,
  },
})

defineEmits(['update:maxTease', 'edit-max-tease', 'start', 'pause', 'reset', 'stop'])
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

.session-form {
  max-width: 180px;
}

.button-row {
  display: flex;
  flex-wrap: wrap;
  gap: 10px;
}
</style>
