<template>
  <el-card shadow="never" class="panel">
    <template #header>
      <div class="panel-header">
        <span>Motor Calibration</span>
        <el-button
          size="small"
          :disabled="emergencyStopped"
          :loading="busy === 'zero-all'"
          @click="$emit('zero-motor')"
        >
          Set All To 0
        </el-button>
      </div>
    </template>

    <div class="motor-list">
      <div v-for="motor in motors" :key="motor.id" class="motor-row">
        <div class="motor-meta">
          <strong>{{ motor.id }}</strong>
          <span>{{ formatAngle(motor.current_angle) }} deg</span>
        </div>

        <div class="motor-controls">
          <el-button
            :disabled="emergencyStopped"
            :loading="busy === `${motor.id}-negative`"
            @click="$emit('jog-motor', { motorId: motor.id, direction: 'negative' })"
          >
            -{{ motor.step_degrees }} deg
          </el-button>
          <el-button
            type="primary"
            :disabled="emergencyStopped"
            :loading="busy === `${motor.id}-positive`"
            @click="$emit('jog-motor', { motorId: motor.id, direction: 'positive' })"
          >
            +{{ motor.step_degrees }} deg
          </el-button>
          <el-button
            :disabled="emergencyStopped"
            :loading="busy === `${motor.id}-zero`"
            @click="$emit('zero-motor', motor.id)"
          >
            Set 0
          </el-button>
        </div>
      </div>
    </div>
  </el-card>
</template>

<script setup>
defineProps({
  motors: {
    type: Array,
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
})

defineEmits(['jog-motor', 'zero-motor'])

const formatAngle = (value) => Number(value || 0).toFixed(1)
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

.motor-list {
  display: grid;
  gap: 10px;
}

.motor-row {
  display: grid;
  grid-template-columns: minmax(120px, 0.8fr) minmax(240px, 1.6fr);
  gap: 12px;
  align-items: center;
  padding: 12px;
  border: 1px solid #e5eaf3;
  border-radius: 8px;
  background: #fbfcff;
}

.motor-meta {
  display: grid;
  gap: 4px;
  min-width: 0;
}

.motor-meta strong,
.motor-meta span {
  overflow-wrap: anywhere;
}

.motor-meta span {
  color: #64748b;
  font-size: 13px;
}

.motor-controls {
  display: grid;
  grid-template-columns: repeat(3, minmax(0, 1fr));
  gap: 8px;
}

.motor-controls :deep(.el-button) {
  margin-left: 0;
  min-height: 38px;
  padding-inline: 10px;
}

@media (max-width: 640px) {
  .panel-header,
  .motor-row {
    align-items: stretch;
    grid-template-columns: 1fr;
  }

  .panel-header {
    display: grid;
  }

  .motor-controls {
    grid-template-columns: 1fr;
  }
}
</style>
