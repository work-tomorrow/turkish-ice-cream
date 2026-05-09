<template>
  <el-card shadow="never" class="panel">
    <template #header>
      <div class="panel-header">
        <span>Motor Calibration</span>
        <div class="panel-actions">
          <el-button
            size="small"
            type="warning"
            :disabled="motors.length === 0"
            :loading="busy === 'home-all'"
            @click="$emit('home-motors')"
          >
            Home All
          </el-button>
          <el-button
            size="small"
            type="primary"
            :disabled="motors.length === 0"
            :loading="busy === 'move-motors'"
            @click="emitAllTargets"
          >
            Move All
          </el-button>
          <el-button
            size="small"
            :loading="busy === 'zero-all'"
            @click="$emit('zero-motor')"
          >
            Set All To 0
          </el-button>
        </div>
      </div>
    </template>

    <div class="motor-list">
      <div v-for="motor in motors" :key="motor.id" class="motor-row">
        <div class="motor-meta">
          <strong>{{ motor.id }}</strong>
          <span>Current {{ formatAngle(motor.current_angle) }} deg</span>
          <span>Target {{ formatAngle(motor.target_angle) }} deg</span>
          <span>Range {{ formatAngle(motor.min_angle) }} ~ {{ formatAngle(motor.max_angle) }} deg</span>
        </div>

        <div class="motor-target">
          <el-input-number
            v-model="draftTargets[motor.id]"
            :min="motor.min_angle"
            :max="motor.max_angle"
            :step="1"
            :precision="1"
            :disabled="busy === 'move-motors'"
            controls-position="right"
          />
          <el-button
            type="primary"
            :loading="busy === 'move-motors'"
            @click="emitSingleTarget(motor.id)"
          >
            Move
          </el-button>
        </div>

        <div class="motor-controls">
          <el-button
            :loading="busy === `${motor.id}-negative`"
            @click="$emit('jog-motor', { motorId: motor.id, direction: 'negative' })"
          >
            -{{ motor.step_degrees }} deg
          </el-button>
          <el-button
            :loading="busy === `${motor.id}-positive`"
            @click="$emit('jog-motor', { motorId: motor.id, direction: 'positive' })"
          >
            +{{ motor.step_degrees }} deg
          </el-button>
          <el-button
            type="primary"
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
import { reactive, watch } from 'vue'

const props = defineProps({
  motors: {
    type: Array,
    required: true,
  },
  busy: {
    type: String,
    required: true,
  },
})

const emit = defineEmits(['home-motors', 'move-motors', 'jog-motor', 'zero-motor'])

const draftTargets = reactive({})

watch(
  () => props.motors,
  (motors) => {
    for (const key of Object.keys(draftTargets)) {
      if (!motors.some((motor) => motor.id === key)) {
        delete draftTargets[key]
      }
    }

    for (const motor of motors) {
      draftTargets[motor.id] = Number(
        draftTargets[motor.id] ?? motor.target_angle ?? motor.current_angle ?? 0,
      )
    }
  },
  { immediate: true, deep: true },
)

const emitSingleTarget = (motorId) => {
  emit('move-motors', { [motorId]: Number(draftTargets[motorId] ?? 0) })
}

const emitAllTargets = () => {
  const targets = Object.fromEntries(
    props.motors.map((motor) => [motor.id, Number(draftTargets[motor.id] ?? 0)]),
  )
  emit('move-motors', targets)
}

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

.panel-actions {
  display: flex;
  gap: 8px;
}

.motor-list {
  display: grid;
  gap: 10px;
}

.motor-row {
  display: grid;
  grid-template-columns: minmax(120px, 0.8fr) minmax(220px, 1.1fr) minmax(240px, 1.6fr);
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

.motor-target {
  display: grid;
  grid-template-columns: minmax(0, 1fr) auto;
  gap: 8px;
  align-items: center;
}

.motor-target :deep(.el-input-number) {
  width: 100%;
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

  .panel-actions,
  .motor-target {
    display: grid;
  }

  .motor-controls {
    grid-template-columns: 1fr;
  }
}
</style>
