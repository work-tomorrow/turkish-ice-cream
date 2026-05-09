<template>
  <el-container class="app-shell" direction="vertical">
    <AppHeader :api-online="apiOnline" :loading="loading" @refresh="refreshDebugSession" />

    <el-main class="debug-dashboard">
      <section class="debug-grid">
        <el-card shadow="never" class="panel">
          <template #header>
            <div class="panel-header">
              <span>Debug Mode</span>
              <el-tag :type="debugSession.active ? 'success' : 'info'">
                {{ debugSession.active ? 'Active' : 'Inactive' }}
              </el-tag>
            </div>
          </template>

          <div class="control-stack">
            <div class="button-row">
              <el-button
                type="primary"
                :icon="VideoPlay"
                :loading="busy === 'start-debug'"
                @click="startDebugSession"
              >
                Start Debug Mode
              </el-button>
              <el-button
                type="danger"
                :icon="Close"
                :loading="busy === 'stop-debug'"
                @click="stopDebugSession"
              >
                End Debug Mode
              </el-button>
            </div>

            <el-form label-position="top">
              <el-form-item label="Move Duration">
                <el-input-number
                  v-model="durationS"
                  :min="0.1"
                  :max="60"
                  :step="0.5"
                  :precision="1"
                  controls-position="right"
                />
              </el-form-item>
            </el-form>

            <el-descriptions :column="1" border>
              <el-descriptions-item label="Moving">
                {{ debugSession.moving ? 'Yes' : 'No' }}
              </el-descriptions-item>
              <el-descriptions-item label="Target Y">
                {{ displayValue(debugSession.target_y_cm) }} cm
              </el-descriptions-item>
              <el-descriptions-item label="Target Z">
                {{ displayValue(debugSession.target_z_cm) }} cm
              </el-descriptions-item>
              <el-descriptions-item label="Duration">
                {{ displayValue(debugSession.duration_s) }} s
              </el-descriptions-item>
            </el-descriptions>
          </div>
        </el-card>

        <el-card shadow="never" class="panel plane-panel">
          <template #header>
            <div class="panel-header">
              <span>Y/Z Target Plane</span>
              <el-button
                type="primary"
                :icon="Position"
                :disabled="!debugSession.active || selectedTarget === null"
                :loading="busy === 'move-debug'"
                @click="sendSelectedTarget"
              >
                Move
              </el-button>
            </div>
          </template>

          <div
            class="target-plane"
            :class="{ disabled: !debugSession.active }"
            @click="selectTarget"
          >
            <div class="axis-label y-min">Y -25</div>
            <div class="axis-label y-max">Y 25</div>
            <div class="axis-label z-max">Z 25</div>
            <div class="axis-label z-min">Z -25</div>
            <div
              v-if="selectedTarget"
              class="target-marker"
              :style="markerStyle"
            />
          </div>

          <div class="target-readout">
            <span>Selected Y {{ displayValue(selectedTarget?.y_cm) }} cm</span>
            <span>Selected Z {{ displayValue(selectedTarget?.z_cm) }} cm</span>
          </div>
        </el-card>
      </section>
    </el-main>
  </el-container>
</template>

<script setup>
import axios from 'axios'
import { computed, onMounted, reactive, ref } from 'vue'
import { ElMessage, ElMessageBox } from 'element-plus'
import { Close, Position, VideoPlay } from '@element-plus/icons-vue'
import AppHeader from '../components/AppHeader.vue'

const WORKSPACE_MIN_CM = -25
const WORKSPACE_MAX_CM = 25
const WORKSPACE_SIZE_CM = WORKSPACE_MAX_CM - WORKSPACE_MIN_CM

const debugSession = reactive({
  active: false,
  moving: false,
  target_y_cm: null,
  target_z_cm: null,
  duration_s: null,
})
const selectedTarget = ref(null)
const durationS = ref(5.0)
const loading = ref(false)
const busy = ref('')
const apiOnline = ref(false)

const markerStyle = computed(() => {
  if (!selectedTarget.value) return {}
  const left = ((selectedTarget.value.y_cm - WORKSPACE_MIN_CM) / WORKSPACE_SIZE_CM) * 100
  const top = ((WORKSPACE_MAX_CM - selectedTarget.value.z_cm) / WORKSPACE_SIZE_CM) * 100
  return {
    left: `${left}%`,
    top: `${top}%`,
  }
})

const displayValue = (value) => {
  if (value === null || value === undefined) return '-'
  if (typeof value === 'number') return value.toFixed(2)
  return value
}

const showError = (title, error) => {
  const detail = error?.response?.data?.detail || error?.message || error
  ElMessage.error(`${title}: ${detail}`)
}

const showTargetAnglesDialog = async (targetMotorAngles) => {
  const entries = Object.entries(targetMotorAngles || {})
  if (!entries.length) return

  const lines = entries.map(([motorId, angle]) => {
    const numericAngle = Number(angle)
    return `${motorId}: ${Number.isFinite(numericAngle) ? numericAngle.toFixed(2) : angle}°`
  })

  await ElMessageBox.alert(lines.join('\n'), 'Final Target Angles', {
    confirmButtonText: 'OK',
  })
}

const applyDebugSession = (payload) => {
  Object.assign(debugSession, {
    active: Boolean(payload.active),
    moving: Boolean(payload.moving),
    target_y_cm: payload.target_y_cm ?? null,
    target_z_cm: payload.target_z_cm ?? null,
    duration_s: payload.duration_s ?? null,
  })
  apiOnline.value = true
}

const refreshDebugSession = async () => {
  loading.value = true
  try {
    const res = await axios.get('/debug/session')
    applyDebugSession(res.data)
  } catch (error) {
    apiOnline.value = false
    showError('API request failed', error)
  } finally {
    loading.value = false
  }
}

const withBusy = async (key, fn) => {
  busy.value = key
  try {
    await fn()
  } catch (error) {
    showError('Command failed', error)
  } finally {
    busy.value = ''
  }
}

const startDebugSession = () =>
  withBusy('start-debug', async () => {
    const res = await axios.post('/debug/session', {})
    applyDebugSession(res.data)
    ElMessage.success('Debug mode started')
  })

const stopDebugSession = () =>
  withBusy('stop-debug', async () => {
    const res = await axios.delete('/debug/session')
    applyDebugSession(res.data)
  })

const selectTarget = (event) => {
  if (!debugSession.active) return

  const rect = event.currentTarget.getBoundingClientRect()
  const xRatio = Math.min(Math.max((event.clientX - rect.left) / rect.width, 0), 1)
  const yRatio = Math.min(Math.max((event.clientY - rect.top) / rect.height, 0), 1)
  selectedTarget.value = {
    y_cm: WORKSPACE_MIN_CM + xRatio * WORKSPACE_SIZE_CM,
    z_cm: WORKSPACE_MAX_CM - yRatio * WORKSPACE_SIZE_CM,
  }
  sendSelectedTarget()
}

const sendSelectedTarget = () =>
  withBusy('move-debug', async () => {
    if (!selectedTarget.value) return
    const res = await axios.post('/debug/move', {
      y_cm: selectedTarget.value.y_cm,
      z_cm: selectedTarget.value.z_cm,
      duration_s: Number(durationS.value),
    })
    applyDebugSession(res.data)
    await showTargetAnglesDialog(res.data.target_motor_angles)
    ElMessage.success('Debug move started')
  })

onMounted(() => {
  refreshDebugSession()
})
</script>

<style scoped>
.app-shell {
  min-height: 100vh;
  background: #f5f7fb;
  color: #1f2937;
}

.debug-dashboard {
  max-width: 1180px;
  width: 100%;
  margin: 0 auto;
  padding: 24px;
}

.debug-grid {
  display: grid;
  grid-template-columns: minmax(280px, 0.7fr) minmax(360px, 1.3fr);
  gap: 16px;
}

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

.control-stack {
  display: grid;
  gap: 16px;
}

.button-row {
  display: flex;
  flex-wrap: wrap;
  gap: 10px;
}

.button-row :deep(.el-button) {
  margin-left: 0;
}

.target-plane {
  position: relative;
  width: min(100%, 640px);
  aspect-ratio: 1;
  margin: 0 auto;
  border: 1px solid #cbd5e1;
  border-radius: 8px;
  background:
    linear-gradient(90deg, rgba(37, 99, 235, 0.08) 1px, transparent 1px),
    linear-gradient(0deg, rgba(37, 99, 235, 0.08) 1px, transparent 1px),
    #fbfcff;
  background-size: 10% 10%;
  cursor: crosshair;
}

.target-plane::before,
.target-plane::after {
  position: absolute;
  content: '';
  background: #94a3b8;
}

.target-plane::before {
  left: 50%;
  top: 0;
  bottom: 0;
  width: 1px;
}

.target-plane::after {
  top: 50%;
  left: 0;
  right: 0;
  height: 1px;
}

.target-plane.disabled {
  cursor: not-allowed;
  opacity: 0.58;
}

.axis-label {
  position: absolute;
  padding: 2px 6px;
  border-radius: 6px;
  background: rgba(255, 255, 255, 0.86);
  color: #475569;
  font-size: 12px;
  font-weight: 700;
  pointer-events: none;
}

.y-min {
  left: 8px;
  top: 50%;
  transform: translateY(-50%);
}

.y-max {
  right: 8px;
  top: 50%;
  transform: translateY(-50%);
}

.z-max {
  left: 50%;
  top: 8px;
  transform: translateX(-50%);
}

.z-min {
  left: 50%;
  bottom: 8px;
  transform: translateX(-50%);
}

.target-marker {
  position: absolute;
  width: 18px;
  height: 18px;
  border: 3px solid #ef4444;
  border-radius: 50%;
  background: rgba(239, 68, 68, 0.18);
  transform: translate(-50%, -50%);
  pointer-events: none;
}

.target-readout {
  display: flex;
  flex-wrap: wrap;
  gap: 12px;
  margin-top: 14px;
  color: #475569;
  font-weight: 700;
}

@media (max-width: 980px) {
  .debug-grid {
    grid-template-columns: 1fr;
  }
}

@media (max-width: 640px) {
  .debug-dashboard {
    padding: 16px;
  }
}
</style>
