<template>
  <el-container class="app-shell" direction="vertical">
    <AppHeader :api-online="apiOnline" :loading="loading" @refresh="refreshAll" />

    <el-main class="dashboard">
      <section class="status-grid">
        <StatusPanel
          :status="status"
          :last-updated="lastUpdated"
          :format-state="formatState"
          :display-value="displayValue"
        />
        <VisionPanel
          :frame-url="frameUrl"
          :stream-url="visionStreamUrl"
          :frame-error="frameError"
          @reload="reloadFrame"
          @frame-error="frameError = true"
        />
      </section>

      <section class="control-grid">
        <SessionPanel
          :session="session"
          :busy="busy"
          @start="startSession"
          @stop="stopSession"
        />
        <MotorCalibrationPanel
          :motors="motors"
          :busy="busy"
          @move-motors="moveMotors"
          @jog-motor="jogMotor"
          @zero-motor="zeroMotor"
        />
      </section>
    </el-main>
  </el-container>
</template>

<script setup>
import axios from 'axios'
import { computed, onBeforeUnmount, onMounted, reactive, ref } from 'vue'
import { ElMessage } from 'element-plus'
import AppHeader from './components/AppHeader.vue'
import MotorCalibrationPanel from './components/MotorCalibrationPanel.vue'
import SessionPanel from './components/SessionPanel.vue'
import StatusPanel from './components/StatusPanel.vue'
import VisionPanel from './components/VisionPanel.vue'

const defaultStatus = {
  system: {
    ready: true,
  },
  robot: {
    state: 'idle',
  },
  vision: {
    hand_detected: false,
    hand: {
      detected: false,
      x: null,
      y: null,
      confidence: null,
    },
  },
}

const status = reactive({
  system: { ...defaultStatus.system },
  robot: { ...defaultStatus.robot },
  vision: {
    ...defaultStatus.vision,
    hand: { ...defaultStatus.vision.hand },
  },
})
const session = reactive({
  active: false,
  robot_state: 'idle',
})

const motors = ref([])
const loading = ref(false)
const busy = ref('')
const apiOnline = ref(false)
const frameVersion = ref(Date.now())
const frameError = ref(false)
const updatedAt = ref(null)
let statusSocket = null
const visionBaseURL = (import.meta.env.VITE_VISION_API_BASE_URL || 'http://localhost:8010').replace(
  /\/$/,
  '',
)

const frameUrl = computed(() => {
  return `${visionBaseURL}/vision/frame?t=${frameVersion.value}`
})

const visionStreamUrl = computed(() => {
  return `${visionBaseURL}/vision/stream`
})

const lastUpdated = computed(() => {
  if (!updatedAt.value) return '-'
  return new Intl.DateTimeFormat('zh-TW', {
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit',
  }).format(updatedAt.value)
})

const applyStatus = (payload) => {
  status.system = payload.system || { ...defaultStatus.system }
  status.robot = payload.robot || { ...defaultStatus.robot }
  status.vision = payload.vision || {
    ...defaultStatus.vision,
    hand: { ...defaultStatus.vision.hand },
  }
  apiOnline.value = true
  updatedAt.value = new Date()
}

const formatState = (value) => {
  if (!value) return '-'
  return value
    .split('_')
    .map((part) => part.charAt(0).toUpperCase() + part.slice(1))
    .join(' ')
}

const displayValue = (value) => {
  if (value === null || value === undefined) return '-'
  if (typeof value === 'number') return value.toFixed(2)
  return value
}

const showError = (title, error) => {
  const detail = error?.response?.data?.detail || error?.message || error
  ElMessage.error(`${title}: ${detail}`)
}

const refreshStatus = async () => {
  const res = await axios.get('/status')
  applyStatus(res.data)
}

const refreshSession = async () => {
  const res = await axios.get('/session')
  Object.assign(session, res.data)
}

const refreshMotors = async () => {
  const res = await axios.get('/motors')
  motors.value = res.data.motors || []
}

const refreshAll = async () => {
  loading.value = true
  try {
    await Promise.all([refreshStatus(), refreshSession(), refreshMotors()])
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
    await refreshAll()
  } catch (error) {
    showError('Command failed', error)
  } finally {
    busy.value = ''
  }
}

const startSession = () =>
  withBusy('start', async () => {
    await axios.post('/session', {})
    ElMessage.success('Session started')
  })

const stopSession = () =>
  withBusy('stop', async () => {
    await axios.delete('/session')
  })

const jogMotor = ({ motorId, direction }) =>
  withBusy(`${motorId}-${direction}`, async () => {
    await axios.post(`/motors/${motorId}/jog`, {
      direction,
      step_degrees: 3,
    })
  })

const moveMotors = (targets) =>
  withBusy('move-motors', async () => {
    await axios.post('/motors/targets', { targets })
    ElMessage.success('Motor targets updated')
  })

const zeroMotor = (motorId) =>
  withBusy(motorId ? `${motorId}-zero` : 'zero-all', async () => {
    await axios.post(motorId ? `/motors/${motorId}/zero` : '/motors/zero')
    ElMessage.success(
      motorId ? `${motorId} zero calibration saved` : 'All motor zero calibrations saved',
    )
  })

const reloadFrame = () => {
  frameError.value = false
  frameVersion.value = Date.now()
}

const connectStatusStream = () => {
  const apiBase = axios.defaults.baseURL || window.location.origin
  const wsBase = apiBase.replace(/^http/, 'ws').replace(/\/$/, '')
  statusSocket = new WebSocket(`${wsBase}/ws/status`)

  statusSocket.onmessage = (event) => {
    applyStatus(JSON.parse(event.data))
  }
  statusSocket.onopen = () => {
    apiOnline.value = true
  }
  statusSocket.onerror = () => {
    apiOnline.value = false
  }
}

onMounted(() => {
  refreshAll()
  connectStatusStream()
})

onBeforeUnmount(() => {
  statusSocket?.close()
})
</script>

<style scoped>
.app-shell {
  min-height: 100vh;
  background: #f5f7fb;
  color: #1f2937;
}

.dashboard {
  max-width: 1180px;
  width: 100%;
  margin: 0 auto;
  padding: 24px;
}

.status-grid,
.control-grid {
  display: grid;
  gap: 16px;
}

.status-grid {
  grid-template-columns: minmax(0, 1.35fr) minmax(320px, 0.8fr);
  margin-bottom: 16px;
}

.control-grid {
  grid-template-columns: minmax(260px, 0.72fr) minmax(380px, 1.28fr);
}

@media (max-width: 980px) {
  .status-grid,
  .control-grid {
    grid-template-columns: 1fr;
  }
}

@media (max-width: 640px) {
  .dashboard {
    padding: 16px;
  }
}
</style>
