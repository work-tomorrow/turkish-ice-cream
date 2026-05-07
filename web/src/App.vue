<template>
  <el-container class="app-shell" direction="vertical">
    <AppHeader :api-online="apiOnline" :loading="loading" @refresh="refreshAll" />

    <el-main class="dashboard">
      <section class="status-grid">
        <StatusPanel
          :status="status"
          :last-updated="lastUpdated"
          :format-action="formatAction"
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
          v-model:max-tease="maxTease"
          :session="session"
          :busy="busy"
          @edit-max-tease="editingMaxTease = $event"
          @start="startSession"
          @pause="setSessionActive(false)"
          @reset="resetSession"
          @stop="stopSession"
        />
        <ActionsPanel
          :actions="actions"
          :current-action="status.current_action"
          :emergency-stopped="status.emergency_stopped"
          :busy="busy"
          :format-action="formatAction"
          @run-action="runAction"
        />
        <EmergencyPanel
          :emergency-stopped="status.emergency_stopped"
          :busy="busy"
          @stop="setEmergencyStop"
          @clear="clearEmergencyStop"
        />
      </section>
    </el-main>
  </el-container>
</template>

<script setup>
import axios from 'axios'
import { computed, onBeforeUnmount, onMounted, reactive, ref } from 'vue'
import { ElMessage } from 'element-plus'
import ActionsPanel from './components/ActionsPanel.vue'
import AppHeader from './components/AppHeader.vue'
import EmergencyPanel from './components/EmergencyPanel.vue'
import SessionPanel from './components/SessionPanel.vue'
import StatusPanel from './components/StatusPanel.vue'
import VisionPanel from './components/VisionPanel.vue'

const defaultStatus = {
  running: false,
  emergency_stopped: false,
  current_action: 'home',
  tease_count: 0,
  max_tease: 3,
  hand_detected: false,
  hand: {
    detected: false,
    x: null,
    y: null,
    confidence: null,
  },
}

const status = reactive({ ...defaultStatus, hand: { ...defaultStatus.hand } })
const session = reactive({
  active: false,
  current_action: 'home',
  tease_count: 0,
  max_tease: 3,
})

const actions = ref([])
const maxTease = ref(3)
const editingMaxTease = ref(false)
const loading = ref(false)
const busy = ref('')
const apiOnline = ref(false)
const frameVersion = ref(Date.now())
const frameError = ref(false)
const updatedAt = ref(null)
let statusSocket = null

const frameUrl = computed(() => {
  const baseURL = axios.defaults.baseURL || ''
  return `${baseURL}/vision/frame?t=${frameVersion.value}`
})

const visionStreamUrl = computed(() => {
  const apiBase = axios.defaults.baseURL || window.location.origin
  return `${apiBase.replace(/^http/, 'ws').replace(/\/$/, '')}/ws/vision`
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
  Object.assign(status, payload)
  status.hand = payload.hand || { ...defaultStatus.hand }
  session.active = payload.running
  session.current_action = payload.current_action
  session.tease_count = payload.tease_count
  session.max_tease = payload.max_tease
  if (!editingMaxTease.value) {
    maxTease.value = payload.max_tease
  }
  apiOnline.value = true
  updatedAt.value = new Date()
}

const formatAction = (value) => {
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

const refreshActions = async () => {
  const res = await axios.get('/actions')
  actions.value = res.data.actions || []
}

const refreshAll = async () => {
  loading.value = true
  try {
    await Promise.all([refreshStatus(), refreshSession(), refreshActions()])
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
    await axios.post('/session', { max_tease: maxTease.value })
    ElMessage.success('Session started')
  })

const setSessionActive = (active) =>
  withBusy(active ? 'resume' : 'pause', async () => {
    await axios.patch('/session', { active })
  })

const resetSession = () =>
  withBusy('reset', async () => {
    await axios.patch('/session', { reset: true })
  })

const stopSession = () =>
  withBusy('stop', async () => {
    await axios.delete('/session')
  })

const runAction = (action) =>
  withBusy(action, async () => {
    await axios.post('/actions', { name: action })
  })

const setEmergencyStop = () =>
  withBusy('emergency', async () => {
    await axios.post('/emergency-stop')
  })

const clearEmergencyStop = () =>
  withBusy('clear-emergency', async () => {
    await axios.delete('/emergency-stop')
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
  grid-template-columns: minmax(280px, 0.8fr) minmax(320px, 1fr) minmax(260px, 0.75fr);
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
