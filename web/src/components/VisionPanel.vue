<template>
  <el-card shadow="never" class="panel vision-panel">
    <template #header>
      <div class="panel-header">
        <div class="title-row">
          <span>Vision Stream</span>
          <el-tag :type="liveConnected ? 'success' : 'info'">
            {{ liveConnected ? 'Live' : 'Snapshot' }}
          </el-tag>
        </div>
        <el-button text :icon="Picture" @click="reloadSnapshot">Reload</el-button>
      </div>
    </template>
    <div class="frame-box">
      <img :src="visibleFrameUrl" alt="Current vision frame" @error="$emit('frame-error')" />
      <el-empty v-if="frameError" description="No frame" />
    </div>
  </el-card>
</template>

<script setup>
import { Picture } from '@element-plus/icons-vue'
import { computed, onBeforeUnmount, onMounted, ref } from 'vue'

const props = defineProps({
  frameUrl: {
    type: String,
    required: true,
  },
  streamUrl: {
    type: String,
    required: true,
  },
  frameError: {
    type: Boolean,
    required: true,
  },
})

const emit = defineEmits(['reload', 'frame-error'])

const liveConnected = ref(false)
const liveFrameUrl = ref('')
let visionSocket = null
let previousFrameUrl = ''

const visibleFrameUrl = computed(() => liveFrameUrl.value || props.frameUrl)

const releasePreviousFrame = () => {
  if (previousFrameUrl) {
    URL.revokeObjectURL(previousFrameUrl)
    previousFrameUrl = ''
  }
}

const setLiveFrame = (data) => {
  releasePreviousFrame()
  const blob = data instanceof Blob ? data : new Blob([data], { type: 'image/jpeg' })
  liveFrameUrl.value = URL.createObjectURL(blob)
  previousFrameUrl = liveFrameUrl.value
}

const connectVisionStream = () => {
  visionSocket = new WebSocket(props.streamUrl)
  visionSocket.binaryType = 'blob'

  visionSocket.onopen = () => {
    liveConnected.value = true
  }
  visionSocket.onmessage = (event) => {
    setLiveFrame(event.data)
  }
  visionSocket.onclose = () => {
    liveConnected.value = false
  }
  visionSocket.onerror = () => {
    liveConnected.value = false
  }
}

const reloadSnapshot = () => {
  emit('reload')
  if (!liveConnected.value) {
    liveFrameUrl.value = ''
    releasePreviousFrame()
  }
}

onMounted(() => {
  connectVisionStream()
})

onBeforeUnmount(() => {
  visionSocket?.close()
  releasePreviousFrame()
})
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

.title-row {
  display: flex;
  align-items: center;
  gap: 10px;
}

.frame-box {
  position: relative;
  display: grid;
  place-items: center;
  min-height: 248px;
  overflow: hidden;
  border: 1px solid #e5eaf3;
  border-radius: 8px;
  background: #111827;
}

.frame-box img {
  width: 100%;
  height: 100%;
  max-height: 320px;
  object-fit: contain;
  image-rendering: pixelated;
}
</style>
