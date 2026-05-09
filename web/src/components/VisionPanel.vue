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
      <img :src="visibleFrameUrl" alt="Current vision frame" @error="handleFrameError" />
      <el-empty v-if="frameError" description="No frame" />
    </div>
  </el-card>
</template>

<script setup>
import { Picture } from '@element-plus/icons-vue'
import { computed, ref } from 'vue'

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

const streamFailed = ref(false)

const liveConnected = computed(() => !streamFailed.value)
const visibleFrameUrl = computed(() => (streamFailed.value ? props.frameUrl : props.streamUrl))

const reloadSnapshot = () => {
  emit('reload')
  streamFailed.value = false
}

const handleFrameError = () => {
  streamFailed.value = true
  emit('frame-error')
}
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
