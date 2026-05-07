<template>
  <el-header class="topbar">
    <div>
      <p class="eyebrow">Turkish Ice Cream Demo</p>
      <h1>Robot Control Panel</h1>
    </div>
    <div class="topbar-actions">
      <el-tag :type="apiOnline ? 'success' : 'danger'" size="large">
        {{ apiOnline ? 'API Online' : 'API Offline' }}
      </el-tag>
      <el-button :icon="Refresh" circle :loading="loading" @click="$emit('refresh')" />
    </div>
  </el-header>
</template>

<script setup>
import { Refresh } from '@element-plus/icons-vue'

defineProps({
  apiOnline: {
    type: Boolean,
    required: true,
  },
  loading: {
    type: Boolean,
    required: true,
  },
})

defineEmits(['refresh'])
</script>

<style scoped>
.topbar {
  height: auto;
  min-height: 82px;
  display: flex;
  align-items: center;
  justify-content: space-between;
  gap: 18px;
  padding: 18px 28px;
  background: #ffffff;
  border-bottom: 1px solid #dfe5ef;
}

.eyebrow {
  margin: 0 0 4px;
  color: #64748b;
  font-size: 13px;
  font-weight: 700;
  text-transform: uppercase;
}

h1 {
  margin: 0;
  color: #111827;
  font-size: 28px;
  font-weight: 700;
  line-height: 1.15;
}

.topbar-actions {
  display: flex;
  align-items: center;
  gap: 12px;
}

@media (max-width: 640px) {
  .topbar {
    align-items: flex-start;
    flex-direction: column;
    padding: 16px;
  }
}
</style>
