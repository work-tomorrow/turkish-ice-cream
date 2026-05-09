<template>
  <el-card shadow="never" class="panel status-panel">
    <template #header>
      <div class="panel-header">
        <span>System Status</span>
        <el-tag :type="status.system.ready ? 'success' : 'danger'">
          {{ status.system.ready ? 'Ready' : 'Blocked' }}
        </el-tag>
      </div>
    </template>

    <div class="metric-grid">
      <div class="metric">
        <span>Robot State</span>
        <strong>{{ formatState(status.robot.state) }}</strong>
      </div>
      <div class="metric">
        <span>Hand</span>
        <strong>{{ status.vision.hand_detected ? 'Detected' : 'Not Found' }}</strong>
      </div>
    </div>

    <el-descriptions :column="2" border class="hand-table">
      <el-descriptions-item label="X">{{
        displayValue(status.vision.hand?.x)
      }}</el-descriptions-item>
      <el-descriptions-item label="Y">{{
        displayValue(status.vision.hand?.y)
      }}</el-descriptions-item>
      <el-descriptions-item label="Area">
        {{ displayValue(status.vision.hand?.area) }}
      </el-descriptions-item>
      <el-descriptions-item label="Confidence">
        {{ displayValue(status.vision.hand?.confidence) }}
      </el-descriptions-item>
      <el-descriptions-item label="Last Update">{{ lastUpdated }}</el-descriptions-item>
    </el-descriptions>
  </el-card>
</template>

<script setup>
defineProps({
  status: {
    type: Object,
    required: true,
  },
  lastUpdated: {
    type: String,
    required: true,
  },
  formatState: {
    type: Function,
    required: true,
  },
  displayValue: {
    type: Function,
    required: true,
  },
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

.metric-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 12px;
}

.metric {
  min-height: 84px;
  padding: 14px;
  border: 1px solid #e5eaf3;
  border-radius: 8px;
  background: #fbfcff;
}

.metric span {
  display: block;
  margin-bottom: 8px;
  color: #64748b;
  font-size: 13px;
  font-weight: 600;
}

.metric strong {
  color: #111827;
  font-size: 20px;
  font-weight: 700;
  overflow-wrap: anywhere;
}

.hand-table {
  margin-top: 16px;
}

@media (max-width: 640px) {
  .metric-grid {
    grid-template-columns: 1fr;
  }
}
</style>
