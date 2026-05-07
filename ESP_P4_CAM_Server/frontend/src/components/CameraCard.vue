<template>
  <v-card class="mb-3 camera-card" :style="cardStyle">
    <v-img :src="imgSrc" :aspect-ratio="frameAspectRatio"
      :class="['camera-frame', `camera-frame--${displayMode}`]"
      :style="{ '--camera-zoom': `${zoomPercent / 100}` }"
      :id="`cam-${props.camNum}-v-img`" crossorigin="anonymous" @error="onImageError">
      <template #placeholder>
        <div class="d-flex align-center justify-center fill-height">
          <v-progress-circular color="grey-lighten-4" indeterminate />
        </div>
      </template>
      <template #error>
        <div class="d-flex align-center justify-center fill-height">
          <div style="height: 100px;" class="d-flex flex-column">
            <div class="my-auto" style="font-size: larger; font-weight: bold;">
              Something went wrong
            </div>
            <div class="mx-auto" style="font-size: smaller; opacity: 70%;">
              Retrying in 3 seconds...
            </div>
          </div>
        </div>
      </template>
    </v-img>
    <div class="d-flex justify-space-between align-center my-2 mx-3">
      <div>
        <div style="font-weight: bold; font-size: larger;">
          {{ camera.name && camera.name.length > 0 ? camera.name : `Camera #${camera.index}` }}
        </div>
        <div style="font-size: smaller; opacity: 70%;">
          {{ camera.currentImageFormatDescription }} @ {{ camera.currentFrameRate }} fps
          <span v-if="camera.currentQuality">
            (Quality: {{ camera.currentQuality }})
          </span>
        </div>
      </div>
      <div class="d-flex">
        <v-btn variant="tonal" @click="settingsDialog = true" :icon="mdiCog" class="mr-1" aria-label="Settings" />
        <v-btn variant="tonal" @click="captureFrame" :icon="mdiCameraOutline" class="mr-1"
          aria-label="Download Frame" />
        <v-btn variant="tonal" @click="recordClip" :icon="mdiVideo" class="mr-1" :loading="isRecording"
          :disabled="isRecording" aria-label="Record 15 Seconds" />
        <v-btn variant="tonal" @click="captureRawFrame" :icon="mdiRaw" class="mr-1"
          aria-label="Download Raw Image (BIN)" />
      </div>
    </div>
    <div class="mx-3 mb-3 d-flex align-center ga-3 flex-wrap">
      <v-btn-toggle v-model="displayMode" density="comfortable" mandatory variant="tonal">
        <v-btn value="fit">Fit</v-btn>
        <v-btn value="fill">Fill</v-btn>
      </v-btn-toggle>
      <div class="zoom-slider-wrap">
        <v-slider v-model="zoomPercent" min="100" max="300" step="10" hide-details density="compact" thumb-label>
          <template #append>
            <span class="zoom-label">{{ zoomPercent }}%</span>
          </template>
        </v-slider>
      </div>
      <v-btn variant="text" density="comfortable" @click="resetZoom">Reset</v-btn>
      <span v-if="isRecording" class="recording-badge">Recording {{ recordingSecondsLeft }}s</span>
    </div>
    <canvas ref="recordCanvas" class="record-canvas" />
  </v-card>

  <v-dialog v-model="settingsDialog" max-width="500">
    <v-card>
      <v-card-title :prepend-icon="mdiCog">
        Camera Settings
      </v-card-title>
      <v-card-text>
        <v-select v-model="selectedImageFormatId" :items="camera.imageFormats" item-title="description" item-value="id"
          :disabled="settingsSaving" label="Image Format" />
        <v-slider v-model="selectedQuality" v-if="selectedFormat?.quality" :min="selectedFormat?.quality.min ?? 80"
          :max="selectedFormat?.quality.max ?? 95" :step="selectedFormat?.quality.step ?? 1" :disabled="settingsSaving"
          label="Quality">
          <template #append>
            {{ selectedQuality }}
          </template>
        </v-slider>
        <div v-else class="text-center mb-4">This image format may not support quality settings</div>
        <v-row>
          <v-col cols="9">
            <v-btn variant="tonal" @click="saveSettings" width="100%" :loading="settingsSaving">Save</v-btn>
          </v-col>
          <v-col cols="3">
            <v-btn variant="tonal" @click="settingsDialog = false" color="error" width="100%"
              :disabled="settingsSaving">Cancel</v-btn>
          </v-col>
        </v-row>
      </v-card-text>
    </v-card>
  </v-dialog>

  <v-snackbar v-model="saveStatusSnackbar" :timeout="2000" color="success">
    {{ saveStatusSnackbarText }}
  </v-snackbar>

  <v-snackbar v-model="recordStatusSnackbar" :timeout="2500" :color="recordStatusSnackbarColor">
    {{ recordStatusSnackbarText }}
  </v-snackbar>
</template>

<script setup lang="ts">
import { ref } from 'vue'
import { mdiCameraOutline, mdiRaw, mdiCog, mdiVideo } from '@mdi/js';
import { useMainStore } from '@/store/mainstore';

const LOADING_IMAGE_SRC = "/loading.jpg"
const RECORD_SECONDS = 15
const RECORD_FPS = 15

const mainStore = useMainStore()

const props = defineProps<{
  camNum: number,
}>()

const camera = computed(() => mainStore.clientCameras[props.camNum])
const zoomStorageKey = computed(() => `esp-p4-cam-zoom-${camera.value.index}`)
const modeStorageKey = computed(() => `esp-p4-cam-mode-${camera.value.index}`)

const imgSrc = ref<string>(LOADING_IMAGE_SRC)
const settingsDialog = ref<boolean>(false)
const selectedImageFormatId = ref<number | string>(camera.value.currentImageFormat)
const selectedQuality = ref<number>(camera.value.currentQuality ?? 80)
const zoomPercent = ref<number>(100)
const displayMode = ref<'fit' | 'fill'>('fit')
const settingsSaving = ref<boolean>(false)
const saveStatusSnackbar = ref<boolean>(false)
const saveStatusSnackbarText = ref<string>("")
const recordStatusSnackbar = ref<boolean>(false)
const recordStatusSnackbarText = ref<string>("")
const recordStatusSnackbarColor = ref<string>("success")
const retryTimeoutId = ref<ReturnType<typeof setTimeout> | null>(null)
const reloadTimeoutId = ref<ReturnType<typeof setTimeout> | null>(null)
const recordingStopTimeoutId = ref<ReturnType<typeof setTimeout> | null>(null)
const recordingCountdownId = ref<ReturnType<typeof setInterval> | null>(null)
const recordingDrawIntervalId = ref<ReturnType<typeof setInterval> | null>(null)
const isRecording = ref<boolean>(false)
const recordingSecondsLeft = ref<number>(RECORD_SECONDS)
const recordCanvas = ref<HTMLCanvasElement | null>(null)
let mediaRecorder: MediaRecorder | null = null
let recordedChunks: Blob[] = []

const selectedFormat = computed(() => {
  return camera.value.imageFormats.find(format => format.id === selectedImageFormatId.value)
})

const frameAspectRatio = computed(() => {
  const width = camera.value.currentResolution?.width ?? 1
  const height = camera.value.currentResolution?.height ?? 1

  if (width <= 0 || height <= 0) {
    return 1
  }

  return width / height
})

const cardStyle = computed(() => {
  const width = camera.value.currentResolution?.width ?? 1
  const height = camera.value.currentResolution?.height ?? 1
  const isPortrait = height > width

  return {
    '--camera-card-max-width': isPortrait ? '560px' : '1100px',
  }
})

const onImageError = () => {
  if (imgSrc.value === LOADING_IMAGE_SRC) return;

  if (retryTimeoutId.value) {
    clearTimeout(retryTimeoutId.value)
  }

  retryTimeoutId.value = setTimeout(() => {
    reloadCameraSrc(1000)
    retryTimeoutId.value = null
  }, 3000)
}

const realCameraUrl = computed(() => {
  let port: number | null = null;
  let path: string | null = null;

  if (camera.value.src.startsWith(':')) {
    const [, portStr, pathStr] = camera.value.src.split(/[:\/]/);
    port = Number(portStr);
    path = pathStr;

    const realUrl = new URL(path, location.href);
    realUrl.port = port.toString();
    return realUrl.toString();
  } else {
    path = camera.value.src;
    return new URL(path, location.href).toString();
  }
})

const reloadCameraSrc = (ms: number = 100) => {
  imgSrc.value = LOADING_IMAGE_SRC;

  if (reloadTimeoutId.value) {
    clearTimeout(reloadTimeoutId.value)
    reloadTimeoutId.value = null
  }

  reloadTimeoutId.value = setTimeout(() => {
    imgSrc.value = realCameraUrl.value;
    reloadTimeoutId.value = null;
  }, ms);
}

const captureFrame = () => {
  const url = `/api/capture_image?source=${camera.value.index}`;

  const link = document.createElement('a');
  link.href = url;
  link.download = `camera_${camera.value.index}_image.jpg`;
  document.body.appendChild(link);
  link.click();
  document.body.removeChild(link);
}

const resetZoom = () => {
  zoomPercent.value = 100;
  displayMode.value = 'fit';
}

const captureRawFrame = () => {
  const url = `/api/capture_binary?source=${camera.value.index}`;

  const link = document.createElement('a');
  link.href = url;
  link.download = `camera_${camera.value.index}_raw.bin`;
  document.body.appendChild(link);
  link.click();
  document.body.removeChild(link);
}

const showRecordStatus = (text: string, color: string = "success") => {
  recordStatusSnackbarText.value = text
  recordStatusSnackbarColor.value = color
  recordStatusSnackbar.value = true
}

const getPreviewImageElement = (): HTMLImageElement | null => {
  const root = document.getElementById(`cam-${props.camNum}-v-img`)
  return root?.querySelector('img.v-img__img') ?? null
}

const getSupportedRecordingMimeType = (): string => {
  if (typeof MediaRecorder === 'undefined') {
    return ''
  }

  const candidates = [
    'video/webm;codecs=vp9',
    'video/webm;codecs=vp8',
    'video/webm',
    'video/mp4',
  ]

  return candidates.find(type => MediaRecorder.isTypeSupported(type)) ?? ''
}

const waitForPreviewFrame = async (timeoutMs: number = 8000): Promise<HTMLImageElement> => {
  const startedAt = Date.now()

  return new Promise((resolve, reject) => {
    const tick = () => {
      const image = getPreviewImageElement()
      if (image && image.complete && image.naturalWidth > 0 && image.naturalHeight > 0) {
        resolve(image)
        return
      }

      if (Date.now() - startedAt > timeoutMs) {
        reject(new Error('Video stream is not ready'))
        return
      }

      window.setTimeout(tick, 120)
    }

    tick()
  })
}

const clearRecordingTimers = () => {
  if (recordingStopTimeoutId.value) {
    clearTimeout(recordingStopTimeoutId.value)
    recordingStopTimeoutId.value = null
  }

  if (recordingCountdownId.value) {
    clearInterval(recordingCountdownId.value)
    recordingCountdownId.value = null
  }

  if (recordingDrawIntervalId.value) {
    clearInterval(recordingDrawIntervalId.value)
    recordingDrawIntervalId.value = null
  }
}

const finishRecording = () => {
  clearRecordingTimers()
  isRecording.value = false
  recordingSecondsLeft.value = RECORD_SECONDS
}

const recordClip = async () => {
  if (isRecording.value) {
    return
  }

  if (typeof MediaRecorder === 'undefined') {
    showRecordStatus('This browser does not support video recording', 'error')
    return
  }

  const mimeType = getSupportedRecordingMimeType()
  if (!mimeType) {
    showRecordStatus('No supported recording format was found in this browser', 'error')
    return
  }

  let image: HTMLImageElement
  try {
    image = await waitForPreviewFrame()
  } catch (error) {
    showRecordStatus(error instanceof Error ? error.message : 'Video stream is not ready', 'error')
    return
  }

  const canvas = recordCanvas.value
  if (!canvas) {
    showRecordStatus('Recording canvas is not ready', 'error')
    return
  }

  const width = image.naturalWidth || camera.value.currentResolution?.width || 800
  const height = image.naturalHeight || camera.value.currentResolution?.height || 600
  canvas.width = width
  canvas.height = height

  const context = canvas.getContext('2d')
  if (!context) {
    showRecordStatus('Failed to initialize recording context', 'error')
    return
  }

  recordedChunks = []
  recordingSecondsLeft.value = RECORD_SECONDS
  isRecording.value = true
  context.drawImage(image, 0, 0, width, height)

  const stream = canvas.captureStream(RECORD_FPS)

  try {
    mediaRecorder = new MediaRecorder(stream, { mimeType })
  } catch {
    finishRecording()
    showRecordStatus('Failed to start browser recorder', 'error')
    return
  }

  mediaRecorder.ondataavailable = (event) => {
    if (event.data.size > 0) {
      recordedChunks.push(event.data)
    }
  }

  mediaRecorder.onerror = () => {
    finishRecording()
    showRecordStatus('Recording failed in browser', 'error')
  }

  mediaRecorder.onstop = () => {
    const fileType = mimeType.includes('mp4') ? 'mp4' : 'webm'
    const blob = new Blob(recordedChunks, { type: mimeType })

    finishRecording()

    if (blob.size === 0) {
      showRecordStatus('Recording finished, but no video data was generated', 'error')
      return
    }

    const link = document.createElement('a')
    link.href = URL.createObjectURL(blob)
    link.download = `camera_${camera.value.index}_video_${Date.now()}.${fileType}`
    document.body.appendChild(link)
    link.click()
    document.body.removeChild(link)
    URL.revokeObjectURL(link.href)
    showRecordStatus('15-second video saved')
  }

  recordingDrawIntervalId.value = setInterval(() => {
    const currentImage = getPreviewImageElement()
    if (currentImage && currentImage.complete && currentImage.naturalWidth > 0) {
      context.drawImage(currentImage, 0, 0, width, height)
    }
  }, 1000 / RECORD_FPS)

  recordingCountdownId.value = setInterval(() => {
    if (recordingSecondsLeft.value > 0) {
      recordingSecondsLeft.value -= 1
    }
  }, 1000)

  recordingStopTimeoutId.value = setTimeout(() => {
    if (mediaRecorder && mediaRecorder.state === 'recording') {
      mediaRecorder.stop()
    }
  }, RECORD_SECONDS * 1000)

  mediaRecorder.start(1000)
  showRecordStatus('Recording started')
}

const saveSettings = async () => {
  imgSrc.value = LOADING_IMAGE_SRC;
  settingsSaving.value = true;
  await fetch('/api/set_camera_config', {
    method: 'POST',
    body: JSON.stringify({
      index: camera.value.index,
      image_format: selectedImageFormatId.value,
      jpeg_quality: selectedQuality.value
    })
  }).then(res => {
    if (res.ok) {
      saveStatusSnackbar.value = true;
      saveStatusSnackbarText.value = "Settings saved";
    } else {
      saveStatusSnackbar.value = true;
      saveStatusSnackbarText.value = "Failed to save settings";
    }
  }).finally(() => {
    settingsSaving.value = false;
    settingsDialog.value = false;
  })

  await mainStore.updateCameraStatus()
  reloadCameraSrc(250)
}

watch(realCameraUrl, (newUrl) => {
  if (imgSrc.value !== LOADING_IMAGE_SRC) {
    imgSrc.value = newUrl;
  }
})

watch(selectedImageFormatId, () => {
  if (selectedFormat.value?.quality) {
    selectedQuality.value = selectedFormat.value?.quality.default ?? selectedFormat.value?.quality.max ?? 90;
  }
})

watch(() => camera.value.currentImageFormat, value => {
  selectedImageFormatId.value = value
})

watch(() => camera.value.currentQuality, value => {
  if (typeof value === 'number') {
    selectedQuality.value = value
  }
})

onMounted(() => {
  const savedZoom = Number(localStorage.getItem(zoomStorageKey.value) ?? "100");
  const savedMode = localStorage.getItem(modeStorageKey.value);

  zoomPercent.value = Number.isFinite(savedZoom) && savedZoom >= 100 && savedZoom <= 300 ? savedZoom : 100;
  displayMode.value = savedMode === 'fill' ? 'fill' : 'fit';
  reloadCameraSrc();
})

watch(zoomPercent, value => {
  localStorage.setItem(zoomStorageKey.value, String(value));
})

watch(displayMode, value => {
  localStorage.setItem(modeStorageKey.value, value);
})

onUnmounted(() => {
  if (mediaRecorder && mediaRecorder.state === 'recording') {
    mediaRecorder.stop()
  }

  clearRecordingTimers()

  if (retryTimeoutId.value) {
    clearTimeout(retryTimeoutId.value)
    retryTimeoutId.value = null
  }
})
</script>

<style scoped>
.camera-frame {
  overflow: hidden;
  background: #000;
}

.camera-card {
  width: min(100%, var(--camera-card-max-width, 1100px));
  margin-left: auto;
  margin-right: auto;
}

.camera-frame :deep(.v-img__img) {
  transform: scale(var(--camera-zoom, 1));
  transform-origin: center center;
}

.camera-frame--fit :deep(.v-img__img) {
  object-fit: contain !important;
}

.camera-frame--fill :deep(.v-img__img) {
  object-fit: cover !important;
}

.zoom-slider-wrap {
  min-width: 220px;
  flex: 1 1 240px;
}

.zoom-label {
  min-width: 44px;
  display: inline-block;
  text-align: right;
}

.record-canvas {
  display: none;
}

.recording-badge {
  color: #c62828;
  font-weight: 700;
  font-size: 0.95rem;
}
</style>
