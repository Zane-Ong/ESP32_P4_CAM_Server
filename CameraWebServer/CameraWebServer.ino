#include "esp_camera.h"
#include <WiFi.h>
#include <WebSocketsServer.h> // 引入 WebSocket 库

// ===========================
// Select camera model in board_config.h
// ===========================
#include "board_config.h"
const int BTN_PHOTO = 12;  // 拍照按键
const int BTN_STREAM = 13; // 预览按键
const int BTN_RECORD = 14; // 录像按键

// 实例化 WebSocket 服务器，运行在 82 端口
WebSocketsServer webSocket = WebSocketsServer(82);

// 按键防抖和状态记录变量
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50; // 50毫秒防抖

// 记录上一次的引脚电平
int lastPhotoState = HIGH;
int lastStreamState = HIGH;
int lastRecordState = HIGH;

// 记录确认后的按键状态
int photoState = HIGH;
int streamState = HIGH;
int recordState = HIGH;
// ===========================
// Enter your WiFi credentials
// ===========================
const char *ssid = "Yukino";
const char *password = "00000000";

void startCameraServer();
void setupLedFlash();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV5640_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash();
#endif

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  Serial.print("WiFi connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  startCameraServer();

  // 1. 初始化按键引脚为上拉输入 (按下为 LOW，松开为 HIGH)
  pinMode(BTN_PHOTO, INPUT_PULLUP);
  pinMode(BTN_STREAM, INPUT_PULLUP);
  pinMode(BTN_RECORD, INPUT_PULLUP);

  // 2. 启动 WebSocket 服务器
  webSocket.begin();
  Serial.println("WebSocket 服务器已启动，端口: 82");

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");


  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");
}

void loop() {
  // Do nothing. Everything is done in another task by the web server
  // 必须不断调用这个函数，WebSocket 才能保持连接和收发数据
  webSocket.loop();

  // 读取当前三个引脚的真实电平
  int readingPhoto = digitalRead(BTN_PHOTO);
  int readingStream = digitalRead(BTN_STREAM);
  int readingRecord = digitalRead(BTN_RECORD);

  // 如果任何一个按键的电平发生了变化，重置防抖计时器
  if (readingPhoto != lastPhotoState || readingStream != lastStreamState || readingRecord != lastRecordState) {
    lastDebounceTime = millis();
  }

  // 如果电平稳定时间超过了设定的防抖延迟 (50ms)
  if ((millis() - lastDebounceTime) > debounceDelay) {
    
    // ==========================================
    // 功能 1：拍照按键 (按下时触发一次)
    // ==========================================
    if (readingPhoto != photoState) {
      photoState = readingPhoto;
      if (photoState == LOW) { // 检测到按下
        Serial.println("硬件按键：触发拍照");
        webSocket.broadcastTXT("TAKE_PHOTO"); 
      }
    }

    // ==========================================
    // 功能 2：长按预览按键 (按下开始，松开停止)
    // ==========================================
    if (readingStream != streamState) {
      streamState = readingStream;
      if (streamState == LOW) { // 检测到按下
        Serial.println("硬件按键：长按开始预览");
        webSocket.broadcastTXT("STREAM_START");
      } else {                  // 检测到松开
        Serial.println("硬件按键：松开停止预览");
        webSocket.broadcastTXT("STREAM_STOP");
      }
    }

    // ==========================================
    // 功能 3：录像按键 (按下时触发一次)
    // ==========================================
    if (readingRecord != recordState) {
      recordState = readingRecord;
      if (recordState == LOW) { // 检测到按下
        Serial.println("硬件按键：触发录制 15 秒");
        webSocket.broadcastTXT("RECORD_START");
      }
    }
  }

  // 保存当前的电平，用于下一次循环比较
  lastPhotoState = readingPhoto;
  lastStreamState = readingStream;
  lastRecordState = readingRecord;
  
  // 极短的延时，防止 watchdog 狗叫，同时不影响视频流
  delay(2);
}
