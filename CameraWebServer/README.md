# ESP32_Cam_server

基于 ESP32-CAM 的局域网相机控制项目。电脑/平板作为客户端，ESP32-CAM 作为服务端，可实现：

- 拍照并保存到本地
- 实时视频预览
- 15 秒视频录制并保存到本地
- 支持网页按钮和硬件按键双控制

## 1. 项目结构

```text
ESP32_Cam_server
├─ CameraWebServer/              # ESP32 固件代码（Arduino）
│  ├─ CameraWebServer.ino        # 主程序
│  ├─ app_httpd.cpp              # 相机 HTTP 服务
│  ├─ board_config.h             # 摄像头型号选择
│  └─ camera_pins.h              # 各型号引脚定义
├─ CameraClinetServer/
│  └─ ClinetServer.html          # 网页客户端
├─ image/README/                 # README 演示图片
└─ 开发环境安装教程/              # 安装与注意事项
```

## 2. 运行效果

烧录并启动后，串口监视器会输出设备 IP。
在同一局域网下，用电脑或平板打开 `CameraClinetServer/ClinetServer.html`，输入该 IP 后连接。

![运行界面1](image/README/1773664593936.png)

连接成功后可进行拍照、预览、录像：

![运行界面2](image/README/1773664666757.png)

## 3. 环境准备

### 3.1 硬件

- ESP32-WROVER_KIT 开发板（带 PSRAM）
- Micro-USB
- 摄像头模组(ov5640)
- 3 个按键（接 IO12 / IO13 / IO14）

### 3.2 软件

- Arduino IDE
- ESP32 开发板支持包（Espressif）

参考教程：
https://blog.csdn.net/qq_34426854/article/details/145853077?spm=1001.2014.3001.5502

## 4. 快速上手（推荐先看）

### 步骤 1：修改 Wi-Fi

打开 `CameraWebServer/CameraWebServer.ino`，修改：

```cpp
const char *ssid = "你的WiFi名称";
const char *password = "你的WiFi密码";
```

### 步骤 2：确认摄像头型号

打开 `CameraWebServer/board_config.h`，只保留你实际使用的型号宏。
当前默认是：

```cpp
#define CAMERA_MODEL_WROVER_KIT
```

摄像头型号选错会导致黑屏、花屏或初始化失败。

### 步骤 3：Arduino IDE 选择参数并烧录

- 选择正确的开发板型号
- 选择对应 COM 口
- 选择带足够 APP 空间的分区方案（ `custom`，至少 3MB APP）
- 烧录程序

参考图：

![烧录参数示例](image/README/1773663885784.png)

### 步骤 4：获取 IP 并连接网页

1. 打开串口监视器（115200 波特率）
2. 按 `RST` 重启，等待输出 `Camera Ready! Use 'http://xxx.xxx.xxx.xxx' to connect`
3. 在同一网络设备上打开 `CameraClinetServer/ClinetServer.html`
4. 输入串口中的 IP，点击“连接硬件”

## 5. 控制方式说明

### 5.1 网页端

- 拍照保存：抓拍一张 JPG 并下载
- 长按预览：按住开始预览，松开停止
- 录制 15 秒：自动录制并下载为 webm

### 5.2 硬件按键（可选）

- IO12：拍照
- IO13：长按预览（按下开始，松开停止）
- IO14：录制 15 秒

## 6. 常见问题（FAQ）

### Q1：烧录失败怎么办？

A：部分板子需要进入下载模式，通常是按住 `BOOT` 再按一下 `RST`（l两个键同时按住）。
如仍失败，检查供电和串口驱动。

### Q2：视频卡顿怎么办？

A：优先降低分辨率和帧率。可在相机控制页面把帧率相关参数从 20 调到 15 后保存，并确认摄像头型号选择正确。

参考图：

![卡顿调参示例](image/README/1773664811860.png)

### Q3：如何调整画面效果（亮度、对比度、翻转等）？

A：先在浏览器访问设备 IP（ESP32 内置控制页），把参数调到满意后记录下来，再写回 `CameraWebServer/CameraWebServer.ino` 的 `setup()` 中作为默认值。

### Q4：为什么网页连接不上？

A：请依次检查：

- 电脑/平板与 ESP32 是否在同一局域网
- 串口中 IP 是否正确
- 防火墙是否阻止浏览器访问局域网端口
- 是否正确输入了 IP（不带 `http://` 也可）

## 7. 已知注意事项

- `CameraClinetServer` 目录名和 `ClinetServer.html` 文件名均为历史命名，请保持原路径避免引用失效。
- 使用高分辨率时建议使用带 PSRAM 的模组，否则可能出现帧率低或图像不完整。
