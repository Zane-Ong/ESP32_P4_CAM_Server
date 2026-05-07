# ESP_P4_CAM_Server

基于 `ESP-IDF` 的 `ESP32-P4` 摄像头网页服务项目。  
运行在 `Waveshare ESP32-P4-WIFI6 Kit A` 上，支持网页预览、拍照、录像、WebSocket 事件与本地 HTML 客户端自动保存文件。

> 本项目已按 `ESP32-P4 + ESP32-C6(SDIO)` 官方推荐结构实现，不使用 Arduino `esp_camera` 旧方案。

---

## 1. 当前支持功能

- 自动连接 Wi-Fi，并打印访问地址
- 浏览器访问板子 IP 进行实时预览
- 网页调整分辨率 / JPEG 质量 / 显示模式 / 缩放
- **单按键逻辑（GPIO46）：**
  - 短按拍照
  - 长按录制（默认 25 秒）
- 本地 HTML 客户端自动保存照片 / 视频
- 支持 WebSocket 事件通知（按钮触发浏览器动作）
- 兼容 `/capture` `/status` `/control` `/stream` 接口
- 支持 `mDNS` 访问，例如：`http://esp-web.local/`

---

## 2. 硬件要求

| 组件 | 说明 |
|------|------|
| 主板 | Waveshare ESP32-P4-WIFI6 Kit A |
| 主控 | ESP32-P4 |
| 协处理器 | ESP32-C6 |
| 摄像头 | OV5647 |
| 框架 | ESP-IDF v5.5.4 |

---

## 3. 项目目录结构

- [`main/simple_video_server_example.c`](./main/simple_video_server_example.c)  
  核心代码（摄像头、HTTP/WebSocket、按键、SD 卡录像等）
- [`main/Kconfig.projbuild`](./main/Kconfig.projbuild)  
  配置入口（按钮 GPIO / 录像时长 / JPEG 质量）
- [`frontend/`](./frontend)  
  板载网页（Vue + Vite）
- [`tools/CameraClientServer.html`](./tools/CameraClientServer.html)  
  本地浏览器客户端（自动保存照片/视频）
- [`partitions.csv`](./partitions.csv)  
  分区表
- [`sdkconfig.defaults`](./sdkconfig.defaults)  
  默认配置模板

---

## 4. 主要接口列表

### HTTP 接口
| 路径 | 说明 |
|------|------|
| `/` | 主网页 |
| `/api/get_camera_info` | 相机参数 JSON |
| `/api/set_camera_config` | 设置分辨率 / 画质 |
| `/api/latest_photo` | 下载最新照片 |
| `/api/latest_video` | 下载最新录像（mjpeg） |
| `/capture` | 兼容拍照接口 |
| `/status` | 兼容状态接口 |
| `/control` | 兼容配置接口 |
| `/stream` | 兼容 MJPEG 推流 |

### WebSocket
| 地址 | 说明 |
|------|------|
| `ws://<IP>/ws` | 按键事件 |
| `ws://<IP>:81/ws_video` | 视频实时推送 |

---

## 5. 按键逻辑（已和代码一致）

> **当前只使用一个按键 GPIO46（低电平触发）**

| 操作 | 行为 |
|------|------|
| 短按（<2s） | 拍照 |
| 长按（>=2s） | 开始录像（默认 25 秒） |

按键端接 GPIO46，另一端接 GND。使用内部上拉。

---

## 6. 默认配置（来自 sdkconfig）

| 项目 | 默认值 |
|------|--------|
| Wi-Fi SSID | `iPhone4S` |
| Wi-Fi 密码 | `12345678` |
| 录像时长 | 25 秒 |
| 录像质量 | 70 |
| 预览质量 | 10 |
| 拍照质量 | 95 |
| mDNS Host | `esp-web` |
| 按键 GPIO | 46 |
| 按键消抖 | 40 ms |

> 可在 `idf.py menuconfig` 或 `sdkconfig.defaults` 中修改

---

## 7. 快速上手（推荐流程）

### Step 1. 配置 Wi-Fi
方法 A（menuconfig）：
```bash
idf.py menuconfig
```

方法 B（直接修改配置）：
```
CONFIG_EXAMPLE_WIFI_SSID="你的热点名"
CONFIG_EXAMPLE_WIFI_PASSWORD="你的密码"
```

### Step 2. 编译
```bash
idf.py build
```

### Step 3. 烧录并监控
```bash
idf.py -p COM10 flash monitor
```

### Step 4. 查看串口日志中的地址
示例输出：
```
Web UI: http://192.168.xxx.xxx/
Legacy capture: http://192.168.xxx.xxx/capture
MJPEG stream: http://192.168.xxx.xxx:81/stream
```

---

## 8. 使用板载网页

浏览器打开：
```
http://<板子IP>/
```

支持操作：
- 查看预览
- 选择分辨率
- 调整 JPEG 质量
- Fit / Fill 显示模式
- 画面缩放（仅影响浏览器显示）

---

## 9. 使用本地 HTML 自动保存文件（推荐）

### Step 1. 打开本地页面
双击：
```
tools/CameraClientServer.html
```

### Step 2. 输入板子 IP
例如：
```
192.168.219.245
```

### Step 3. 点击 Connect
成功后会：
- 建立事件 WebSocket
- 读取相机配置

### Step 4. 使用功能按钮
- Start Preview
- Capture Photo
- Record 15s Video
- Apply Camera Settings

> 当按硬件按键时，浏览器会自动下载照片或录像。

---

## 10. SD 卡录像说明（与代码一致）

- 录像文件写入 SD 卡
- 录制时先写入 `.tmp`，完成后自动重命名为 `.mjpeg`
- 下载接口：
  - `/api/latest_video`

---

## 11. 常见问题

### Q1: 串口没打印 IP
- Wi-Fi 配置是否正确
- 是否 2.4GHz
- 是否识别到 ESP32-C6

### Q2: 预览卡顿
- 降低分辨率
- 降低 JPEG 质量
- 使用 Fit + 100%

### Q3: 本地 HTML 无法连接
- 是否同一局域网
- IP 是否输入正确
- `ws://<IP>/ws` 是否可访问

### Q4: 按键没反应
- GPIO46 是否接对
- 是否接地
- 低电平触发
- 浏览器是否已连接事件通道

---

## 12. 常用命令汇总

```bash
idf.py build
idf.py -p COM10 flash
idf.py -p COM10 flash monitor
idf.py -p COM10 monitor
idf.py menuconfig
```

---

## 13. 建议阅读顺序

1. [`main/simple_video_server_example.c`](./main/simple_video_server_example.c)
2. [`main/Kconfig.projbuild`](./main/Kconfig.projbuild)
3. [`tools/CameraClientServer.html`](./tools/CameraClientServer.html)
4. [`frontend/`](./frontend)

---

## 14. ESP-IDF 安装参考

参考微雪文档：

- [ESP-IDF 开发 | 微雪文档平台](https://docs.waveshare.net/ESP32-P4-WIFI6/Development-Environment-Setup-IDF)  
- [Bilibili 视频教程](https://www.bilibili.com/video/BV1EPisBWEUX?spm_id_from=333.788.videopod.episodes&vd_source=16653787726583a107f817924f9f09fe&p=6)

---

## 15. 建议先完成这些动作

- 串口打印 IP
- 浏览器能看到视频
- 修改分辨率成功
- 拍照下载成功
- 录像下载成功
- 按键触发成功
