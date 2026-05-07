/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: ESPRESSIF MIT
 */

#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>
#include <sys/errno.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "cJSON.h"
#include "esp_event.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_http_client.h"
#include "nvs_flash.h"
#include "esp_check.h"
#include "esp_http_server.h"
#include "driver/gpio.h"
#include "protocol_examples_common.h"
#include "mdns.h"
#include "lwip/inet.h"
#include "lwip/apps/netbiosns.h"
#include "example_video_common.h"
#include "esp_wifi.h"
#include "LED.h"
#include "esp_vfs_fat.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
#include "esp32_p4_function_ev_board.h"
#include "mbedtls/base64.h"  // 用于Base64编码
#include "cJSON.h"
#include <dirent.h>      // 用于 opendir/closedir
#include <errno.h>       // 用于 errno
#include <sys/stat.h>   // for stat()
#include <unistd.h>     // for write(), close()

#define EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER  CONFIG_EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER

#define EXAMPLE_JPEG_ENC_QUALITY            CONFIG_EXAMPLE_JPEG_COMPRESSION_QUALITY

#define EXAMPLE_MDNS_INSTANCE               CONFIG_EXAMPLE_MDNS_INSTANCE
#define EXAMPLE_MDNS_HOST_NAME              CONFIG_EXAMPLE_MDNS_HOST_NAME
#define EXAMPLE_STREAM_MAX_FPS              45
#define EXAMPLE_RECORD_DURATION_SEC         CONFIG_EXAMPLE_RECORD_DURATION_SEC
#define EXAMPLE_BUTTON_DEBOUNCE_MS          CONFIG_EXAMPLE_BUTTON_DEBOUNCE_MS
#define EXAMPLE_CAPTURE_BUTTON_GPIO         CONFIG_EXAMPLE_BUTTON_GPIO_CAPTURE
#define EXAMPLE_PREVIEW_BUTTON_GPIO         CONFIG_EXAMPLE_BUTTON_GPIO_PREVIEW
#define EXAMPLE_RECORD_BUTTON_GPIO          CONFIG_EXAMPLE_BUTTON_GPIO_RECORD
#define EXAMPLE_MAX_EVENT_CLIENTS           2

#define EXAMPLE_PART_BOUNDARY               CONFIG_EXAMPLE_HTTP_PART_BOUNDARY
#define MOUNT_POINT "/sdcard"


static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" EXAMPLE_PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" EXAMPLE_PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

static sdmmc_card_t *bsp_sdcard = NULL;           // uSD card handle
static sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL; // SD LDO handle

extern const uint8_t index_html_gz_start[] asm("_binary_index_html_gz_start");
extern const uint8_t index_html_gz_end[] asm("_binary_index_html_gz_end");
extern const uint8_t loading_jpg_gz_start[] asm("_binary_loading_jpg_gz_start");
extern const uint8_t loading_jpg_gz_end[] asm("_binary_loading_jpg_gz_end");
extern const uint8_t favicon_ico_gz_start[] asm("_binary_favicon_ico_gz_start");
extern const uint8_t favicon_ico_gz_end[] asm("_binary_favicon_ico_gz_end");
extern const uint8_t assets_index_js_gz_start[] asm("_binary_index_js_gz_start");
extern const uint8_t assets_index_js_gz_end[] asm("_binary_index_js_gz_end");
extern const uint8_t assets_index_css_gz_start[] asm("_binary_index_css_gz_start");
extern const uint8_t assets_index_css_gz_end[] asm("_binary_index_css_gz_end");

/**
 * @brief Web cam control structure
 */
typedef struct web_cam_video {
    int fd;
    uint8_t index;
    char dev_name[32];

    example_encoder_handle_t encoder_handle;
    uint8_t *jpeg_out_buf;
    uint32_t jpeg_out_size;

    uint8_t *buffer[EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER];
    uint32_t buffer_size;

    uint32_t width;
    uint32_t height;
    uint32_t pixel_format;
    uint8_t jpeg_quality;

    uint32_t frame_rate;

    SemaphoreHandle_t sem;
    SemaphoreHandle_t io_lock;

    struct web_cam_image_format_option *format_options;
    uint32_t format_option_count;
    int current_format_index;

    uint32_t support_control_jpeg_quality   : 1;
    uint32_t stream_on                      : 1;
    
    /* 软件裁剪相关字段 */
    uint32_t software_crop_enabled;
    uint32_t crop_src_width;
    uint32_t crop_src_height;
    uint32_t crop_left;
    uint32_t crop_top;
    uint32_t crop_width;
    uint32_t crop_height;
    
    /* 画质管理字段 */
    uint8_t preview_quality;
    uint8_t capture_quality;
    uint8_t record_quality;
    uint8_t current_quality;
} web_cam_video_t;

static QueueHandle_t s_frame_queue = NULL;
static TaskHandle_t s_sd_write_task_handle = NULL;

#define RECORD_HEADER_SIZE 256

// 录像相关变量
static bool s_recording = false;
static char s_record_filename[256];
static char s_record_tmp_filename[256];
static int s_record_fd = -1;
static int64_t s_record_start_us = 0;
static int64_t s_record_duration_us = 0;
static SemaphoreHandle_t s_record_mutex = NULL;
static web_cam_video_t *s_recording_video = NULL;
static TaskHandle_t s_record_stop_task_handle = NULL;
static volatile bool s_record_finalize_pending = false;

// 异步写入队列
typedef struct {
    uint8_t *data;
    uint32_t size;
    uint32_t frame_index;
    bool is_exit;
    uint32_t avg_fps_x1000;
    uint32_t frame_count;
    uint32_t width;
    uint32_t height;
    uint8_t quality;
} frame_buffer_t;

// 画质配置
#define PREVIEW_JPEG_QUALITY    10
#define CAPTURE_JPEG_QUALITY    95
#define RECORD_JPEG_QUALITY     70

static void crop_rgb565_optimized(const uint8_t *src, int src_width, int src_height,
                                  uint8_t *dst, int crop_x, int crop_y,
                                  int crop_width, int crop_height)
{
    const int bytes_per_pixel = 2;  // RGB565 每像素2字节
    const int src_stride = src_width * bytes_per_pixel;
    const int dst_stride = crop_width * bytes_per_pixel;
    
    const uint8_t *src_row = src + (crop_y * src_stride) + (crop_x * bytes_per_pixel);
    uint8_t *dst_row = dst;
    
    for (int y = 0; y < crop_height; y++) {
        memcpy(dst_row, src_row, dst_stride);
        src_row += src_stride;
        dst_row += dst_stride;
    }
}

static esp_err_t write_record_header(int fd, uint32_t avg_fps_x1000, uint32_t frame_count,
                                     uint32_t width, uint32_t height, uint8_t quality)
{
    char header[RECORD_HEADER_SIZE];
    char text[RECORD_HEADER_SIZE];
    int text_len;

    memset(header, ' ', sizeof(header));
    text_len = snprintf(text, sizeof(text),
                        "#MJPEG\nAVG_FPS=%lu.%03lu\nWIDTH=%lu\nHEIGHT=%lu\nQUALITY=%u\nFRAMES=%lu\n",
                        (unsigned long)(avg_fps_x1000 / 1000),
                        (unsigned long)(avg_fps_x1000 % 1000),
                        (unsigned long)width,
                        (unsigned long)height,
                        (unsigned int)quality,
                        (unsigned long)frame_count);
    if (text_len <= 0) {
        return ESP_FAIL;
    }

    if (text_len >= (int)sizeof(header)) {
        return ESP_FAIL;
    }

    memcpy(header, text, text_len);
    header[sizeof(header) - 1] = '\n';

    if (lseek(fd, 0, SEEK_SET) < 0) {
        return ESP_FAIL;
    }

    if (write(fd, header, sizeof(header)) != sizeof(header)) {
        return ESP_FAIL;
    }

    return ESP_OK;
}

typedef struct web_cam {
    uint8_t video_count;
    web_cam_video_t video[0];
} web_cam_t;

typedef struct web_cam_video_config {
    const char *dev_name;
    uint32_t buffer_count;
} web_cam_video_config_t;

typedef struct request_desc {
    int index;
} request_desc_t;

typedef struct web_cam_image_format_option {
    uint32_t pixel_format;
    uint32_t width;
    uint32_t height;
    uint32_t frame_interval_num;
    uint32_t frame_interval_den;
} web_cam_image_format_option_t;

typedef struct camera_runtime_state {
    web_cam_t *web_cam;
    SemaphoreHandle_t lock;
    TimerHandle_t record_timer;
    bool preview_requested;
    bool recording_active;
    bool capture_in_progress;
    bool stream_enabled;
} camera_runtime_state_t;

typedef struct event_client_registry {
    httpd_handle_t server;
    SemaphoreHandle_t lock;
    int fds[EXAMPLE_MAX_EVENT_CLIENTS];
} event_client_registry_t;

static const char *TAG = "example";
static camera_runtime_state_t s_runtime = {0};
static event_client_registry_t s_event_clients = {0};

static esp_err_t init_web_cam_video(web_cam_video_t *video, const web_cam_video_config_t *config, int index,
                                    const web_cam_image_format_option_t *requested_format);
static esp_err_t deinit_web_cam_video(web_cam_video_t *video);
static bool is_valid_web_cam(web_cam_video_t *video);
static esp_err_t set_runtime_stream_enabled(bool enable);
static esp_err_t apply_runtime_stream_policy(void);
static bool runtime_should_stream(void);
static esp_err_t broadcast_button_event(const char *event_name);
static esp_err_t video_set_quality(web_cam_video_t *video, uint8_t quality);
static esp_err_t capture_photo_to_sdcard(web_cam_video_t *video, const char *filename);
static esp_err_t start_recording(web_cam_video_t *video, const char *filename, uint32_t duration_sec);
static esp_err_t stop_recording(void);
static void record_stop_task(void *arg);
static void independent_recording_task(void *arg);
static esp_err_t ws_video_handler(httpd_req_t *req);


static void log_access_urls(void)
{
    esp_netif_t *netif = get_example_netif();
    esp_netif_ip_info_t ip_info = {0};

    if (netif == NULL) {
        ESP_LOGW(TAG, "No active network interface found, skip access URL logging");
        return;
    }

    if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK || ip_info.ip.addr == 0) {
        ESP_LOGW(TAG, "Network interface has no IPv4 address yet");
        return;
    }

    ESP_LOGI(TAG, "Web UI: http://" IPSTR "/", IP2STR(&ip_info.ip));
    ESP_LOGI(TAG, "Legacy capture: http://" IPSTR "/capture", IP2STR(&ip_info.ip));
    ESP_LOGI(TAG, "Legacy status: http://" IPSTR "/status", IP2STR(&ip_info.ip));
    ESP_LOGI(TAG, "MJPEG stream: http://" IPSTR ":81/stream", IP2STR(&ip_info.ip));
    ESP_LOGI(TAG, "mDNS URL: http://%s.local/", EXAMPLE_MDNS_HOST_NAME);
}

static esp_err_t init_event_client_registry(void)
{
    if (s_event_clients.lock == NULL) {
        s_event_clients.lock = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(s_event_clients.lock != NULL, ESP_ERR_NO_MEM, TAG, "failed to create event client lock");
    }

    for (size_t i = 0; i < EXAMPLE_MAX_EVENT_CLIENTS; i++) {
        s_event_clients.fds[i] = -1;
    }
    return ESP_OK;
}

static void register_event_client(httpd_handle_t server, int fd)
{
    if (fd < 0 || s_event_clients.lock == NULL) {
        return;
    }

    if (xSemaphoreTake(s_event_clients.lock, portMAX_DELAY) != pdPASS) {
        return;
    }

    s_event_clients.server = server;
    for (size_t i = 0; i < EXAMPLE_MAX_EVENT_CLIENTS; i++) {
        if (s_event_clients.fds[i] == fd) {
            xSemaphoreGive(s_event_clients.lock);
            return;
        }
    }

    for (size_t i = 0; i < EXAMPLE_MAX_EVENT_CLIENTS; i++) {
        if (s_event_clients.fds[i] == -1) {
            s_event_clients.fds[i] = fd;
            ESP_LOGI(TAG, "event client connected on socket %d", fd);
            xSemaphoreGive(s_event_clients.lock);
            return;
        }
    }

    xSemaphoreGive(s_event_clients.lock);
    ESP_LOGW(TAG, "event client list is full, ignoring socket %d", fd);
}

static void unregister_event_client(int fd)
{
    if (fd < 0 || s_event_clients.lock == NULL) {
        return;
    }

    if (xSemaphoreTake(s_event_clients.lock, portMAX_DELAY) != pdPASS) {
        return;
    }

    for (size_t i = 0; i < EXAMPLE_MAX_EVENT_CLIENTS; i++) {
        if (s_event_clients.fds[i] == fd) {
            s_event_clients.fds[i] = -1;
            ESP_LOGI(TAG, "event client disconnected on socket %d", fd);
            break;
        }
    }

    xSemaphoreGive(s_event_clients.lock);
}

static esp_err_t broadcast_button_event(const char *event_name)
{
    httpd_ws_frame_t frame = {
        .final = true,
        .fragmented = false,
        .type = HTTPD_WS_TYPE_TEXT,
        .payload = (uint8_t *)event_name,
        .len = strlen(event_name),
    };
    int fds[EXAMPLE_MAX_EVENT_CLIENTS];
    httpd_handle_t server;

    ESP_RETURN_ON_FALSE(event_name != NULL, ESP_ERR_INVALID_ARG, TAG, "event name is null");
    ESP_RETURN_ON_FALSE(s_event_clients.lock != NULL, ESP_ERR_INVALID_STATE, TAG, "event registry is not initialized");

    if (xSemaphoreTake(s_event_clients.lock, portMAX_DELAY) != pdPASS) {
        return ESP_FAIL;
    }

    server = s_event_clients.server;
    for (size_t i = 0; i < EXAMPLE_MAX_EVENT_CLIENTS; i++) {
        fds[i] = s_event_clients.fds[i];
    }
    xSemaphoreGive(s_event_clients.lock);

    ESP_RETURN_ON_FALSE(server != NULL, ESP_ERR_INVALID_STATE, TAG, "event server is not ready");

    for (size_t i = 0; i < EXAMPLE_MAX_EVENT_CLIENTS; i++) {
        if (fds[i] < 0) {
            continue;
        }

        if (httpd_ws_get_fd_info(server, fds[i]) != HTTPD_WS_CLIENT_WEBSOCKET) {
            unregister_event_client(fds[i]);
            continue;
        }

        if (httpd_ws_send_frame_async(server, fds[i], &frame) != ESP_OK) {
            ESP_LOGW(TAG, "failed to push event '%s' to socket %d", event_name, fds[i]);
            unregister_event_client(fds[i]);
        }
    }

    return ESP_OK;
}

static esp_err_t set_cors_headers(httpd_req_t *req)
{
    ESP_RETURN_ON_ERROR(httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*"), TAG, "failed to set allow-origin");
    ESP_RETURN_ON_ERROR(httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "*"), TAG, "failed to set allow-headers");
    ESP_RETURN_ON_ERROR(httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET,POST,OPTIONS"), TAG, "failed to set allow-methods");
    return ESP_OK;
}

static bool runtime_should_stream(void)
{
    return s_runtime.preview_requested || s_runtime.recording_active;
}

static esp_err_t set_runtime_stream_enabled(bool enable)
{
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    ESP_RETURN_ON_FALSE(s_runtime.web_cam != NULL, ESP_ERR_INVALID_STATE, TAG, "runtime camera is not initialized");

    if (s_runtime.stream_enabled == enable) {
        return ESP_OK;
    }

    for (int i = 0; i < s_runtime.web_cam->video_count; i++) {
        web_cam_video_t *video = &s_runtime.web_cam->video[i];

        if (!is_valid_web_cam(video)) {
            continue;
        }

        if (enable) {
            ESP_RETURN_ON_FALSE(xSemaphoreTake(video->io_lock, portMAX_DELAY) == pdPASS, ESP_FAIL, TAG, "failed to take io lock");
            int ret = ioctl(video->fd, VIDIOC_STREAMON, &type);
            xSemaphoreGive(video->io_lock);
            ESP_RETURN_ON_FALSE(ret == 0, ESP_FAIL, TAG, "failed to stream on video%d", video->index);
            video->stream_on = 1;
        } else if (video->stream_on) {
            ESP_RETURN_ON_FALSE(xSemaphoreTake(video->io_lock, portMAX_DELAY) == pdPASS, ESP_FAIL, TAG, "failed to take io lock");
            ioctl(video->fd, VIDIOC_STREAMOFF, &type);
            xSemaphoreGive(video->io_lock);
            video->stream_on = 0;
        }
    }

    s_runtime.stream_enabled = enable;
    ESP_LOGI(TAG, "camera stream %s", enable ? "enabled" : "disabled");
    return ESP_OK;
}

static esp_err_t apply_runtime_stream_policy(void)
{
    bool should_stream;

    ESP_RETURN_ON_FALSE(s_runtime.lock != NULL, ESP_ERR_INVALID_STATE, TAG, "runtime lock is not initialized");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(s_runtime.lock, portMAX_DELAY) == pdPASS, ESP_FAIL, TAG, "failed to take runtime lock");
    should_stream = runtime_should_stream();
    xSemaphoreGive(s_runtime.lock);

    return set_runtime_stream_enabled(should_stream);
}

static bool is_valid_web_cam(web_cam_video_t *video)
{
    return video->fd != -1;
}

static uint32_t get_effective_stream_fps(const web_cam_video_t *video)
{
    if (video->frame_rate == 0) {
        return EXAMPLE_STREAM_MAX_FPS;
    }
    return video->frame_rate > EXAMPLE_STREAM_MAX_FPS ? EXAMPLE_STREAM_MAX_FPS : video->frame_rate;
}

static void free_video_format_options(web_cam_video_t *video)
{
    free(video->format_options);
    video->format_options = NULL;
    video->format_option_count = 0;
    video->current_format_index = -1;
}

static bool is_same_format_option(const web_cam_image_format_option_t *lhs, const web_cam_image_format_option_t *rhs)
{
    return lhs->pixel_format == rhs->pixel_format &&
           lhs->width == rhs->width &&
           lhs->height == rhs->height &&
           lhs->frame_interval_num == rhs->frame_interval_num &&
           lhs->frame_interval_den == rhs->frame_interval_den;
}

static bool is_encoder_pixel_format_supported(uint32_t pixel_format)
{
#if CONFIG_EXAMPLE_SELECT_JPEG_HW_DRIVER
    switch (pixel_format) {
    case V4L2_PIX_FMT_JPEG:
    case V4L2_PIX_FMT_SBGGR8:
    case V4L2_PIX_FMT_GREY:
    case V4L2_PIX_FMT_RGB565:
    case V4L2_PIX_FMT_RGB24:
    case V4L2_PIX_FMT_UYVY:
#if CONFIG_ESP32P4_REV_MIN_FULL >= 300
    case V4L2_PIX_FMT_YUV420:
    case V4L2_PIX_FMT_YUV444:
#endif
        return true;
    default:
        return false;
    }
#else
    switch (pixel_format) {
    case V4L2_PIX_FMT_JPEG:
    case V4L2_PIX_FMT_SBGGR8:
    case V4L2_PIX_FMT_GREY:
    case V4L2_PIX_FMT_UYVY:
    case V4L2_PIX_FMT_YUYV:
    case V4L2_PIX_FMT_RGB565:
    case V4L2_PIX_FMT_RGB565X:
        return true;
    default:
        return false;
    }
#endif
}

static esp_err_t append_video_format_option(web_cam_video_t *video, const web_cam_image_format_option_t *option)
{
    for (uint32_t i = 0; i < video->format_option_count; i++) {
        if (is_same_format_option(&video->format_options[i], option)) {
            return ESP_OK;
        }
    }

    web_cam_image_format_option_t *new_options = realloc(video->format_options,
                                                         sizeof(web_cam_image_format_option_t) * (video->format_option_count + 1));
    ESP_RETURN_ON_FALSE(new_options, ESP_ERR_NO_MEM, TAG, "failed to grow format option list");

    video->format_options = new_options;
    video->format_options[video->format_option_count] = *option;
    video->format_option_count++;
    return ESP_OK;
}

static int find_current_format_index(const web_cam_video_t *video)
{
    for (uint32_t i = 0; i < video->format_option_count; i++) {
        const web_cam_image_format_option_t *option = &video->format_options[i];
        uint32_t fps = 0;

        if (option->frame_interval_num != 0) {
            fps = option->frame_interval_den / option->frame_interval_num;
        }

        if (option->pixel_format == video->pixel_format &&
            option->width == video->width &&
            option->height == video->height &&
            (fps == 0 || fps == video->frame_rate)) {
            return (int)i;
        }
    }

    return -1;
}

static esp_err_t enumerate_video_format_options(web_cam_video_t *video, int fd)
{
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    free_video_format_options(video);

    for (int fmt_index = 0; ; fmt_index++) {
        struct v4l2_fmtdesc fmtdesc = {
            .index = fmt_index,
            .type = type,
        };

        if (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) != 0) {
            break;
        }

        if (!is_encoder_pixel_format_supported(fmtdesc.pixelformat)) {
            ESP_LOGI(TAG, "skip unsupported stream format %c%c%c%c",
                     V4L2_FMT_STR_ARG(fmtdesc.pixelformat));
            continue;
        }

        for (int size_index = 0; ; size_index++) {
            struct v4l2_frmsizeenum frmsize = {
                .index = size_index,
                .pixel_format = fmtdesc.pixelformat,
                .type = type,
            };

            if (ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &frmsize) != 0) {
                if (size_index == 0) {
                    web_cam_image_format_option_t option = {
                        .pixel_format = fmtdesc.pixelformat,
                        .width = video->width,
                        .height = video->height,
                        .frame_interval_num = 1,
                        .frame_interval_den = video->frame_rate,
                    };
                    ESP_RETURN_ON_ERROR(append_video_format_option(video, &option), TAG, "failed to append fallback format option");
                }
                break;
            }

            bool has_interval = false;
            if (frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
                for (int interval_index = 0; ; interval_index++) {
                    struct v4l2_frmivalenum frmival = {
                        .index = interval_index,
                        .pixel_format = fmtdesc.pixelformat,
                        .type = type,
                        .width = frmsize.discrete.width,
                        .height = frmsize.discrete.height,
                    };

                    if (ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival) != 0) {
                        break;
                    }

                    if (frmival.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
                        web_cam_image_format_option_t option = {
                            .pixel_format = fmtdesc.pixelformat,
                            .width = frmsize.discrete.width,
                            .height = frmsize.discrete.height,
                            .frame_interval_num = frmival.discrete.numerator,
                            .frame_interval_den = frmival.discrete.denominator,
                        };
                        ESP_RETURN_ON_ERROR(append_video_format_option(video, &option), TAG, "failed to append format option");
                        has_interval = true;
                    }
                }
            }

            if (!has_interval && frmsize.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
                web_cam_image_format_option_t option = {
                    .pixel_format = fmtdesc.pixelformat,
                    .width = frmsize.discrete.width,
                    .height = frmsize.discrete.height,
                    .frame_interval_num = 1,
                    .frame_interval_den = video->frame_rate,
                };
                ESP_RETURN_ON_ERROR(append_video_format_option(video, &option), TAG, "failed to append default fps format option");
            }
        }
    }

    video->current_format_index = find_current_format_index(video);
    return ESP_OK;
}

static esp_err_t release_video_buffers(web_cam_video_t *video)
{
    struct v4l2_requestbuffers req = {
        .count = 0,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };

    for (int i = 0; i < EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER; i++) {
        if (video->buffer[i]) {
            munmap(video->buffer[i], video->buffer_size);
            video->buffer[i] = NULL;
        }
    }
    video->buffer_size = 0;

    if (video->fd != -1) {
        ioctl(video->fd, VIDIOC_REQBUFS, &req);
    }

    return ESP_OK;
}

static void format_option_description(const web_cam_image_format_option_t *option, char *buffer, size_t buffer_size)
{
    uint32_t fps = 0;
    uint32_t display_fps = 0;

    if (option->frame_interval_num != 0) {
        fps = option->frame_interval_den / option->frame_interval_num;
    }

    if (fps > 0) {
        display_fps = fps > EXAMPLE_STREAM_MAX_FPS ? EXAMPLE_STREAM_MAX_FPS : fps;
        snprintf(buffer, buffer_size, "%c%c%c%c %" PRIu32 "x%" PRIu32 " @ %" PRIu32 "fps",
                 V4L2_FMT_STR_ARG(option->pixel_format), option->width, option->height, display_fps);
    } else {
        snprintf(buffer, buffer_size, "%c%c%c%c %" PRIu32 "x%" PRIu32,
                 V4L2_FMT_STR_ARG(option->pixel_format), option->width, option->height);
    }
}

static web_cam_video_t *get_default_video(web_cam_t *web_cam)
{
    for (int i = 0; i < web_cam->video_count; i++) {
        if (is_valid_web_cam(&web_cam->video[i])) {
            return &web_cam->video[i];
        }
    }
    return NULL;
}

static esp_err_t decode_request(web_cam_t *web_cam, httpd_req_t *req, request_desc_t *desc)
{
    esp_err_t ret;
    int index = -1;
    char buffer[32];

    if ((ret = httpd_req_get_url_query_str(req, buffer, sizeof(buffer))) != ESP_OK) {
        return ret;
    }
    ESP_LOGD(TAG, "source: %s", buffer);

    for (int i = 0; i < web_cam->video_count; i++) {
        char source_str[16];

        if (snprintf(source_str, sizeof(source_str), "source=%d", i) <= 0) {
            return ESP_FAIL;
        }

        if (strcmp(buffer, source_str) == 0) {
            index = i;
            break;
        }
    }
    if (index == -1) {
        return ESP_ERR_INVALID_ARG;
    }

    desc->index = index;
    return ESP_OK;
}

static esp_err_t capture_video_image(httpd_req_t *req, web_cam_video_t *video, bool is_jpeg)
{
    esp_err_t ret;
    struct v4l2_buffer buf = {0};
    const char *type_str = is_jpeg ? "JPEG" : "binary";
    uint32_t jpeg_encoded_size;
    bool io_locked = false;
    bool sem_locked = false;

    ESP_GOTO_ON_FALSE(xSemaphoreTake(video->io_lock, portMAX_DELAY) == pdPASS, ESP_FAIL, fail0, TAG, "failed to take io lock");
    io_locked = true;
    memset(&buf, 0, sizeof(buf));
    buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    ESP_RETURN_ON_ERROR(ioctl(video->fd, VIDIOC_DQBUF, &buf), TAG, "failed to receive video frame");
    if (!(buf.flags & V4L2_BUF_FLAG_DONE)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    if (!is_jpeg || video->pixel_format == V4L2_PIX_FMT_JPEG) {
        /* 直接发送原始数据（JPEG 格式） */
        ESP_GOTO_ON_ERROR(httpd_resp_send(req, (char *)video->buffer[buf.index], buf.bytesused), fail0, TAG, "failed to send %s", type_str);
        jpeg_encoded_size = buf.bytesused;
    } else {
        ESP_GOTO_ON_FALSE(xSemaphoreTake(video->sem, portMAX_DELAY) == pdPASS, ESP_FAIL, fail0, TAG, "failed to take semaphore");
        sem_locked = true;
        
        /* ========== 添加软件裁剪处理 ========== */
        if (video->software_crop_enabled) {
            size_t cropped_size = video->crop_width * video->crop_height * 2;
            uint8_t *cropped_buffer = malloc(cropped_size);
            
            if (cropped_buffer) {
                crop_rgb565_optimized(video->buffer[buf.index],
                                     video->crop_src_width,
                                     video->crop_src_height,
                                     cropped_buffer,
                                     video->crop_left,
                                     video->crop_top,
                                     video->crop_width,
                                     video->crop_height);
                
                ret = example_encoder_process(video->encoder_handle,
                                             cropped_buffer, cropped_size,
                                             video->jpeg_out_buf, video->jpeg_out_size,
                                             &jpeg_encoded_size);
                free(cropped_buffer);
            } else {
                ESP_LOGW(TAG, "Crop buffer alloc failed for capture, using full frame");
                ret = example_encoder_process(video->encoder_handle,
                                             video->buffer[buf.index], video->buffer_size,
                                             video->jpeg_out_buf, video->jpeg_out_size,
                                             &jpeg_encoded_size);
            }
        } else {
            ret = example_encoder_process(video->encoder_handle,
                                         video->buffer[buf.index], video->buffer_size,
                                         video->jpeg_out_buf, video->jpeg_out_size,
                                         &jpeg_encoded_size);
        }
        
        xSemaphoreGive(video->sem);
        sem_locked = false;
        ESP_GOTO_ON_ERROR(ret, fail0, TAG, "failed to encode video frame");
        ESP_GOTO_ON_ERROR(httpd_resp_send(req, (char *)video->jpeg_out_buf, jpeg_encoded_size), fail0, TAG, "failed to send %s", type_str);
    }

    ESP_RETURN_ON_ERROR(ioctl(video->fd, VIDIOC_QBUF, &buf), TAG, "failed to queue video frame");
    ESP_GOTO_ON_ERROR(httpd_resp_sendstr_chunk(req, NULL), fail0, TAG, "failed to send null");

    ESP_LOGD(TAG, "send %s image%d size: %" PRIu32, type_str, video->index, jpeg_encoded_size);

    xSemaphoreGive(video->io_lock);
    return ESP_OK;

fail0:
    if (sem_locked) {
        xSemaphoreGive(video->sem);
    }
    if (io_locked) {
        xSemaphoreGive(video->io_lock);
    }
    ioctl(video->fd, VIDIOC_QBUF, &buf);
    return ret;
}

static char *get_cameras_json(web_cam_t *web_cam)
{
    cJSON *root = cJSON_CreateObject();
    cJSON *cameras = cJSON_CreateArray();
    cJSON_AddItemToObject(root, "cameras", cameras);

    for (int i = 0; i < web_cam->video_count; i++) {
        char src_str[32];

        if (!is_valid_web_cam(&web_cam->video[i])) {
            continue;
        }

        cJSON *camera = cJSON_CreateObject();
        web_cam_image_format_option_t current_option = {
            .pixel_format = web_cam->video[i].pixel_format,
            .width = web_cam->video[i].width,
            .height = web_cam->video[i].height,
            .frame_interval_num = 1,
            .frame_interval_den = web_cam->video[i].frame_rate,
        };
        cJSON_AddNumberToObject(camera, "index", i);
        assert(snprintf(src_str, sizeof(src_str), ":%d/stream", i + 81) > 0);
        cJSON_AddStringToObject(camera, "src", src_str);
        cJSON_AddNumberToObject(camera, "currentFrameRate", get_effective_stream_fps(&web_cam->video[i]));
        cJSON_AddNumberToObject(camera, "currentImageFormat", web_cam->video[i].current_format_index >= 0 ? web_cam->video[i].current_format_index : 0);
        format_option_description(&current_option, src_str, sizeof(src_str));
        cJSON_AddStringToObject(camera, "currentImageFormatDescription", src_str);

        if (web_cam->video[i].support_control_jpeg_quality) {
            cJSON_AddNumberToObject(camera, "currentQuality", web_cam->video[i].jpeg_quality);
        }

        cJSON *current_resolution = cJSON_CreateObject();
        cJSON_AddNumberToObject(current_resolution, "width", web_cam->video[i].width);
        cJSON_AddNumberToObject(current_resolution, "height", web_cam->video[i].height);
        cJSON_AddItemToObject(camera, "currentResolution", current_resolution);

        cJSON *image_formats = cJSON_CreateArray();

        for (uint32_t format_index = 0; format_index < web_cam->video[i].format_option_count; format_index++) {
            cJSON *image_format = cJSON_CreateObject();
            cJSON_AddNumberToObject(image_format, "id", format_index);
            format_option_description(&web_cam->video[i].format_options[format_index], src_str, sizeof(src_str));
            cJSON_AddStringToObject(image_format, "description", src_str);

            if (web_cam->video[i].support_control_jpeg_quality) {
                cJSON *image_format_quality = cJSON_CreateObject();

                int min_quality = 1;
                int max_quality = 100;
                int step_quality = 1;
                int default_quality = EXAMPLE_JPEG_ENC_QUALITY;
                if (web_cam->video[i].pixel_format == V4L2_PIX_FMT_JPEG) {
                    struct v4l2_query_ext_ctrl qctrl = {0};

                    qctrl.id = V4L2_CID_JPEG_COMPRESSION_QUALITY;
                    if (ioctl(web_cam->video[i].fd, VIDIOC_QUERY_EXT_CTRL, &qctrl) == 0) {
                        min_quality = qctrl.minimum;
                        max_quality = qctrl.maximum;
                        step_quality = qctrl.step;
                        default_quality = qctrl.default_value;
                    }
                }

                cJSON_AddNumberToObject(image_format_quality, "min", min_quality);
                cJSON_AddNumberToObject(image_format_quality, "max", max_quality);
                cJSON_AddNumberToObject(image_format_quality, "step", step_quality);
                cJSON_AddNumberToObject(image_format_quality, "default", default_quality);
                cJSON_AddItemToObject(image_format, "quality", image_format_quality);
            }
            cJSON_AddItemToArray(image_formats, image_format);
        }

        cJSON_AddItemToObject(camera, "imageFormats", image_formats);
        cJSON_AddItemToArray(cameras, camera);
    }

    char *output = cJSON_Print(root);
    cJSON_Delete(root);
    return output;
}

static esp_err_t set_camera_jpeg_quality(web_cam_video_t *video, int quality)
{
    esp_err_t ret = ESP_OK;
    int quality_reset = quality;

    if (video->pixel_format == V4L2_PIX_FMT_JPEG) {
        struct v4l2_ext_controls controls = {0};
        struct v4l2_ext_control control[1];
        struct v4l2_query_ext_ctrl qctrl = {0};

        qctrl.id = V4L2_CID_JPEG_COMPRESSION_QUALITY;
        if (ioctl(video->fd, VIDIOC_QUERY_EXT_CTRL, &qctrl) == 0) {
            if ((quality > qctrl.maximum) || (quality < qctrl.minimum) ||
                    (((quality - qctrl.minimum) % qctrl.step) != 0)) {

                if (quality > qctrl.maximum) {
                    quality_reset = qctrl.maximum;
                } else if (quality < qctrl.minimum) {
                    quality_reset = qctrl.minimum;
                } else {
                    quality_reset = qctrl.minimum + ((quality - qctrl.minimum) / qctrl.step) * qctrl.step;
                }

                ESP_LOGW(TAG, "video%d: JPEG compression quality=%d is out of sensor's range, reset to %d", video->index, quality, quality_reset);
            }

            controls.ctrl_class = V4L2_CID_JPEG_CLASS;
            controls.count = 1;
            controls.controls = control;
            control[0].id = V4L2_CID_JPEG_COMPRESSION_QUALITY;
            control[0].value = quality_reset;
            ESP_RETURN_ON_ERROR(ioctl(video->fd, VIDIOC_S_EXT_CTRLS, &controls), TAG, "failed to set jpeg compression quality");

            video->jpeg_quality = quality_reset;
            video->support_control_jpeg_quality = 1;
        } else {
            video->support_control_jpeg_quality = 0;
            ESP_LOGW(TAG, "video%d: JPEG compression quality control is not supported", video->index);
        }
    } else {
        ESP_RETURN_ON_ERROR(example_encoder_set_jpeg_quality(video->encoder_handle, quality_reset), TAG, "failed to set jpeg quality");
        video->jpeg_quality = quality_reset;
    }

    if (video->support_control_jpeg_quality) {
        ESP_LOGI(TAG, "video%d: set jpeg quality %d success", video->index, quality_reset);
    }

    return ret;
}

static esp_err_t camera_info_handler(httpd_req_t *req)
{
    esp_err_t ret;
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;
    char *output = get_cameras_json(web_cam);

    set_cors_headers(req);
    httpd_resp_set_type(req, "application/json");
    ret = httpd_resp_sendstr(req, output);
    free(output);

    return ret;
}

static esp_err_t set_camera_image_format(web_cam_video_t *video, int image_format_index)
{
    esp_err_t ret;
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    int stream_ret;
    web_cam_image_format_option_t selected_format;
    web_cam_image_format_option_t previous_format = {
        .pixel_format = video->pixel_format,
        .width = video->width,
        .height = video->height,
        .frame_interval_num = 1,
        .frame_interval_den = video->frame_rate,
    };
    web_cam_video_config_t config = {
        .dev_name = video->dev_name,
    };

    ESP_RETURN_ON_FALSE(image_format_index >= 0 && image_format_index < (int)video->format_option_count,
                        ESP_ERR_INVALID_ARG, TAG, "invalid image format index");

    if (image_format_index == video->current_format_index) {
        return ESP_OK;
    }

    selected_format = video->format_options[image_format_index];
    ESP_RETURN_ON_FALSE(is_encoder_pixel_format_supported(selected_format.pixel_format), ESP_ERR_NOT_SUPPORTED,
                        TAG, "selected pixel format is not supported by encoder");
    ESP_RETURN_ON_FALSE(xSemaphoreTake(video->io_lock, portMAX_DELAY) == pdPASS, ESP_FAIL, TAG, "failed to take io lock");

    deinit_web_cam_video(video);
    ret = init_web_cam_video(video, &config, video->index, &selected_format);
    if (ret == ESP_OK) {
        stream_ret = ioctl(video->fd, VIDIOC_STREAMON, &type);
        if (stream_ret == 0) {
            video->stream_on = 1;
            ret = ESP_OK;
        } else {
            ret = ESP_FAIL;
        }
    }

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "video%d: failed to switch format, rolling back to previous format", video->index);
        deinit_web_cam_video(video);
        if (init_web_cam_video(video, &config, video->index, &previous_format) == ESP_OK) {
            if (ioctl(video->fd, VIDIOC_STREAMON, &type) != 0) {
                ESP_LOGE(TAG, "video%d: rollback stream on failed", video->index);
            } else {
                video->stream_on = 1;
            }
        } else {
            ESP_LOGE(TAG, "video%d: rollback init failed", video->index);
        }
    }

    xSemaphoreGive(video->io_lock);
    if (ret == ESP_OK && !runtime_should_stream()) {
        ESP_RETURN_ON_ERROR(set_runtime_stream_enabled(false), TAG, "failed to restore stream policy after format switch");
    }
    return ret;
}

static esp_err_t legacy_status_handler(httpd_req_t *req)
{
    esp_err_t ret = ESP_OK;
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;
    web_cam_video_t *video = get_default_video(web_cam);
    cJSON *root = cJSON_CreateObject();

    ESP_RETURN_ON_FALSE(root, ESP_ERR_NO_MEM, TAG, "failed to allocate status json");
    ESP_GOTO_ON_FALSE(video, ESP_ERR_NOT_FOUND, fail0, TAG, "no active camera stream found");

    cJSON_AddNumberToObject(root, "index", video->index);
    cJSON_AddNumberToObject(root, "width", video->width);
    cJSON_AddNumberToObject(root, "height", video->height);
    cJSON_AddNumberToObject(root, "frame_rate", get_effective_stream_fps(video));
    cJSON_AddNumberToObject(root, "jpeg_quality", video->jpeg_quality);
    cJSON_AddStringToObject(root, "stream_path", "/stream");
    cJSON_AddStringToObject(root, "capture_path", "/capture");

    char *output = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    ESP_RETURN_ON_FALSE(output, ESP_ERR_NO_MEM, TAG, "failed to render status json");

    httpd_resp_set_type(req, "application/json");
    ret = httpd_resp_sendstr(req, output);
    free(output);
    return ret;

fail0:
    cJSON_Delete(root);
    httpd_resp_send_500(req);
    return ESP_FAIL;
}

static esp_err_t camera_settings_handler(httpd_req_t *req)
{
    esp_err_t ret = ESP_OK;
    char *content = NULL;
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;

    set_cors_headers(req);
    content = (char *)calloc(1, req->content_len + 1);
    ESP_RETURN_ON_FALSE(content, ESP_ERR_NO_MEM, TAG, "failed to allocate memory");

    ESP_GOTO_ON_FALSE(httpd_req_recv(req, content, req->content_len) > 0, ESP_FAIL, fail0, TAG, "failed to recv content");
    ESP_LOGI(TAG, "Received JSON: %s", content);

    cJSON *json_root = cJSON_Parse(content);
    free(content);
    content = NULL;
    ESP_GOTO_ON_FALSE(json_root, ESP_FAIL, fail0, TAG, "failed to parse JSON");

    // 解析 index
    cJSON *json_index = cJSON_GetObjectItem(json_root, "index");
    ESP_GOTO_ON_FALSE(json_index && cJSON_IsNumber(json_index), ESP_ERR_INVALID_ARG, fail1, 
                      TAG, "missing or invalid index field");
    int index = json_index->valueint;
    ESP_GOTO_ON_FALSE(index >= 0 && index < web_cam->video_count && is_valid_web_cam(&web_cam->video[index]), 
                      ESP_ERR_INVALID_ARG, fail1, TAG, "invalid index");

    web_cam_video_t *video = &web_cam->video[index];

    cJSON *json_image_format = cJSON_GetObjectItem(json_root, "image_format");
    if (!json_image_format) {
        json_image_format = cJSON_GetObjectItem(json_root, "imageFormat");
    }
    
    int image_format = video->current_format_index;
    if (json_image_format && cJSON_IsNumber(json_image_format)) {
        image_format = json_image_format->valueint;
    }

    cJSON *json_jpeg_quality = cJSON_GetObjectItem(json_root, "jpeg_quality");
    if (!json_jpeg_quality) {
        json_jpeg_quality = cJSON_GetObjectItem(json_root, "quality");
    }
    
    int jpeg_quality = video->jpeg_quality;
    if (json_jpeg_quality && cJSON_IsNumber(json_jpeg_quality)) {
        jpeg_quality = json_jpeg_quality->valueint;
    }

    cJSON_Delete(json_root);
    json_root = NULL;

    if (image_format != video->current_format_index) {
        ESP_GOTO_ON_ERROR(set_camera_image_format(video, image_format), fail1, TAG, "failed to set camera image format");
    }
    
    if (jpeg_quality != video->jpeg_quality) {
        ESP_GOTO_ON_ERROR(set_camera_jpeg_quality(video, jpeg_quality), fail1, TAG, "failed to set camera jpeg quality");
    }

    httpd_resp_sendstr(req, "OK");
    return ESP_OK;

fail1:
    if (json_root) {
        cJSON_Delete(json_root);
    }
fail0:
    if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
        httpd_resp_send_408(req);
    } else {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON format");
    }
    if (content) {
        free(content);
    }
    return ret;
}

static esp_err_t button_event_ws_handler(httpd_req_t *req)
{
    int fd = httpd_req_to_sockfd(req);

    if (req->method == HTTP_GET) {
        register_event_client(req->handle, fd);
        return ESP_OK;
    }

    httpd_ws_frame_t ws_pkt = {0};
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        unregister_event_client(fd);
        return ret;
    }

    if (ws_pkt.len > 0) {
        ws_pkt.payload = calloc(1, ws_pkt.len + 1);
        ESP_RETURN_ON_FALSE(ws_pkt.payload != NULL, ESP_ERR_NO_MEM, TAG, "failed to alloc ws payload");
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            free(ws_pkt.payload);
            unregister_event_client(fd);
            return ret;
        }
    }

    if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
        unregister_event_client(fd);
    } else if (ws_pkt.type == HTTPD_WS_TYPE_TEXT && ws_pkt.payload != NULL) {
        ESP_LOGI(TAG, "button event client says: %s", (char *)ws_pkt.payload);
    }

    free(ws_pkt.payload);
    return ESP_OK;
}

static esp_err_t static_file_handler(httpd_req_t *req)
{
    const char *uri = req->uri;

    if (strcmp(uri, "/") == 0) {
        httpd_resp_set_type(req, "text/html");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)index_html_gz_start, index_html_gz_end - index_html_gz_start);
    } else if (strcmp(uri, "/loading.jpg") == 0) {
        httpd_resp_set_type(req, "image/jpeg");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)loading_jpg_gz_start, loading_jpg_gz_end - loading_jpg_gz_start);
    } else if (strcmp(uri, "/favicon.ico") == 0) {
        httpd_resp_set_type(req, "image/x-icon");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)favicon_ico_gz_start, favicon_ico_gz_end - favicon_ico_gz_start);
    } else if (strcmp(uri, "/assets/index.js") == 0) {
        httpd_resp_set_type(req, "application/javascript");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)assets_index_js_gz_start, assets_index_js_gz_end - assets_index_js_gz_start);
    } else if (strcmp(uri, "/assets/index.css") == 0) {
        httpd_resp_set_type(req, "text/css");
        httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
        return httpd_resp_send(req, (const char *)assets_index_css_gz_start, assets_index_css_gz_end - assets_index_css_gz_start);
    }

    ESP_LOGW(TAG, "File not found: %s", uri);
    httpd_resp_send_404(req);
    return ESP_FAIL;
}

static esp_err_t image_stream_handler(httpd_req_t *req)
{
    esp_err_t ret;
    struct v4l2_buffer buf;
    char http_string[128];
    bool locked = false;
    bool io_locked = false;
    int64_t next_frame_deadline_us = 0;
    web_cam_video_t *video = (web_cam_video_t *)req->user_ctx;
    uint32_t stream_fps = get_effective_stream_fps(video);
    int64_t stream_interval_us = 1000000LL / stream_fps;
    
    static uint32_t frame_total_count = 0;

    if (!video->stream_on) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_sendstr(req, "preview disabled");
        return ESP_OK;
    }

    ESP_RETURN_ON_FALSE(snprintf(http_string, sizeof(http_string), "%" PRIu32, stream_fps) > 0,
                        ESP_FAIL, TAG, "failed to format framerate buffer");

    ESP_RETURN_ON_ERROR(httpd_resp_set_type(req, STREAM_CONTENT_TYPE), TAG, "failed to set content type");
    ESP_RETURN_ON_ERROR(set_cors_headers(req), TAG, "failed to set cors headers");
    ESP_RETURN_ON_ERROR(httpd_resp_set_hdr(req, "X-Framerate", http_string), TAG, "failed to set x framerate");

    while (1) {
        int hlen;
        struct timespec ts;
        uint32_t jpeg_encoded_size = 0;

        locked = false;
        io_locked = false;

        if (xSemaphoreTake(video->io_lock, portMAX_DELAY) != pdPASS) {
            break;
        }
        io_locked = true;

        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        
        ret = ioctl(video->fd, VIDIOC_DQBUF, &buf);
        if (ret != 0) {
            xSemaphoreGive(video->io_lock);
            io_locked = false;
            continue;
        }
        
        if (!(buf.flags & V4L2_BUF_FLAG_DONE)) {
            ioctl(video->fd, VIDIOC_QBUF, &buf);
            xSemaphoreGive(video->io_lock);
            io_locked = false;
            continue;
        }

        int64_t now_us = esp_timer_get_time();
        if (next_frame_deadline_us != 0 && now_us < next_frame_deadline_us) {
            ioctl(video->fd, VIDIOC_QBUF, &buf);
            xSemaphoreGive(video->io_lock);
            io_locked = false;
            continue;
        }

        frame_total_count++;
        video_set_quality(video, video->preview_quality);

        if (video->pixel_format == V4L2_PIX_FMT_JPEG) {
            video->jpeg_out_buf = video->buffer[buf.index];
            jpeg_encoded_size = buf.bytesused;
        } else {
            if (xSemaphoreTake(video->sem, portMAX_DELAY) != pdPASS) {
                goto fail0;
            }
            locked = true;

            if (video->software_crop_enabled) {
                size_t cropped_size = video->crop_width * video->crop_height * 2;
                uint8_t *cropped_buffer = heap_caps_malloc(cropped_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                
                if (cropped_buffer) {
                    crop_rgb565_optimized(video->buffer[buf.index],
                                         video->crop_src_width,
                                         video->crop_src_height,
                                         cropped_buffer,
                                         video->crop_left,
                                         video->crop_top,
                                         video->crop_width,
                                         video->crop_height);
                    
                    ret = example_encoder_process(video->encoder_handle,
                                                 cropped_buffer, cropped_size,
                                                 video->jpeg_out_buf, video->jpeg_out_size,
                                                 &jpeg_encoded_size);
                    heap_caps_free(cropped_buffer);
                } else {
                    ret = example_encoder_process(video->encoder_handle,
                                                 video->buffer[buf.index], video->buffer_size,
                                                 video->jpeg_out_buf, video->jpeg_out_size,
                                                 &jpeg_encoded_size);
                }
            } else {
                ret = example_encoder_process(video->encoder_handle,
                                             video->buffer[buf.index], video->buffer_size,
                                             video->jpeg_out_buf, video->jpeg_out_size,
                                             &jpeg_encoded_size);
            }
            
            if (ret != ESP_OK) {
                goto fail0;
            }
        }
        
        ret = clock_gettime(CLOCK_MONOTONIC, &ts);
        if (ret != 0) {
            goto fail0;
        }
        
        hlen = snprintf(http_string, sizeof(http_string), STREAM_PART, jpeg_encoded_size, ts.tv_sec, ts.tv_nsec);
        if (hlen <= 0) {
            goto fail0;
        }
        
        ret = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
        if (ret != ESP_OK) {
            goto cleanup;
        }
        
        ret = httpd_resp_send_chunk(req, http_string, hlen);
        if (ret != ESP_OK) {
            goto cleanup;
        }
        
        ret = httpd_resp_send_chunk(req, (char *)video->jpeg_out_buf, jpeg_encoded_size);
        if (ret != ESP_OK) {
            goto cleanup;
        }

        if (locked) {
            xSemaphoreGive(video->sem);
            locked = false;
        }

        ret = ioctl(video->fd, VIDIOC_QBUF, &buf);
        if (ret != 0) {
            ESP_LOGE(TAG, "Failed to queue video frame");
        }
        next_frame_deadline_us = now_us + stream_interval_us;
        xSemaphoreGive(video->io_lock);
        io_locked = false;
        
        continue;

cleanup:
        if (locked) {
            xSemaphoreGive(video->sem);
            locked = false;
        }
        if (io_locked) {
            xSemaphoreGive(video->io_lock);
            io_locked = false;
        }
        ioctl(video->fd, VIDIOC_QBUF, &buf);
        break;

fail0:
        if (locked) {
            xSemaphoreGive(video->sem);
        }
        if (io_locked) {
            xSemaphoreGive(video->io_lock);
        }
        ioctl(video->fd, VIDIOC_QBUF, &buf);
        if (ret == ESP_FAIL || ret == ESP_ERR_HTTPD_RESP_SEND) {
            return ESP_OK;
        }
        return ret;
    }

    return ESP_OK;
}

static esp_err_t capture_image_handler(httpd_req_t *req)
{
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;

    request_desc_t desc;
    ESP_RETURN_ON_ERROR(decode_request(web_cam, req, &desc), TAG, "failed to decode request");

    ESP_RETURN_ON_ERROR(set_cors_headers(req), TAG, "failed to set cors headers");
    char type_ptr[32];
    ESP_RETURN_ON_FALSE(snprintf(type_ptr, sizeof(type_ptr), "image/jpeg;name=image%d.jpg", desc.index) > 0, ESP_FAIL, TAG, "failed to format buffer");
    ESP_RETURN_ON_ERROR(httpd_resp_set_type(req, type_ptr), TAG, "failed to set content type");

    return capture_video_image(req, &web_cam->video[desc.index], true);
}

static esp_err_t capture_binary_handler(httpd_req_t *req)
{
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;

    request_desc_t desc;
    ESP_RETURN_ON_ERROR(decode_request(web_cam, req, &desc), TAG, "failed to decode request");

    ESP_RETURN_ON_ERROR(set_cors_headers(req), TAG, "failed to set cors headers");
    char type_ptr[56];
    ESP_RETURN_ON_FALSE(snprintf(type_ptr, sizeof(type_ptr), "application/octet-stream;name=image_binary%d.bin", desc.index) > 0, ESP_FAIL, TAG, "failed to format buffer");
    ESP_RETURN_ON_ERROR(httpd_resp_set_type(req, type_ptr), TAG, "failed to set content type");

    return capture_video_image(req, &web_cam->video[desc.index], false);
}

static esp_err_t legacy_capture_handler(httpd_req_t *req)
{
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;
    web_cam_video_t *video = get_default_video(web_cam);

    ESP_RETURN_ON_FALSE(video, ESP_ERR_NOT_FOUND, TAG, "no active camera stream found");
    ESP_RETURN_ON_ERROR(set_cors_headers(req), TAG, "failed to set cors headers");
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    return capture_video_image(req, video, true);
}

static esp_err_t legacy_control_handler(httpd_req_t *req)
{
    esp_err_t ret;
    char query[64];
    char variable[32];
    char value[16];
    web_cam_t *web_cam = (web_cam_t *)req->user_ctx;
    web_cam_video_t *video = get_default_video(web_cam);

    ESP_RETURN_ON_FALSE(video, ESP_ERR_NOT_FOUND, TAG, "no active camera stream found");

    ret = httpd_req_get_url_query_str(req, query, sizeof(query));
    if (ret != ESP_OK) {
        httpd_resp_send_404(req);
        return ret;
    }

    if (httpd_query_key_value(query, "var", variable, sizeof(variable)) != ESP_OK ||
        httpd_query_key_value(query, "val", value, sizeof(value)) != ESP_OK) {
        httpd_resp_send_404(req);
        return ESP_ERR_INVALID_ARG;
    }

    if (strcmp(variable, "quality") == 0 || strcmp(variable, "jpeg_quality") == 0) {
        ESP_RETURN_ON_ERROR(set_camera_jpeg_quality(video, atoi(value)), TAG, "failed to set jpeg quality");
    } else {
        ESP_LOGW(TAG, "legacy control '%s' is not implemented on esp_video backend", variable);
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "unsupported var on esp32-p4 backend");
        return ESP_ERR_NOT_SUPPORTED;
    }

    set_cors_headers(req);
    return httpd_resp_send(req, NULL, 0);
}

static esp_err_t init_web_cam_video(web_cam_video_t *video, const web_cam_video_config_t *config, int index,
                                    const web_cam_image_format_option_t *requested_format)
{
    int fd;
    int ret;
    struct v4l2_format format;
    struct v4l2_streamparm sparm;
    struct v4l2_requestbuffers req;
    struct v4l2_captureparm *cparam = &sparm.parm.capture;
    struct v4l2_fract *timeperframe = &cparam->timeperframe;

    fd = open(config->dev_name, O_RDWR);
    ESP_RETURN_ON_FALSE(fd >= 0, ESP_ERR_NOT_FOUND, TAG, "Open video device %s failed", config->dev_name);

    snprintf(video->dev_name, sizeof(video->dev_name), "%s", config->dev_name);

    memset(&format, 0, sizeof(format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = 1280;
    format.fmt.pix.height = 960;
    format.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    
    if (ioctl(fd, VIDIOC_S_FMT, &format) == 0) {
        video->pixel_format = V4L2_PIX_FMT_MJPEG;
        video->width = format.fmt.pix.width;
        video->height = format.fmt.pix.height;
        video->encoder_handle = NULL;
        video->jpeg_out_buf = NULL;
        video->jpeg_out_size = 0;
        video->support_control_jpeg_quality = 0;
        
        memset(&sparm, 0, sizeof(sparm));
        sparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        sparm.parm.capture.timeperframe.numerator = 1;
        sparm.parm.capture.timeperframe.denominator = 45;
        ioctl(fd, VIDIOC_S_PARM, &sparm);
        goto mjpeg_setup;
    }

    if (requested_format) {
        memset(&format, 0, sizeof(struct v4l2_format));
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.width = requested_format->width;
        format.fmt.pix.height = requested_format->height;
        format.fmt.pix.pixelformat = requested_format->pixel_format;
        ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_S_FMT, &format), fail0, TAG, "Failed set fmt on %s", config->dev_name);

        memset(&sparm, 0, sizeof(sparm));
        sparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        sparm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
        sparm.parm.capture.timeperframe.numerator = requested_format->frame_interval_num;
        sparm.parm.capture.timeperframe.denominator = requested_format->frame_interval_den;
        ioctl(fd, VIDIOC_S_PARM, &sparm);
    }

    memset(&format, 0, sizeof(struct v4l2_format));
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_G_FMT, &format), fail0, TAG, "Failed get fmt from %s", config->dev_name);

#if CONFIG_EXAMPLE_SELECT_JPEG_HW_DRIVER
    if (format.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB565X) {
#if CONFIG_ESP_VIDEO_ENABLE_SWAP_BYTE
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB565;
        ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_S_FMT, &format), fail0, TAG, "failed to set fmt to %s", config->dev_name);
#else
        ESP_GOTO_ON_ERROR(ESP_FAIL, fail0, TAG, "Please enable the byte swap function ESP_VIDEO_ENABLE_SWAP_BYTE in menuconfig.");
#endif
    }
#endif

    memset(&sparm, 0, sizeof(sparm));
    sparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_G_PARM, &sparm), fail0, TAG, "failed to get frame rate from %s", config->dev_name);
    video->frame_rate = timeperframe->denominator / timeperframe->numerator;

    bool hw_crop_supported = false;
    
#if CONFIG_EXAMPLE_ENABLE_MIPI_CSI_CROP
    struct v4l2_selection selection;
    memset(&selection, 0, sizeof(selection));
    selection.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    selection.target = V4L2_SEL_TGT_CROP;
    selection.r.left = 0;
    selection.r.top = 0;
    selection.r.width = 1280;
    selection.r.height = 960;
    
    if (ioctl(fd, VIDIOC_S_SELECTION, &selection) != 0) {
        hw_crop_supported = false;
    } else {
        hw_crop_supported = true;
    }
#endif

    memset(&req, 0, sizeof(req));
    req.count  = EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_REQBUFS, &req), fail0, TAG, "failed to req buffers from %s", config->dev_name);

    for (int i = 0; i < EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory      = V4L2_MEMORY_MMAP;
        buf.index       = i;
        ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_QUERYBUF, &buf), fail0, TAG, "failed to query vbuf from %s", config->dev_name);

        video->buffer[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        ESP_GOTO_ON_FALSE(video->buffer[i] != MAP_FAILED, ESP_ERR_NO_MEM, fail0, TAG, "failed to mmap buffer");
        video->buffer_size = buf.length;

        ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_QBUF, &buf), fail0, TAG, "failed to queue frame vbuf from %s", config->dev_name);
    }

    video->fd = fd;
    video->width = format.fmt.pix.width;
    video->height = format.fmt.pix.height;
    video->pixel_format = format.fmt.pix.pixelformat;
    video->jpeg_quality = EXAMPLE_JPEG_ENC_QUALITY;
    video->software_crop_enabled = 0;
    
#if CONFIG_EXAMPLE_ENABLE_MIPI_CSI_CROP
    if (!hw_crop_supported && video->width == 1280 && video->height == 960) {
        video->software_crop_enabled = 1;
        video->crop_src_width = 1280;
        video->crop_src_height = 960;
        video->crop_left = 0;
        video->crop_top = 0;
        video->crop_width = 1280;
        video->crop_height = 960;
    }
#endif

    if (video->pixel_format == V4L2_PIX_FMT_JPEG) {
        ESP_GOTO_ON_ERROR(set_camera_jpeg_quality(video, EXAMPLE_JPEG_ENC_QUALITY), fail0, TAG, "failed to set jpeg quality");
    } else {
        example_encoder_config_t encoder_config = {0};
        if (video->software_crop_enabled) {
            encoder_config.width = video->crop_width;
            encoder_config.height = video->crop_height;
        } else {
            encoder_config.width = video->width;
            encoder_config.height = video->height;
        }
        
        encoder_config.pixel_format = video->pixel_format;
        encoder_config.quality = EXAMPLE_JPEG_ENC_QUALITY;
        ESP_GOTO_ON_ERROR(example_encoder_init(&encoder_config, &video->encoder_handle), fail0, TAG, "failed to init encoder");

        ESP_GOTO_ON_ERROR(example_encoder_alloc_output_buffer(video->encoder_handle, &video->jpeg_out_buf, &video->jpeg_out_size),
                          fail1, TAG, "failed to alloc jpeg output buf");

        video->support_control_jpeg_quality = 1;
    }

    video->sem = xSemaphoreCreateBinary();
    ESP_GOTO_ON_FALSE(video->sem, ESP_ERR_NO_MEM, fail2, TAG, "failed to create semaphore");
    xSemaphoreGive(video->sem);

    if (video->io_lock == NULL) {
        video->io_lock = xSemaphoreCreateMutex();
        ESP_GOTO_ON_FALSE(video->io_lock, ESP_ERR_NO_MEM, fail3, TAG, "failed to create io lock");
    }

    ESP_GOTO_ON_ERROR(enumerate_video_format_options(video, fd), fail3, TAG, "failed to enumerate video formats");
    video->preview_quality = PREVIEW_JPEG_QUALITY;
    video->capture_quality = CAPTURE_JPEG_QUALITY;
    video->record_quality = RECORD_JPEG_QUALITY;
    video->current_quality = PREVIEW_JPEG_QUALITY;

    return ESP_OK;
mjpeg_setup:
    memset(&req, 0, sizeof(req));
    req.count = EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_REQBUFS, &req), fail0, TAG, "failed to req buffers");

    for (int i = 0; i < EXAMPLE_CAMERA_VIDEO_BUFFER_NUMBER; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_QUERYBUF, &buf), fail0, TAG, "failed to query vbuf");
        
        video->buffer[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        ESP_GOTO_ON_FALSE(video->buffer[i] != MAP_FAILED, ESP_ERR_NO_MEM, fail0, TAG, "failed to mmap buffer");
        video->buffer_size = buf.length;
        
        ESP_GOTO_ON_ERROR(ioctl(fd, VIDIOC_QBUF, &buf), fail0, TAG, "failed to queue frame vbuf");
    }
    
    video->fd = fd;
    video->sem = xSemaphoreCreateBinary();
    ESP_GOTO_ON_FALSE(video->sem, ESP_ERR_NO_MEM, fail2, TAG, "failed to create semaphore");
    xSemaphoreGive(video->sem);
    
    if (video->io_lock == NULL) {
        video->io_lock = xSemaphoreCreateMutex();
        ESP_GOTO_ON_FALSE(video->io_lock, ESP_ERR_NO_MEM, fail3, TAG, "failed to create io lock");
    }
    
    video->preview_quality = PREVIEW_JPEG_QUALITY;
    video->capture_quality = CAPTURE_JPEG_QUALITY;
    video->record_quality = RECORD_JPEG_QUALITY;
    video->current_quality = PREVIEW_JPEG_QUALITY;
    return ESP_OK;        
fail3:
    if (video->sem) {
        vSemaphoreDelete(video->sem);
        video->sem = NULL;
    }
fail2:
    if (video->pixel_format != V4L2_PIX_FMT_JPEG) {
        example_encoder_free_output_buffer(video->encoder_handle, video->jpeg_out_buf);
        video->jpeg_out_buf = NULL;
    }
fail1:
    if (video->pixel_format != V4L2_PIX_FMT_JPEG) {
        example_encoder_deinit(video->encoder_handle);
        video->encoder_handle = NULL;
    }
fail0:
    close(fd);
    video->fd = -1;
    return ret;
}

static esp_err_t video_set_quality(web_cam_video_t *video, uint8_t quality)
{
    if (!video) return ESP_ERR_INVALID_ARG;
    if (video->current_quality == quality && video->jpeg_quality == quality) return ESP_OK;
    
    if (video->pixel_format != V4L2_PIX_FMT_JPEG) {
        ESP_RETURN_ON_ERROR(example_encoder_set_jpeg_quality(video->encoder_handle, quality),
                            TAG, "failed to set encoder quality");
    } else {
        ESP_RETURN_ON_ERROR(set_camera_jpeg_quality(video, quality), TAG, "failed to set sensor jpeg quality");
    }
    video->current_quality = quality;
    video->jpeg_quality = quality;
    
    return ESP_OK;
}

static esp_err_t deinit_web_cam_video(web_cam_video_t *video)
{
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (video->sem) {
        vSemaphoreDelete(video->sem);
        video->sem = NULL;
    }

    if (video->pixel_format != V4L2_PIX_FMT_JPEG) {
        example_encoder_free_output_buffer(video->encoder_handle, video->jpeg_out_buf);
        example_encoder_deinit(video->encoder_handle);
    }

    video->encoder_handle = NULL;
    video->jpeg_out_buf = NULL;
    video->jpeg_out_size = 0;

    if (video->fd != -1) {
        ioctl(video->fd, VIDIOC_STREAMOFF, &type);
        video->stream_on = 0;
        release_video_buffers(video);
        close(video->fd);
        video->fd = -1;
    }

    free_video_format_options(video);
    return ESP_OK;
}

static esp_err_t new_web_cam(const web_cam_video_config_t *config, int config_count, web_cam_t **ret_wc)
{
    int i;
    web_cam_t *wc;
    esp_err_t ret = ESP_FAIL;
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    wc = calloc(1, sizeof(web_cam_t) + config_count * sizeof(web_cam_video_t));
    ESP_RETURN_ON_FALSE(wc, ESP_ERR_NO_MEM, TAG, "failed to alloc web cam");
    wc->video_count = config_count;

    for (i = 0; i < config_count; i++) {
        wc->video[i].index = i;
        wc->video[i].fd = -1;
        wc->video[i].stream_on = 0;

        ret = init_web_cam_video(&wc->video[i], &config[i], i, NULL);
        if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "failed to find web_cam %d", i);
            continue;
        } else if (ret != ESP_OK) {
            ESP_LOGE(TAG, "failed to initialize web_cam %d", i);
            goto fail0;
        }
    }

    for (i = 0; i < config_count; i++) {
        if (is_valid_web_cam(&wc->video[i])) {
            ESP_GOTO_ON_ERROR(ioctl(wc->video[i].fd, VIDIOC_STREAMON, &type), fail1, TAG, "failed to start stream");
            wc->video[i].stream_on = 1;
        }
    }

    *ret_wc = wc;
    return ESP_OK;

fail1:
    for (int j = i - 1; j >= 0; j--) {
        if (is_valid_web_cam(&wc->video[j])) {
            ioctl(wc->video[j].fd, VIDIOC_STREAMOFF, &type);
        }
    }
    i = config_count;
fail0:
    for (int j = i - 1; j >= 0; j--) {
        if (is_valid_web_cam(&wc->video[j])) {
            deinit_web_cam_video(&wc->video[j]);
        }
    }
    free(wc);
    return ret;
}

static void free_web_cam(web_cam_t *web_cam)
{
    for (int i = 0; i < web_cam->video_count; i++) {
        if (is_valid_web_cam(&web_cam->video[i])) {
            deinit_web_cam_video(&web_cam->video[i]);
        }
    }
    free(web_cam);
}

static void clear_photo_files_on_sdcard(void)
{
    DIR *dir = opendir("/sdcard");

    if (!dir) {
        ESP_LOGW(TAG, "failed to open sdcard directory for photo cleanup");
        return;
    }

    struct dirent *entry;
    char filepath[512];

    while ((entry = readdir(dir)) != NULL) {
        const char *name = entry->d_name;

        if (strstr(name, ".jpg") || strstr(name, ".jpeg")) {
            snprintf(filepath, sizeof(filepath), "/sdcard/%s", name);
            if (remove(filepath) == 0) {
                ESP_LOGI(TAG, "removed old photo file: %s", filepath);
            } else {
                ESP_LOGW(TAG, "failed to remove old photo file: %s", filepath);
            }
        }
    }

    closedir(dir);
}

static void clear_video_files_on_sdcard(void)
{
    DIR *dir = opendir("/sdcard");

    if (!dir) {
        ESP_LOGW(TAG, "failed to open sdcard directory for video cleanup");
        return;
    }

    struct dirent *entry;
    char filepath[512];

    while ((entry = readdir(dir)) != NULL) {
        const char *name = entry->d_name;

        if (strstr(name, ".mjpeg") || strstr(name, ".mjpg")) {
            snprintf(filepath, sizeof(filepath), "/sdcard/%s", name);
            if (remove(filepath) == 0) {
                ESP_LOGI(TAG, "removed old video file: %s", filepath);
            } else {
                ESP_LOGW(TAG, "failed to remove old video file: %s", filepath);
            }
        }
    }

    closedir(dir);
}

// 获取最新照片
static esp_err_t get_latest_photo_handler(httpd_req_t *req)
{
    DIR *dir;
    struct dirent *entry;
    struct stat file_stat;
    char latest_file[256] = {0};
    char filepath[512];
    time_t latest_time = 0;
    
    dir = opendir("/sdcard");
    if (!dir) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No SD card");
        return ESP_FAIL;
    }
    
    while ((entry = readdir(dir)) != NULL) {
        if (strstr(entry->d_name, ".jpg") || strstr(entry->d_name, ".jpeg")) {
            snprintf(filepath, sizeof(filepath), "/sdcard/%s", entry->d_name);
            if (stat(filepath, &file_stat) == 0) {
                if (file_stat.st_mtime > latest_time) {
                    latest_time = file_stat.st_mtime;
                    strlcpy(latest_file, entry->d_name, sizeof(latest_file));
                }
            }
        }
    }
    closedir(dir);
    
    if (latest_file[0] == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No photo found");
        return ESP_FAIL;
    }
    
    snprintf(filepath, sizeof(filepath), "/sdcard/%s", latest_file);
    FILE *f = fopen(filepath, "rb");
    if (!f) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_FAIL;
    }
    
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"photo.jpg\"");
    
    char *buf = malloc(4096);
    if (!buf) {
        fclose(f);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }
    
    while (size > 0) {
        int read = fread(buf, 1, 4096, f);
        if (read > 0) {
            httpd_resp_send_chunk(req, buf, read);
            size -= read;
        } else {
            break;
        }
    }
    free(buf);
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    
    return ESP_OK;
}

// 获取最新录像
static esp_err_t get_latest_video_handler(httpd_req_t *req)
{
    struct dirent *entry;
    struct stat file_stat;
    char latest_file[256] = {0};
    char filepath[512] = {0};
    bool tmp_fallback = false;
    FILE *f = NULL;
    long selected_size = 0;
    const int max_retry = 20;
    const TickType_t retry_delay = pdMS_TO_TICKS(100);

    // 录像刚结束时可能还在 flush/rename，短暂重试可避免首次请求 404
    for (int attempt = 0; attempt < max_retry; attempt++) {
        DIR *dir = opendir("/sdcard");
        time_t latest_time = 0;

        latest_file[0] = '\0';
        tmp_fallback = false;

        if (!dir) {
            httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No SD card");
            return ESP_FAIL;
        }

        while ((entry = readdir(dir)) != NULL) {
            if (strstr(entry->d_name, ".mjpeg") || strstr(entry->d_name, ".mjpg")) {
                snprintf(filepath, sizeof(filepath), "/sdcard/%s", entry->d_name);
                if (stat(filepath, &file_stat) == 0 && file_stat.st_mtime > latest_time) {
                    latest_time = file_stat.st_mtime;
                    strlcpy(latest_file, entry->d_name, sizeof(latest_file));
                }
            }
        }
        closedir(dir);

        if (latest_file[0] == 0) {
            dir = opendir("/sdcard");
            if (!dir) {
                httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "No SD card");
                return ESP_FAIL;
            }
            latest_time = 0;
            while ((entry = readdir(dir)) != NULL) {
                if (strstr(entry->d_name, ".tmp")) {
                    snprintf(filepath, sizeof(filepath), "/sdcard/%s", entry->d_name);
                    if (stat(filepath, &file_stat) == 0 && file_stat.st_mtime > latest_time) {
                        latest_time = file_stat.st_mtime;
                        strlcpy(latest_file, entry->d_name, sizeof(latest_file));
                        tmp_fallback = true;
                    }
                }
            }
            closedir(dir);
        }

        if (latest_file[0] != 0) {
            snprintf(filepath, sizeof(filepath), "/sdcard/%s", latest_file);
            f = fopen(filepath, "rb");
            if (f) {
                if (fseek(f, 0, SEEK_END) == 0) {
                    long candidate_size = ftell(f);
                    if (candidate_size > 0) {
                        fseek(f, 0, SEEK_SET);
                        selected_size = candidate_size;
                        break;
                    }
                }
                ESP_LOGW(TAG, "video file not ready yet: %s", filepath);
                fclose(f);
                f = NULL;
            }
        }

        if (attempt < (max_retry - 1)) {
            vTaskDelay(retry_delay);
        }
    }

    if (!f) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File not found");
        return ESP_FAIL;
    }

    long size = selected_size;
    
    // 如果是回退到临时文件，尝试把下载文件名设为去掉 .tmp 后缀的最终名字
    char disposition[256];
    if (tmp_fallback) {
        char download_name[200];
        strlcpy(download_name, latest_file, sizeof(download_name));
        // 去掉末尾的 .tmp
        size_t ln = strlen(download_name);
        if (ln > 4 && strcmp(download_name + ln - 4, ".tmp") == 0) {
            download_name[ln - 4] = '\0';
        }
        snprintf(disposition, sizeof(disposition), "attachment; filename=\"%s\"", download_name);
        ESP_LOGW(TAG, "Serving temporary recording file: %s (as %s)", filepath, download_name);
    } else {
        snprintf(disposition, sizeof(disposition), "attachment; filename=\"video.mjpeg\"");
    }
    httpd_resp_set_type(req, "video/x-mjpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", disposition);
    
    char *buf = malloc(4096);
    if (!buf) {
        fclose(f);
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Out of memory");
        return ESP_FAIL;
    }
    
    while (size > 0) {
        int read = fread(buf, 1, 4096, f);
        if (read > 0) {
            httpd_resp_send_chunk(req, buf, read);
            size -= read;
        } else {
            break;
        }
    }
    free(buf);
    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    
    return ESP_OK;
}

static esp_err_t http_server_init(web_cam_t *web_cam)
{
    httpd_handle_t main_httpd = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.max_uri_handlers = 15;
    config.task_priority = 5;
    config.stack_size = 10240;
    ESP_RETURN_ON_ERROR(init_event_client_registry(), TAG, "failed to init event registry");

    httpd_uri_t capture_image_uri = {
        .uri = "/api/capture_image",
        .method = HTTP_GET,
        .handler = capture_image_handler,
        .user_ctx = (void *)web_cam
    };

    httpd_uri_t capture_binary_uri = {
        .uri = "/api/capture_binary",
        .method = HTTP_GET,
        .handler = capture_binary_handler,
        .user_ctx = (void *)web_cam
    };

    httpd_uri_t camera_info_uri = {
        .uri = "/api/get_camera_info",
        .method = HTTP_GET,
        .handler = camera_info_handler,
        .user_ctx = (void *)web_cam
    };

    httpd_uri_t camera_settings_uri = {
        .uri = "/api/set_camera_config",
        .method = HTTP_POST,
        .handler = camera_settings_handler,
        .user_ctx = (void *)web_cam
    };

    httpd_uri_t legacy_capture_uri = {
        .uri = "/capture",
        .method = HTTP_GET,
        .handler = legacy_capture_handler,
        .user_ctx = (void *)web_cam
    };

    httpd_uri_t legacy_status_uri = {
        .uri = "/status",
        .method = HTTP_GET,
        .handler = legacy_status_handler,
        .user_ctx = (void *)web_cam
    };

    httpd_uri_t legacy_control_uri = {
        .uri = "/control",
        .method = HTTP_GET,
        .handler = legacy_control_handler,
        .user_ctx = (void *)web_cam
    };

    httpd_uri_t button_event_ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = button_event_ws_handler,
        .user_ctx = NULL,
        .is_websocket = true,
    };
    
    httpd_uri_t latest_photo_uri = {
        .uri = "/api/latest_photo",
        .method = HTTP_GET,
        .handler = get_latest_photo_handler,
        .user_ctx = NULL
    };

    httpd_uri_t latest_video_uri = {
        .uri = "/api/latest_video",
        .method = HTTP_GET,
        .handler = get_latest_video_handler,
        .user_ctx = NULL
    };

    httpd_uri_t static_file_uri = {
        .uri = "/*",
        .method = HTTP_GET,
        .handler = static_file_handler,
        .user_ctx = (void *)web_cam
    };

    config.stack_size = 1024 * 6;
    config.server_port = 80;
    config.ctrl_port = 80;
    
    if (httpd_start(&main_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(main_httpd, &capture_image_uri);
        httpd_register_uri_handler(main_httpd, &capture_binary_uri);
        httpd_register_uri_handler(main_httpd, &camera_info_uri);
        httpd_register_uri_handler(main_httpd, &camera_settings_uri);
        httpd_register_uri_handler(main_httpd, &legacy_capture_uri);
        httpd_register_uri_handler(main_httpd, &legacy_status_uri);
        httpd_register_uri_handler(main_httpd, &legacy_control_uri);
        httpd_register_uri_handler(main_httpd, &button_event_ws_uri);
        httpd_register_uri_handler(main_httpd, &latest_photo_uri);
        httpd_register_uri_handler(main_httpd, &latest_video_uri);
        httpd_register_uri_handler(main_httpd, &static_file_uri);
    } else {
        config.server_port = 8080;
        config.ctrl_port = 8080;
        if (httpd_start(&main_httpd, &config) == ESP_OK) {
            httpd_register_uri_handler(main_httpd, &latest_photo_uri);
            httpd_register_uri_handler(main_httpd, &latest_video_uri);
        }
    }

    for (int i = 0; i < web_cam->video_count; i++) {
        if (!is_valid_web_cam(&web_cam->video[i])) {
            continue;
        }

        httpd_config_t stream_config = HTTPD_DEFAULT_CONFIG();
        stream_config.server_port = 81;
        stream_config.ctrl_port = 81;
        stream_config.stack_size = 8192;
        stream_config.task_priority = 5;
        stream_config.uri_match_fn = httpd_uri_match_wildcard;
        
        httpd_uri_t stream_0_uri = {
            .uri = "/stream",
            .method = HTTP_GET,
            .handler = image_stream_handler,
            .user_ctx = (void *) &web_cam->video[i]
        };

        httpd_uri_t ws_video_uri = {
            .uri = "/ws_video",
            .method = HTTP_GET,
            .handler = ws_video_handler,
            .user_ctx = (void *) &web_cam->video[i],
            .is_websocket = true,
        };

        httpd_handle_t video_httpd = NULL;
        if (httpd_start(&video_httpd, &stream_config) == ESP_OK) {
            httpd_register_uri_handler(video_httpd, &stream_0_uri);
            httpd_register_uri_handler(video_httpd, &ws_video_uri);
        }
    }

    return ESP_OK;
}

static esp_err_t start_cam_web_server(const web_cam_video_config_t *config, int config_count)
{
    esp_err_t ret;
    web_cam_t *web_cam;

    ESP_RETURN_ON_ERROR(new_web_cam(config, config_count, &web_cam), TAG, "Failed to new web cam");
    ESP_GOTO_ON_ERROR(http_server_init(web_cam), fail0, TAG, "Failed to init http server");
    s_runtime.web_cam = web_cam;

    return ESP_OK;

fail0:
    free_web_cam(web_cam);
    return ret;
}

static void record_timer_callback(TimerHandle_t timer)
{
    if (s_runtime.lock == NULL) {
        return;
    }

    if (xSemaphoreTake(s_runtime.lock, portMAX_DELAY) == pdPASS) {
        s_runtime.recording_active = false;
        xSemaphoreGive(s_runtime.lock);
        apply_runtime_stream_policy();
        led_stop_recording_blink();
    }
}

static esp_err_t init_runtime_state(void)
{
    if (s_runtime.lock == NULL) {
        s_runtime.lock = xSemaphoreCreateMutex();
        ESP_RETURN_ON_FALSE(s_runtime.lock != NULL, ESP_ERR_NO_MEM, TAG, "failed to create runtime lock");
    }

    if (s_runtime.record_timer == NULL) {
        s_runtime.record_timer = xTimerCreate("record_timer",
                                              pdMS_TO_TICKS(EXAMPLE_RECORD_DURATION_SEC * 1000),
                                              pdFALSE,
                                              NULL,
                                              record_timer_callback);
        ESP_RETURN_ON_FALSE(s_runtime.record_timer != NULL, ESP_ERR_NO_MEM, TAG, "failed to create record timer");
    }

    s_runtime.preview_requested = true;
    s_runtime.recording_active = false;
    s_runtime.capture_in_progress = false;
    s_runtime.stream_enabled = true;
    return ESP_OK;
}

static void capture_button_task(void *arg)
{
    static bool is_capturing = false;
    static uint32_t photo_counter = 0;
    web_cam_video_t *video = get_default_video(s_runtime.web_cam);
    
    if (is_capturing || !video) {
        vTaskDelete(NULL);
        return;
    }
    is_capturing = true;
    
    led_trigger_capture_pulse(100);
    broadcast_button_event("TAKE_PHOTO");
    
    char filename[64];
    snprintf(filename, sizeof(filename), "photo_%lu.jpg", (unsigned long)(photo_counter++));
    
    capture_photo_to_sdcard(video, filename);
    
    is_capturing = false;
    vTaskDelete(NULL);
}

static void record_button_task(void *arg)
{
    static uint32_t record_counter = 0;
    web_cam_video_t *video = get_default_video(s_runtime.web_cam);
    
    if (!video || s_recording) {
        vTaskDelete(NULL);
        return;
    }
    
    broadcast_button_event("RECORD_START");
    
    char filename[64];
    snprintf(filename, sizeof(filename), "rec_%lu.mjpeg", (unsigned long)(record_counter++));
    
    esp_err_t ret = start_recording(video, filename, EXAMPLE_RECORD_DURATION_SEC);
    if (ret == ESP_OK) {
        led_start_recording_blink(500);
        
        if (xSemaphoreTake(s_runtime.lock, portMAX_DELAY) == pdPASS) {
            s_runtime.recording_active = true;
            xSemaphoreGive(s_runtime.lock);
        }
        
        xTaskCreate(independent_recording_task, "indep_rec", 16384, video, 3, NULL);
        
        if (s_record_stop_task_handle) {
            vTaskDelete(s_record_stop_task_handle);
        }
        xTaskCreate(record_stop_task, "record_stop", 8192, NULL, 2, &s_record_stop_task_handle);
    }
    else {
        // 启动录像失败，关闭之前为长按设置的常亮指示灯
        led_set_level(0);
        ESP_LOGW(TAG, "start_recording failed");
    }
    
    vTaskDelete(NULL);
}

static void ensure_preview_from_button(void)
{
    if (xSemaphoreTake(s_runtime.lock, portMAX_DELAY) == pdPASS) {
        s_runtime.preview_requested = true;
        xSemaphoreGive(s_runtime.lock);
    }

    if (apply_runtime_stream_policy() == ESP_OK) {
        broadcast_button_event("STREAM_START");
    }
}

// 单键模式：由 button_control_task 处理短按/长按逻辑
static void handle_button_press(gpio_num_t gpio)
{
    (void)gpio; // 兼容旧调用，实际逻辑在 button_control_task 内实现
}

static void button_control_task(void *arg)
{
    const gpio_num_t button = EXAMPLE_CAPTURE_BUTTON_GPIO; // 单键
    bool stable_level = true;
    bool last_sample = true;
    int64_t last_change_ms = 0;
    bool in_pressed = false;
    int64_t press_start_ms = 0;
    bool long_started = false;

    while (1) {
        int64_t now_ms = esp_timer_get_time() / 1000;
        bool sample_high = gpio_get_level(button) != 0;

        if (sample_high != last_sample) {
            last_sample = sample_high;
            last_change_ms = now_ms;
        }

        if (sample_high != stable_level && (now_ms - last_change_ms) >= EXAMPLE_BUTTON_DEBOUNCE_MS) {
            stable_level = sample_high;

            if (!stable_level) {
                // 按下开始
                in_pressed = true;
                press_start_ms = now_ms;
                long_started = false;
                // 长按期间先常亮
                led_set_level(1);
            } else {
                // 松开
                if (in_pressed) {
                    int64_t duration = now_ms - press_start_ms;
                    in_pressed = false;

                    if (!long_started && duration < 2000) {
                        // 短按 -> 拍照
                        xTaskCreate(capture_button_task, "capture_button", 8192, NULL, 5, NULL);
                    }
                    // 若 long_started 为 true，则录像已在 2s 时开始，录像保持按配置时长
                    // 释放键后不主动停止录像（录像时长由 EXAMPLE_RECORD_DURATION_SEC 决定），
                    // 如需按释放停止，可在此处调用 stop_recording();
                    // 恢复指示灯（若未进入录像）
                    if (!long_started) {
                        // 短按后的拍照函数会处理脉冲灯，此处确保非录像时灯为关闭
                        led_set_level(0);
                    }
                }
            }
        }

        // 当按住并且达到 2s 时触发录像开始（只触发一次）
        if (in_pressed && !long_started && (now_ms - press_start_ms) >= 2000) {
            long_started = true;
            // 开始录像任务（任务内会在成功后启用闪烁指示）
            xTaskCreate(record_button_task, "record_button", 8192, NULL, 5, NULL);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static esp_err_t init_button_control(void)
{
    const gpio_num_t button = EXAMPLE_CAPTURE_BUTTON_GPIO;
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << button,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "failed to config button gpio %d", button);

    ESP_RETURN_ON_FALSE(xTaskCreate(button_control_task, "button_ctrl", 4096, NULL, 4, NULL) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "failed to create button control task");

    return ESP_OK;
}

static void initialise_mdns(void)
{
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(EXAMPLE_MDNS_HOST_NAME));
    ESP_ERROR_CHECK(mdns_instance_name_set(EXAMPLE_MDNS_INSTANCE));

    mdns_txt_item_t serviceTxtData[] = {
        {"board", CONFIG_IDF_TARGET},
        {"path", "/"}
    };

    ESP_ERROR_CHECK(mdns_service_add("ESP32-WebServer", "_http", "_tcp", 80, serviceTxtData,
                                     sizeof(serviceTxtData) / sizeof(serviceTxtData[0])));
}

sdmmc_card_t *bsp_sdcard_get_handle(void)
{
    return bsp_sdcard;
}

void bsp_sdcard_get_sdmmc_host(const int slot, sdmmc_host_t *config)
{
    assert(config);

    sdmmc_host_t host_config = SDMMC_HOST_DEFAULT();
    host_config.slot = slot;
    host_config.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

    memcpy(config, &host_config, sizeof(sdmmc_host_t));
}

void bsp_sdcard_sdmmc_get_slot(const int slot, sdmmc_slot_config_t *config)
{
    assert(config);
    memset(config, 0, sizeof(sdmmc_slot_config_t));

    config->cd = SDMMC_SLOT_NO_CD;
    config->wp = SDMMC_SLOT_NO_WP;
    config->width = 4;
    config->flags = 0;
}

esp_err_t bsp_sdcard_sdmmc_mount(bsp_sdcard_cfg_t *cfg)
{
    sdmmc_host_t sdhost = {0};
    sdmmc_slot_config_t sdslot = {0};
    const esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_BSP_SD_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
        .max_files = 5,
        .allocation_unit_size = 64 * 1024
    };
    assert(cfg);

    if (!cfg->mount) {
        cfg->mount = &mount_config;
    }

    if (!cfg->host) {
        bsp_sdcard_get_sdmmc_host(SDMMC_HOST_SLOT_0, &sdhost);
        cfg->host = &sdhost;
    }

    if (!cfg->slot.sdmmc) {
        bsp_sdcard_sdmmc_get_slot(SDMMC_HOST_SLOT_0, &sdslot);
        cfg->slot.sdmmc = &sdslot;
    }

    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = 4,
    };
    esp_err_t ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    cfg->host->pwr_ctrl_handle = pwr_ctrl_handle;

    return esp_vfs_fat_sdmmc_mount(BSP_SD_MOUNT_POINT, cfg->host, cfg->slot.sdmmc, cfg->mount, &bsp_sdcard);
}

esp_err_t bsp_sdcard_mount(void)
{
    bsp_sdcard_cfg_t cfg = {0};
    return bsp_sdcard_sdmmc_mount(&cfg);
}

static esp_err_t capture_photo_to_sdcard(web_cam_video_t *video, const char *filename)
{
    uint8_t *jpeg_data = NULL;
    uint32_t jpeg_size = 0;
    esp_err_t ret = ESP_OK;
    bool io_locked = false;
    
    if (!video) return ESP_ERR_INVALID_ARG;
    if (!filename || strlen(filename) == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    char filepath[256];
    snprintf(filepath, sizeof(filepath), "/sdcard/%s", filename);
    
    example_encoder_handle_t temp_encoder = NULL;
    uint8_t *temp_jpeg_buf = NULL;
    uint32_t temp_jpeg_size = 0;
    
    example_encoder_config_t encoder_config = {
        .width = video->width,
        .height = video->height,
        .pixel_format = video->pixel_format,
        .quality = video->capture_quality,
    };
    
    if (example_encoder_init(&encoder_config, &temp_encoder) != ESP_OK) {
        return ESP_FAIL;
    }
    if (example_encoder_alloc_output_buffer(temp_encoder, &temp_jpeg_buf, &temp_jpeg_size) != ESP_OK) {
        example_encoder_deinit(temp_encoder);
        return ESP_FAIL;
    }
    
    struct v4l2_buffer buf = {0};
    
    if (xSemaphoreTake(video->io_lock, portMAX_DELAY) != pdPASS) {
        ret = ESP_FAIL;
        goto cleanup;
    }
    io_locked = true;
    
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    if (ioctl(video->fd, VIDIOC_DQBUF, &buf) != 0) {
        ret = ESP_FAIL;
        goto cleanup;
    }
    
    if (!(buf.flags & V4L2_BUF_FLAG_DONE)) {
        ret = ESP_FAIL;
        goto cleanup;
    }
    
    if (video->pixel_format == V4L2_PIX_FMT_JPEG) {
        jpeg_size = buf.bytesused;
        jpeg_data = (uint8_t *)malloc(jpeg_size);
        if (jpeg_data) {
            memcpy(jpeg_data, video->buffer[buf.index], jpeg_size);
        } else {
            ret = ESP_ERR_NO_MEM;
        }
    } else {
        if (example_encoder_process(temp_encoder,
                                   video->buffer[buf.index],
                                   video->buffer_size,
                                   temp_jpeg_buf,
                                   temp_jpeg_size,
                                   &jpeg_size) == ESP_OK) {
            jpeg_data = (uint8_t *)malloc(jpeg_size);
            if (jpeg_data) {
                memcpy(jpeg_data, temp_jpeg_buf, jpeg_size);
            } else {
                ret = ESP_ERR_NO_MEM;
            }
        } else {
            ret = ESP_FAIL;
        }
    }
    
    ioctl(video->fd, VIDIOC_QBUF, &buf);
    xSemaphoreGive(video->io_lock);
    io_locked = false;
    
    if (ret == ESP_OK && jpeg_data && jpeg_size > 0) {
        FILE *f = fopen(filepath, "wb");
        if (f) {
            size_t written = fwrite(jpeg_data, 1, jpeg_size, f);
            fclose(f);
            if (written == jpeg_size) {
                ret = ESP_OK;
            } else {
                ret = ESP_FAIL;
            }
        } else {
            ret = ESP_FAIL;
        }
        free(jpeg_data);
    }

cleanup:
    if (io_locked) {
        ioctl(video->fd, VIDIOC_QBUF, &buf);
        xSemaphoreGive(video->io_lock);
    }
    if (temp_encoder) {
        if (temp_jpeg_buf) {
            example_encoder_free_output_buffer(temp_encoder, temp_jpeg_buf);
        }
        example_encoder_deinit(temp_encoder);
    }
    
    return ret;
}

static esp_err_t start_recording(web_cam_video_t *video, const char *filename, uint32_t duration_sec)
{
    esp_err_t ret = ESP_OK;

    if (!video) return ESP_ERR_INVALID_ARG;
    if (s_recording || s_record_finalize_pending || s_record_fd >= 0 || s_sd_write_task_handle != NULL || s_frame_queue != NULL) {
        ESP_LOGW(TAG, "recording is busy, ignore new start request");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (!filename || strlen(filename) == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_record_mutex) {
        s_record_mutex = xSemaphoreCreateMutex();
        if (!s_record_mutex) return ESP_ERR_NO_MEM;
    }
    
    xSemaphoreTake(s_record_mutex, portMAX_DELAY);
    snprintf(s_record_filename, sizeof(s_record_filename), "/sdcard/%s", filename);
    snprintf(s_record_tmp_filename, sizeof(s_record_tmp_filename), "/sdcard/%s.tmp", filename);
    ESP_GOTO_ON_ERROR(video_set_quality(video, video->record_quality), fail0, TAG, "failed to set record quality");
    
    DIR *dir = opendir("/sdcard");
    if (!dir) {
        video_set_quality(video, video->preview_quality);
        goto fail0;
    }
    closedir(dir);
    
    // 使用系统调用 open 替代 fopen 进行写入，避免带有标准库缓存的冲突
    s_record_fd = open(s_record_tmp_filename, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (s_record_fd < 0) {
        video_set_quality(video, video->preview_quality);
        goto fail0;
    }

    if (write_record_header(s_record_fd, 0, 0, video->width, video->height, video->record_quality) != ESP_OK) {
        close(s_record_fd);
        s_record_fd = -1;
        video_set_quality(video, video->preview_quality);
        unlink(s_record_tmp_filename);
        goto fail0;
    }
    
    s_recording = true;
    s_recording_video = video;
    s_record_start_us = esp_timer_get_time();
    s_record_duration_us = (int64_t)duration_sec * 1000000LL;
    
    xSemaphoreGive(s_record_mutex);
    
    return ESP_OK;

fail0:
    xSemaphoreGive(s_record_mutex);
    return ret;
}

static esp_err_t stop_recording(void)
{
    if (!s_recording) return ESP_OK;
    
    // 只负责将录制状态置否，让 independent_recording_task 跳出循环
    s_recording = false;
    
    if (s_recording_video) {
        video_set_quality(s_recording_video, s_recording_video->preview_quality);
        s_recording_video = NULL;
    }
    
    return ESP_OK;
}

static void record_stop_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(EXAMPLE_RECORD_DURATION_SEC * 1000));
    
    if (s_recording) {
        stop_recording();
        led_stop_recording_blink();
        
        if (s_runtime.lock) {
            xSemaphoreTake(s_runtime.lock, portMAX_DELAY);
            s_runtime.recording_active = false;
            xSemaphoreGive(s_runtime.lock);
        }
    }
    
    s_record_stop_task_handle = NULL;
    vTaskDelete(NULL);
}

// 独立的SD卡写入任务
static void sd_write_task(void *arg)
{
    frame_buffer_t frame;
    s_record_finalize_pending = false;
    
    while (1) {
        if (xQueueReceive(s_frame_queue, &frame, portMAX_DELAY) == pdTRUE) {
            // 收到退出信号，先回写平均帧率，再执行优雅关闭文件
            if (frame.is_exit) {
                s_record_finalize_pending = true;
                if (s_record_fd >= 0) {
                    if (write_record_header(s_record_fd, frame.avg_fps_x1000, frame.frame_count,
                                            frame.width, frame.height, frame.quality) != ESP_OK) {
                        ESP_LOGW(TAG, "failed to update record header with average fps");
                    }
                    fsync(s_record_fd);     // 强制将文件系统缓存写入SD卡
                    close(s_record_fd);     // 关闭文件，更新 FAT 表（必须执行，否则文件为 0b）
                    s_record_fd = -1;
                    if (rename(s_record_tmp_filename, s_record_filename) != 0) {
                        ESP_LOGW(TAG, "failed to rename temp record file to final name: %s", s_record_filename);
                    }
                    ESP_LOGI(TAG, "✅ Recording file closed and saved successfully.");
                }
                s_record_finalize_pending = false;
                break; // 结束任务循环
            }

            // 正常收到一帧视频数据
            if (s_record_fd >= 0 && frame.data && frame.size > 0) {
                // 直接写入 SD 卡 (FATFS 底层自带缓存，直接写单帧速度最稳)
                int written = write(s_record_fd, frame.data, frame.size);
                if (written != frame.size) {
                    ESP_LOGE(TAG, "❌ SD write failed! written=%d, expected=%lu", written, (unsigned long)frame.size);
                }
            }
            
            // 释放当前帧的内存
            if (frame.data) {
                free(frame.data);
            }
        }
    }
    
    s_sd_write_task_handle = NULL;
    vTaskDelete(NULL);
}

static void independent_recording_task(void *arg)
{
    web_cam_video_t *video = (web_cam_video_t *)arg;
    if (!video) {
        vTaskDelete(NULL);
        return;
    }
    
    if (s_frame_queue == NULL) {
        s_frame_queue = xQueueCreate(200, sizeof(frame_buffer_t));
        if (!s_frame_queue) {
            vTaskDelete(NULL);
            return;
        }
    }
    
    if (s_sd_write_task_handle == NULL) {
        xTaskCreatePinnedToCore(sd_write_task, "sd_write", 8192, NULL, 5, &s_sd_write_task_handle, 0);
    }
    
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (!video->stream_on) {
        if (xSemaphoreTake(video->io_lock, portMAX_DELAY) == pdPASS) {
            int ret = ioctl(video->fd, VIDIOC_STREAMON, &type);
            xSemaphoreGive(video->io_lock);
            if (ret == 0) {
                video->stream_on = 1;
            }
        }
    }
    
    int64_t start_us = esp_timer_get_time();
    int64_t duration_us = EXAMPLE_RECORD_DURATION_SEC * 1000000LL;
    uint32_t frame_count = 0;
    int64_t last_frame_us = 0;
    int64_t target_frame_interval_us = 1000000 / 15;
    
    // 【修改点】增加对 s_recording 的判断，保证能够及时响应停止
    while (s_recording && (esp_timer_get_time() - start_us) < duration_us) {
        int64_t now_us1 = esp_timer_get_time();
        if (now_us1 - last_frame_us < target_frame_interval_us) {
            vTaskDelay(pdMS_TO_TICKS((target_frame_interval_us - (now_us1 - last_frame_us)) / 1000));
            continue;
        }
        last_frame_us = now_us1;
        struct v4l2_buffer buf = {0};
        uint32_t jpeg_encoded_size = 0;
        
        if (xSemaphoreTake(video->io_lock, portMAX_DELAY) != pdPASS) {
            continue;
        }
        
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        
        if (ioctl(video->fd, VIDIOC_DQBUF, &buf) != 0) {
            xSemaphoreGive(video->io_lock);
            continue;
        }
        
        if (!(buf.flags & V4L2_BUF_FLAG_DONE)) {
            ioctl(video->fd, VIDIOC_QBUF, &buf);
            xSemaphoreGive(video->io_lock);
            continue;
        }
        
        if (video->pixel_format == V4L2_PIX_FMT_JPEG) {
            if (s_recording && video == s_recording_video && buf.bytesused > 0) {
                uint8_t *frame_data = (uint8_t *)heap_caps_malloc(buf.bytesused, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                if (!frame_data) {
                    frame_data = (uint8_t *)malloc(buf.bytesused);
                }

                if (frame_data) {
                    memcpy(frame_data, video->buffer[buf.index], buf.bytesused);

                    frame_buffer_t frame = {
                        .data = frame_data,
                        .size = buf.bytesused,
                        .frame_index = frame_count
                    };

                    if (xQueueSend(s_frame_queue, &frame, 0) != pdTRUE) {
                        free(frame_data);
                    } else {
                        frame_count++;
                    }
                }
            }
        } else if (xSemaphoreTake(video->sem, portMAX_DELAY) == pdPASS) {
            int ret = example_encoder_process(video->encoder_handle,
                                             video->buffer[buf.index], 
                                             video->buffer_size,
                                             video->jpeg_out_buf, 
                                             video->jpeg_out_size,
                                             &jpeg_encoded_size);
            xSemaphoreGive(video->sem);
            
            if (ret == ESP_OK && jpeg_encoded_size > 0 && s_recording && video == s_recording_video) {
                // 【修改点】优先使用外部 SPIRAM 申请内存，防止内存不足引发崩溃
                uint8_t *frame_data = (uint8_t *)heap_caps_malloc(jpeg_encoded_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
                if (!frame_data) {
                    frame_data = (uint8_t *)malloc(jpeg_encoded_size); // 降级备用
                }
                
                if (frame_data) {
                    memcpy(frame_data, video->jpeg_out_buf, jpeg_encoded_size);
                    
                    frame_buffer_t frame = {
                        .data = frame_data,
                        .size = jpeg_encoded_size,
                        .frame_index = frame_count
                    };
                    
                    if (xQueueSend(s_frame_queue, &frame, 0) != pdTRUE) {
                        free(frame_data);
                    } else {
                        frame_count++;
                    }
                }
            }
        }
        
        ioctl(video->fd, VIDIOC_QBUF, &buf);
        xSemaphoreGive(video->io_lock);
    }
    
    // 优雅退出：发送空信号告知 SD 写入任务结束
    if (s_frame_queue) {
        int64_t elapsed_us = esp_timer_get_time() - start_us;
        uint32_t avg_fps_x1000 = 0;

        if (elapsed_us > 0) {
            avg_fps_x1000 = (uint32_t)(((uint64_t)frame_count * 1000000ULL * 1000ULL) / (uint64_t)elapsed_us);
        }

        frame_buffer_t exit_frame = {
            .data = NULL,
            .size = 0,
            .frame_index = 0,
            .is_exit = true,
            .avg_fps_x1000 = avg_fps_x1000,
            .frame_count = frame_count,
            .width = video->width,
            .height = video->height,
            .quality = video->record_quality,
        };
        xQueueSend(s_frame_queue, &exit_frame, portMAX_DELAY);
        
        // 等待写卡任务处理完所有队列数据，关闭文件后销毁
        while (s_sd_write_task_handle != NULL) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        vQueueDelete(s_frame_queue);
        s_frame_queue = NULL;
    }
    
    if (s_recording) {
        stop_recording();
        led_stop_recording_blink();
    }
    
    vTaskDelete(NULL);
}

// WebSocket 视频流处理函数
static esp_err_t ws_video_handler(httpd_req_t *req)
{
    web_cam_video_t *video = (web_cam_video_t *)req->user_ctx;
    
    if (!video) {
        return ESP_FAIL;
    }
    
    if (video->encoder_handle == NULL) {
        example_encoder_config_t encoder_config = {
            .width = video->width,
            .height = video->height,
            .pixel_format = video->pixel_format,
            .quality = video->preview_quality,
        };
        if (example_encoder_init(&encoder_config, &video->encoder_handle) != ESP_OK) {
            return ESP_FAIL;
        }
        if (example_encoder_alloc_output_buffer(video->encoder_handle, &video->jpeg_out_buf, &video->jpeg_out_size) != ESP_OK) {
            return ESP_FAIL;
        }
    }
    
    const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (!video->stream_on) {
        if (xSemaphoreTake(video->io_lock, portMAX_DELAY) == pdPASS) {
            int ret = ioctl(video->fd, VIDIOC_STREAMON, &type);
            xSemaphoreGive(video->io_lock);
            if (ret == 0) {
                video->stream_on = 1;
            } else {
                return ESP_FAIL;
            }
        } else {
            return ESP_FAIL;
        }
    }
    
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_BINARY;
    ws_pkt.final = true;
    
    ESP_RETURN_ON_ERROR(video_set_quality(video, video->preview_quality), TAG, "failed to apply ws preview quality");
    
    int encoder_fail_count = 0;
    int64_t last_heartbeat = esp_timer_get_time();
    
    // 【新增】分配 WebSocket 专属发送缓存，避免与录像任务产生数据竞争(修复花屏)
    uint8_t *ws_local_buf = heap_caps_malloc(1024 * 512, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!ws_local_buf) {
        ESP_LOGE(TAG, "[WS_VIDEO] Failed to allocate local buffer");
        return ESP_FAIL;
    }

    while (1) {
        struct v4l2_buffer buf_v4l = {0};
        uint32_t jpeg_encoded_size = 0;
        bool frame_ok = false;
        
        int64_t now = esp_timer_get_time();
        if (now - last_heartbeat > 3000000) {
            httpd_ws_frame_t heartbeat = {
                .final = true,
                .fragmented = false,
                .type = HTTPD_WS_TYPE_TEXT,
                .payload = (uint8_t *)"ping",
                .len = 4,
            };
            esp_err_t ret = httpd_ws_send_frame(req, &heartbeat);
            if (ret == ESP_OK) {
                last_heartbeat = now;
            } else {
                break;
            }
        }
        
        if (xSemaphoreTake(video->io_lock, portMAX_DELAY) != pdPASS) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        buf_v4l.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf_v4l.memory = V4L2_MEMORY_MMAP;
        
        if (ioctl(video->fd, VIDIOC_DQBUF, &buf_v4l) != 0) {
            xSemaphoreGive(video->io_lock);
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        
        if (!(buf_v4l.flags & V4L2_BUF_FLAG_DONE)) {
            ioctl(video->fd, VIDIOC_QBUF, &buf_v4l);
            xSemaphoreGive(video->io_lock);
            continue;
        }
        
        if (video->encoder_handle == NULL) {
            example_encoder_config_t encoder_config = {
                .width = video->width,
                .height = video->height,
                .pixel_format = video->pixel_format,
                .quality = video->preview_quality,
            };
            if (example_encoder_init(&encoder_config, &video->encoder_handle) == ESP_OK) {
                example_encoder_alloc_output_buffer(video->encoder_handle, &video->jpeg_out_buf, &video->jpeg_out_size);
                encoder_fail_count = 0;
            }
        }
        
        if (video->encoder_handle && xSemaphoreTake(video->sem, portMAX_DELAY) == pdPASS) {
            int ret = example_encoder_process(video->encoder_handle,
                                             video->buffer[buf_v4l.index],
                                             video->buffer_size,
                                             video->jpeg_out_buf,
                                             video->jpeg_out_size,
                                             &jpeg_encoded_size);
            
            if (ret == ESP_OK && jpeg_encoded_size > 0 && jpeg_encoded_size <= 1024 * 512) {
                // 【修改】在释放信号量之前，将数据拷贝到专属缓存
                memcpy(ws_local_buf, video->jpeg_out_buf, jpeg_encoded_size);
                frame_ok = true;
                encoder_fail_count = 0;
            } else {
                encoder_fail_count++;
                if (encoder_fail_count >= 3 && video->encoder_handle) {
                    example_encoder_free_output_buffer(video->encoder_handle, video->jpeg_out_buf);
                    example_encoder_deinit(video->encoder_handle);
                    video->encoder_handle = NULL;
                    
                    example_encoder_config_t encoder_config = {
                        .width = video->width,
                        .height = video->height,
                        .pixel_format = video->pixel_format,
                        .quality = video->preview_quality,
                    };
                    if (example_encoder_init(&encoder_config, &video->encoder_handle) == ESP_OK) {
                        example_encoder_alloc_output_buffer(video->encoder_handle, &video->jpeg_out_buf, &video->jpeg_out_size);
                        encoder_fail_count = 0;
                    }
                }
            }
            xSemaphoreGive(video->sem);
        }
        
        ioctl(video->fd, VIDIOC_QBUF, &buf_v4l);
        xSemaphoreGive(video->io_lock);
        
        if (frame_ok) {
            ws_pkt.payload = ws_local_buf; // 【修改】使用专属缓存发送
            ws_pkt.len = jpeg_encoded_size;
            
            esp_err_t ret = httpd_ws_send_frame(req, &ws_pkt);
            if (ret != ESP_OK) {
                break;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(33));
    }
    
    if (ws_local_buf) free(ws_local_buf);
    return ESP_OK;
}

void app_main(void)
{
    esp_err_t cd1 = bsp_sdcard_mount();
    if (cd1 == ESP_OK) {
        clear_photo_files_on_sdcard();
        clear_video_files_on_sdcard();

        FILE *test = fopen("/sdcard/test.txt", "w");
        if (test) {
            fprintf(test, "test");
            fclose(test);
            unlink("/sdcard/test.txt");
        }
    }
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(example_video_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(led_indicator_init());
    initialise_mdns();
    netbiosns_init();
    netbiosns_set_name(EXAMPLE_MDNS_HOST_NAME);

    ret = example_connect();
    if (ret != ESP_OK) {
        return;
    }
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    log_access_urls();

    web_cam_video_config_t config[] = {
#if EXAMPLE_ENABLE_MIPI_CSI_CAM_SENSOR
        {
            .dev_name = ESP_VIDEO_MIPI_CSI_DEVICE_NAME,
        },
#endif
#if EXAMPLE_ENABLE_DVP_CAM_SENSOR
        {
            .dev_name = ESP_VIDEO_DVP_DEVICE_NAME,
        },
#endif
#if EXAMPLE_ENABLE_SPI_CAM_0_SENSOR
        {
            .dev_name = ESP_VIDEO_SPI_DEVICE_NAME,
        },
#endif
#if EXAMPLE_ENABLE_SPI_CAM_1_SENSOR
        {
            .dev_name = ESP_VIDEO_SPI_DEVICE_1_NAME,
        },
#endif
#if EXAMPLE_ENABLE_USB_UVC_CAM_SENSOR
        {
            .dev_name = ESP_VIDEO_USB_UVC_DEVICE_NAME(0),
        },
#endif
    };

    int config_count = sizeof(config) / sizeof(config[0]);

    assert(config_count > 0);
    ESP_ERROR_CHECK(start_cam_web_server(config, config_count));
    ESP_ERROR_CHECK(init_runtime_state());
#if CONFIG_EXAMPLE_ENABLE_GPIO_BUTTON_CONTROL
    ESP_ERROR_CHECK(apply_runtime_stream_policy());
    ESP_ERROR_CHECK(init_button_control());
#endif

    ESP_LOGI(TAG, "Camera web server starts");
}