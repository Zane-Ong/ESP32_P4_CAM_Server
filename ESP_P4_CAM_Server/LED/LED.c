/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: ESPRESSIF MIT
 */

#include "LED.h"
#include "driver/gpio.h"          // GPIO 驱动
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"

// 静态日志标签
static const char *TAG = "LED";

// GPIO 配置
#define LED_GPIO_PIN                51      // LED 指示灯 GPIO
#define LED_CAPTURE_PULSE_MS        100     // 拍照脉冲默认持续时间（毫秒）
#define LED_BLINK_PERIOD_MS         500     // 录像闪烁默认周期（毫秒）

// 静态变量（模块私有）
static SemaphoreHandle_t s_gpio_mutex = NULL;      // GPIO 操作互斥锁
static TimerHandle_t s_blink_timer = NULL;         // 闪烁定时器
static TimerHandle_t s_pulse_timer = NULL;         // 脉冲定时器
static bool s_blink_state = false;                 // 当前闪烁状态
static bool s_is_recording = false;                // 是否正在录像中

/**
 * @brief 脉冲定时器回调函数（拍照用）
 * 定时结束后关闭 LED
 */
static void pulse_timer_callback(TimerHandle_t xTimer)
{
    if (s_gpio_mutex != NULL) {
        if (xSemaphoreTake(s_gpio_mutex, portMAX_DELAY) == pdPASS) {
            gpio_set_level(LED_GPIO_PIN, 0);
            ESP_LOGD(TAG, "Capture pulse OFF");
            xSemaphoreGive(s_gpio_mutex);
        }
    }
    
    // 删除一次性定时器
    if (s_pulse_timer != NULL) {
        xTimerDelete(s_pulse_timer, 0);
        s_pulse_timer = NULL;
    }
}

/**
 * @brief 闪烁定时器回调函数（录像用）
 * 周期性地翻转 LED 状态
 */
static void blink_timer_callback(TimerHandle_t xTimer)
{
    if (s_gpio_mutex == NULL) {
        return;
    }
    
    if (xSemaphoreTake(s_gpio_mutex, portMAX_DELAY) == pdPASS) {
        s_blink_state = !s_blink_state;
        gpio_set_level(LED_GPIO_PIN, s_blink_state ? 1 : 0);
        ESP_LOGD(TAG, "Blink: %s", s_blink_state ? "ON" : "OFF");
        xSemaphoreGive(s_gpio_mutex);
    }
}

esp_err_t led_indicator_init(void)
{
    esp_err_t ret = ESP_OK;
    
    // 配置 GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure GPIO %d", LED_GPIO_PIN);
        return ret;
    }
    
    // 初始状态为低电平
    gpio_set_level(LED_GPIO_PIN, 0);
    
    // 创建互斥锁
    if (s_gpio_mutex == NULL) {
        s_gpio_mutex = xSemaphoreCreateMutex();
        if (s_gpio_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create GPIO mutex");
            return ESP_ERR_NO_MEM;
        }
    }
    
    ESP_LOGI(TAG, "LED indicator initialized on GPIO %d", LED_GPIO_PIN);
    return ESP_OK;
}

esp_err_t led_trigger_capture_pulse(int duration_ms)
{
    if (s_gpio_mutex == NULL) {
        ESP_LOGE(TAG, "LED not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 如果正在录像中，不干扰录像闪烁
    if (s_is_recording) {
        ESP_LOGW(TAG, "Recording in progress, capture pulse ignored");
        return ESP_OK;
    }
    
    // 如果已经有脉冲定时器在运行，先删除
    if (s_pulse_timer != NULL) {
        xTimerDelete(s_pulse_timer, 0);
        s_pulse_timer = NULL;
    }
    
    // 设置 GPIO 为高电平
    if (xSemaphoreTake(s_gpio_mutex, portMAX_DELAY) == pdPASS) {
        gpio_set_level(LED_GPIO_PIN, 1);
        ESP_LOGI(TAG, "Capture pulse ON (GPIO %d, %d ms)", LED_GPIO_PIN, duration_ms);
        xSemaphoreGive(s_gpio_mutex);
    } else {
        ESP_LOGE(TAG, "Failed to take GPIO mutex");
        return ESP_FAIL;
    }
    
    // 创建一次性定时器来恢复低电平
    s_pulse_timer = xTimerCreate(
        "pulse_timer",
        pdMS_TO_TICKS(duration_ms > 0 ? duration_ms : LED_CAPTURE_PULSE_MS),
        pdFALSE,  // 单次触发
        NULL,
        pulse_timer_callback
    );
    
    if (s_pulse_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create pulse timer");
        // 无法创建定时器，立即恢复电平
        if (xSemaphoreTake(s_gpio_mutex, portMAX_DELAY) == pdPASS) {
            gpio_set_level(LED_GPIO_PIN, 0);
            xSemaphoreGive(s_gpio_mutex);
        }
        return ESP_ERR_NO_MEM;
    }
    
    if (xTimerStart(s_pulse_timer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start pulse timer");
        xTimerDelete(s_pulse_timer, 0);
        s_pulse_timer = NULL;
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t led_start_recording_blink(int period_ms)
{
    if (s_gpio_mutex == NULL) {
        ESP_LOGE(TAG, "LED not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 如果已经在闪烁中，先停止
    if (s_is_recording) {
        led_stop_recording_blink();
    }
    
    // 停止可能正在运行的脉冲定时器
    if (s_pulse_timer != NULL) {
        xTimerDelete(s_pulse_timer, 0);
        s_pulse_timer = NULL;
    }
    
    // 创建周期定时器
    int blink_period = (period_ms > 0) ? period_ms : LED_BLINK_PERIOD_MS;
    s_blink_timer = xTimerCreate(
        "blink_timer",
        pdMS_TO_TICKS(blink_period),
        pdTRUE,   // 周期触发
        NULL,
        blink_timer_callback
    );
    
    if (s_blink_timer == NULL) {
        ESP_LOGE(TAG, "Failed to create blink timer");
        return ESP_ERR_NO_MEM;
    }
    
    // 设置初始状态为 LED 亮
    s_blink_state = true;
    if (xSemaphoreTake(s_gpio_mutex, portMAX_DELAY) == pdPASS) {
        gpio_set_level(LED_GPIO_PIN, 1);
        xSemaphoreGive(s_gpio_mutex);
    }
    
    // 启动定时器
    if (xTimerStart(s_blink_timer, 0) != pdPASS) {
        ESP_LOGE(TAG, "Failed to start blink timer");
        xTimerDelete(s_blink_timer, 0);
        s_blink_timer = NULL;
        return ESP_FAIL;
    }
    
    s_is_recording = true;
    ESP_LOGI(TAG, "Recording blink started (period: %d ms)", blink_period);
    return ESP_OK;
}

esp_err_t led_stop_recording_blink(void)
{
    if (!s_is_recording) {
        return ESP_OK;
    }
    
    // 停止定时器
    if (s_blink_timer != NULL) {
        xTimerStop(s_blink_timer, 0);
        xTimerDelete(s_blink_timer, 0);
        s_blink_timer = NULL;
    }
    
    // 关闭 LED
    if (s_gpio_mutex != NULL) {
        if (xSemaphoreTake(s_gpio_mutex, portMAX_DELAY) == pdPASS) {
            gpio_set_level(LED_GPIO_PIN, 0);
            s_blink_state = false;
            xSemaphoreGive(s_gpio_mutex);
        }
    }
    
    s_is_recording = false;
    ESP_LOGI(TAG, "Recording blink stopped");
    return ESP_OK;
}

bool led_is_recording_blinking(void)
{
    return s_is_recording;
}

esp_err_t led_set_level(int level)
{
    if (s_gpio_mutex == NULL) {
        ESP_LOGE(TAG, "LED not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 如果正在录像中，不允许直接设置电平
    if (s_is_recording) {
        ESP_LOGW(TAG, "Recording in progress, direct level set ignored");
        return ESP_OK;
    }
    
    if (xSemaphoreTake(s_gpio_mutex, portMAX_DELAY) == pdPASS) {
        gpio_set_level(LED_GPIO_PIN, level ? 1 : 0);
        ESP_LOGD(TAG, "LED level set to %d", level);
        xSemaphoreGive(s_gpio_mutex);
        return ESP_OK;
    }
    
    return ESP_FAIL;
}