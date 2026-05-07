/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: ESPRESSIF MIT
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 LED 指示灯 GPIO
 * 
 * @return ESP_OK 成功，其他值表示失败
 */
esp_err_t led_indicator_init(void);

/**
 * @brief 拍照时触发 LED 脉冲（拉高一段时间后恢复）
 * 
 * @param duration_ms 脉冲持续时间（毫秒）
 * @return ESP_OK 成功，其他值表示失败
 */
esp_err_t led_trigger_capture_pulse(int duration_ms);

/**
 * @brief 开始录像闪烁模式（周期闪烁）
 * 
 * @param period_ms 闪烁周期（毫秒），例如 500 = 2Hz 闪烁
 * @return ESP_OK 成功，其他值表示失败
 */
esp_err_t led_start_recording_blink(int period_ms);

/**
 * @brief 停止录像闪烁模式
 * 
 * @return ESP_OK 成功，其他值表示失败
 */
esp_err_t led_stop_recording_blink(void);

/**
 * @brief 检查是否正在录像闪烁中
 * 
 * @return true 正在闪烁中，false 未闪烁
 */
bool led_is_recording_blinking(void);

/**
 * @brief 直接设置 LED 电平
 * 
 * @param level 电平值：1=高电平(3.3V)，0=低电平(0V)
 * @return ESP_OK 成功，其他值表示失败
 */
esp_err_t led_set_level(int level);

#ifdef __cplusplus
}
#endif