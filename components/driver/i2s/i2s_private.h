/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "soc/lldesc.h"
#include "soc/soc_caps.h"
#include "hal/i2s_types.h"
#include "driver/i2s_types.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#if SOC_GDMA_SUPPORTED
#include "esp_private/gdma.h"
#endif
#include "esp_pm.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// If ISR handler is allowed to run whilst cache is disabled,
// Make sure all the code and related variables used by the handler are in the SRAM
#if CONFIG_I2S_ISR_IRAM_SAFE
#define I2S_INTR_ALLOC_FLAGS    (ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_LOWMED)
#define I2S_MEM_ALLOC_CAPS      (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)
#else
#define I2S_INTR_ALLOC_FLAGS    (ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_LOWMED)
#define I2S_MEM_ALLOC_CAPS      MALLOC_CAP_DEFAULT
#endif //CONFIG_I2S_ISR_IRAM_SAFE
#define I2S_DMA_ALLOC_CAPS      (MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA)

#define I2S_NULL_POINTER_CHECK(tag, p)          ESP_RETURN_ON_FALSE((p), ESP_ERR_INVALID_ARG, tag, "input parameter '"#p"' is NULL")



/**
 * @brief i2s platform level configurations
 * @note  All i2s controllers' resources are involved
 */
typedef struct {
    portMUX_TYPE            spinlock;                   /*!< Platform level lock */
    i2s_controller_t        *controller[SOC_I2S_NUM];   /*!< Controller object */
    const char              *comp_name[SOC_I2S_NUM];    /*!< The component name that occupied i2s controller */
} i2s_platform_t;

extern i2s_platform_t g_i2s;

/**
 * @brief Initialize I2S DMA interrupt
 *
 * @param handle        I2S channel handle
 * @param intr_flag     I2S interrupt flags, `ESP_INTR_FLAG_XXX` defined in `esp_intr_alloc.h`
 * @return
 *      - ESP_OK                Initialize interrupt success
 *      - ESP_ERR_INVALID_ARG   Wrong port id or NULL pointer
 */
esp_err_t i2s_init_dma_intr(i2s_chan_handle_t handle, int intr_flag);

esp_err_t i2s_init_dma_intr_single_tx(i2s_chan_handle_t handle, int intr_flag);

/**
 * @brief Free I2S DMA descriptor and DMA buffer
 *
 * @param handle        I2S channel handle
 * @return
 *      - ESP_OK                Free success
 *      - ESP_ERR_INVALID_ARG   NULL pointer
 */
esp_err_t i2s_free_dma_desc(i2s_chan_handle_t handle);

/**
 * @brief Allocate memory for I2S DMA descriptor and DMA buffer
 *
 * @param handle        I2S channel handle
 * @param num           Number of DMA descriptors
 * @param bufsize       The DMA buffer size
 *
 * @return
 *      - ESP_OK                Allocate memory success
 *      - ESP_ERR_INVALID_ARG   NULL pointer or bufsize is too big
 *      - ESP_ERR_NO_MEM        No memory for DMA descriptor and DMA buffer
 */
esp_err_t i2s_alloc_dma_desc(i2s_chan_handle_t handle, uint32_t num, uint32_t bufsize);

/**
 * @brief Get DMA buffer size
 *
 * @param handle        I2S channel handle
 * @param data_bit_width Data bit width in one slot
 * @param dma_frame_num  Frame number in one DMA buffer
 *
 * @return
 *      - DMA buffer size
 */
uint32_t i2s_get_buf_size(i2s_chan_handle_t handle, uint32_t data_bit_width, uint32_t dma_frame_num);

/**
 * @brief Get the frequency of the source clock
 *
 * @param clk_src       clock source
 * @param mclk_freq_hz  Expected mclk frequency in Hz
 * @return
 *      - Actual source clock frequency
 */
uint32_t i2s_get_source_clk_freq(i2s_clock_src_t clk_src, uint32_t mclk_freq_hz);

/**
 * @brief Check gpio validity and attach to corresponding signal
 *
 * @param gpio          GPIO number
 * @param signal_idx    Signal index
 * @param is_input      Is input gpio
 * @param is_invert     Is invert gpio
 */
void i2s_gpio_check_and_set(gpio_num_t gpio, uint32_t signal_idx, bool is_input, bool is_invert);

/**
 * @brief Check gpio validity and output mclk signal
 *
 * @param id            I2S port id
 * @param gpio_num      GPIO number
 * @param is_apll       Is using APLL as clock source
 * @param is_invert     Is invert the GPIO
 * @return
 *      - ESP_OK                Set mclk output gpio success
 *      - ESP_ERR_INVALID_ARG   Invalid GPIO number
 */
esp_err_t i2s_check_set_mclk(i2s_port_t id, gpio_num_t gpio_num, bool is_apll, bool is_invert);

/**
 * @brief Attach data out signal and data in signal to a same gpio
 *
 * @param gpio          GPIO number
 * @param out_sig_idx   Data out signal index
 * @param in_sig_idx    Data in signal index
 */
void i2s_gpio_loopback_set(gpio_num_t gpio, uint32_t out_sig_idx, uint32_t in_sig_idx);

#ifdef __cplusplus
}
#endif
