/*
 * SPDX-FileCopyrightText: 2015-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "driver/i2s_types.h"
#include "hal/i2s_types.h"

#include "esp_types.h"
#include "esp_err.h"

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
#include "hal/i2s_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief get default I2S property
 */
#define I2S_CHANNEL_DEFAULT_CONFIG(i2s_num, i2s_role) { \
    .id = i2s_num, \
    .role = i2s_role, \
    .dma_desc_num = 6, \
    .dma_frame_num = 240, \
    .auto_clear = false, \
}

#define I2S_GPIO_UNUSED         GPIO_NUM_NC         /*!< Used in i2s_gpio_config_t for signals which are not used */



/**
 * @brief Group of I2S callbacks
 * @note The callbacks are all running under ISR environment
 * @note When CONFIG_I2S_ISR_IRAM_SAFE is enabled, the callback itself and functions called by it should be placed in IRAM.
 *       The variables used in the function should be in the SRAM as well.
 */
typedef struct {
    i2s_isr_callback_t on_recv;             /**< Callback of data received event, only for rx channel
                                             *   The event data includes DMA buffer address and size that just finished receiving data
                                             */
    i2s_isr_callback_t on_recv_q_ovf;       /**< Callback of receiving queue overflowed event, only for rx channel
                                             *   The event data includes buffer size that has been overwritten
                                             */
    i2s_isr_callback_t on_sent;             /**< Callback of data sent event, only for tx channel
                                             *   The event data includes DMA buffer address and size that just finished sending data
                                             */
    i2s_isr_callback_t on_send_q_ovf;       /**< Callback of sending queue overflowed event, only for tx channel
                                             *   The event data includes buffer size that has been overwritten
                                             */
} i2s_event_callbacks_t;

/**
 * @brief I2S controller channel configuration
*/
typedef struct {
    i2s_port_t          id;                 /*!< I2S port id */
    i2s_role_t          role;               /*!< I2S role, I2S_ROLE_MASTER or I2S_ROLE_SLAVE */

    /* DMA configurations */
    uint32_t            dma_desc_num;       /*!< I2S DMA buffer number, it is also the number of DMA descriptor */
    uint32_t            dma_frame_num;      /*!< I2S frame number in one DMA buffer. One frame means one-time sample data in all slots,
                                             *   it should be the multiple of '3' when the data bit width is 24.
                                             */
    bool                auto_clear;         /*!< Set to auto clear DMA TX buffer, i2s will always send zero automatically if no data to send */
} i2s_chan_config_t;

/**
 * @brief I2S channel information
 */
typedef struct {
    i2s_port_t          id;                 /*!< I2S port id */
    i2s_role_t          role;               /*!< I2S role, I2S_ROLE_MASTER or I2S_ROLE_SLAVE */
    i2s_dir_t           dir;                /*!< I2S channel direction */
    i2s_comm_mode_t     mode;               /*!< I2S channel communication mode */
    i2s_chan_handle_t   pair_chan;          /*!< I2S pair channel handle in duplex mode, always NULL in simplex mode */
} i2s_chan_info_t;

/**
 * @brief i2s channel state for checking if the operation in under right driver state
 */
typedef enum {
    I2S_CHAN_STATE_REGISTER,                /*!< i2s channel is registered (not initialized)  */
    I2S_CHAN_STATE_READY,                   /*!< i2s channel is disabled (initialized) */
    I2S_CHAN_STATE_RUNNING,                 /*!< i2s channel is idling (initialized and enabled) */
} i2s_state_t;

/**
 * @brief i2s channel level configurations
 * @note  It performs as channel handle
 */
typedef struct {
#if SOC_GDMA_SUPPORTED
    gdma_channel_handle_t   dma_chan;       /*!< gdma channel handle */
#else
    intr_handle_t           dma_chan;       /*!< interrupt channel handle */
#endif
    uint32_t                desc_num;       /*!< I2S DMA buffer number, it is also the number of DMA descriptor */
    uint32_t                frame_num;      /*!< I2S frame number in one DMA buffer. One frame means one-time sample data in all slots */
    uint32_t                buf_size;       /*!< dma buffer size */
    bool                    auto_clear;     /*!< Set to auto clear DMA TX descriptor, i2s will always send zero automatically if no data to send */
    uint32_t                rw_pos;         /*!< reading/writing pointer position */
    void                    *curr_ptr;      /*!< Pointer to current dma buffer */
    lldesc_t                **desc;         /*!< dma descriptor array */
    uint8_t                 **bufs;         /*!< dma buffer array */
} i2s_dma_t;

/**
 * @brief i2s controller level configurations
 * @note  Both i2s rx and tx channel are under its control
 */
typedef struct {
    i2s_port_t              id;             /*!< i2s port id */
    i2s_hal_context_t       hal;            /*!< hal context */
    uint32_t                chan_occupancy; /*!< channel occupancy (rx/tx) */
    bool                    full_duplex;    /*!< is full_duplex */
    i2s_chan_handle_t       tx_chan;        /*!< tx channel handler */
    i2s_chan_handle_t       rx_chan;        /*!< rx channel handler */
    int                     mclk;           /*!< MCK out pin, shared by tx/rx*/
} i2s_controller_t;

struct i2s_channel_obj_t {
    /* Channel basic information */
    i2s_controller_t        *controller;    /*!< Parent pointer to controller object */
    i2s_comm_mode_t         mode;           /*!< i2s channel communication mode */
    i2s_role_t              role;           /*!< i2s role */
    i2s_dir_t               dir;            /*!< i2s channel direction */
    i2s_dma_t               dma;            /*!< i2s dma object */
    i2s_state_t             state;          /*!< i2s driver state. Ensuring the driver working in a correct sequence */
    /* Stored configurations */
    void                    *mode_info;     /*!< Slot, clock and gpio information of each mode */
#if SOC_I2S_SUPPORTS_APLL
    bool                    apll_en;        /*!< Flag of wether APLL enabled */
#endif
    uint32_t                active_slot;    /*!< Active slot number */
    uint32_t                total_slot;     /*!< Total slot number */
    /* Locks and queues */
    SemaphoreHandle_t       mutex;          /*!< Mutex semaphore for the channel operations */
    SemaphoreHandle_t       binary;         /*!< Binary semaphore for writing / reading / enabling / disabling */
#if CONFIG_PM_ENABLE
    esp_pm_lock_handle_t    pm_lock;        /*!< Power management lock, to avoid apb clock frequency changes while i2s is working */
#endif
#if CONFIG_I2S_ISR_IRAM_SAFE
    StaticSemaphore_t       *mutex_struct;      /*!< Static mutex struct */
    StaticSemaphore_t       *binary_struct;     /*!< Static binary struct */
    StaticQueue_t           *msg_que_struct;    /*!< Static message queue struct */
    void                    *msg_que_storage;   /*!< Static message queue storage */
#endif
    QueueHandle_t           msg_queue;      /*!< Message queue handler, used for transporting data between interrupt and read/write task */
    i2s_event_callbacks_t   callbacks;      /*!< Callback functions */
    void                    *user_data;     /*!< User data for callback functions */
    void      (*start)(i2s_chan_handle_t);  /*!< start tx/rx channel */
    void      (*stop)(i2s_chan_handle_t);   /*!< stop tx/rx channel */
};

/**
 * @brief Allocate new I2S channel(s)
 * @note  The new created I2S channel handle will be REGISTERED state after it is allocated successfully.
 * @note  When the port id in channel configuration is I2S_NUM_AUTO, driver will allocate I2S port automatically
 *        on one of the i2s controller, otherwise driver will try to allocate the new channel on the selected port.
 * @note  If both tx_handle and rx_handle are not NULL, it means this I2S controller will work at full-duplex mode,
 *        the rx and tx channels will be allocated on a same I2S port in this case.
 *        Note that some configurations of tx/rx channel are shared on ESP32 and ESP32S2,
 *        so please make sure they are working at same condition and under same status(start/stop).
 *        Currently, full-duplex mode can't guarantee tx/rx channels write/read synchronously,
 *        they can only share the clock signals for now.
 * @note  If tx_handle OR rx_handle is NULL, it means this I2S controller will work at simplex mode.
 *        For ESP32 and ESP32S2, the whole I2S controller (i.e. both rx and tx channel) will be occupied,
 *        even if only one of rx or tx channel is registered.
 *        For the other targets, another channel on this controller will still available.
 *
 * @param[in]   chan_cfg    I2S controller channel configurations
 * @param[out]  ret_tx_handle   I2S channel handler used for managing the sending channel(optional)
 * @param[out]  ret_rx_handle   I2S channel handler used for managing the receiving channel(optional)
 * @return
 *      - ESP_OK    Allocate new channel(s) success
 *      - ESP_ERR_NOT_SUPPORTED The communication mode is not supported on the current chip
 *      - ESP_ERR_INVALID_ARG   NULL pointer or illegal parameter in i2s_chan_config_t
 *      - ESP_ERR_NOT_FOUND     No available I2S channel found
 */
esp_err_t i2s_new_channel(const i2s_chan_config_t *chan_cfg, i2s_chan_handle_t *ret_tx_handle, i2s_chan_handle_t *ret_rx_handle);

/**
 * @brief Delete the i2s channel
 * @note  Only allowed to be called when the i2s channel is at REGISTERED or READY state (i.e., it should stop before deleting it).
 * @note  Resource will be free automatically if all channels in one port are deleted
 *
 * @param[in]   handle      I2S channel handler
 *      - ESP_OK    Delete successfully
 *      - ESP_ERR_INVALID_ARG   NULL pointer
 */
esp_err_t i2s_del_channel(i2s_chan_handle_t handle);

/**
 * @brief Get I2S channel information
 *
 * @param[in]   handle      I2S channel handler
 * @param[out]  chan_info   I2S channel basic information
 * @return
 *      - ESP_OK    Get i2s channel information success
 *      - ESP_ERR_NOT_FOUND     The input handle doesn't match any registered I2S channels, it may not an i2s channel handle or not available any more
 *      - ESP_ERR_INVALID_ARG   The input handle or chan_info pointer is NULL
 */
esp_err_t i2s_channel_get_info(i2s_chan_handle_t handle, i2s_chan_info_t *chan_info);

/**
 * @brief Enable the i2s channel
 * @note  Only allowed to be called when the channel state is READY, (i.e., channel has been initialized, but not started)
 *        the channel will enter RUNNING state once it is enabled successfully.
 * @note  Enable the channel can start the I2S communication on hardware. It will start outputting bclk and ws signal.
 *        For mclk signal, it will start to output when initialization is finished
 *
 * @param[in]   handle      I2S channel handler
 *      - ESP_OK    Start successfully
 *      - ESP_ERR_INVALID_ARG   NULL pointer
 *      - ESP_ERR_INVALID_STATE This channel has not initialized or already started
 */
esp_err_t i2s_channel_enable(i2s_chan_handle_t handle);

/**
 * @brief Disable the i2s channel
 * @note  Only allowed to be called when the channel state is RUNNING, (i.e., channel has been started)
 *        the channel will enter READY state once it is disabled successfully.
 * @note  Disable the channel can stop the I2S communication on hardware. It will stop bclk and ws signal but not mclk signal
 *
 * @param[in]   handle      I2S channel handler
 * @return
 *      - ESP_OK    Stop successfully
 *      - ESP_ERR_INVALID_ARG   NULL pointer
 *      - ESP_ERR_INVALID_STATE This channel has not stated
 */
esp_err_t i2s_channel_disable(i2s_chan_handle_t handle);

/**
 * @brief Preload the data into TX DMA buffer
 * @note  Only allowed to be called when the channel state is READY, (i.e., channel has been initialized, but not started)
 * @note  As the initial DMA buffer has no data inside, it will transmit the empty buffer after enabled the channel,
 *        this function is used to preload the data into the DMA buffer, so that the valid data can be transmitted immediately
 *        after the channel is enabled.
 * @note  This function can be called multiple times before enabling the channel, the buffer that loaded later will be concatenated
 *        behind the former loaded buffer. But when all the DMA buffers have been loaded, no more data can be preload then, please
 *        check the `bytes_loaded` parameter to see how many bytes are loaded successfully, when the `bytes_loaded` is smaller than
 *        the `size`, it means the DMA buffers are full.
 *
 * @param[in]   tx_handle   I2S TX channel handler
 * @param[in]   src         The pointer of the source buffer to be loaded
 * @param[in]   size        The source buffer size
 * @param[out]  bytes_loaded    The bytes that successfully been loaded into the TX DMA buffer
 * @return
 *      - ESP_OK    Load data successful
 *      - ESP_ERR_INVALID_ARG   NULL pointer or not TX direction
 *      - ESP_ERR_INVALID_STATE This channel has not stated
 */
esp_err_t i2s_channel_preload_data(i2s_chan_handle_t tx_handle, const void *src, size_t size, size_t *bytes_loaded);

/**
 * @brief I2S write data
 * @note  Only allowed to be called when the channel state is RUNNING, (i.e., tx channel has been started and is not writing now)
 *        but the RUNNING only stands for the software state, it doesn't mean there is no the signal transporting on line.
 *
 * @param[in]   handle      I2S channel handler
 * @param[in]   src         The pointer of sent data buffer
 * @param[in]   size        Max data buffer length
 * @param[out]  bytes_written   Byte number that actually be sent, can be NULL if not needed
 * @param[in]   timeout_ms      Max block time
 * @return
 *      - ESP_OK    Write successfully
 *      - ESP_ERR_INVALID_ARG   NULL pointer or this handle is not tx handle
 *      - ESP_ERR_TIMEOUT       Writing timeout, no writing event received from ISR within ticks_to_wait
 *      - ESP_ERR_INVALID_STATE I2S is not ready to write
 */
esp_err_t i2s_channel_write(i2s_chan_handle_t handle, const void *src, size_t size, size_t *bytes_written, uint32_t timeout_ms);

/**
 * @brief I2S read data
 * @note  Only allowed to be called when the channel state is RUNNING
 *        but the RUNNING only stands for the software state, it doesn't mean there is no the signal transporting on line.
 *
 * @param[in]   handle      I2S channel handler
 * @param[in]   dest        The pointer of receiving data buffer
 * @param[in]   size        Max data buffer length
 * @param[out]  bytes_read      Byte number that actually be read, can be NULL if not needed
 * @param[in]   timeout_ms      Max block time
 * @return
 *      - ESP_OK    Read successfully
 *      - ESP_ERR_INVALID_ARG   NULL pointer or this handle is not rx handle
 *      - ESP_ERR_TIMEOUT       Reading timeout, no reading event received from ISR within ticks_to_wait
 *      - ESP_ERR_INVALID_STATE I2S is not ready to read
 */
esp_err_t i2s_channel_read(i2s_chan_handle_t handle, void *dest, size_t size, size_t *bytes_read, uint32_t timeout_ms);

/**
 * @brief Set event callbacks for I2S channel
 *
 * @note Only allowed to be called when the channel state is REGISTERED / READY, (i.e., before channel starts)
 * @note User can deregister a previously registered callback by calling this function and setting the callback member in the `callbacks` structure to NULL.
 * @note When CONFIG_I2S_ISR_IRAM_SAFE is enabled, the callback itself and functions called by it should be placed in IRAM.
 *       The variables used in the function should be in the SRAM as well. The `user_data` should also reside in SRAM or internal RAM as well.
 *
 * @param[in] handle        I2S channel handler
 * @param[in] callbacks     Group of callback functions
 * @param[in] user_data     User data, which will be passed to callback functions directly
 * @return
 *      - ESP_OK                Set event callbacks successfully
 *      - ESP_ERR_INVALID_ARG   Set event callbacks failed because of invalid argument
 *      - ESP_ERR_INVALID_STATE Set event callbacks failed because the current channel state is not REGISTERED or READY
 */
esp_err_t i2s_channel_register_event_callback(i2s_chan_handle_t handle, const i2s_event_callbacks_t *callbacks, void *user_data);

void i2s_tx_setup_single_tx_dma(i2s_chan_handle_t handle);
void i2s_tx_single_buffer_dma(i2s_chan_handle_t handle, const void *src, size_t size);

esp_err_t i2s_channel_setup_but_dont_start(i2s_chan_handle_t handle);
esp_err_t i2s_channel_start_already_setup(i2s_chan_handle_t handle);
uint32_t i2s_get_dma_desc_num(i2s_chan_handle_t handle);
uint8_t* i2s_get_dma_bufs(i2s_chan_handle_t handle, int index);
void* i2s_get_dma_curr_ptr(i2s_chan_handle_t handle);

#ifdef __cplusplus
}
#endif
