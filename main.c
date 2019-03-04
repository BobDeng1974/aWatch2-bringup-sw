/**
 * Copyright (c) 2015 - 2018, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/**
 * @brief Blinky Sample Application main file.
 *
 * This file contains the source code for a sample server application using the LED Button service.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
//#include "boards.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_lbs.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_spi.h"
#include "nrf_drv_qspi.h"
#include "nrf_drv_power.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_pdm.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_cli.h"
#include "nrf_cli_types.h"
#include "nrf_cli_uart.h"


#define BACK_BUTTON NRF_GPIO_PIN_MAP(1,01)
#define UP_BUTTON   NRF_GPIO_PIN_MAP(1,02)
#define CTR_BUTTON  NRF_GPIO_PIN_MAP(1,03)
#define DOWN_BUTTON NRF_GPIO_PIN_MAP(1,04)

#define DEVICE_NAME                     "aWatch2"                               /**< Name of device. Will be included in the advertising data. */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */


#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


BLE_LBS_DEF(m_lbs);                                                             /**< LED Button Service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};

/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
//    bsp_board_init(BSP_INIT_LEDS);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    ret_code_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;

    ble_uuid_t adv_uuids[] = {{LBS_UUID_SERVICE, m_lbs.uuid_type}};

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling write events to the LED characteristic.
 *
 * @param[in] p_lbs     Instance of LED Button Service to which the write applies.
 * @param[in] led_state Written/desired state of the LED.
 */
static void led_write_handler(uint16_t conn_handle, ble_lbs_t * p_lbs, uint8_t led_state)
{
    nrf_gpio_cfg_output(13);
    if (led_state)
    {
        nrf_gpio_pin_set(13);
        NRF_LOG_INFO("Received LED ON!");
    }
    else
    {
        nrf_gpio_pin_clear(13);
        NRF_LOG_INFO("Received LED OFF!");
    }
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_lbs_init_t     init     = {0};
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize LBS.
    init.led_write_handler = led_write_handler;

    err_code = ble_lbs_init(&m_lbs, &init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t           err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

//    bsp_board_led_on(ADVERTISING_LED);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
//            bsp_board_led_on(CONNECTED_LED);
//            bsp_board_led_off(ADVERTISING_LED);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
//            bsp_board_led_off(CONNECTED_LED);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    switch (pin_no)
    {
        case BACK_BUTTON:
            NRF_LOG_INFO("back button %d", button_action);
            break;
        case UP_BUTTON:
            NRF_LOG_INFO("up button %d", button_action);
            break;
        case DOWN_BUTTON:
            NRF_LOG_INFO("down button %d", button_action);
            break;
        case CTR_BUTTON:
            NRF_LOG_INFO("center button %d", button_action);
            err_code = ble_lbs_on_button_change(m_conn_handle, &m_lbs, button_action);
            if (err_code != NRF_SUCCESS &&
                err_code != BLE_ERROR_INVALID_CONN_HANDLE &&
                err_code != NRF_ERROR_INVALID_STATE &&
                err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            APP_ERROR_HANDLER(pin_no);
            break;
    }
}


/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {BACK_BUTTON, false, NRF_GPIO_PIN_PULLUP, button_event_handler},
        {UP_BUTTON, false, NRF_GPIO_PIN_PULLUP, button_event_handler},
        {CTR_BUTTON, false, NRF_GPIO_PIN_PULLUP, button_event_handler},
        {DOWN_BUTTON, false, NRF_GPIO_PIN_PULLUP, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               BUTTON_DETECTION_DELAY);
    app_button_enable();
    APP_ERROR_CHECK(err_code);
}


static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

NRF_CLI_UART_DEF(m_cli_uart_transport, 0, 64, 16);
NRF_CLI_DEF(m_cli_uart,
            "Asterix# ",
            &m_cli_uart_transport.transport,
            '\r',
            4);

static void cli_init() {
    ret_code_t ret;
    
    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
    uart_config.pseltxd = 8;
    uart_config.pselrxd = 6;
    uart_config.hwfc    = NRF_UART_HWFC_DISABLED;
    ret = nrf_cli_init(&m_cli_uart, &uart_config, true, true, NRF_LOG_SEVERITY_INFO);
    APP_ERROR_CHECK(ret);
    
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
    
    NRF_LOG_INFO("DC/DC converter is now %s", nrf_power_dcdcen_get() ? "ON" : "OFF");
    nrf_power_dcdcen_set(1);
    NRF_LOG_INFO("DC/DC converter is now %s", nrf_power_dcdcen_get() ? "ON" : "OFF");
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


static const nrf_drv_twi_t i2c_pmic = NRF_DRV_TWI_INSTANCE(0);
static const nrf_drv_twi_t i2c_sens = NRF_DRV_TWI_INSTANCE(1);

static void i2c_read(const nrf_drv_twi_t *bus, uint8_t dev, uint8_t addr)
{
    ret_code_t ret;
    uint8_t datum;
    
    ret = nrf_drv_twi_tx(bus, dev, &addr, 1, true);
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("twi tx %02x %d", dev, ret);
        return;
    }
    
    ret = nrf_drv_twi_rx(bus, dev, &datum, 1);
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("twi rx %d", ret);
        return;
    }
    
    NRF_LOG_INFO("%02x[%02x] = %02x", dev, addr, datum);
}

static void i2c_write(const nrf_drv_twi_t *bus, uint8_t dev, uint8_t addr, uint8_t datum)
{
    ret_code_t ret;
    uint8_t data[2];
    
    data[0] = addr;
    data[1] = datum;
    
    ret = nrf_drv_twi_tx(bus, dev, data, 2, false);
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("twi tx %02x %d", dev, ret);
        return;
    }
    
    NRF_LOG_INFO("%02x[%02x] <- %02x", dev, addr, datum);
}

static void i2c_init()
{
    ret_code_t ret;
    const nrf_drv_twi_config_t config_pmic =
    {
       .scl                = 4,
       .sda                = 5,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = true
    };

    ret = nrf_drv_twi_init(&i2c_pmic, &config_pmic, NULL, NULL);
    APP_ERROR_CHECK(ret);
    
    const nrf_drv_twi_config_t config_sens =
    {
       .scl                = 27,
       .sda                = 26,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = true
    };

    ret = nrf_drv_twi_init(&i2c_sens, &config_sens, NULL, NULL);
    APP_ERROR_CHECK(ret);

    nrf_drv_twi_enable(&i2c_pmic);
    nrf_drv_twi_enable(&i2c_sens);

}

static void cmd_i2c(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    if ((argc != 4 && argc != 5) || nrf_cli_help_requested(p_cli)) {
        nrf_cli_help_print(p_cli, NULL, 0);
        return;
    }
    
    const nrf_drv_twi_t *bus;
    if (!strcmp(argv[1], "pmic"))
        bus = &i2c_pmic;
    else if (!strcmp(argv[1], "sens"))
        bus = &i2c_sens;
    else {
        nrf_cli_help_print(p_cli, NULL, 0);
        return;
    }
    
    uint8_t dev = strtol(argv[2], NULL, 16);
    uint8_t adr = strtol(argv[3], NULL, 16);
    if (argc == 5) {
        uint8_t data = strtol(argv[4], NULL, 16);
        i2c_write(bus, dev, adr, data);
    } else
        i2c_read(bus, dev, adr);
}
NRF_CLI_CMD_REGISTER(i2c, NULL, "i2c [pmic|sens] dev adr [data]", cmd_i2c);


static void pmic_init() 
{
    i2c_read(&i2c_pmic, 0x28, 0x00); /* chipid maxim, 0x01 */
    i2c_write(&i2c_pmic, 0x28, 0x1E, 0x81); /* Don't panic, little Maxim. */
    i2c_write(&i2c_pmic, 0x28, 0x13, 0x19); /* LDO1 3.3v */
    i2c_write(&i2c_pmic, 0x28, 0x12, 0x02); /* LDO1 enable */
    i2c_write(&i2c_pmic, 0x28, 0x0F, 0xE8); /* Buck2 enable, burst mode, 2.2uH */
    i2c_write(&i2c_pmic, 0x28, 0x10, 0x28); /* buck2, 2.8v */
    i2c_read(&i2c_pmic, 0x28, 0x13);
    i2c_read(&i2c_pmic, 0x28, 0x12);
    
    return;
}

static void cmd_sensor(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    NRF_LOG_INFO("Sensor test...");
    /* Set up the IMU to power up the magnetometer */
    i2c_write(&i2c_sens, 0x68, 0x7e /* command */, 0x19 /* mag power mode normal */);
    i2c_read(&i2c_sens, 0x76, 0x00); /* BMP388 should be 0x50 */
    i2c_read(&i2c_sens, 0x68, 0x00); /* BMI160 chipid should be 0xD1 */
    i2c_read(&i2c_sens, 0x68, 0x03 /* PMU_STATUS */); /* hope [1:0] == 2'b01 */ 
    i2c_write(&i2c_sens, 0x68, 0x4C /* MAG_IF[1] */, 0x80 /* MAG_MANUAL_EN */);
    i2c_write(&i2c_sens, 0x68, 0x4F /* MAG_IF[4] */, 0x01 /* power on */);
    i2c_write(&i2c_sens, 0x68, 0x4E /* MAG_IF[3] */, 0x4B /* magnetometer power control */);
    i2c_write(&i2c_sens, 0x68, 0x4D /* MAG_IF[2] */, 0x40 /* BMM150 chipid */);
    i2c_read(&i2c_sens, 0x68, 0x04 /* mag X data */); /* should be 0x32 */
    i2c_read(&i2c_sens, 0x68, 0x1B /* mag X data */); /* should be 0x32 */
}
NRF_CLI_CMD_REGISTER(sensor, NULL, "Run sensor chipid tests.", cmd_sensor);

volatile static int saadc_is_done = 0;
static void saadc_event_handler(nrfx_saadc_evt_t const *p_event)
{
    if (p_event->type == NRFX_SAADC_EVT_DONE || p_event->type == NRFX_SAADC_EVT_CALIBRATEDONE)
        saadc_is_done = 1;
}

static void saadc_sample_one(uint8_t mon, const char *name, int ratio)
{
    ret_code_t ret;
    
    i2c_write(&i2c_pmic, 0x28, 0x1A, mon);

    nrf_saadc_value_t val;
    ret = nrfx_saadc_sample_convert(0, &val);
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("SAADC sample %d", ret);
        return;
    }

    /* The floats are sinking! */
    int mv = val * 600 * ratio / 0x3FFF;
    
    NRF_LOG_INFO("SAADC: %s (%02x): %d mV (%d raw)", name, mon, mv, val);
    
}

static void cmd_pmicmon(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    ret_code_t ret;

    nrfx_saadc_config_t config = NRFX_SAADC_DEFAULT_CONFIG;
    config.resolution = NRF_SAADC_RESOLUTION_14BIT;
    config.oversample = NRF_SAADC_OVERSAMPLE_DISABLED;
    
    ret = nrfx_saadc_init(&config, saadc_event_handler);
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("SAADC init %d", ret);
        return;
    }
    
    saadc_is_done = 0;
    ret = nrfx_saadc_calibrate_offset();
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("SAADC calibrate %d", ret);
        return;
    }
    while (!saadc_is_done)
        ;
    
    nrf_saadc_channel_config_t chconf = NRFX_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1);
    chconf.gain = NRF_SAADC_GAIN1_2;
    chconf.acq_time = NRF_SAADC_ACQTIME_40US;

    ret = nrfx_saadc_channel_init(0, &chconf);
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("SAADC channel %d", ret);
        return;
    }

    saadc_sample_one(0x01, "BATT ", 8);
    saadc_sample_one(0x02, "SYS  ", 8);
    saadc_sample_one(0x04, "BUCK2", 8);
    saadc_sample_one(0x05, "LDO1 ", 8);
}
NRF_CLI_CMD_REGISTER(pmicmon, NULL, "Display PMIC monitor voltages.", cmd_pmicmon);

static const nrf_drv_spi_t m_spi_master_1 = NRF_DRV_SPI_INSTANCE(2);

static const uint8_t rebble[] = {
#include "rebble-bw.h"
};

static void cmd_display(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    ret_code_t ret;

    nrf_drv_spi_config_t config = NRF_DRV_SPI_DEFAULT_CONFIG;
    config.mosi_pin  = 15;
    config.sck_pin   = 14;
    config.frequency = NRF_DRV_SPI_FREQ_500K;
//    config.ss_pin    = 16;
// Active high -- not supported by the driver.
    nrf_gpio_cfg_output(16);
    nrf_gpio_pin_clear(16);

    
    ret = nrf_drv_spi_init(&m_spi_master_1, &config, NULL, NULL);
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("spi init %d", ret);
    }
    
    NRF_LOG_INFO("spi init OK");
    
    for (int i = 1; i <= 168; i++) {
        uint8_t displaybuf[23];
        int bufp = 0;
#define display_write(c) do { displaybuf[bufp] = c; bufp++; } while(0)
    
        nrf_gpio_pin_set(16);
    
        display_write(0x80);
        display_write(__RBIT(__REV(i)));
        for (int j = 0; j < 18; j++) {
            display_write(rebble[(168-i)*18+j]);
        }
        display_write(0);
        display_write(0);
        
        ret = nrf_drv_spi_transfer(&m_spi_master_1, displaybuf, bufp, NULL, 0);
        if (ret != NRF_SUCCESS) {
            NRF_LOG_INFO("spi tx %d", ret);
        }
        nrf_gpio_pin_clear(16);
    }
}
NRF_CLI_CMD_REGISTER(display, NULL, "Run display test.", cmd_display);


static volatile int mic_ncap;
#define MICBUFSIZ 32000
static int16_t mic_buf[MICBUFSIZ];

static void mic_event_handler(nrfx_pdm_evt_t const *const p_evt)
{
    NRF_LOG_INFO("PDM event, buffer request %d, compl bufr %p, error %d (%d left)", p_evt->buffer_requested, p_evt->buffer_released, p_evt->error, mic_ncap);
    if (p_evt->buffer_requested)
        nrfx_pdm_buffer_set(mic_buf, MICBUFSIZ);
    if (p_evt->buffer_released) {
        if (!mic_ncap) {
            nrfx_pdm_stop();
        } else
            mic_ncap--;
    }
}

static void cmd_mic(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    ret_code_t ret;
    
    /* mic power on */
    nrf_gpio_cfg_output(2);
    nrf_gpio_pin_set(2);
    
    nrfx_pdm_config_t config = NRFX_PDM_DEFAULT_CONFIG(1, 0);
    config.edge = NRF_PDM_EDGE_LEFTRISING;
    config.gain_l = NRF_PDM_GAIN_MAXIMUM;
    config.gain_r = NRF_PDM_GAIN_MAXIMUM;
    
    ret = nrfx_pdm_init(&config, mic_event_handler);
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("pdm init %d", ret);
        return;
    }
    
    mic_ncap = 2;
    NRF_LOG_INFO("capture start");
    mic_buf[2000] = 444;
    mic_buf[2001] = -555;
    nrfx_pdm_start();
    while (mic_ncap) {
        nrf_cli_process(&m_cli_uart);
        idle_state_handle();
    }
    NRF_LOG_INFO("capture end");
    nrfx_pdm_stop();
    
    int16_t min = 0x7fff;
    int16_t max = -0x7ffe;
    for (int i = 0; i < MICBUFSIZ; i++) {
        if (mic_buf[i] < min) min = mic_buf[i];
        if (mic_buf[i] > max) max = mic_buf[i];
    }
    NRF_LOG_INFO("PDM min %d %x, max %d %x", min, min, max, max);
}
NRF_CLI_CMD_REGISTER(mic, NULL, "Run mic test.", cmd_mic);


#define BSP_QSPI_SCK_PIN 19
#define BSP_QSPI_CSN_PIN 17
#define BSP_QSPI_IO0_PIN 20
#define BSP_QSPI_IO1_PIN 21
#define BSP_QSPI_IO2_PIN 22
#define BSP_QSPI_IO3_PIN 23

static void cmd_qspi(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    nrfx_qspi_config_t config = NRF_DRV_QSPI_DEFAULT_CONFIG;
    config.phy_if.sck_freq = NRF_QSPI_FREQ_32MDIV1;
    config.pins.sck_pin = 19;
    config.pins.csn_pin = 17;
    config.pins.io0_pin = 20;
    config.pins.io1_pin = 21;
    config.pins.io2_pin = 22;
    config.pins.io3_pin = 23;
    
    ret_code_t ret;
    ret = nrfx_qspi_init(&config, NULL, NULL);
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("qspi init %d", ret);
        return;
    }
    
    nrf_qspi_cinstr_conf_t instr = NRFX_QSPI_DEFAULT_CINSTR(0x9F /* JEDEC ID */, 4);
    uint8_t buf[16];
    uint8_t golden[16] = {0xAA, 0x55, 0xF0, 0xF0, 0xEA, 0x80, 0xE0, 0xB3, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF};
    ret = nrfx_qspi_cinstr_xfer(&instr, NULL, buf);
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("qspi jedec id %d", ret);
        return;
    }
    NRF_LOG_INFO("QSPI: JEDEC ID %02x %02x %02x\n", buf[0], buf[1], buf[2]);
    
    instr.opcode = 0x35 /* read SR2 */;
    instr.length = 2;
    ret = nrfx_qspi_cinstr_xfer(&instr, NULL, buf);
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("qspi sr2 %d", ret);
        return;
    }
    NRF_LOG_INFO("QSPI: SR2.QE = %d\n", !!(buf[0] & 2));
    
    ret = nrfx_qspi_read(buf, 16, 0);
    if (ret != NRF_SUCCESS) {
        NRF_LOG_INFO("qspi read %d", ret);
        return;
    }
    if (memcmp(buf, golden, 16)) {
        NRF_LOG_INFO("QSPI: writing at 0x0...");
        ret = nrfx_qspi_erase(NRF_QSPI_ERASE_LEN_4KB, 0x0);
        if (ret != NRF_SUCCESS) {
            NRF_LOG_INFO("qspi erase %d", ret);
            return;
        }
        
        ret = nrfx_qspi_write(golden, 16, 0);
        if (ret != NRF_SUCCESS) {
            NRF_LOG_INFO("qspi write %d", ret);
            return;
        }
    } else {
        NRF_LOG_INFO("QSPI: readback matches golden");
    }
}
NRF_CLI_CMD_REGISTER(qspi, NULL, "Run QSPI test.", cmd_qspi);


/**@brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    cli_init();
    log_init();

    nrf_gpio_cfg_output(13);
    nrf_gpio_pin_set(13);

    leds_init();
    timers_init();
    buttons_init();
    power_management_init();
    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    i2c_init();
    pmic_init();

    // Start execution.
    NRF_LOG_INFO("hello from aWatch2");
    advertising_start();

    ret_code_t ret = nrf_cli_start(&m_cli_uart);
    APP_ERROR_CHECK(ret);
    
    nrf_gpio_pin_clear(13);

    // Enter main loop.
    for (;;)
    {
        nrf_cli_process(&m_cli_uart);
        idle_state_handle();
    }
}


/**
 * @}
 */
