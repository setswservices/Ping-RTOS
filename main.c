

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
//#include "sensorsim.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "bsp_btn_ble.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_drv_clock.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"

#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_ping.h"

#include "ping_config.h"
#include "Drv_sgtl5000.h"


#define DEVICE_NAME                         "Ping"                            /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "PingLLC"                   /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                    300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_DURATION                    18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(400, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(650, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                       0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      5000                                    /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       30000                                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define OSTIMER_WAIT_FOR_QUEUE              2                                       /**< Number of ticks to wait for the timer queue to be ready */

/////////////////////////////////////////////////////////////////////////////////////////////
//  Variable and Data Structure Declarations                                                                                                                                           //
/////////////////////////////////////////////////////////////////////////////////////////////

BLE_PING_DEF(m_ping);				/**< BLE Ping service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */

xSemaphoreHandle SendDataMutex = NULL;
ble_gap_addr_t MAC_Address;

uint8_t BLE_buffer[21];

volatile bool bBleConnected = false;
uint16_t currentConnectionInterval = 0;
volatile bool bPingConnected = false;
volatile bool bSendParameters = false;


static uint16_t m_conn_handle         = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t m_adv_uuids[] =                                   /**< Universally unique service identifiers. */
{
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};


static TaskHandle_t m_logger_thread;                                /**< Definition of Logger thread. */


// Each I2S access/interrupt provides AUDIO_FRAME_NUM_SAMPLES of 32-bit stereo pairs
// And, m_i2s_rx_buffer holds two input buffers for double buffering, so twice the size or I2S_BUFFER_SIZE_WORDS long, where "words" are 32-bit pairs
// So, each vector is I2S_BUFFER_SIZE_WORDS * sizeof(uint32_t) bytes long
// And each buffer contains AUDIO_FRAME_NUM_SAMPLES 32-bit pairs


uint32_t  m_i2s_tx_buffer[I2S_BUFFER_SIZE_WORDS];
uint32_t  m_i2s_rx_buffer[I2S_BUFFER_SIZE_WORDS];

int16_t Rx_Buffer[FFT_SAMPLE_SIZE * 2];	// twice the number of stereo pairs

static uint32_t sample_idx          = 0;

#define SAMPLE_LEN                  67200
static uint8_t * p_sample ;



static bool i2s_sgtl5000_driver_evt_handler(drv_sgtl5000_evt_t * p_evt)
{
    bool ret = false;
    //NRF_LOG_INFO("i2s_sgtl5000_driver_evt_handler %d", p_evt->evt);

    switch (p_evt->evt)
    {
        case DRV_SGTL5000_EVT_I2S_RX_BUF_RECEIVED:
            {
                //NRF_LOG_INFO("i2s_sgtl5000_driver_evt_handler RX BUF RECEIVED");
                // TODO: Handle RX as desired - for this example, we dont use RX for anything
                //uint16_t * p_buffer  = (uint16_t *) p_evt->param.rx_buf_received.p_data_received;
            }
            break;
        case DRV_SGTL5000_EVT_I2S_TX_BUF_REQ:
            {
                //NRF_LOG_INFO("i2s_sgtl5000_driver_evt_handler TX BUF REQ");
                
                /* Play sample! 16kHz sample played on 32kHz frequency! If frequency is changed, this approach needs to change. */
                /* Playback of this 16kHz sample depends on I2S MCK, RATIO, Alignment, format, and channels! Needs to be DIV8, RATIO 128X, alignment LEFT, format I2S, channels LEFT. */
                
                uint16_t * p_buffer  = (uint16_t *) p_evt->param.tx_buf_req.p_data_to_send;
                uint32_t i2s_buffer_size_words = p_evt->param.rx_buf_received.number_of_words;
                int16_t pcm_stream[i2s_buffer_size_words];  // int16_t - i2s_buffer_size_words size; means we only cover half of data_to_send_buffer, which is fine since we are only using LEFT channel
                
                /* Clear pcm buffer */
                memset(pcm_stream, 0, sizeof(pcm_stream));

                /* Check if playing the next part of the sample will exceed the sample size, if not, copy over part of sample to be played */
                if (sample_idx < SAMPLE_LEN)
                {
                    /* Copy sample bytes into pcm_stream (or remaining part of sample). This should fill up half the actual I2S transmit buffer. */
                    /* We only want half becuase the sample is a 16kHz sample, and we are running the SGTL500 at 32kHz; see DRV_SGTL5000_FS_31250HZ */
                    uint32_t bytes_to_copy = ((sample_idx + sizeof(pcm_stream)) < SAMPLE_LEN) ? sizeof(pcm_stream) : SAMPLE_LEN - sample_idx - 1;
                    memcpy(pcm_stream, &p_sample[sample_idx], bytes_to_copy);
                    sample_idx += bytes_to_copy;
                    ret = true;
                }
                else 
                {
                    /* End of buffer reached. */
                    sample_idx = 0;
                    ret = false;
                }
                
                /* Upsample the decompressed audio */
                /* i < i2s_buffer_size_words * 2 because we have a uint16_t buffer pointer */
                for (int i = 0, pcm_stream_idx = 0; i < i2s_buffer_size_words * 2; i += 2)
                {
                    for (int j = i; j < (i + 2); ++j)
                    {
                        p_buffer[j] = pcm_stream[pcm_stream_idx];
                    }
                    ++pcm_stream_idx;
                }
            }
            break;
    }

    return ret;
}





static void advertising_start(void * p_erase_bonds);


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_id)
    {
        case PM_EVT_BONDED_PEER_CONNECTED:
        {
            NRF_LOG_INFO("Connected to a previously bonded device.");
        } break;

        case PM_EVT_CONN_SEC_SUCCEEDED:
        {
            NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
                         ble_conn_state_role(p_evt->conn_handle),
                         p_evt->conn_handle,
                         p_evt->params.conn_sec_succeeded.procedure);
        } break;

        case PM_EVT_CONN_SEC_FAILED:
        {
            /* Often, when securing fails, it shouldn't be restarted, for security reasons.
             * Other times, it can be restarted directly.
             * Sometimes it can be restarted, but only after changing some Security Parameters.
             * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
             * Sometimes it is impossible, to secure the link, or the peer device does not support it.
             * How to handle this error is highly application dependent. */
        } break;

        case PM_EVT_CONN_SEC_CONFIG_REQ:
        {
            // Reject pairing request from an already bonded peer.
            pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
            pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
        } break;

        case PM_EVT_STORAGE_FULL:
        {
            // Run garbage collection on the flash.
            err_code = fds_gc();
            if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
            {
                // Retry.
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }
        } break;

        case PM_EVT_PEERS_DELETE_SUCCEEDED:
        {
            bool delete_bonds = false;
            advertising_start(&delete_bonds);
        } break;

        case PM_EVT_PEER_DATA_UPDATE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
        } break;

        case PM_EVT_PEER_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
        } break;

        case PM_EVT_PEERS_DELETE_FAILED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
        } break;

        case PM_EVT_ERROR_UNEXPECTED:
        {
            // Assert.
            APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
        } break;

        case PM_EVT_CONN_SEC_START:
        case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
        case PM_EVT_PEER_DELETE_SUCCEEDED:
        case PM_EVT_LOCAL_DB_CACHE_APPLIED:
        case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
            // This can happen when the local DB has changed.
        case PM_EVT_SERVICE_CHANGED_IND_SENT:
        case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
        default:
            break;
    }
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

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HEART_RATE_SENSOR_HEART_RATE_BELT);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module. */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
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


/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_hrs_init_t     hrs_init;
    ble_bas_init_t     bas_init;
    ble_dis_init_t     dis_init;
    nrf_ble_qwr_init_t qwr_init = {0};
    uint8_t            body_sensor_location;

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
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
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module. */
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


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;


    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

#ifdef NOTNEC
    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);
#endif

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
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


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief Function for the Peer Manager initialization. */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage. */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality. */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

#ifdef NOTNEC
/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void 
buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}
#endif


/**@brief Function for starting advertising. */
static void advertising_start(void * p_erase_bonds)
{
    bool erase_bonds = *(bool*)p_erase_bonds;

    if (erase_bonds)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}


#if NRF_LOG_ENABLED
/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are deferred.
 *          Thread flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void logger_thread(void * arg)
{
    UNUSED_PARAMETER(arg);

    while (1)
    {
        NRF_LOG_FLUSH();

        vTaskSuspend(NULL); // Suspend myself
    }
}
#endif //NRF_LOG_ENABLED

/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{
#if NRF_LOG_ENABLED
     vTaskResume(m_logger_thread);
#endif
}


/**@brief Function for initializing the clock.
 */
static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
}

//////////////////////////////////////////////////////////////////////////////
//  Additional Support Routines
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//
// The ble_log_mac_address() function retrieves the MAC address and forms a mask name.
//
// Parameter(s):
//
//	devname		string representing  mask name
//
// May terminate in APP_ERROR_CHECK
//
//////////////////////////////////////////////////////////////////////////////

void GetMacAddress(void)
{
	// Log our BLE address (6 bytes).
	static uint32_t err_code;
// Get BLE address.
//#if (NRF_SD_BLE_API_VERSION >= 3)
	err_code = sd_ble_gap_addr_get(&MAC_Address);
//#else
//	err_code = sd_ble_gap_address_get(&MAC_Address);
//#endif

	APP_ERROR_CHECK(err_code);

	// Because the NRF_LOG macros only support up to five varargs after the format string, we need to break this into two calls.
	//NRF_LOG_RAW_INFO("\r\n---------------------------------------------------------------------\r\n");
	//NRF_LOG_RAW_INFO("\r\nMAC Address = %02X:%02X:%02X:", MAC_Address.addr[5], MAC_Address.addr[4], MAC_Address.addr[3]);
	//NRF_LOG_RAW_INFO("%02X:%02X:%02X\r\n", MAC_Address.addr[2], MAC_Address.addr[1], MAC_Address.addr[0]);

	//if (devname != NULL)
	//	sprintf((char *)devname, "-%02X%02X%02X", MAC_Address.addr[2], MAC_Address.addr[1], MAC_Address.addr[0]);
}

//////////////////////////////////////////////////////////////////////////////
//
// The Ble_ping_send_data() function sends an arbitrary packet with Packet Type over BLE.   
//
// Parameter(s):
//
//	PingPacketType		string representing  mask name
//	BLEpacket			pointer to packet buffer
//	BLEpacketLen			packet buffer length
//
// Returns SDK error number or NRF_SUCCESS
//
// May terminate in APP_ERROR_CHECK
//
//////////////////////////////////////////////////////////////////////////////

#define	MAX_BLE_RETRIES		20

int Ble_ping_send_data(uint8_t PingPacketType, uint8_t *BLEpacket, uint8_t BLEpacketLen)
{
	uint32_t err_code;
	uint16_t nIdx, nJdx;

	
	if( BLEpacket == NULL)
	{
		return NRF_ERROR_INVALID_DATA;
	}

	if (m_ping.conn_handle == BLE_CONN_HANDLE_INVALID)
	{
		return NRF_ERROR_INVALID_STATE;
	}	

	if (xSemaphoreTake(SendDataMutex, (portMAX_DELAY - 10)) != pdPASS)
		APP_ERROR_CHECK(NRF_ERROR_MUTEX_LOCK_FAILED);


	memset(BLE_buffer, 0, sizeof(BLE_buffer));

	if(BLEpacketLen > (sizeof(BLE_buffer) - 1))
	{
		BLEpacketLen = sizeof(BLE_buffer) - 1;
	}
	
	memcpy(&BLE_buffer[1], BLEpacket, BLEpacketLen);

	
#if ENABLE_BLE_SEND_DATA_DEBUG
	NRF_LOG_RAW_INFO("*** Ble_ping_send_data: PingPacketType = %d, Len=%d, BLE_PING_MAX_DATA_LEN = %d\r\n", PingPacketType, BLEpacketLen, BLE_PING_MAX_DATA_LEN);
#endif
	

	BLE_buffer[0] = PingPacketType;

	if (m_ping.conn_handle == BLE_CONN_HANDLE_INVALID)
	{
		if (xSemaphoreGive(SendDataMutex) != pdPASS)
			APP_ERROR_CHECK(NRF_ERROR_MUTEX_UNLOCK_FAILED);

		return NRF_ERROR_INVALID_STATE;
	}

	// We could also check hvx_sent_count, which is getting tracked in ble_ping.c

	for (nJdx = 0; nJdx < 5; nJdx++)
	{
		if (hvx_sent_count > 0)
			vTaskDelay((TickType_t)(pdMS_TO_TICKS(20))); // wait 20 msec
		else
			break;
	}

	for (nIdx = 0; nIdx < MAX_BLE_RETRIES; nIdx++)
	{

#if ENABLE_BLE_SEND_DATA_DEBUG
		NRF_LOG_RAW_INFO("[%8d]Ble_ping_send_data:  PT=%02x, Len=%d, \r\n", global_msec_counter, PingPacketType, BLEpacketLen);
#endif

		ping_ble_msg_len = (BLEpacketLen + 1);
		err_code = ble_ping_string_send(&m_ping, (uint8_t *)BLE_buffer, &ping_ble_msg_len);

		if (err_code == NRF_ERROR_RESOURCES)
		{
			if (nIdx > 3)
				NRF_LOG_RAW_INFO("[%8d, %d]Ble_ping_send_data:  Insufficient resources, hvx_sent_count=%d, nIdx=%d\r\n", 
					global_msec_counter, nIdx, hvx_sent_count, nIdx);

			vTaskDelay((TickType_t)(pdMS_TO_TICKS(25 * nIdx)));
			; // wait 100 msec
		}
		else
			break;
	}

//
// We're disabling this as of Rev1.1.62 as it seems like missing a packet 
// should not stop the therapy session
//

#ifdef DISABLING_16APR2019
	if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) // We'l just miss this data, but don't stop mask
	)
	{
		APP_ERROR_CHECK(err_code);
	}
#endif

	if (xSemaphoreGive(SendDataMutex) != pdPASS)
		APP_ERROR_CHECK(NRF_ERROR_MUTEX_UNLOCK_FAILED);

	return err_code;
}


//////////////////////////////////////////////////////////////////////////////
//
// The AppTask() function is the main application task.  It initializes, manages and controls
// other functions and tasks that can only be managed after all tasks have been
// defined and activated.
//
// Parameter(s):
//
//	pvParameters		void * pointer to arbitrary argument, which is always NULL for us
//
//////////////////////////////////////////////////////////////////////////////


#define CAPTURE

#ifndef CAPTURE
int16_t TestData[256] =
{
-13751, 	-31063, -3568, 21496, 16378, -14022, -24752, 7928, 22866, 19673, -6642, -16679, 12597, 23857, 15086, -19363, -26866, 4776, 23593, 9230, 
-22033, 	-20760, 13333, 23088, 15742, -13908, -10655, 18543, 23662, 11258, -23047, -19363, 12601, 23472, -1043, -31848, -18924, 15405, 23211, 7052, 
-22065, 	-5900, 20412, 23319, 6482, -24994, -10347, 18716, 21724, -6584, -32768, -13167, 18959, 21549, -1309, -24989, 589, 21846, 22181, 1459, 
-24197, 	-590, 22373, 18756, -13143, -32768, -6361, 21525, 18410, -9893, -24029, 7310, 22683, 20411, -5127, -20712, 7970, 23844, 14938, -19492, 
-30035, 	720, 22737, 12967, -18333, -20842, 12837, 22796, 17716, -12311, -14380, 15344, 23928, 10392, -24696, -23784, 9154, 23072, 2095, -28035, 
-18982, 15162, 23219, 10016, -19550, -7187, 20086, 23459, 5036, -27703, -16069, 15580, 22021, -7961, -32768, -23602, 10124, 13929, -17325, -32768, 
-18528, 16865, 20726, -6481, -28806, -3675, 22500, 20305, -9035, -31409, -2527, 21551, 21032, -1943, -20220, 9032, 22766, 21500, -3494, -23245, 
4267, 23326, 16424, -17946, -31991, -3556, 21750, 8506, -23859, -30009, 4125, 23438, 15652, -14596, -17713, 14109, 24038, 12891, -22073, -25559, 
6765, 23433, 7817, -23659, -18966, 14714, 23129, 13878, -14979, -8174, 18585, 23725, 7651, -25902, -18563, 13727, 22412, -3819, -31568, -16005,
17098, 22793, 5195, -22881, -2968, 21781, 22822, 2070, -27568, -9702, 19276, 20400, -10720, -32768, -8797, 20263, 21164, -2325, -23625, 3687, 
22486, 21742, -4177, -26423, -1075, 22481, 16418, -18051, -32768, -7885, 21290, 15683, -15301, -26012, 6690, 22780, 19445, -8112, -21068, 8933, 
24057, 12672, -22657, -29764, 2678, 23269, 11515, -19737, -19143, 14329, 22727, 16667, -13174, -14124, 15657, 23991, 7544, -27141, -23466, 9759, 
22417, -3219, -32072, -20331, 14605, 23172, 5925, -23154, -7100, 20561, 23189, 3908, -27545, -13579, 17216, 21516,
};
#endif  // CAPTURE

uint32_t GetDominantIndexSolo(void)
{
	uint16_t nIdx, nJdx;
	uint32_t Dominant_Index;
	float fBinSize = 0.0;

	// Signal that we want to capture
	bDoCaptureRx = true;

	// Wait for capture
	while(bDoCaptureRx)   vTaskDelay((TickType_t)(pdMS_TO_TICKS(0)));

	nJdx = 0;
	nJdx = 0;

	for(nIdx=0; nIdx < FFT_SAMPLE_SIZE * 2; nIdx += 2)
	{

		fFFTin[nJdx] = (float)  Rx_Buffer[nIdx];

#ifdef PRINTIT
		sprintf(cOutbuf, "%6.0f\r\n",fFFTin[nJdx]);
		NRF_LOG_RAW_INFO("%s",cOutbuf);
		NRF_LOG_FLUSH();
#endif

		nJdx++;
	}

	Dominant_Index = ping_fft(fBinSize);
	return Dominant_Index;

}

// Get two values in a row

uint32_t GetDominantIndex(void)
{
	uint16_t nIdx, nJdx;
	uint32_t Dominant_Index, LastDominant_Index;
	float fBinSize = 0.0;

	LastDominant_Index = 9999;

	while(true)
	{
		Dominant_Index = GetDominantIndexSolo();
		if(Dominant_Index != 25) Dominant_Index = 0;
		if(Dominant_Index == LastDominant_Index) break;
		LastDominant_Index = Dominant_Index;
	}

	return Dominant_Index;

}

//  temporal 3 or the T3 pattern, smoke detectors must generate an audible signal that consists of a three pulse tonal sequence of .5 second on and .5 second off 
//  with an additional one second of silence following the third and final .5 second off portion of the cycle. One full T3 pattern takes four seconds to complete.

// 500 msec of 3200 Hz
// 500 msec of silence
// 500 msec of 3200 Hz
// 500 msec of silence
// 500 msec of 3200 Hz
// 1500 msec of silence
// 500 msec of 3200 Hz
// 500 msec of silence
// 500 msec of 3200 Hz
// 500 msec of silence
// .
// .

uint32_t nKdx;

extern uint32_t NewI2sTime, OldI2sTime, DeltaI2sTime;

void TurnOnLEDs(void)
{
	uint16_t nJdx;
	// Turn On LEDs
	for(nJdx=LED_1; nJdx <= LED_4; nJdx++)  
	{
		//NRF_LOG_RAW_INFO("LED %d OFF\r\n", nJdx);
		nrf_gpio_pin_clear(nJdx);
	}
}

void TurnOffLEDs(void)
{
	uint16_t nJdx;
	// Turn Off LEDs
	for(nJdx=LED_1; nJdx <= LED_4; nJdx++)  
	{
		//NRF_LOG_RAW_INFO("LED %d OFF\r\n", nJdx);
		nrf_gpio_pin_set(nJdx);
	}
}

static void AppTask(void *pvParameters)
{
	(void)pvParameters;

	uint32_t err_code = NRF_SUCCESS;
	uint16_t nIdx, nJdx;
	uint32_t nSeed = 0;

	static bool bBeenHere = false;
	float fBinSize;
	uint32_t Dominant_Index;
	bool bGot3200HZ = false;

	uint32_t BegTime, EndTime, DeltaTime=0;
	uint32_t BegCapture, EndCapture, DeltaCapture=0;
	
	uint32_t nIterations = 10;
	uint16_t State = 1, LastState=0;
	bool bDebugPrint = false;

#if FUNCTION_START_DEBUG
	NRF_LOG_RAW_INFO("%s Entering ..\r\n", (uint32_t) __func__);
#endif

#ifndef CAPTURE
			for(nKdx=0; nKdx < FFT_SAMPLE_SIZE; nKdx++)
			{

				fFFTin[nKdx] = (float)  TestData[nKdx];
				sprintf(cOutbuf, "[%3d]  %6.0f \n\r", nKdx, fFFTin[nKdx]);
				NRF_LOG_RAW_INFO("%s",cOutbuf);

				vTaskDelay((TickType_t)(pdMS_TO_TICKS(20)));
			}

#endif // CAPTURE

	// Get MAC Address

	GetMacAddress();

	// Because the NRF_LOG macros only support up to five varargs after the format string, we need to break this into two calls.
	NRF_LOG_RAW_INFO("\r\n---------------------------------------------------------------------\r\n");
	NRF_LOG_RAW_INFO("\r\nMAC Address = %02X:%02X:%02X:", MAC_Address.addr[5], MAC_Address.addr[4], MAC_Address.addr[3]);
	NRF_LOG_RAW_INFO("%02X:%02X:%02X\r\n", MAC_Address.addr[2], MAC_Address.addr[1], MAC_Address.addr[0]);

	// Generate a random seed

	for(nIdx=0; nIdx < 6 ; nIdx++)
	{
		nSeed += ((uint32_t) MAC_Address.addr[nIdx] ) << nIdx;	
	}

	srand(nSeed);

	NRF_LOG_RAW_INFO("Random seed is %d\r\n", nSeed);

	Timer1_Init(TIMER1_REPEAT_RATE);

	// Turn Off LEDs
	for(nJdx=LED_1; nJdx <= LED_4; nJdx++)  
	{
		NRF_LOG_RAW_INFO("LED %d OFF\r\n", nJdx);
		nrf_gpio_pin_set(nJdx);
	}

	NRF_LOG_RAW_INFO("Init started\r\n");

	// Enable audio
	drv_sgtl5000_init_t sgtl_drv_params;
	sgtl_drv_params.i2s_tx_buffer           = (void*)m_i2s_tx_buffer;
	sgtl_drv_params.i2s_rx_buffer           = (void*)m_i2s_rx_buffer;
	sgtl_drv_params.i2s_buffer_size_words   =  I2S_BUFFER_SIZE_WORDS; ; //I2S_BUFFER_SIZE_WORDS/2;
	sgtl_drv_params.i2s_evt_handler         = i2s_sgtl5000_driver_evt_handler;
	sgtl_drv_params.fs                      = DRV_SGTL5000_FS_31250HZ;

#ifdef DOING_TX
	m_i2s_tx_buffer[0] = 167;
	m_i2s_tx_buffer[I2S_BUFFER_SIZE_WORDS/2] = 167;
#endif

	NRF_LOG_RAW_INFO("AUDIO_FRAME_NUM_SAMPLES = %d\r\n", AUDIO_FRAME_NUM_SAMPLES);
	NRF_LOG_RAW_INFO("size of  m_i2s_rx_buffer %d =  %d 32-bit samples\r\n", sizeof(m_i2s_rx_buffer) / sizeof(uint32_t), I2S_BUFFER_SIZE_WORDS);
	NRF_LOG_RAW_INFO("i2s_initial_Rx_buffer addr1: %d, addr2: %d\r\n", m_i2s_rx_buffer, m_i2s_rx_buffer + I2S_BUFFER_SIZE_WORDS/2);

#ifdef CAPTURE
	drv_sgtl5000_init(&sgtl_drv_params);
	drv_sgtl5000_stop();
	NRF_LOG_RAW_INFO("Audio initialization done.\r\n");

	/* Demonstrate Mic loopback */
	NRF_LOG_RAW_INFO("Loop in main and loopback MIC data.\r\n");
	drv_sgtl5000_start_mic_listen();

#endif

// 500 msec of 3200 Hz
// 500 msec of silence
// 500 msec of 3200 Hz
// 500 msec of silence
// 500 msec of 3200 Hz
// 1500 msec of silence
// 500 msec of 3200 Hz


	/////////////////////////////////////////////////////////////
	//  State 1:  Indeterminate, listening 3200 Hz 
	//  State 2:  Heard 3200 Hz, note time and speed up sampling to 20 times a second
	//  State 3:  Entered absence of 3200 Hz, dead zone, keep sampling
	//  State 4:  Heard 3200  Hz, note time and compute delta
	//  State 5:  Delta was > 400 msec, keep listening
	//  State 6:  Entered dead zone, note time and compute delta
	//  State 7:  Delta was > 

	State = 1;
	bDebugPrint = true;

	BegTime = ElapsedTimeInMilliseconds();

	while(true)
	{

		BegCapture = ElapsedTimeInMilliseconds();

		// Signal that we want to capture
		//bDoCaptureRx = true;

		// Wait for capture
		//while(bDoCaptureRx)   vTaskDelay((TickType_t)(pdMS_TO_TICKS(0)));

			
		Dominant_Index = GetDominantIndex();
										
		EndCapture = ElapsedTimeInMilliseconds();

		DeltaCapture = EndCapture - BegCapture;
	
		if(bDebugPrint && (LastState != State))  
		{
			if(State==1) NRF_LOG_RAW_INFO("\r\n");
			NRF_LOG_RAW_INFO("S %d, LS %d (%s)", State, LastState, bGot3200HZ?"QUIET":"TONE");
			NRF_LOG_RAW_INFO(", DI=%d, TM=%d, CT=%d, I2ST=%d, NT=%d\r\n", Dominant_Index, DeltaTime, DeltaCapture, DeltaI2sTime, ElapsedTimeInMilliseconds());

			//NRF_LOG_FLUSH();
		}

		if((Dominant_Index >= 25) && (Dominant_Index <= 27)) 	
			bGot3200HZ = true;
		else 	
			bGot3200HZ = false;

		LastState = State;
		
		switch(State)
		{
			
			case 1:	// Indeterminate, listening 3200 Hz 
					TurnOffLEDs();
					if(!bGot3200HZ) 
					{
						//NRF_LOG_RAW_INFO("DI = %d\r\n", Dominant_Index);
						//NRF_LOG_FLUSH();
						vTaskDelay((TickType_t)(pdMS_TO_TICKS(25)));		// Wait 400 msec
					}
					else 
					{
						EndTime = ElapsedTimeInMilliseconds();
						DeltaTime = EndTime - BegTime;
						BegTime = ElapsedTimeInMilliseconds();

						if(DeltaTime > 600)	// We were in a long silence
							State = 2;
						else 				
							State = 1;		// We were in a short silence, or in tone, stay here					
					}
					break;
					
			case 2:	// Heard 3200 Hz, note time and speed up sampling to 20 times a second

					if(bGot3200HZ) vTaskDelay((TickType_t)(pdMS_TO_TICKS(25)));		// Wait 50 msec
					else 
					{
						EndTime = ElapsedTimeInMilliseconds();
						DeltaTime = EndTime - BegTime;
						BegTime = ElapsedTimeInMilliseconds();
						if((DeltaTime > 300) && (DeltaTime < 700))
							State = 3;
						else 
							State = 1;		// Start over
					}
					break;
					
			case 3:	// Entered absence of 3200 Hz, dead zone, keep sampling
						
					if(!bGot3200HZ) vTaskDelay((TickType_t)(pdMS_TO_TICKS(25)));		// Wait 50 msec
					else 
					{
						EndTime = ElapsedTimeInMilliseconds();
						DeltaTime = EndTime - BegTime;
						BegTime = ElapsedTimeInMilliseconds();
						if((DeltaTime > 300) && (DeltaTime < 700))

							State = 4;

						else 
							State = 1;		// Start over
					}
					break;

			case 4:	// Heard 3200  Hz, note time and compute delta
									
					if(bGot3200HZ) vTaskDelay((TickType_t)(pdMS_TO_TICKS(25)));		// Wait 50 msec
					else 
					{
						EndTime = ElapsedTimeInMilliseconds();
						DeltaTime = EndTime - BegTime;
						BegTime = ElapsedTimeInMilliseconds();
						if((DeltaTime > 300) && (DeltaTime < 700))
						
							State = 5;

						else 
							State = 1;		// Start over
					}
					break;
					
			case 5:	// Entered absence of 3200 Hz, dead zone, keep sampling
					
					if(!bGot3200HZ) vTaskDelay((TickType_t)(pdMS_TO_TICKS(25)));		// Wait 50 msec
					else 
					{
						EndTime = ElapsedTimeInMilliseconds();
						DeltaTime = EndTime - BegTime;
						BegTime = ElapsedTimeInMilliseconds();
						if((DeltaTime > 300) && (DeltaTime < 700))
							State = 6;
						else 
							State = 1;		// Start over
					}
					break;

			case 6:	// Heard 3200  Hz, note time and compute delta
				
					if(bGot3200HZ) vTaskDelay((TickType_t)(pdMS_TO_TICKS(25)));		// Wait 50 msec
					else 
					{
						EndTime = ElapsedTimeInMilliseconds();
						DeltaTime = EndTime - BegTime;
						BegTime = ElapsedTimeInMilliseconds();
						//if((DeltaTime > 300) && (DeltaTime < 700))

									
			{
					NRF_LOG_RAW_INFO("!!!  ALARM !!!\r\n", State);
					BegTime = ElapsedTimeInMilliseconds();
					State = 1;
					TurnOnLEDs();
					vTaskDelay((TickType_t)(pdMS_TO_TICKS(100)));		// Wait 50 msec
					break;

					
			}

						
						//else 
							//State = 1;		// Start over
					}
					break;

			case 7:	// Entered absence of 3200 Hz, dead zone, keep sampling

					State = 1;
					NRF_LOG_RAW_INFO("!!!  ALARM !!!\r\n", State);
					BegTime = ElapsedTimeInMilliseconds();
					break;
							
					if(!bGot3200HZ) vTaskDelay((TickType_t)(pdMS_TO_TICKS(25)));		// Wait 50 msec
					else 
					{
						EndTime = ElapsedTimeInMilliseconds();
						DeltaTime = EndTime - BegTime;
						BegTime = ElapsedTimeInMilliseconds();
						if((DeltaTime > 300) && (DeltaTime < 700))
						{
							State = 1;
							NRF_LOG_RAW_INFO("!!!  ALARM !!!\r\n", State);
						}
						else 
							State = 1;		// Start over
					}
					break;
			
			
		}	


	}


	for (;;)
	{


		if(ElapsedTimeInMilliseconds() > 1000)
		{

#ifdef CAPTURE
			// Signal that we want to capture
			bDoCaptureRx = true;

			// Wait for capture
			while(bDoCaptureRx)   vTaskDelay((TickType_t)(pdMS_TO_TICKS(1)));

			//NRF_LOG_RAW_INFO("\r\nCopied RX in %d msec\r\n", RxTimeDelta);
#endif
				
			//NRF_LOG_RAW_INFO("[%d] Num_Mic_Samples = %d, Mono FFT Sample Size = %d\n\r",ElapsedTimeInMilliseconds(), Num_Mic_Samples, FFT_SAMPLE_SIZE);
			fBinSize = ( 31250.0 /2 ) / (FFT_SAMPLE_SIZE /2 );

			//sprintf(cOutbuf, "fBinSize = %f\n\r", fBinSize); 	NRF_LOG_RAW_INFO("%s", (uint32_t) cOutbuf);
			//NRF_LOG_RAW_INFO("%s",cOutbuf);
			//NRF_LOG_FLUSH();

			// Convert stereo 16-bit samples to mono float samples, half as many

#ifdef CAPTURE
			nJdx = 0;
			nKdx = 0;

			for(nIdx=0; nIdx < FFT_SAMPLE_SIZE * 2; nIdx += 2)
			{
							
				fFFTin[nKdx] = (float)  Rx_Buffer[nIdx];

#ifdef PRINTIT
				sprintf(cOutbuf, "%6.0f\r\n",fFFTin[nKdx]);
				NRF_LOG_RAW_INFO("%s",cOutbuf);
				NRF_LOG_FLUSH();
#endif

				nKdx++;

				//vTaskDelay((TickType_t)(pdMS_TO_TICKS(20)));
			}
#endif




			BegTime = ElapsedTimeInMilliseconds();

			Dominant_Index = ping_fft(fBinSize);

			EndTime = ElapsedTimeInMilliseconds();

			DeltaTime = EndTime - BegTime;

			//if((Dominant_Index >= 51) && (Dominant_Index <= 53))
			{
				NRF_LOG_RAW_INFO("Dominant_Index = %d\r\n", Dominant_Index);
				NRF_LOG_FLUSH();
				nrf_gpio_pin_clear(LED_3);
			}
			//else
			{
				nrf_gpio_pin_set(LED_3);
			}

			//   NRF_LOG_RAW_INFO("BegTime = %d EndTime = %d\r\n", BegTime, EndTime);
			//NRF_LOG_RAW_INFO("For %d Iterations, took %d msec\r\n", nIterations, DeltaTime);


			bBeenHere = true;

			////  !!!!!!!!!!!!!!!!!!!!!!
			//NRF_LOG_RAW_INFO("!! WAITING !");
			//while(1) vTaskDelay((TickType_t)(pdMS_TO_TICKS(1000)));
			
		}

		vTaskDelay((TickType_t)(pdMS_TO_TICKS(1000)));
		NRF_LOG_FLUSH();
	}
}


//////////////////////////////////////////////////////////////////////////////
//
// The AppStart() function creates and starts FreeRTOS main application task.
// Like all task start function, it has two parameters.
//
// Parameter(s):
//
//	usStackSize	stack size to be used when invoking xTaskCreate()
//	uxPriority		scheduling priority to be used when invoking xTaskCreate()
//
//////////////////////////////////////////////////////////////////////////////

TaskHandle_t m_app_task;

void AppStart(uint16_t usStackSize, portBASE_TYPE uxPriority)
{

	BaseType_t xReturned = 0;

	NRF_LOG_RAW_INFO("%s Entered ..\r\n", (uint32_t) __func__);

	xReturned = xTaskCreate(AppTask,				/* The task that implements the main applicationl. */
							"App",		 		/* Text name assigned to the task.  This is just to assist debugging.  The kernel does not use this name itself. */
							usStackSize, 		/* The size of the stack allocated to the task. */
							NULL,				/* The parameter is not used, so NULL is passed. */
							uxPriority,  			/* The priority allocated to the task. */
							&m_app_task);		/* handle for the task */

	if (xReturned != pdPASS)
	{
		NRF_LOG_ERROR("AppTask task not created.");
		APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
	}
}


/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds;

    // Initialize modules.
    log_init();
    clock_init();

    // Do not start any interrupt that uses system functions before system initialisation.
    // The best solution is to start the OS before any other initalisation.

#if NRF_LOG_ENABLED
    // Start execution.
    if (pdPASS != xTaskCreate(logger_thread, "LOGGER", 256, NULL, 1, &m_logger_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#endif

    // Activate deep sleep mode.
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // Configure and initialize the BLE stack.
    ble_stack_init();

    // buttons_leds_init(&erase_bonds);
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();

    conn_params_init();
    peer_manager_init();

    // Create a FreeRTOS task for the BLE stack.
    // The task will run advertising_start() before entering its loop.
    nrf_sdh_freertos_init(advertising_start, &erase_bonds);

    NRF_LOG_INFO("HRS FreeRTOS example started.");

	AppStart(APP_STACKSIZE, 	APP_PRIORITY);
	
    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    for (;;)
    {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}


