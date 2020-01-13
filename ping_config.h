#ifndef __PING_CONFIG_H__
#define __PING_CONFIG_H__

#include <ble.h>
#include <ble_gap.h>


///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Ping BLE-Related
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////
extern void ble_ping_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);



#define BLE_PING_BLE_OBSERVER_PRIO			2

#define BLE_PING_DEF(_name)                                                                          \
static ble_ping_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_PING_BLE_OBSERVER_PRIO,                                                     \
                     ble_ping_on_ble_evt, &_name)


#define OPCODE_LENGTH 1
#define HANDLE_LENGTH 2

static uint16_t ping_ble_msg_len = 0;

#ifdef NOTNEC

extern void ble_ping_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context);



#define BLE_PING_BLE_OBSERVER_PRIO			2

#define BLE_PING_DEF(_name)                                                                          \
static ble_ping_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_PING_BLE_OBSERVER_PRIO,                                                     \
                     ble_ping_on_ble_evt, &_name)

#define BLE_UUID_PING_SERVICE 0x0001                      /**< The UUID of the Ping/Nordic UART Service. */


/**@brief   Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_PING_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
#else
    #define BLE_PING_MAX_DATA_LEN (BLE_GATT_MTU_SIZE_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif

typedef enum
{
	BLE_PING_EVT_RX_DATA,           /**< Data received. */
	BLE_PING_EVT_TX_RDY,            /**< Service is ready to accept new data to be transmitted. */
	BLE_PING_EVT_COMM_STARTED,      /**< Notification has been enabled. */
	BLE_PING_EVT_COMM_STOPPED,      /**< Notification has been disabled. */
} ble_ping_evt_type_t;




/* Forward declaration of the ble_nus_t type. */
typedef struct ble_ping_s ble_ping_t;

/**@brief   Ping/Nordic UART Service @ref BLE_SPING_EVT_RX_DATA event data.
 *
 * @details This structure is passed to an event when @ref BLE_PING_EVT_RX_DATA occurs.
 */
typedef struct
{
	uint8_t const * p_data;           /**< A pointer to the buffer with received data. */
	uint16_t        length;           /**< Length of received data. */
} ble_ping_evt_rx_data_t;

/**@brief   Ping/Nordic UART Service event structure.
 *
 * @details This structure is passed to an event coming from service.
 */
typedef struct
{
	ble_ping_evt_type_t type;           /**< Event type. */
	ble_ping_t * p_ping;                 /**< A pointer to the instance. */
	union
	{
		ble_ping_evt_rx_data_t rx_data; /**< @ref BLE_PING_EVT_RX_DATA event data. */
	} params;
} ble_ping_evt_t;

/**@brief   Ping/Nordic UART Service event handler type. */
typedef void (*ble_ping_data_handler_t) (ble_ping_evt_t * p_evt);

struct ble_ping_s
{
	uint8_t                  uuid_type;               /**< UUID type for Nordic UART Service Base UUID. */
	uint16_t                 service_handle;          /**< Handle of Nordic UART Service (as provided by the SoftDevice). */
	ble_gatts_char_handles_t tx_handles;              /**< Handles related to the TX characteristic (as provided by the SoftDevice). */
	ble_gatts_char_handles_t rx_handles;              /**< Handles related to the RX characteristic (as provided by the SoftDevice). */
	uint16_t                 conn_handle;             /**< Handle of the current connection (as provided by the SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection. */
	bool                     is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the RX characteristic.*/
	ble_ping_data_handler_t   data_handler;            /**< Event handler to be called for handling received data. */
};

#endif

#define TIMER1_REPEAT_RATE 		(1000) 	// 5 millisecond repeating timer in microseconds

#define	SEC_PARAM_MITM_1		1
#define	SEC_PARAM_MITM_0		0

///////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//  Audio-Related
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////

// Number of stereo pairs per I2S access
#define AUDIO_FRAME_NUM_SAMPLES                   			128

// Size of the Rx buffer in terms of samples, or 32-bit stereo pairs
#define I2S_BUFFER_SIZE_WORDS               					AUDIO_FRAME_NUM_SAMPLES * 2   // Double buffered, with AUDIO_FRAME_NUM_SAMPLES

#define FFT_SAMPLE_SIZE 						(AUDIO_FRAME_NUM_SAMPLES)
#define COMPLEX_FFT_SAMPLE_SIZE				(FFT_SAMPLE_SIZE * 2)



extern void Timer1_Init(uint32_t repeat_rate);
extern uint32_t ElapsedTimeInMilliseconds(void);
extern uint32_t ping_fft(float fBinSize);
extern void GetMacAddress(void);
extern 	ble_gap_addr_t MAC_Address;
extern void DoBLE(void);


extern uint32_t Num_Mic_Samples;

extern float fFFTin[FFT_SAMPLE_SIZE];

extern float fFFTout[COMPLEX_FFT_SAMPLE_SIZE+2];
extern char cOutbuf[128];

extern int16_t *Current_RX_Buffer;
extern bool bDoCaptureRx;
extern int16_t Rx_Buffer[FFT_SAMPLE_SIZE * 2];

extern uint32_t RxTimeBeg;
extern uint32_t RxTimeDelta;

extern uint32_t  m_i2s_tx_buffer[I2S_BUFFER_SIZE_WORDS];
extern uint32_t  m_i2s_rx_buffer[I2S_BUFFER_SIZE_WORDS];

extern volatile bool bBleConnected;
extern uint16_t hvx_sent_count;
extern volatile bool bSendParameters;
extern volatile bool bPingConnected;
extern  uint16_t currentConnectionInterval;
extern bool connectedToBondedDevice;
extern bool bEraseBonds;
extern uint32_t global_msec_counter;

#define	APP_PRIORITY					( tskIDLE_PRIORITY + 1 )
#define	APP_STACKSIZE					( 512)


#endif //  __PING_CONFIG_H__

