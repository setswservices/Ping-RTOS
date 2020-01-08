
/////////////////////////////////////////////////////////////////////////////////////////////
//
//	File Name:		ble_ping.c
//	Author(s):		Jeffery Bahr, Dmitriy Antonets
//	Copyright Notice:	Copyright, 2019, Ping, LLC
//
//	Purpose/Functionality:	Implements Ping custom BLE service
//
/////////////////////////////////////////////////////////////////////////////////////////////



#include "app_config.h"

#include <stdint.h>
#include <string.h>

#include "ble_ping.h"
//#include "ping_ble.h"

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "peer_manager.h"
#include "fds.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "ble_conn_state.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "timer.h"


#include "ping_config.h"


/////////////////////////////////////////////////////////////////////////////////////////////
//  Defines                                                                                                                                               //
/////////////////////////////////////////////////////////////////////////////////////////////

#define BLE_UUID_PING_TX_CHARACTERISTIC 0x0003 /**< The UUID of the TX Characteristic. */
#define BLE_UUID_PING_RX_CHARACTERISTIC 0x0002 /**< The UUID of the RX Characteristic. */

#define BLE_PING_MAX_RX_CHAR_LEN BLE_PING_MAX_DATA_LEN /**< Maximum length of the RX Characteristic (in bytes). */
#define BLE_PING_MAX_TX_CHAR_LEN BLE_PING_MAX_DATA_LEN /**< Maximum length of the TX Characteristic (in bytes). */

#define PING_BASE_UUID                                                                                    						 	\
	{                                                                                                      									\
		{                                                                                                									\
			0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E 	\
		}      																					\
	} /**< Used vendor specific UUID. */

/////////////////////////////////////////////////////////////////////////////////////////////
//  Variable and Data Structure Declarations                                                                                               //
/////////////////////////////////////////////////////////////////////////////////////////////

uint16_t hvx_sent_count = 0;

/////////////////////////////////////////////////////////////////////////////////////////////
//  Function Prototypes                                                                                                                              //
/////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////
//  Code Begins                                                                                                                                        //
/////////////////////////////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////////////////////////////
//  Note original Nordic comments, which follow                                                                                                                                                 //
/////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
//
// The on_connect() function is handling the BLE_GAP_EVT_CONNECTED event from the 
// SoftDevice.
//
// Parameter(s):
//
//	p_ping     	Ping/Nordic UART Service structure.
//	p_ble_evt 	Pointer to the event received from BLE stack.
//
//////////////////////////////////////////////////////////////////////////////

static void on_connect(ble_ping_t *p_ping, ble_evt_t const *p_ble_evt)
{
	p_ping->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

//////////////////////////////////////////////////////////////////////////////
//
// The on_disconnect() function is handling the BLE_GAP_EVT_DISCONNECTED event from the 
// SoftDevice.
//
// Parameter(s):
//
//	p_ping     	Ping/Nordic UART Service structure.
//	p_ble_evt 	Pointer to the event received from BLE stack.
//
//////////////////////////////////////////////////////////////////////////////

static void on_disconnect(ble_ping_t *p_ping, ble_evt_t const *p_ble_evt)
{
	UNUSED_PARAMETER(p_ble_evt);
	p_ping->conn_handle = BLE_CONN_HANDLE_INVALID;
}

//////////////////////////////////////////////////////////////////////////////
//
// The on_write() function is handling the BLE_GATTS_EVT_WRITE event from the 
// SoftDevice.
//
// Parameter(s):
//
//	p_ping     	Ping/Nordic UART Service structure.
//	p_ble_evt 	Pointer to the event received from BLE stack.
//
//////////////////////////////////////////////////////////////////////////////

static void on_write(ble_ping_t *p_ping, ble_evt_t const *p_ble_evt)
{
	ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
	ble_ping_evt_t evt;
	evt.p_ping = p_ping;
	if ((p_evt_write->handle == p_ping->tx_handles.cccd_handle) && (p_evt_write->len == 2))
	{
		if (ble_srv_is_notification_enabled(p_evt_write->data))
		{
			p_ping->is_notification_enabled = true;
			evt.type = BLE_PING_EVT_COMM_STARTED;
		}
		else
		{
			p_ping->is_notification_enabled = false;
			evt.type = BLE_PING_EVT_COMM_STOPPED;
		}
		p_ping->data_handler(&evt);
	}
	else if ((p_evt_write->handle == p_ping->rx_handles.value_handle) && (p_ping->data_handler != NULL))
	{
		evt.params.rx_data.p_data = p_evt_write->data;
		evt.params.rx_data.length = p_evt_write->len;
		evt.type = BLE_PING_EVT_RX_DATA;
		p_ping->data_handler(&evt);
	}
	else
	{
		// Do Nothing. This event is not relevant for this service.
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// The tx_char_add() function adds a Tx characteristic.
//
// Parameter(s):
//
//	p_ping       	Ping/Nordic UART Service structure.
//	p_ping_init  	Information needed to initialize the service.
//
// Returns NRF_SUCCESS on success, otherwise an error code.
//
//////////////////////////////////////////////////////////////////////////////

static uint32_t tx_char_add(ble_ping_t *p_ping, ble_ping_init_t const *p_ping_init)
{
	/**@snippet [Adding proprietary characteristic to the SoftDevice] */
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_md_t cccd_md;
	ble_gatts_attr_t attr_char_value;
	ble_uuid_t ble_uuid;
	ble_gatts_attr_md_t attr_md;

	memset(&cccd_md, 0, sizeof(cccd_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

	cccd_md.vloc = BLE_GATTS_VLOC_STACK;

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.notify = 1;
	char_md.p_char_user_desc = NULL;
	char_md.p_char_pf = NULL;
	char_md.p_user_desc_md = NULL;
	char_md.p_cccd_md = &cccd_md;
	char_md.p_sccd_md = NULL;

	ble_uuid.type = p_ping->uuid_type;
	ble_uuid.uuid = BLE_UUID_PING_TX_CHARACTERISTIC;

	memset(&attr_md, 0, sizeof(attr_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

	attr_md.vloc = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth = 0;
	attr_md.wr_auth = 0;
	attr_md.vlen = 1;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len = sizeof(uint8_t);
	attr_char_value.init_offs = 0;
	attr_char_value.max_len = BLE_PING_MAX_TX_CHAR_LEN;

	return sd_ble_gatts_characteristic_add(p_ping->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_ping->tx_handles);

}

//////////////////////////////////////////////////////////////////////////////
//
// The rx_char_add() function adds a Rx characteristic.
//
// Parameter(s):
//
//	p_ping       	Ping/Nordic UART Service structure.
//	p_ping_init  	Information needed to initialize the service.
//
// Returns NRF_SUCCESS on success, otherwise an error code.
//
//////////////////////////////////////////////////////////////////////////////

static uint32_t rx_char_add(ble_ping_t *p_ping, const ble_ping_init_t *p_ping_init)
{
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t attr_char_value;
	ble_uuid_t ble_uuid;
	ble_gatts_attr_md_t attr_md;

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.write = 1;
	char_md.char_props.write_wo_resp = 1;
	char_md.p_char_user_desc = NULL;
	char_md.p_char_pf = NULL;
	char_md.p_user_desc_md = NULL;
	char_md.p_cccd_md = NULL;
	char_md.p_sccd_md = NULL;

	ble_uuid.type = p_ping->uuid_type;
	ble_uuid.uuid = BLE_UUID_PING_RX_CHARACTERISTIC;

	memset(&attr_md, 0, sizeof(attr_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

	attr_md.vloc = BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth = 0;
	attr_md.wr_auth = 0;
	attr_md.vlen = 1;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid = &ble_uuid;
	attr_char_value.p_attr_md = &attr_md;
	attr_char_value.init_len = 1;
	attr_char_value.init_offs = 0;
	attr_char_value.max_len = BLE_PING_MAX_RX_CHAR_LEN;

	return sd_ble_gatts_characteristic_add(p_ping->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_ping->rx_handles);
}

//////////////////////////////////////////////////////////////////////////////
//
// The ble_ping_on_ble_evt() function is the event-handler for the Ping UART custom service.
//
//  Parameter(s) are:
//
//	p_ble_evt		an event structure pointer that reports the event information
//	p_context		a context pointer that must be a parameter, but is unused
//
//////////////////////////////////////////////////////////////////////////////

void ble_ping_on_ble_evt(ble_evt_t const *p_ble_evt, void *p_context)
{
	if ((p_context == NULL) || (p_ble_evt == NULL))
	{
		return;
	}

	ble_ping_t *p_ping = (ble_ping_t *)p_context;

	switch (p_ble_evt->header.evt_id)
	{
	case BLE_GAP_EVT_CONNECTED:
		on_connect(p_ping, p_ble_evt);
			
		bBleConnected = true;
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		on_disconnect(p_ping, p_ble_evt);
		bBleConnected = false;
		break;

	case BLE_GATTS_EVT_WRITE:
		on_write(p_ping, p_ble_evt);
		break;

	case BLE_GATTS_EVT_HVN_TX_COMPLETE:
	{
		//notify with empty data that some tx was completed.
		ble_ping_evt_t evt = {
			.type = BLE_PING_EVT_TX_RDY,
			.p_ping = p_ping};
		p_ping->data_handler(&evt);

		if (hvx_sent_count > 0)
			hvx_sent_count--;

		break;
	}
	
	default:
		// No implementation needed.
		break;
	}
}

//////////////////////////////////////////////////////////////////////////////
//
// The ble_ping_init() function is the initialization function for the Ping UART custom service.
//
//  Parameter(s) are:
//
//	p_ping			the custom service structure pointer
//	p_ping_init		a pointer to a structure with the event handler
//
//////////////////////////////////////////////////////////////////////////////

uint32_t ble_ping_init(ble_ping_t *p_ping, ble_ping_init_t const *p_ping_init)
{
	uint32_t err_code;
	ble_uuid_t ble_uuid;
	ble_uuid128_t ping_base_uuid = PING_BASE_UUID;

	VERIFY_PARAM_NOT_NULL(p_ping);
	VERIFY_PARAM_NOT_NULL(p_ping_init);

	// Initialize the service structure.
	p_ping->conn_handle = BLE_CONN_HANDLE_INVALID;
	p_ping->data_handler = p_ping_init->data_handler;
	p_ping->is_notification_enabled = false;

	/**@snippet [Adding proprietary Service to the SoftDevice] */
	// Add a custom base UUID.

	err_code = sd_ble_uuid_vs_add(&ping_base_uuid, &p_ping->uuid_type);
	VERIFY_SUCCESS(err_code);

	ble_uuid.type = p_ping->uuid_type;
	ble_uuid.uuid = BLE_UUID_PING_SERVICE;

	// Add the service.
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
										&ble_uuid,
										&p_ping->service_handle);

	/**@snippet [Adding proprietary Service to the SoftDevice] */
	VERIFY_SUCCESS(err_code);

	// Add the RX Characteristic.
	err_code = rx_char_add(p_ping, p_ping_init);
	VERIFY_SUCCESS(err_code);

	// Add the TX Characteristic.
	err_code = tx_char_add(p_ping, p_ping_init);
	VERIFY_SUCCESS(err_code);

	return NRF_SUCCESS;
}

//////////////////////////////////////////////////////////////////////////////
//
// The ble_ping_string_send() function sends a string over BLE using the custom Ping UART
// service.
//
//  Parameter(s) are:
//
//	p_ping		the custom service structure pointer
//	p_string		the string to send
//	p_length		the length of the string to send
//
// Returns an SDK error or NRF_SUCCESS.
//
//////////////////////////////////////////////////////////////////////////////

uint32_t ble_ping_string_send(ble_ping_t *p_ping, uint8_t *p_string, uint16_t *p_length)
{
	ble_gatts_hvx_params_t hvx_params;
	uint32_t err_code;

	VERIFY_PARAM_NOT_NULL(p_ping);

	if ((p_ping->conn_handle == BLE_CONN_HANDLE_INVALID) || (!p_ping->is_notification_enabled))
	{
		return NRF_ERROR_INVALID_STATE;
	}

	if (*p_length > BLE_PING_MAX_DATA_LEN)
	{
		NRF_LOG_RAW_INFO("*p_length = %d > BLE_PING_MAX_DATA_LEN = %d\r\n", *p_length, BLE_PING_MAX_DATA_LEN);
	
		return NRF_ERROR_INVALID_PARAM;
	}

	memset(&hvx_params, 0, sizeof(hvx_params));

	hvx_params.handle = p_ping->tx_handles.value_handle;
	hvx_params.p_data = p_string;
	hvx_params.p_len = p_length;
	hvx_params.type = BLE_GATT_HVX_NOTIFICATION;

	err_code = sd_ble_gatts_hvx(p_ping->conn_handle, &hvx_params);

	if (err_code == NRF_SUCCESS)
		hvx_sent_count++;

	return err_code;
}

