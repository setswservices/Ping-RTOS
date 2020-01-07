#ifndef __PING_CONFIG_H__
#define __PING_CONFIG_H__

#include <ble.h>
#include <ble_gap.h>


#define TIMER1_REPEAT_RATE 		(1000) 	// 5 millisecond repeating timer in microseconds

#define 	NRF_SDH_BLE_GATT_MAX_MTU_SIZE	247

#define	SEC_PARAM_MITM_1		1
#define	SEC_PARAM_MITM_0		0

// Number of stereo pairs per I2S access
#define AUDIO_FRAME_NUM_SAMPLES                   			256

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
extern bool bCaptureRx;
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



#endif //  __PING_CONFIG_H__

