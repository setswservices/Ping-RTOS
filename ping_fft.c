#undef ARM_MATH_CM7

#include <stdint.h>
#include <string.h>

#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nordic_common.h"
#include "FreeRTOS.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "arm_math.h"
#include "arm_const_structs.h"

#include "ping_config.h"

/* ----------------------------------------------------------------------
* Copyright (C) 2010-2012 ARM Limited. All rights reserved.
*
* $Date:         17. January 2013
* $Revision:     V1.4.0
*
* Project:       CMSIS DSP Library
* Title:             arm_fft_bin_example_f32.c
*
* Description:   Example code demonstrating calculation of Max energy bin of
*                frequency domain of input signal.
*
* Target Processor: Cortex-M4/Cortex-M3
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*   - Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   - Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in
*     the documentation and/or other materials provided with the
*     distribution.
*   - Neither the name of ARM LIMITED nor the names of its contributors
*     may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
 * -------------------------------------------------------------------- */

///////////////////////////////////////////////////////////////////////////////////


// buckets to save FFT inputs


static float32_t FFT_magnitudes[COMPLEX_FFT_SAMPLE_SIZE];

char cOutbuf[128];

float fFFTin[FFT_SAMPLE_SIZE];

float fFFTout[COMPLEX_FFT_SAMPLE_SIZE+2];

float32_t maxvalue;

uint32_t maxindex;

arm_rfft_fast_instance_f32 S;



/* -------------------------------------------------------------------
* External Input and Output buffer Declarations for FFT Bin Example
* ------------------------------------------------------------------- */

/* ------------------------------------------------------------------
* Global variables for FFT Bin Example
* ------------------------------------------------------------------- */
uint32_t fftSize = COMPLEX_FFT_SAMPLE_SIZE;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;
/* Reference index at which max energy of bin ocuurs */
uint32_t refIndex = 213, testIndex = 0;


float32_t fft_out[FFT_SAMPLE_SIZE];
float32_t fft_magnitude[FFT_SAMPLE_SIZE];
arm_rfft_fast_instance_f32 fftInstance;
uint32_t nIdx, nJdx;



/* ----------------------------------------------------------------------
* Max magnitude FFT Bin test
* ------------------------------------------------------------------- */
uint32_t  ping_fft(float fBinSize)
{
	arm_status status;
	float32_t maxValue;

	static bool bBeenHere = false;


	float fMax;
	uint32_t MaxIdx;

	// First way

#ifdef INIT_VECTORS
	for(nIdx=0 ; nIdx < FFT_SAMPLE_SIZE; nIdx++) 
	{
		fft_out[nIdx] = 0.0;
		fft_magnitude[nIdx] = 0.0;
	}
#endif

	//if(!bBeenHere)
	{
		arm_rfft_fast_init_f32(&fftInstance, FFT_SAMPLE_SIZE);
		//NRF_LOG_RAW_INFO("Initializing FFT with Sample Size %d\r\n", FFT_SAMPLE_SIZE);
		//NRF_LOG_FLUSH();
		bBeenHere = true;
	}


	arm_rfft_fast_f32(&fftInstance, fFFTin, fft_out, 0);
	arm_cmplx_mag_f32(fft_out, fft_magnitude, FFT_SAMPLE_SIZE);
	arm_max_f32(fft_out, FFT_SAMPLE_SIZE, &maxValue, &maxindex);

#ifdef PRINT_RESULTS    
	sprintf(cOutbuf, "Max:[%ld]:%f Output=[",maxindex,maxvalue);
	NRF_LOG_RAW_INFO("%s\r\n", (uint32_t) cOutbuf);

	NRF_LOG_FLUSH();
#endif


	fMax = 0.0;
	MaxIdx = 0;
	nJdx=0;

	for(nIdx=0; nIdx<FFT_SAMPLE_SIZE / 2; nIdx++)
	{

		if(fft_magnitude[nIdx] > fMax)  
		{
			fMax = fft_magnitude[nIdx];
			MaxIdx = nIdx;
		}

#ifdef PRINT_RESULTS    
		sprintf(cOutbuf, "[%3d] %8.0f + %8.0fi,  %8.0f -> %8.0f\r\n", nIdx, fft_out[nJdx], fft_out[nJdx+1], fft_magnitude[nIdx], fft_magnitude[nIdx] * 2.0 /FFT_SAMPLE_SIZE);
		NRF_LOG_RAW_INFO("%s", (uint32_t) cOutbuf);
		vTaskDelay((TickType_t)(pdMS_TO_TICKS(50)));
		NRF_LOG_FLUSH();
#endif

		nJdx += 2;

	}

	//sprintf(cOutbuf, "MaxMag = %f, MaxIdx = %d\r\n", fMax, MaxIdx);
	//NRF_LOG_RAW_INFO("%s", (uint32_t) cOutbuf);

#ifdef PRINT_RESULTS      

	float RealPart, ImaginaryPart, Magnitude, OtherMagnitude;

	nJdx = 0;
	for(nIdx=0; nIdx<FFT_SAMPLE_SIZE; nIdx += 2)
	{
		sprintf(cOutbuf, "[%6d]", ( uint32_t)(fBinSize * nJdx));
		NRF_LOG_RAW_INFO("%s", (uint32_t) cOutbuf);
		NRF_LOG_FLUSH();

		sprintf(cOutbuf, "  %f", fFFTin[nJdx]);
		NRF_LOG_RAW_INFO("%s", (uint32_t) cOutbuf);
		NRF_LOG_FLUSH();

		RealPart = fft_out[nIdx];

		sprintf(cOutbuf, "  %f", RealPart);

		NRF_LOG_RAW_INFO("%s", (uint32_t) cOutbuf);
		NRF_LOG_FLUSH();


		ImaginaryPart = fft_out[nIdx+1];

		sprintf(cOutbuf, "  %f", ImaginaryPart);
		NRF_LOG_RAW_INFO("%s", (uint32_t) cOutbuf);
		NRF_LOG_FLUSH();

		Magnitude = abs(sqrt(RealPart*RealPart +  ImaginaryPart *ImaginaryPart));

		OtherMagnitude = fft_magnitude[nJdx];

		sprintf(cOutbuf, "  %f %f\r\n", Magnitude, OtherMagnitude);
		NRF_LOG_RAW_INFO("%s", (uint32_t) cOutbuf);
		NRF_LOG_FLUSH();

		nJdx++;

	}

#endif

	return MaxIdx;

}



