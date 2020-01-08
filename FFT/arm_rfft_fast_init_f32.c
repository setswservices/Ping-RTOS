/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_cfft_init_f32.c
 * Description:  Split Radix Decimation in Frequency CFFT Floating point processing function
 *
 * $Date:        27. January 2017
 * $Revision:    V.1.5.1
 *
 * Target Processor: Cortex-M cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2010-2017 ARM Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nordic_common.h"
#ifdef FROM_SANA
#include "audio.h"
#include "gpio.h"
#include "sana_data.h"
#include "sana_config.h"
#include "system.h"
#include "timer.h"
#include "sana_modes.h"
#include "sana_config.h"
#include "sana_data.h"
#include "system.h"
#include "lights.h"  
#include "timer.h"
#include "sana_modes.h"
#include "data_manage.h"
#include "Valencell_HR.h"
#define NRF_LOG_MODULE_NAME "Arm_cfft_radix8_f32"
#endif // FROM_SANA
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define ARM_MATH_CM4
#include "arm_math.h"
#include "./arm_const_structs.h"

#define ARMBITREVINDEXTABLE_16_TABLE_LENGTH ((uint16_t)20)
#define ARMBITREVINDEXTABLE_32_TABLE_LENGTH ((uint16_t)48)
#define ARMBITREVINDEXTABLE_64_TABLE_LENGTH ((uint16_t)56)


/**
 * @ingroup groupTransforms
 */

/**
 * @addtogroup RealFFT
 * @{
 */

/**
* @brief  Initialization function for the floating-point real FFT.
* @param[in,out] *S             points to an arm_rfft_fast_instance_f32 structure.
* @param[in]     fftLen         length of the Real Sequence.
* @return        The function returns ARM_MATH_SUCCESS if initialization is successful or ARM_MATH_ARGUMENT_ERROR if <code>fftLen</code> is not a supported value.
*
* \par Description:
* \par
* The parameter <code>fftLen</code>	Specifies length of RFFT/CIFFT process. Supported FFT Lengths are 32, 64, 128, 256, 512, 1024, 2048, 4096.
* \par
* This Function also initializes Twiddle factor table pointer and Bit reversal table pointer.
*/
arm_status arm_rfft_fast_init_f32(
  arm_rfft_fast_instance_f32 * S,
  uint16_t fftLen)
{
  arm_cfft_instance_f32 * Sint;
  /*  Initialise the default arm status */
  arm_status status = ARM_MATH_SUCCESS;
  /*  Initialise the FFT length */
  Sint = &(S->Sint);
  Sint->fftLen = fftLen/2;
  S->fftLenRFFT = fftLen;

  	NRF_LOG_RAW_INFO("Entered!\r\n");

  /*  Initializations of structure parameters depending on the FFT length */
  switch (Sint->fftLen)
  {

  #ifdef NOT_NEC
  case 2048U:
    /*  Initializations of structure parameters for 2048 point FFT */
    /*  Initialise the bit reversal table length */
    Sint->bitRevLength = ARMBITREVINDEXTABLE_2048_TABLE_LENGTH;
    /*  Initialise the bit reversal table pointer */
    Sint->pBitRevTable = (uint16_t *)armBitRevIndexTable2048;
    /*  Initialise the Twiddle coefficient pointers */
		Sint->pTwiddle     = (float32_t *) twiddleCoef_2048;
		S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_4096;
    break;
  case 1024U:
    Sint->bitRevLength = ARMBITREVINDEXTABLE_1024_TABLE_LENGTH;
    Sint->pBitRevTable = (uint16_t *)armBitRevIndexTable1024;
		Sint->pTwiddle     = (float32_t *) twiddleCoef_1024;
		S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_2048;
    break;
  case 512U:
    Sint->bitRevLength = ARMBITREVINDEXTABLE_512_TABLE_LENGTH;
    Sint->pBitRevTable = (uint16_t *)armBitRevIndexTable512;
		Sint->pTwiddle     = (float32_t *) twiddleCoef_512;
		S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_1024;
    break;

#endif // NOT_NEC
	
  case 256U:
    Sint->bitRevLength = ARMBITREVINDEXTABLE_256_TABLE_LENGTH;
    Sint->pBitRevTable = (uint16_t *)armBitRevIndexTable256;
		Sint->pTwiddle     = (float32_t *) twiddleCoef_256;
		S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_512;
    break;
  case 128U:
    Sint->bitRevLength = ARMBITREVINDEXTABLE_128_TABLE_LENGTH;
    Sint->pBitRevTable = (uint16_t *)armBitRevIndexTable128;
		Sint->pTwiddle     = (float32_t *) twiddleCoef_128;
		S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_256;
    break;
  case 64U:
    Sint->bitRevLength = ARMBITREVINDEXTABLE_64_TABLE_LENGTH;
    Sint->pBitRevTable = (uint16_t *)armBitRevIndexTable64;
		Sint->pTwiddle     = (float32_t *) twiddleCoef_64;
		S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_128;
    break;
  case 32U:
    Sint->bitRevLength = ARMBITREVINDEXTABLE_32_TABLE_LENGTH;
    Sint->pBitRevTable = (uint16_t *)armBitRevIndexTable32;
		Sint->pTwiddle     = (float32_t *) twiddleCoef_32;
		S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_64;
    break;
  case 16U:
    Sint->bitRevLength = ARMBITREVINDEXTABLE_16_TABLE_LENGTH;
    Sint->pBitRevTable = (uint16_t *)armBitRevIndexTable16;
		Sint->pTwiddle     = (float32_t *) twiddleCoef_16;
		S->pTwiddleRFFT    = (float32_t *) twiddleCoef_rfft_32;
    break;
  default:
    /*  Reporting argument error if fftSize is not valid value */
    status = ARM_MATH_ARGUMENT_ERROR;
    break;
  }

  return (status);
}

/**
 * @} end of RealFFT group
 */
