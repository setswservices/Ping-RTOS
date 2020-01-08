/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_cmplx_mag_f32.c
 * Description:  Floating-point complex magnitude
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

#include "arm_const_structs.h"

/**
 * @ingroup groupCmplxMath
 */

/**
 * @defgroup cmplx_mag Complex Magnitude
 *
 * Computes the magnitude of the elements of a complex data vector.
 *
 * The <code>pSrc</code> points to the source data and
 * <code>pDst</code> points to the where the result should be written.
 * <code>numSamples</code> specifies the number of complex samples
 * in the input array and the data is stored in an interleaved fashion
 * (real, imag, real, imag, ...).
 * The input array has a total of <code>2*numSamples</code> values;
 * the output array has a total of <code>numSamples</code> values.
 * The underlying algorithm is used:
 *
 * <pre>
 * for(n=0; n<numSamples; n++) {
 *     pDst[n] = sqrt(pSrc[(2*n)+0]^2 + pSrc[(2*n)+1]^2);
 * }
 * </pre>
 *
 * There are separate functions for floating-point, Q15, and Q31 data types.
 */

/**
 * @addtogroup cmplx_mag
 * @{
 */
/**
 * @brief Floating-point complex magnitude.
 * @param[in]       *pSrc points to complex input buffer
 * @param[out]      *pDst points to real output buffer
 * @param[in]       numSamples number of complex samples in the input vector
 * @return none.
 *
 */


void arm_cmplx_mag_f32(
  float32_t * pSrc,
  float32_t * pDst,
  uint32_t numSamples)
{
  float32_t realIn, imagIn;                      /* Temporary variables to hold input values */

#if defined (ARM_MATH_DSP)

#if APP_USE_SERIAL_DEBUG || APP_USE_SEGGER_DEBUG
	NRF_LOG_RAW_INFO("%s: Using M3\r\n ", (uint32_t)__func__);
#endif

  /* Run the below code for Cortex-M4 and Cortex-M3 */
  uint32_t blkCnt;                               /* loop counter */

  /*loop Unrolling */
  blkCnt = numSamples >> 2U;

  /* First part of the processing with loop unrolling.  Compute 4 outputs at a time.
   ** a second loop below computes the remaining 1 to 3 samples. */
  while (blkCnt > 0U)
  {

    /* C[0] = sqrt(A[0] * A[0] + A[1] * A[1]) */
    realIn = *pSrc++;
    imagIn = *pSrc++;
    /* store the result in the destination buffer. */
    arm_sqrt_f32((realIn * realIn) + (imagIn * imagIn), pDst++);

    realIn = *pSrc++;
    imagIn = *pSrc++;
    arm_sqrt_f32((realIn * realIn) + (imagIn * imagIn), pDst++);

    realIn = *pSrc++;
    imagIn = *pSrc++;
    arm_sqrt_f32((realIn * realIn) + (imagIn * imagIn), pDst++);

    realIn = *pSrc++;
    imagIn = *pSrc++;
    arm_sqrt_f32((realIn * realIn) + (imagIn * imagIn), pDst++);


    /* Decrement the loop counter */
    blkCnt--;
  }

  /* If the numSamples is not a multiple of 4, compute any remaining output samples here.
   ** No loop unrolling is used. */
  blkCnt = numSamples % 0x4U;

  while (blkCnt > 0U)
  {
    /* C[0] = sqrt(A[0] * A[0] + A[1] * A[1]) */
    realIn = *pSrc++;
    imagIn = *pSrc++;
    /* store the result in the destination buffer. */
    arm_sqrt_f32((realIn * realIn) + (imagIn * imagIn), pDst++);

    /* Decrement the loop counter */
    blkCnt--;
  }

#else

  /* Run the below code for Cortex-M0 */

#if APP_USE_SERIAL_DEBUG || APP_USE_SEGGER_DEBUG
	NRF_LOG_RAW_INFO("%s: Using M0\r\n ", (uint32_t)__func__);
#endif

  while (numSamples > 0U)
  {
    /* out = sqrt((real * real) + (imag * imag)) */
    realIn = *pSrc++;
    imagIn = *pSrc++;
    /* store the result in the destination buffer. */
    arm_sqrt_f32((realIn * realIn) + (imagIn * imagIn), pDst++);

    /* Decrement the loop counter */
    numSamples--;
  }

#endif /* #if defined (ARM_MATH_DSP) */

}

/**
 * @} end of cmplx_mag group
 */
