/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_max_f32.c
 * Description:  Maximum value of a floating-point vector
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
 * @ingroup groupStats
 */

/**
 * @defgroup Max Maximum
 *
 * Computes the maximum value of an array of data.
 * The function returns both the maximum value and its position within the array.
 * There are separate functions for floating-point, Q31, Q15, and Q7 data types.
 */

/**
 * @addtogroup Max
 * @{
 */


/**
 * @brief Maximum value of a floating-point vector.
 * @param[in]       *pSrc points to the input vector
 * @param[in]       blockSize length of the input vector
 * @param[out]      *pResult maximum value returned here
 * @param[out]      *pIndex index of maximum value returned here
 * @return none.
 */

void arm_max_f32(
  float32_t * pSrc,
  uint32_t blockSize,
  float32_t * pResult,
  uint32_t * pIndex)
{
#if defined (ARM_MATH_DSP)
  /* Run the below code for Cortex-M4 and Cortex-M3 */

  float32_t maxVal1, maxVal2, out;               /* Temporary variables to store the output value. */
  uint32_t blkCnt, outIndex, count;              /* loop counter */

  /* Initialise the count value. */
  count = 0U;
  /* Initialise the index value to zero. */
  outIndex = 0U;
  /* Load first input value that act as reference value for comparision */
  out = *pSrc++;

  /* Loop unrolling */
  blkCnt = (blockSize - 1U) >> 2U;

  while (blkCnt > 0U)
  {
    /* Initialize maxVal to the next consecutive values one by one */
    maxVal1 = *pSrc++;
    maxVal2 = *pSrc++;

    /* compare for the maximum value */
    if (out < maxVal1)
    {
      /* Update the maximum value and its index */
      out = maxVal1;
      outIndex = count + 1U;
    }

    /* compare for the maximum value */
    if (out < maxVal2)
    {
      /* Update the maximum value and its index */
      out = maxVal2;
      outIndex = count + 2U;
    }

    /* Initialize maxVal to the next consecutive values one by one */
    maxVal1 = *pSrc++;
    maxVal2 = *pSrc++;

    /* compare for the maximum value */
    if (out < maxVal1)
    {
      /* Update the maximum value and its index */
      out = maxVal1;
      outIndex = count + 3U;
    }

    /* compare for the maximum value */
    if (out < maxVal2)
    {
      /* Update the maximum value and its index */
      out = maxVal2;
      outIndex = count + 4U;
    }

    count += 4U;

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* if (blockSize - 1U) is not multiple of 4 */
  blkCnt = (blockSize - 1U) % 4U;

#else
  /* Run the below code for Cortex-M0 */

#if APP_USE_SERIAL_DEBUG || APP_USE_SEGGER_DEBUG
	NRF_LOG_RAW_INFO("%s: M0 code\r\n ", (uint32_t)__func__);
#endif

  float32_t maxVal1, out;                        /* Temporary variables to store the output value. */
  uint32_t blkCnt, outIndex;                     /* loop counter */

  /* Initialise the index value to zero. */
  outIndex = 0U;
  /* Load first input value that act as reference value for comparision */
  out = *pSrc++;

  blkCnt = (blockSize - 1U);

#endif /* #if defined (ARM_MATH_DSP) */

  while (blkCnt > 0U)
  {
    /* Initialize maxVal to the next consecutive values one by one */
    maxVal1 = *pSrc++;

    /* compare for the maximum value */
    if (out < maxVal1)
    {
      /* Update the maximum value and it's index */
      out = maxVal1;
      outIndex = blockSize - blkCnt;
    }

    /* Decrement the loop counter */
    blkCnt--;
  }

  /* Store the maximum value and it's index into destination pointers */
  *pResult = out;
  *pIndex = outIndex;
}

/**
 * @} end of Max group
 */
