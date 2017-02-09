/*
 * Copyright (c) 2015, Swiss Federal Institute of Technology (ETH Zurich).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author:  Reto Da Forno
 */

/**
 * @addtogroup  Dev
 * @{
 *
 * @defgroup    bolt Bolt
 * @{
 *
 * @file
 *
 * This library provides functionality to configure and use Bolt, an
 * asynchronous data interface with real-time guarantees.
 *
 * Set BOLT_CONF_ON to 1 to use this library. By default, DMA usage is disabled
 * and the max. message length is 48 bytes. To adjust the max. message length, 
 * define BOLT_CONF_MAX_MSG_LEN in your configuration file.
 * @note The data transfer over the SPI can either be synchronous (blocking,
 * polling/busy wait) or asynchronous (interrupt/DMA-driven).
 * @remark This lib does not require any peripherals other than an SPI module.
 */

#ifndef __BOLT_H__
#define __BOLT_H__

/* necessary include to enable overwriting of default settings in platform.h */
#include "platform.h"

#if BOLT_CONF_ON

#ifndef BOLT_CONF_SPI                   /* must be of type spi_module_t */
#define BOLT_CONF_SPI                   SPI_1   
#endif /* BOLT_CONF_SPI */

#ifndef BOLT_CONF_REQ_PIN
#define BOLT_CONF_TIMEREQ_PIN           PORT2, PIN1
#define BOLT_CONF_REQ_PIN               PORT2, PIN2
#define BOLT_CONF_IND_PIN               PORT2, PIN3
#define BOLT_CONF_ACK_PIN               PORT2, PIN4
#define BOLT_CONF_MODE_PIN              PORT2, PIN5
#define BOLT_CONF_IND_OUT_PIN           PORT2, PIN6
#endif /* BOLT_CONF_REQ_PIN */

#ifndef BOLT_CONF_TIMEREQ_ENABLE
#define BOLT_CONF_TIMEREQ_ENABLE        0
#endif /* BOLT_CONF_TIMEREQ_ENABLE */

/* high frequency mode? uses HF timer module instead of LF if available */
#ifndef BOLT_CONF_TIMEREQ_HF_MODE
#define BOLT_CONF_TIMEREQ_HF_MODE       0
#endif /* BOLT_CONF_TIMEREQ_HF_MODE */

/* this is just an example (works on the cc430), hardware specific definitions
 * should be set in platform.h! */
#if !BOLT_CONF_TIMEREQ_HF_MODE
 #ifndef BOLT_CONF_TIMEREQ_TIMERID
  #define BOLT_CONF_TIMEREQ_TIMERID     RTIMER_LF_0
 #endif /* BOLT_CONF_TIMEREQ_TIMERID */
 #ifndef BOLT_CONF_TIMEREQ_DMATRG
  #define BOLT_CONF_TIMEREQ_DMATRG      DMA_TRCSRC_TA1CCR0
 #endif /* BOLT_CONF_TIMEREQ_DMATRG */
 #ifndef BOLT_CONF_TIMEREQ_PINMAP
  #define BOLT_CONF_TIMEREQ_PINMAP      PM_TA1CCR0A
 #endif /* BOLT_CONF_TIMEREQ_PINMAP */
 #ifndef BOLT_CONF_TIMEREQ_CCR
  #define BOLT_CONF_TIMEREQ_CCR         TA1CCR0
 #endif /* BOLT_CONF_TIMEREQ_CCR */
#endif /* BOLT_CONF_TIMEREQ_HF_MODE */

#ifndef BOLT_CONF_MAX_MSG_LEN
#define BOLT_CONF_MAX_MSG_LEN           48  /* bytes */
#endif /* BOLT_CONF_MAX_MSG_LEN */

#ifndef BOLT_CONF_USE_DMA
#define BOLT_CONF_USE_DMA               0
#endif /* BOLT_CONF_USE_DMA */

#ifndef BOLT_CONF_SCLK_SPEED            /* serial clock speed */
#define BOLT_CONF_SCLK_SPEED            (SMCLK_SPEED)
#endif /* BOLT_CONF_SCLK_SPEED */

#ifndef BOLT_CONF_DEBUG_ON
#define BOLT_CONF_DEBUG_ON              0
#endif /* BOLT_CONF_DEBUG_ON */

/* to ensure compatibility to Bolt driver < v2.0 */
#define BOLT_WRITE(d, l)                bolt_write(d, l)
#define BOLT_READ(d, l)                 ((l) = bolt_read(d))


/**
 * @brief checks whether there is data to read from BOLT
 * @remark IND line high means data is available
 */
#define BOLT_DATA_AVAILABLE             PIN_GET(BOLT_CONF_IND_PIN)



/**
 * @brief initializes all required GPIO pins and peripherals for Bolt
 * @return 1 if successful (= Bolt accessible), 0 otherwise
 */
uint8_t bolt_init(void);

/**
B * @return 1 if BOLT is active/ready (= responds to a write request) and
 * 0 otherwise
 */
uint8_t bolt_status(void);

/**
 * @brief read a message from Bolt
 * @param[out] out_data the output buffer to hold the received data, must be
 *                      at least BOLT_CONF_MAX_MSG_LEN long
 * @return the number of read bytes
 */
uint16_t bolt_read(uint8_t* out_data);

/**
 * @brief read a message from Bolt
 * @param[in] data the data to send to Bolt
 * @param[in] len the number of bytes to write
 * @return 1 if successful, 0 otherwise
 */
uint8_t bolt_write(const uint8_t* data, uint16_t len);


#if BOLT_CONF_TIMEREQ_ENABLE
/**
 * @brief set the callback function for the timestamp request functionality
 * @param[in] func a pointer to the callback function. Set this parameter to
 * 0 disable the timestamp request interrupt and to use polling instead
 */
void bolt_set_timereq_callback(void (*func)(void)); 

/**
 * @brief checks whether a timestamp request is pending and handles it if so
 * @param out timestamp a pointer to a buffer to hold the timestamp
 * @return 1 if successful, 0 otherwise
 * @note this function should be used in polling mode only
 */
uint8_t bolt_handle_timereq(rtimer_clock_t* timestamp);
#endif /* BOLT_CONF_TIMEREQ_ENABLE */

/**
 * @brief set the callback function for the IND line interrupt
 * @note this will enable the port interrupt on the IND pin
 * @param func a pointer to the callback function. Pass 0 to disable the 
 * IND interrupt.
 */
void bolt_set_ind_callback(void (*func)(void));

/**
 * @brief handles IRQs if there are pending interrupts
 * call this function from the appropriate port ISR (e.g. Port 2 ISR)
 */
void bolt_handle_irq(void);

#endif /* BOLT_CONF_ON */

#endif /* __BOLT_H__ */

/**
 * @}
 * @}
 */
