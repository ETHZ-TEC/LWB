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
 * @addtogroup  Platform
 * @{
 *
 * @defgroup    uart UART
 * @{
 *
 * @file
 * @brief interface defintion for UART
 * these functions must be implemented in 'uart.c' in the architecture
 * specific folder
 */

#ifndef __UART_H__
#define __UART_H__


#ifndef UART_CONF_BAUDRATE
/* don't forget to adjust uart_init() if you change the baudrate here */
#define UART_CONF_BAUDRATE  115200LU
#endif /* UART_CONF_BAUDRATE */

/* use interrupt driven UART transmission */
#ifndef UART_CONF_TX_INTERRUPT
#define UART_CONF_TX_INTERRUPT  0
#endif /* UART_CONF_TX_INTERRUPT */

#if UART_CONF_TX_INTERRUPT
/* set the buffer size for interrupt driven transmission */
#ifndef UART_CONF_TXBUF_SIZE 
#define UART_CONF_TXBUF_SIZE    64      /* characters/bytes */
#endif /* UART_CONF_TX_BUFSIZE */
#endif /* UART_CONF_TX_INTERRUPT */

/** 
 * @brief this macro can be defined in a platform specific file
 * to add instructions that need to be executed before enabling UART
 */
#ifndef UART_BEFORE_ENABLE
#define UART_BEFORE_ENABLE
#endif /* UART_BEFORE_ENABLE */

/** 
 * @brief this macro can be defined in a platform specific file
 * to add instructions that need to be executed after disabling UART
 */
#ifndef UART_AFTER_DISABLE
#define UART_AFTER_DISABLE
#endif /* UART_AFTER_DISABLE */

/**
 * @brief set the input handler for the UART
 * @param[in] input the function to be called from the UART ISR (RX interrupt)
 * @note this will enable the RX interrupt
 */
void uart_set_input_handler(int (*input)(unsigned char c));

/**
 * @brief initialize the UART module (port RS232)
 * @note this does not enable the module
 */
void uart_init(void);

/**
 * @brief re-initialize the UART module 
 * @remark The UART module is driven by the SMCLK.
 * 
 * This function does only a minimal reconfig, which is useful e.g. when using
 * the same USCI module for SPI and UART for switching between the two modes.
 */
void uart_reinit(void);

/**
 * @brief enable or disable the UART module
 */
void uart_enable(uint8_t enable);


#endif /* __UART_H__ */

/**
 * @}
 * @}
 */