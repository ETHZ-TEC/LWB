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
 * @defgroup    adc ADC
 * @{
 *
 * @file
 * @brief provides functionality to configure the USCI A0 module in UART mode
 * @author rdaforno
 */

#ifndef __UART_H__
#define __UART_H__

#ifndef UART_CONF_BAUDRATE
/* don't forget to adjust uart_init() if you change the baudrate here */
#define UART_CONF_BAUDRATE  115200LU
#endif /* UART_CONF_BAUDRATE */

/* pin definitions */
#define UART_A0_RX          PORT1, PIN5         /* input */
#define UART_A0_TX          PORT1, PIN6         /* output */

/**
 * @brief check if the UART module is active / busy (same as USCI_A0_ACTIVE)
 */
#define UART_ACTIVE         (UCA0STAT & UCBUSY)

/**
 * @brief set the input handler for the UART
 * @param[in] input the function to be called from the UART ISR (RX interrupt)
 */
void uart_set_input_handler(int (*input)(unsigned char c));

/**
 * @brief initialize the USCI_A0 module in UART mode (port RS232)
 * @note set the desired baudrate in hal.h (UART0_BAUDRATE).
 * @remark The UART module is driven by the SMCLK.
 */
void uart_init(void);

/**
 * @brief re-initialize the USCI_A0 module in UART mode (when it is configured
 *in SPI mode)
 * @remark The UART module is driven by the SMCLK.
 */
void uart_reinit(void);

#endif /* __UART0_H__ */

/**
 * @}
 * @}
 */