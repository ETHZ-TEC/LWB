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
 * @defgroup    usci USCI
 * @{
 *
 * @file
 * @author
 *              Reto Da Forno
 *
 * @brief Universal Serial Communication Interface
 * 
 * There are two modules: A0 and B0.
 * Both can be configured in SPI or UART mode.
 */

#ifndef __USCI_H__
#define __USCI_H__


#define USCI_A0             UCA0CTLW0  /* base address of the USCI A0 module */
#define USCI_B0             UCB0CTLW0  /* base address of the USCI A0 module */

/**
 * @brief check if the USCI A0 module is active / busy (i.e. a transmission is
 *ongoing)
 */
#define USCI_A0_ACTIVE      (UCA0STAT & UCBUSY)

/**
 * @brief check if the USCI A0 module is configured in SPI mode
 */
#define USCI_A0_IN_SPI_MODE (UCA0CTL0 & UCSYNC)

/**
 * @brief disable the USCI A0 module
 */
#define USCI_A0_DISABLE          (UCA0CTL1 |= UCSWRST)

/**
 * @brief disable the USCI B0 module
 */
#define USCI_B0_DISABLE          (UCB0CTL1 |= UCSWRST)

/**
 * @brief enable the USCI A0 module
 */
#define USCI_A0_ENABLE          (UCA0CTL1 &= ~UCSWRST)

/**
 * @brief enable the USCI B0 module
 */
#define USCI_B0_ENABLE          (UCB0CTL1 &= ~UCSWRST)


#endif /* __USCI_H__ */

/**
 * @}
 * @}
 */
