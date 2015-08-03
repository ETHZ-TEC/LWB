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
 * @defgroup    leds LEDs
 * @{
 *
 * @file
 * @author
 *              Reto Da Forno
 *
 * @brief access the LEDs
 */

#ifndef __LEDS_H__
#define __LEDS_H__

#if LEDS_CONF_ON
#define LED_ON(portandpin)      PIN_SET_I(portandpin)
#define LED_OFF(portandpin)     PIN_CLR_I(portandpin)
#define LED_TOGGLE(portandpin)  PIN_XOR_I(portandpin)
#else
#define LED_ON(portandpin)
#define LED_OFF(portandpin)
#define LED_TOGGLE(portandpin)
#endif

#define LEDS_ON                 { PIN_SET(LED_0); PIN_SET(LED_1); PIN_SET(LED_2); PIN_SET(LED_3); }
#define LEDS_OFF                { PIN_CLR(LED_0); PIN_CLR(LED_1); PIN_CLR(LED_2); PIN_CLR(LED_3); }
#define LEDS_TOGGLE             { PIN_XOR(LED_0); PIN_XOR(LED_1); PIN_XOR(LED_2); PIN_XOR(LED_3); }
#define LEDS_INIT               { LEDS_OFF; PIN_UNSEL(LED_0); PIN_UNSEL(LED_1); PIN_UNSEL(LED_2); PIN_UNSEL(LED_3); \
                                  PIN_CFG_OUT(LED_0); PIN_CFG_OUT(LED_1); PIN_CFG_OUT(LED_2); PIN_CFG_OUT(LED_3); }
#endif /* __LEDS_H__ */

/**
 * @}
 * @}
 */
