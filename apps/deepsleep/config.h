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

#ifndef __CONFIG_H__
#define __CONFIG_H__

/*
 * application specific config file to override default settings
 */

#define RF_CONF_ON          0
#define BOLT_CONF_ON        1

#define ENTER_LPM45()       { SVS_DISABLE; PMMCTL0 = (PMMPW | PMMREGOFF); __bis_status_register(GIE | SCG0 | SCG1 | CPUOFF | OSCOFF); __no_operation(); }

#define BOLT_PIN_CFG()      { PIN_CFG_IN(BOLT_CONF_IND_PIN); \
                              PIN_UNSEL(BOLT_CONF_MODE_PIN); \
                              PIN_CLR(BOLT_CONF_MODE_PIN); \
                              PIN_CFG_OUT(BOLT_CONF_MODE_PIN); \
                              PIN_UNSEL(BOLT_CONF_REQ_PIN); \
                              PIN_CLR(BOLT_CONF_REQ_PIN); \
                              PIN_CFG_OUT(BOLT_CONF_REQ_PIN); \
                              PIN_CFG_IN(BOLT_CONF_ACK_PIN); \
                              PIN_PULLDOWN_EN(BOLT_CONF_ACK_PIN); \
                              PIN_CFG_IN(BOLT_CONF_IND_OUT_PIN); }

#define SYSTEM_INIT_HOOK()  { BOLT_PIN_CFG(); BEFORE_DEEPSLEEP(); PIN_SET(LED_STATUS); __delay_cycles(1000); PIN_CLR(LED_STATUS); ENTER_LPM45(); }

#endif /* __CONFIG_H__ */
