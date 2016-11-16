/*
 * Copyright (c) 2016, Swiss Federal Institute of Technology (ETH Zurich).
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
 */

#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

/*
 * contiki configuration, architecture specific
 */
 
/* standard libraries */
#include <stdio.h>
#include <stdlib.h>
#include <isr_compat.h>
#include <string.h>

/* application specific config */
#include "config.h"

#define CLIF
#define CCIF

#ifndef ENERGEST_CONF_ON
#define ENERGEST_CONF_ON        0
#endif /* ENERGEST_CONF_ON */

#ifndef AUTOSTART_ENABLE
#define AUTOSTART_ENABLE        1
#endif /* AUTOSTART_ENABLE */

#ifndef RTIMER_NOW
/* LF clock is the default rtimer */
#define RTIMER_NOW              rtimer_now_lf
#endif

#ifdef NODE_ID
#define node_id                 NODE_ID
#else /* NODE_ID */
extern volatile uint16_t node_id;
#endif /* NODE_ID */

/* Contiki requires the definition of the following data types: */
typedef uint32_t clock_time_t;
typedef uint64_t rtimer_clock_t;


clock_time_t clock_time(void);

#endif /* __CONTIKI_CONF_H__ */
