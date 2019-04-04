/*
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich).
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
 * Author:  Reto Da Forno
 */

#ifndef __EVENT_H__
#define __EVENT_H__


/*
 * write an event message to a target
 */

#ifndef EVENT_CONF_ON
#define EVENT_CONF_ON           0
#endif /* EVENT_CONF_ON */


/* CONFIG */

#ifndef EVENT_CONF_TARGET
#define EVENT_CONF_TARGET               EVENT_TARGET_UART
#endif /* EVENT_CONF_TARGET */

/* compile time fixed logging level? */
#ifndef EVENT_CONF_LEVEL_FIXED
#define EVENT_CONF_LEVEL_FIXED          0
#endif /* EVENT_CONF_LEVEL_FIXED */

/* default logging level */
#ifndef EVENT_CONF_LEVEL
#define EVENT_CONF_LEVEL                EVENT_LEVEL_INFO
#endif /* EVENT_CONF_LVEL */


/* DEFINITIONS */

/* event notification level (equivalent to debug_level_t scale) */
typedef enum {
  EVENT_LEVEL_QUIET,      /* no notifications */
  EVENT_LEVEL_ERROR,      /* report only errors */
  EVENT_LEVEL_WARNING,    /* report warnings and errors */
  EVENT_LEVEL_INFO,       /* report information, warnings and errors */
  EVENT_LEVEL_VERBOSE,    /* report all */
  NUM_EVENT_LEVELS,
} event_level_t;

typedef enum {
  EVENT_TARGET_NONE, 
  EVENT_TARGET_UART,
  EVENT_TARGET_BOLT,
  EVENT_TARGET_LWB,
#if FRAM_CONF_ON
  EVENT_TARGET_XMEM,      /* TODO not yet implemented */
#endif /* FRAM_CONF_ON */
} event_target_t;


#if EVENT_CONF_ON

/* MACROS */

/* print out strings when using UART */
#define EVENT(lvl, evt, val)      event(lvl, evt, val)
#define EVENT_INFO(evt, val)      if(event_lvl >= EVENT_LEVEL_INFO) { \
    event_write(EVENT_LEVEL_INFO, evt, val); }
#define EVENT_WARNING(evt, val)   if(event_lvl >= EVENT_LEVEL_WARNING) { \
    event_write(EVENT_LEVEL_WARNING, evt, val); }
#define EVENT_ERROR(evt, val)     if(event_lvl >= EVENT_LEVEL_ERROR) { \
    event_write(EVENT_LEVEL_ERROR, evt, val); }
#define EVENT_VERBOSE(evt, val)   if(event_lvl >= EVENT_LEVEL_VERBOSE) { \
    event_write(EVENT_LEVEL_VERBOSE, evt, val); }


/* GLOBAL VARIABLES and FUNCTIONS */

#if !EVENT_CONF_LEVEL_FIXED
extern event_level_t event_lvl;

static __inline void event_set_level(event_level_t lvl) { event_lvl = lvl; }
#else
#define event_lvl               EVENT_CONF_LEVEL
#endif /* EVENT_CONF_LEVEL_FIXED */


void event_write(event_level_t lvl, dpp_event_type_t type, uint32_t val);


#else /* EVENT_CONF_ON */

/* dummy macros */
#define EVENT(x,y,z)
#define EVENT_INFO(x,y)
#define EVENT_WARNING(x,y)
#define EVENT_ERROR(x,y)
#define EVENT_VERBOSE(x,y)

#endif /* EVENT_CONF_ON */

#endif /* __EVENT_H__ */
