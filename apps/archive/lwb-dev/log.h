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
 * Author:  Reto Da Forno
 */

#ifndef __LOG_H__
#define __LOG_H__


/*
 * log a message to a target
 *
 * This logging mechanism is independent of debug-print.c
 *
 * Only immediate logging (write directly to the target) is supported,
 * no buffering.
 */

#ifndef LOG_CONF_ON
#define LOG_CONF_ON           0
#endif /* LOG_CONF_ON */


/* CONFIG */

#ifndef LOG_CONF_TARGET
#define LOG_CONF_TARGET               LOG_TARGET_UART
#endif /* LOG_CONF_TARGET */

/* compile time fixed logging level? */
#ifndef LOG_CONF_LEVEL_FIXED
#define LOG_CONF_LEVEL_FIXED          1
#endif /* LOG_CONF_LEVEL_FIXED */

/* default logging level */
#ifndef LOG_CONF_LEVEL
#define LOG_CONF_LEVEL                LOG_LEVEL_INFO
#endif /* LOG_CONF_LVEL */

/* enable/disable UART before/after each printf statement? */
#ifndef LOG_CONF_DISABLE_UART
#define LOG_CONF_DISABLE_UART 1
#endif /* LOG_CONF_DISABLE_UART */

#ifndef LOG_CONF_STRING_BUFFER_SIZE
#define LOG_CONF_STRING_BUFFER_SIZE   64
#endif /* LOG_CONF_STRING_BUFFER_SIZE */

#ifndef LOG_CONF_LWB_STREAM_ID
#define LOG_CONF_LWB_STREAM_ID        0
#endif /* LOG_CONF_LWB_STREAM_ID */

/* format the UART output string (add node ID, timestamp and msg type) */
#ifndef LOG_CONF_FORMAT_OUTPUT
#define LOG_CONF_FORMAT_OUTPUT        0
#endif /* LOG_CONF_FORMAT_OUTPUT */


/* DEFINITIONS */

/* verbosity */
typedef enum {
  LOG_LEVEL_DISABLED,
  LOG_LEVEL_ERROR,
  LOG_LEVEL_WARNING,
  LOG_LEVEL_INFO,
  LOG_LEVEL_VERBOSE,
  NUM_LOG_LEVELS,
} log_level_t;


/* must use defines (typedef enum doesn't work) */
#define LOG_TARGET_NONE     0
#define LOG_TARGET_UART     1
#define LOG_TARGET_XMEM     2
#define LOG_TARGET_BOLT     3
#define LOG_TARGET_LWB      4


#if LOG_CONF_ON

/* MACROS */

/* print out strings when using UART */
#if LOG_CONF_TARGET == LOG_TARGET_UART
#define LOG(lvl, evt, val)      log_event_str(lvl, evt##_STR, val)
#define LOG_STR(lvl, ...)       if(log_lvl >= lvl) { \
    snprintf(log_buffer, LOG_CONF_STRING_BUFFER_SIZE, __VA_ARGS__); \
    log_generic(lvl, log_buffer); }
#define LOG_INFO(evt, val)      if(log_lvl >= LOG_LEVEL_INFO) { \
    log_event_str(LOG_LEVEL_INFO, evt##_STR, val); }
#define LOG_WARNING(evt, val)   if(log_lvl >= LOG_LEVEL_WARNING) { \
    log_event_str(LOG_LEVEL_WARNING, evt##_STR, val); }
#define LOG_ERROR(evt, val)     if(log_lvl >= LOG_LEVEL_ERROR) { \
    log_event_str(LOG_LEVEL_ERROR, evt##_STR, val); }
#define LOG_VERBOSE(evt, val)   if(log_lvl >= LOG_LEVEL_VERBOSE) { \
    log_event_str(LOG_LEVEL_VERBOSE, evt##_STR, val); }
#else /* LOG_CONF_TARGET */
#define LOG(lvl, evt, val)      log_event(lvl, evt, val)
#define LOG_STR(lvl, ...)       if(log_lvl >= lvl) { \
    snprintf(log_buffer, LOG_CONF_STRING_BUFFER_SIZE, __VA_ARGS__); \
    log_generic(lvl, msg); }
#define LOG_INFO(evt, val)      if(log_lvl >= LOG_LEVEL_INFO) { \
    log_event(LOG_LEVEL_INFO, evt, val); }
#define LOG_WARNING(evt, val)   if(log_lvl >= LOG_LEVEL_WARNING) { \
    log_event(LOG_LEVEL_WARNING, evt, val); }
#define LOG_ERROR(evt, val)     if(log_lvl >= LOG_LEVEL_ERROR) { \
    log_event(LOG_LEVEL_ERROR, evt, val); }
#define LOG_VERBOSE(evt, val)   if(log_lvl >= LOG_LEVEL_VERBOSE) { \
    log_event(LOG_LEVEL_VERBOSE, evt, val); }
#endif /* LOG_CONF_TARGET */

/* log the position in the code (i.e. file name and line) as string */
#define LOG_POS(lvl)            if(log_lvl >= lvl) { \
    snprintf(log_buffer, LOG_CONF_STRING_BUFFER_SIZE, "%s, line %u", \
             FILENAME, __LINE__); \
    log_generic(lvl, log_buffer); }

/* strrchr() will be evaluated at compile time and the file name inlined */
#define FILENAME  (strrchr(__FILE__, '/') ? \
                   strrchr(__FILE__, '/') + 1 : __FILE__)



/* GLOBAL VARIABLES and FUNCTIONS */

#if !LOG_CONF_LEVEL_FIXED
extern log_level_t log_lvl;

static __inline void log_set_level(log_level_t lvl) { log_lvl = lvl; }
#else
#define log_lvl               LOG_CONF_LEVEL
#endif /* LOG_CONF_LEVEL_FIXED */

extern char log_buffer[LOG_CONF_STRING_BUFFER_SIZE];


void log_event(log_level_t lvl, log_event_type_t type, uint16_t val);
void log_event_str(log_level_t lvl, const char* evt_str, uint16_t val);
void log_generic(log_level_t lvl, const char* str);


#else /* LOG_CONF_ON */

/* dummy macros */
#define LOG(x,y,z)
#define LOG_INFO(x,y)
#define LOG_WARNING(x,y)
#define LOG_ERROR(x,y)
#define LOG_VERBOSE(x,y)
#define LOG_STR(x, ...)


#endif /* LOG_CONF_ON */

#endif /* __LOG_H__ */
