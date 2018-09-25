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
 * Author:  Jonas Baechli
 *          Reto Da Forno
 */

#ifndef FLOCKLAB_H_
#define FLOCKLAB_H_

#include "gpio.h"

/* platform dependent GPIO definitions */
#ifdef PLATFORM_SKY

  /* the following pins assignments are given by the FlockLAB testbed */
  #define FLOCKLAB_LED1       PORT6, PIN7  /* for GPIO tracing */
  #define FLOCKLAB_LED2       PORT6, PIN6  /* for GPIO tracing */
  #define FLOCKLAB_LED3       PORT6, PIN2  /* for GPIO tracing */
  #define FLOCKLAB_INT1       PORT6, PIN0  /* for GPIO tracing */
  #define FLOCKLAB_INT2       PORT6, PIN1  /* for GPIO tracing */
  #define FLOCKLAB_SIG1       PORT2, PIN7  /* target actuation */
  #define FLOCKLAB_SIG2       PORT2, PIN3  /* target actuation */

#elif defined PLATFORM_DPP_CC430

  #define FLOCKLAB_LED1       PORT3, PIN0  /* for GPIO tracing */
  #define FLOCKLAB_INT1       PORT3, PIN5  /* for GPIO tracing */
  #define FLOCKLAB_INT2       PORT3, PIN4  /* for GPIO tracing */
  /* target actuation not available */

#endif /* PLATFORM_... */


#ifdef FLOCKLAB_INT1
  #define FLOCKLAB_INT1_INIT  { PIN_CFG_OUT(FLOCKLAB_INT1); \
                                PIN_CLR(FLOCKLAB_INT1); }
#else  /* FLOCKLAB_INT1 */
  #define FLOCKLAB_INT1_INIT
#endif /* FLOCKLAB_INT1 */

#ifdef FLOCKLAB_INT2
  #define FLOCKLAB_INT2_INIT  { PIN_CFG_OUT(FLOCKLAB_INT2); \
                                PIN_CLR(FLOCKLAB_INT2); }
#else  /* FLOCKLAB_INT2 */
  #define FLOCKLAB_INT2_INIT
#endif /* FLOCKLAB_INT2 */

#ifdef FLOCKLAB_LED1
  #define FLOCKLAB_LED1_INIT  { PIN_CFG_OUT(FLOCKLAB_LED1); \
                                PIN_CLR(FLOCKLAB_LED1); }
#else  /* FLOCKLAB_LED1 */
  #define FLOCKLAB_LED1_INIT
#endif /* FLOCKLAB_LED1 */

#ifdef FLOCKLAB_LED2
  #define FLOCKLAB_LED2_INIT  { PIN_CFG_OUT(FLOCKLAB_LED2); \
                                PIN_CLR(FLOCKLAB_LED2); }
#else  /* FLOCKLAB_LED2 */
  #define FLOCKLAB_LED2_INIT
#endif /* FLOCKLAB_LED2 */

#ifdef FLOCKLAB_LED3
  #define FLOCKLAB_LED3_INIT  { PIN_CFG_OUT(FLOCKLAB_LED3); \
                                PIN_CLR(FLOCKLAB_LED3); }
#else  /* FLOCKLAB_LED3 */
  #define FLOCKLAB_LED3_INIT
#endif /* FLOCKLAB_LED3 */

#ifdef FLOCKLAB_SIG1
  #define FLOCKLAB_SIG1_INIT  { PIN_CFG_OUT(FLOCKLAB_SIG1); \
                                PIN_CLR(FLOCKLAB_SIG1); }
#else  /* FLOCKLAB_SIG1 */
  #define FLOCKLAB_SIG1_INIT
#endif /* FLOCKLAB_SIG1 */

#ifdef FLOCKLAB_SIG2
  #define FLOCKLAB_SIG2_INIT  { PIN_CFG_OUT(FLOCKLAB_SIG2); \
                                PIN_CLR(FLOCKLAB_SIG2); }
#else  /* FLOCKLAB_SIG2 */
  #define FLOCKLAB_SIG2_INIT
#endif /* FLOCKLAB_SIG2 */


#ifdef FLOCKLAB
  #define FLOCKLAB_INIT()     { \
                                FLOCKLAB_INT1_INIT; \
                                FLOCKLAB_INT2_INIT; \
                                FLOCKLAB_LED1_INIT; \
                                FLOCKLAB_LED2_INIT; \
                                FLOCKLAB_LED3_INIT; \
                                FLOCKLAB_SIG1_INIT; \
                                FLOCKLAB_SIG2_INIT; \
                              }
#endif /* FLOCKLAB */


#endif /* FLOCKLAB_H_ */
