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
 * Author:  Federico Ferrari
 *          Reto Da Forno
 */

/**
 * @brief rtimer implementation
 * 
 * TA0 is a high-frequency (HF) timer with 5 CCRs (SMCLK_SPEED)
 * TA1 is a low-frequency (LF) timer with 3 CCRs (ACLK_SPEED)
 * 
 * TA0 runs at 3.25 MHz
 * TA1 runs at 32768 Hz
 */

#include "contiki.h"
#include "platform.h"

/*---------------------------------------------------------------------------*/
static rtimer_t rt[NUM_OF_RTIMERS];     /* rtimer structs */
volatile rtimer_clock_t ta0_sw_ext;     /* SW extension for timer A0 */
volatile rtimer_clock_t ta1_sw_ext;
#define MAX_TICKS (~((clock_time_t)0) / 2)
static volatile clock_time_t count = 0;
/*---------------------------------------------------------------------------*/
static inline void
update_rtimer_state(uint16_t timer)
{
  /* update the state only if the rtimer has not been manually */
  /* stopped or re-scheduled by the callback function */
  if(rt[timer].state == RTIMER_JUST_EXPIRED) {
    if(rt[timer].period > 0) {
      /* if it is periodic, schedule the new expiration */
      rt[timer].time += rt[timer].period;
      if(timer >= RTIMER_LF_0) {
        *(&TA1CCR0 + (timer - RTIMER_LF_0)) += (uint16_t)(rt[timer].period);
      } else {
        *(&TA0CCR0 + timer) += (uint16_t)(rt[timer].period);
      }
      rt[timer].state = RTIMER_SCHEDULED;
    } else {
      if(timer >= RTIMER_LF_0) {
        /* otherwise, just stop it */
        *(&TA1CCTL0 + (timer - RTIMER_LF_0)) = 0;
      } else {
        *(&TA0CCTL0 + timer) = 0;
      }
      rt[timer].state = RTIMER_INACTIVE;
    }
  }
}
/*---------------------------------------------------------------------------*/
#define RTIMER_HF_CALLBACK(timer) \
  if((rtimer_now_hf() >= rt[timer].time) && \
     (rt[timer].state == RTIMER_SCHEDULED)) { \
    /* the timer has expired! */ \
    rt[timer].state = RTIMER_JUST_EXPIRED; \
    /* execute the proper callback function */ \
    rt[timer].func(&rt[timer]); \
    /* update or stop the timer */ \
    update_rtimer_state(timer); \
    if(process_nevents() > 0) { \
      LPM4_EXIT; \
    } \
  } else if (rt[timer].state == RTIMER_WFE) { \
    rt[timer].func(&rt[timer]); \
  }

#define RTIMER_LF_CALLBACK(timer) \
  if((rtimer_now_lf() >= rt[timer].time) && \
     (rt[timer].state == RTIMER_SCHEDULED)) { \
    /* the timer has expired! */ \
    rt[timer].state = RTIMER_JUST_EXPIRED; \
    /* execute the proper callback function */ \
    rt[timer].func(&rt[timer]); \
    /* update or stop the timer */ \
    update_rtimer_state(timer); \
    if(process_nevents() > 0) { \
      LPM4_EXIT; \
    } \
  } else if (rt[timer].state == RTIMER_WFE) { \
    rt[timer].func(&rt[timer]); \
  }
/*---------------------------------------------------------------------------*/
void
rtimer_init(void)
{
  /* initialize timer A0: */
  /* SMCLK (3.25 MHz), continuous mode, clear TA0R, overflow interrupt */
  ta0_sw_ext = 0;
  /* make sure the input divider expansion is set to 0 before setting the 
   * TACLR bit */
  TA0EX0 = 0;           
  TA0CTL = TASSEL_2 | MC_2 | ID__1 | TACLR | TAIE; /* SMCLK, input divider 1 */

  /* initialize timer A1: */
  /* ACLK, continuous mode, clear TA1R */
  ta1_sw_ext = 0;
  TA1EX0 = 0;
  TA1CTL = TASSEL_1 | MC_2 | ID__1 | TACLR | TAIE; /* ACLK */

  memset(rt, 0, sizeof(rt));
}
/*---------------------------------------------------------------------------*/
void
rtimer_schedule(rtimer_id_t timer,
                rtimer_clock_t start,
                rtimer_clock_t period,
                rtimer_callback_t func)
{
  if((timer < NUM_OF_RTIMERS) && (rt[timer].state != RTIMER_SCHEDULED)) {
    rt[timer].func = func;
    rt[timer].period = period;
    rt[timer].time = start + period;
    rt[timer].state = RTIMER_SCHEDULED;
    if(timer >= RTIMER_LF_0) {
      *(&TA1CCR0 + (timer - RTIMER_LF_0)) = (uint16_t)(start + period);
      *(&TA1CCTL0 + (timer - RTIMER_LF_0)) = CCIE | OUTMOD_4;
    } else {
      *(&TA0CCR0 + timer) = (uint16_t)(start + period);
      *(&TA0CCTL0 + timer) = CCIE | OUTMOD_4;            /* enable interrupt */
    }
  } else {
    DEBUG_PRINT_ERROR("invalid rtimer ID %u", timer);
  }
}
/*---------------------------------------------------------------------------*/
void 
rtimer_wait_for_event(rtimer_id_t timer, rtimer_callback_t func)
{
  /* must be an unscheduled timer */
  if((timer < NUM_OF_RTIMERS) && (rt[timer].state != RTIMER_SCHEDULED)) {
    rt[timer].func = func;
    rt[timer].state = RTIMER_WFE;
    if(timer >= RTIMER_LF_0) {
      /* set the timer to capture mode */
      /* rising edge, synchronize the capture with the next timer clock to
       * prevent race conditions, capture input select */
      *(&TA1CCTL0 + timer - RTIMER_LF_0) = CAP | CM_1 | SCS; 
      /* only enable interrupts when a callback function is provided */
      if (func) {       
        *(&TA1CCTL0 + timer - RTIMER_LF_0) |= CCIE;
      }
    } else {
      /* set the timer to capture mode */
      *(&TA0CCTL0 + timer) = CAP | CM_1 | SCS; 
      /* only enable interrupts when a callback function is provided */
      if (func) {       
        *(&TA0CCTL0 + timer) |= CCIE;
      }
    }
  } 
}
/*---------------------------------------------------------------------------*/
void
rtimer_stop(rtimer_id_t timer)
{
  if(timer < NUM_OF_RTIMERS) {
    if(timer >= RTIMER_LF_0) {
      *(&TA1CCTL0 + timer - RTIMER_LF_0) = 0;
      rt[timer].state = RTIMER_INACTIVE;
    } else {
      *(&TA0CCTL0 + timer) = 0;
      rt[timer].state = RTIMER_INACTIVE;
    }
  }
}
/*---------------------------------------------------------------------------*/
void
rtimer_reset(void)
{
  TA0R = 0;
  TA1R = 0;
  TA0CTL &= ~TAIFG;
  TA1CTL &= ~TAIFG;
  ta0_sw_ext = 0;
  ta1_sw_ext = 0;    
}
/*---------------------------------------------------------------------------*/
inline void
rtimer_update_enable(uint8_t enable)
{
  if(enable) {
    TA0CTL |= TAIE; 
    TA1CTL |= TAIE;
    //*(&TA0CCTL0 + LWB_CONF_RTIMER_ID) |= CCIE;
  } else {
    TA0CTL &= ~TAIE; 
    TA1CTL &= ~TAIE;
    //*(&TA0CCTL0 + LWB_CONF_RTIMER_ID) &= ~CCIE;
  }
}
/*---------------------------------------------------------------------------*/
inline uint8_t 
rtimer_update_enabled(void)
{
  return ((TA0CTL & TAIE) > 0);
  // ((TA1CTL & TAIE) > 0)
}
/*---------------------------------------------------------------------------*/
void
rtimer_interrupts_enable(uint8_t enable)
{
  static uint16_t int_state = 0;
  if(enable) {
    /* re-enable interrupts */
    TA0CTL |= TAIE;
    TA1CTL |= TAIE;
    TA0CCTL0 |= (int_state << 4) & CCIE;
    TA0CCTL1 |= (int_state << 3) & CCIE;
    TA0CCTL2 |= (int_state << 2) & CCIE;
    TA0CCTL3 |= (int_state << 1) & CCIE;
    TA1CCTL0 |= (int_state) & CCIE;
    TA1CCTL1 |= (int_state >> 1) & CCIE;
    TA1CCTL2 |= (int_state >> 2) & CCIE;
  } else {
    /* disable all timer related interrupts (overflow and ccr) except for
     * radio/glossy! */
    /* store current interrupt enable state */
    int_state = (TA0CCTL0 & CCIE) >> 4 |
                (TA0CCTL1 & CCIE) >> 3 |
                (TA0CCTL2 & CCIE) >> 2 |
                (TA0CCTL3 & CCIE) >> 1 |
                (TA1CCTL0 & CCIE)      |
                (TA1CCTL1 & CCIE) << 1 |
                (TA1CCTL2 & CCIE) << 2;
    TA0CCTL0 &= ~CCIE;
    TA0CCTL1 &= ~CCIE;
    TA0CCTL2 &= ~CCIE;
    TA0CCTL3 &= ~CCIE;
    TA1CCTL0 &= ~CCIE;
    TA1CCTL1 &= ~CCIE;
    TA1CCTL2 &= ~CCIE;
    TA0CTL &= ~TAIE;
    TA1CTL &= ~TAIE;
  }
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t
rtimer_now_hf(void)
{
  /* disable all interrupts */
  uint16_t interrupt_enabled = __get_interrupt_state() & GIE;
  /* or use: __get_SR_register()  or  (READ_SR & GIE) */
  __dint(); __nop();

  /* take a snapshot of both the HW timer and the SW extension */
  rtimer_clock_t sw = ta0_sw_ext;
  uint16_t hw = TA0R;
  if(TA0CTL & TAIFG) {
    /* in the meantime there has been an overflow of the HW timer: */
    /* manually increment the SW extension */
    sw++;
    /* and take a new snapshot of the HW timer */
    hw = TA0R;
  }
  /* shift the SW extension to the left and append the HW timer */
  rtimer_clock_t time = (sw << 16) | hw;

  /* only enable interrupts if the GIE bit was set before! otherwise interrupt
   * nesting will be enabled if rtimer_now_hf() is called from an ISR! */
  if(interrupt_enabled) {
    __eint(); __nop();
  }

  return time;
}
/*---------------------------------------------------------------------------*/
static uint16_t
rtimer_now_lf_hw(void)
{
  uint16_t hw1, hw2;
  do {
    /* majority vote: loop until both value are the same 
     * (necessary because clock sources of the CPU and TA1 are different) */
    hw1 = TA1R;
    hw2 = TA1R;
  } while (hw1 != hw2);
  return hw1;
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t
rtimer_now_lf(void)
{
  /* disable all interrupts */
  uint16_t interrupt_enabled = __get_interrupt_state() & GIE; //READ_SR & GIE;
  __dint(); __nop();

  /* take a snapshot of both the HW timer and the SW extension */
  rtimer_clock_t sw = ta1_sw_ext;
  uint16_t hw = rtimer_now_lf_hw();
  if(TA1CTL & TAIFG) {
    /* in the meantime there has been an overflow of the HW timer: */
    /* manually increment the SW extension */
    sw++;
    /* and take a new snapshot of the HW timer */
    hw = rtimer_now_lf_hw();
  }
  /* shift the SW extension to the left and append the HW timer */
  rtimer_clock_t time = (sw << 16) | hw;

  /* only enable interrupts if the GIE bit was set before! otherwise interrupt
   * nesting will be enabled if rtimer_now_hf() is called from an ISR! */
  if(interrupt_enabled) {
    __eint(); __nop();
  }
  
  return time;
}
/*---------------------------------------------------------------------------*/
void
rtimer_now(rtimer_clock_t* const hf_val, rtimer_clock_t* const lf_val)
{
  /* NOTE: This function will only work properly if the CPU and timer TA0 (HF)
   * are running from the same clock source */
  if(hf_val && lf_val) {
    /* disable all interrupts */
    uint16_t interrupt_enabled = __get_interrupt_state() & GIE;//READ_SR & GIE;
    __dint(); __nop();
    
    /* take a snapshot of the SW extension */
    rtimer_clock_t sw_hf = ta0_sw_ext;
    rtimer_clock_t sw_lf = ta1_sw_ext;
capture_values: ;
    uint16_t hw_hf = TA0R;
    uint16_t hw_lf = TA1R;
    uint16_t hw_lf2 = TA1R;
    if(hw_lf != hw_lf2) { 
      goto capture_values;
    }
    if((TA0CTL & TAIFG) && (sw_hf == ta0_sw_ext)) {
        /* in the meantime there has been an overflow of the HW timer: */
        /* manually increment the SW extension and recapture all values */
        sw_hf++;
        goto capture_values;        
    }
    if((TA1CTL & TAIFG) && (sw_lf == ta1_sw_ext)) {
        /* in the meantime there has been an overflow of the HW timer: */
        /* manually increment the SW extension and recapture all values */
        sw_lf++;
        goto capture_values;
    }
    /* compose the final timestamps (shift the SW extension to the left and 
    * append the HW timer */
    *hf_val = (sw_hf << 16) | hw_hf;
    *lf_val = (sw_lf << 16) | hw_lf;

    /* only enable interrupts if the GIE bit was set before! otherwise ISR
     * nesting will be enabled if rtimer_now_hf() is called from an ISR! */
    if(interrupt_enabled) {
        __eint(); __nop();
    }
  }
}
/*---------------------------------------------------------------------------*/
uint16_t
rtimer_swext_addr(rtimer_id_t timer)
{
   if(timer < RTIMER_CONF_NUM_HF) {
     return (uint16_t)&ta0_sw_ext;
   } 
   return (uint16_t)&ta1_sw_ext;
}
/*---------------------------------------------------------------------------*/
uint8_t
rtimer_next_expiration(rtimer_id_t timer, rtimer_clock_t* exp_time)
{
   if(timer < NUM_OF_RTIMERS) {
     *exp_time = rt[timer].time;
     return (rt[timer].state == RTIMER_SCHEDULED);
   }
   return 0;
}
/*---------------------------------------------------------------------------*/
clock_time_t
clock_time(void)
{
  clock_time_t t1, t2;
  do {
    t1 = count;
    t2 = count;
  } while(t1 != t2);
  return t1;
}
/*---------------------------------------------------------------------------*/
/* Timer A0, CCR0 interrupt service routine */
ISR(TIMER0_A0, timer0_a0_interrupt) 
{
  ENERGEST_ON(ENERGEST_TYPE_CPU);
  
  RTIMER_HF_CALLBACK(RTIMER_HF_0);
  if(process_nevents() > 0) {
    LPM4_EXIT;
  }

  ENERGEST_OFF(ENERGEST_TYPE_CPU);
}
/*---------------------------------------------------------------------------*/
/* Timer A0, CCR1-4 interrupt service routine */
ISR(TIMER0_A1, timer0_a1_interrupt) 
{
  ENERGEST_ON(ENERGEST_TYPE_CPU);

  switch(TA0IV) {
  case TA0IV_TA0CCR1:
    RTIMER_HF_CALLBACK(RTIMER_HF_1);
    break;
  case TA0IV_TA0CCR2:
    RTIMER_HF_CALLBACK(RTIMER_HF_2);
    break;
  case TA0IV_TA0CCR3:
    RTIMER_HF_CALLBACK(RTIMER_HF_3);
    break;
#if !RF_CONF_ON
  case TA0IV_TA0CCR4:
    RTIMER_HF_CALLBACK(RTIMER_HF_4);
    break;
#endif /* RF_CONF_ON */
  case TA0IV_TA0IFG:
    /* overflow of timer A0: increment its software extension */
    ta0_sw_ext++;
    /* increment also the etimer count */
    count++;
    /* check whether there are etimers ready to be served */
    if(etimer_pending() &&
       (etimer_next_expiration_time() - count - 1) > MAX_TICKS) {
      etimer_request_poll();
      LPM4_EXIT;
    }
    break;
  default: break;
  }
  
  ENERGEST_OFF(ENERGEST_TYPE_CPU);
}
/*---------------------------------------------------------------------------*/
/* Timer A1, CCR0 interrupt service routine (higher priority than TIMER1_A1) */
ISR(TIMER1_A0, timer1_a0_interrupt) 
{
  ENERGEST_ON(ENERGEST_TYPE_CPU);
  
  RTIMER_LF_CALLBACK(RTIMER_LF_0);

  ENERGEST_OFF(ENERGEST_TYPE_CPU);
}
/*---------------------------------------------------------------------------*/
/* Timer A1, CCR1-2 interrupt service routine */
ISR(TIMER1_A1, timer1_a1_interrupt) 
{
  ENERGEST_ON(ENERGEST_TYPE_CPU);
  
#if RTIMER_CONF_LF_UPDATE_LED_ON
  PIN_XOR(LED_STATUS);    /* to indicate activity */
#endif /* RTIMER_CONF_TOGGLE_LED_ON_LF_UPDATE */

  switch(TA1IV) {
  case TA1IV_TA1CCR1:
    RTIMER_LF_CALLBACK(RTIMER_LF_1);
    break;
  case TA1IV_TA1CCR2:
    RTIMER_LF_CALLBACK(RTIMER_LF_2);
    break;
  case TA1IV_TA1IFG:
    /* overflow of timer A1: increment its software extension */
    ta1_sw_ext++;
#if WATCHDOG_CONF_ON && WATCHDOG_CONF_RESET_ON_TA1IFG
    watchdog_reset();
#endif /* WATCHDOG */
    break;
  default: break;
  }

#if RTIMER_CONF_LF_UPDATE_LED_ON
  PIN_XOR(LED_STATUS);    /* to indicate activity */
#endif /* RTIMER_CONF_TOGGLE_LED_ON_LF_UPDATE */

  ENERGEST_OFF(ENERGEST_TYPE_CPU);
}
/*---------------------------------------------------------------------------*/
