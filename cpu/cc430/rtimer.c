
#include "contiki.h"
#include "platform.h"

/* 
 * modified by rdaforno:
 * - Type of timer ID changed from uint16_t to rtimer_id (enum)
 * - TA1 fed by SMCLK
 * - TA1 rt struct added 
 */


 
// rtimer structs
static rtimer_t rt[NUM_OF_RTIMERS];

// SW extension for timer A0
volatile rtimer_clock_t ta0_sw_ext;
volatile rtimer_clock_t ta1_sw_ext;
static volatile uint16_t reenable_interrupts;

#define MAX_TICKS (~((clock_time_t)0) / 2)
static volatile clock_time_t count = 0;



static inline void update_rtimer_state(uint16_t timer) {
    // update the state only if the rtimer has not been manually
    // stopped or re-scheduled by the callback function
    if (rt[timer].state == RTIMER_JUST_EXPIRED) {
        if (rt[timer].period > 0) {
            // if it is periodic, schedule the new expiration
            rt[timer].time += rt[timer].period;
            if (timer >= RTIMER_TA1_0) { 
                *(&TA1CCR0 + timer - RTIMER_TA1_0) += (uint16_t)(rt[timer].period);
            } else {
                *(&TA0CCR0 + timer) += (uint16_t)(rt[timer].period);                
            }
            rt[timer].state = RTIMER_SCHEDULED;
        } else {
            if (timer >= RTIMER_TA1_0) { 
                // otherwise, just stop it
                *(&TA1CCTL0 + timer - RTIMER_TA1_0) = 0;
            } else {
                *(&TA0CCTL0 + timer) = 0;
            }
            rt[timer].state = RTIMER_INACTIVE;
        }
    }
}

#define RTIMER_CALLBACK(timer) \
    if ((rtimer_now() >= rt[timer].time) && (rt[timer].state == RTIMER_SCHEDULED)) { \
        /* the timer has expired! */ \
        rt[timer].state = RTIMER_JUST_EXPIRED; \
        /* execute the proper callback function */ \
        rt[timer].func(&rt[timer]); \
        /* update or stop the timer */ \
        update_rtimer_state(timer); \
        if (process_nevents() > 0) { \
            LPM4_EXIT; \
        } \
    }

    
#define RTIMER_TA1_CALLBACK(timer) \
    if ((rtimer_now_ta1() >= rt[timer].time) && (rt[timer].state == RTIMER_SCHEDULED)) { \
        /* the timer has expired! */ \
        rt[timer].state = RTIMER_JUST_EXPIRED; \
        /* execute the proper callback function */ \
        rt[timer].func(&rt[timer]); \
        /* update or stop the timer */ \
        update_rtimer_state(timer); \
        if (process_nevents() > 0) { \
            LPM4_EXIT; \
        } \
    }
    
    
/**
 * @brief initialize the rtimers
 */
void rtimer_init(void) {
    // initialize timer A0:
    // SMCLK (3.25 MHz), continuous mode, clear TA0R, overflow interrupt
    ta0_sw_ext = 0;
    TA0EX0 = 0;         // make sure the input divider expansion is set to 0 before setting the TACLR bit
    TA0CTL = TASSEL_2 | MC_2 | ID__1 | TACLR | TAIE;    // SMCLK, input divider 1

    // initialize timer A1:
    // ACLK, continuous mode, clear TA1R
    ta1_sw_ext = 0;
    TA1EX0 = 0;
    TA1CTL = TASSEL_1 | MC_2 | ID__1 | TACLR | TAIE;    // ACLK

    memset(rt, 0, sizeof(rt));
}

/**
 * @brief schedule (start) an rtimer
 * @param[in] timer the ID of the rtimer, must by of type rtimer_id_t
 * @param[in] start the expiration time (absolute counter value)
 * @param[in] period the period; set this to zero if you don't want this timer to expire and trigger an interrupt periodically
 * @param[in] func the callback function; will be executed as soon as the timer has expired
 */
void rtimer_schedule(rtimer_id_t timer, rtimer_clock_t start, rtimer_clock_t period, rtimer_callback_t func) {
    if ((timer < NUM_OF_RTIMERS) && (rt[timer].state != RTIMER_SCHEDULED)) {
        rt[timer].func = func;
        rt[timer].period = period;
        rt[timer].time = start + period;
        rt[timer].state = RTIMER_SCHEDULED;
        if (timer >= RTIMER_TA1_0) {        
            *(&TA1CCR0 + timer - RTIMER_TA1_0) = (uint16_t)(start + period);
            *(&TA1CCTL0 + timer - RTIMER_TA1_0) = CCIE | OUTMOD_4;      // enable interrupt            
        } else if (timer < N_RTIMERS) {
            *(&TA0CCR0 + timer) = (uint16_t)(start + period);
            *(&TA0CCTL0 + timer) = CCIE | OUTMOD_4;      // enable interrupt
        } else {
            // this timer ID may not be used!
            rt[timer].state = RTIMER_INACTIVE;
        }
    } else {
        DEBUG_PRINT_VERBOSE("invalid rtimer ID");
    }
}

/**
 * @brief stop an rtimer
 * @param[in] timer the ID of the rtimer, must by of type rtimer_id_t
 */
void rtimer_stop(rtimer_id_t timer) {
    if (timer < NUM_OF_RTIMERS) {
        if (timer >= RTIMER_TA1_0) { 
            *(&TA1CCTL0 + timer - RTIMER_TA1_0) = 0;
            rt[timer].state = RTIMER_INACTIVE;
        } else {
            *(&TA0CCTL0 + timer) = 0;
            rt[timer].state = RTIMER_INACTIVE;
        }
    }
}

/**
 * @brief get the period of the specified rtimer
 * @param[in] timer the ID of the rtimer, must by of type rtimer_id_t
 */
rtimer_clock_t rtimer_get_period(rtimer_id_t timer) {
    if ((timer < N_RTIMERS) && (rt[timer].state != RTIMER_INACTIVE)) {
        return rt[timer].period;
    } else {
        return 0;
    }
}

/**
 * @brief get the expiration time of the specified rtimer
 * @param[in] timer the ID of the rtimer, must by of type rtimer_id_t
 * @return expiration time in timer clock ticks (timestamp)
 */
rtimer_clock_t rtimer_get_expiration_time(rtimer_id_t timer) {
    if ((timer < N_RTIMERS) && (rt[timer].state != RTIMER_INACTIVE)) {
        return rt[timer].time;
    } else {
        return 0;
    }
}

/**
 * @brief get the current timer value of TA0
 * @return timer value in timer clock ticks (timestamp)
 */
rtimer_clock_t rtimer_now(void) {
    // check whether interrupts must be re-enabled at the end of the function
    // depending on the current value of the GIE bit
    reenable_interrupts = __read_status_register() & GIE;

    // disable all interrupts
    __dint(); __nop();

    // take a snapshot of both the HW timer and the SW extension
    rtimer_clock_t sw = ta0_sw_ext;
    uint16_t hw = TA0R;
    if (TA0CTL & TAIFG) {
        // in the meantime there has been an overflow of the HW timer:
        // manually increment the SW extension
        sw++;
        // and take a new snapshot of the HW timer
        hw = TA0R;
    }
    // shift the SW extension to the left and append the HW timer
    rtimer_clock_t time = (sw << 16) | hw;

    if (reenable_interrupts) {
        // re-enable all interrupts
        __eint(); __nop();
    }

    return time;
}


/**
 * @brief get the current timer value of TA1
 * @return timer value in timer clock ticks (timestamp)
 */
rtimer_clock_t rtimer_now_ta1(void) {
    // check whether interrupts must be re-enabled at the end of the function
    // depending on the current value of the GIE bit
    reenable_interrupts = __read_status_register() & GIE;

    // disable all interrupts
    __dint(); __nop();

    // take a snapshot of both the HW timer and the SW extension
    rtimer_clock_t sw = ta1_sw_ext;
    uint16_t hw = TA1R;
    if (TA1CTL & TAIFG) {
        // in the meantime there has been an overflow of the HW timer:
        // manually increment the SW extension
        sw++;
        // and take a new snapshot of the HW timer
        hw = TA1R;
    }
    // shift the SW extension to the left and append the HW timer
    rtimer_clock_t time = (sw << 16) | hw;

    if (reenable_interrupts) {
        // re-enable all interrupts
        __eint(); __nop();
    }

    return time;
}


// Timer A0, CCR0 interrupt service routine
ISR(TIMER0_A0, timer0_a0_interrupt) {

    ENERGEST_ON(ENERGEST_TYPE_CPU);

    RTIMER_CALLBACK(RTIMER_TA0_0);
    if (process_nevents() > 0) {
        LPM4_EXIT;
    }

    ENERGEST_OFF(ENERGEST_TYPE_CPU);

}

// Timer A0, CCR1-4 interrupt service routine
ISR(TIMER0_A1, timer0_a1_interrupt) {

    ENERGEST_ON(ENERGEST_TYPE_CPU);

    switch (TA0IV) {
    case TA0IV_TA0CCR1:
        RTIMER_CALLBACK(RTIMER_TA0_1);
        break;
    case TA0IV_TA0CCR2:
        RTIMER_CALLBACK(RTIMER_TA0_2);
        break;
    case TA0IV_TA0CCR3:
        RTIMER_CALLBACK(RTIMER_TA0_3);
        break;
#ifndef WITH_RADIO
    case TA0IV_TA0CCR4:
        RTIMER_CALLBACK(RTIMER_TA0_4);
        break;
#endif /* WITH_RADIO */
    case TA0IV_TA0IFG:
        // overflow of timer A0: increment its software extension
        ta0_sw_ext++;
        
        // increment also the etimer count
        count++;
        // check whether there are etimers ready to be served
        if (etimer_pending() && (etimer_next_expiration_time() - count - 1) > MAX_TICKS) {
            etimer_request_poll();
            LPM4_EXIT;
        }
        break;
    default: break;
    }

    ENERGEST_OFF(ENERGEST_TYPE_CPU);

}

// Timer A1, CCR0 interrupt service routine (higher priority than TIMER1_A1)
ISR(TIMER1_A0, timer1_a0_interrupt) {
    
    ENERGEST_ON(ENERGEST_TYPE_CPU);
    
    /*if (rt_ta1[0] != 0) {
        rt_ta1[0]();   // call the callback function    
    }
    TA1CCTL0 &= ~(CCIFG + COV);*/
    
    RTIMER_TA1_CALLBACK(RTIMER_TA1_0);
    
    ENERGEST_OFF(ENERGEST_TYPE_CPU);
}

// Timer A1, CCR1-2 interrupt service routine
ISR(TIMER1_A1, timer1_a1_interrupt) {

    ENERGEST_ON(ENERGEST_TYPE_CPU);

    switch (TA1IV) {
    case TA1IV_TA1CCR1:
        //if (rt_ta1[1] != 0) {
        //    rt_ta1[1]();   // call the callback function    
        //}
        RTIMER_TA1_CALLBACK(RTIMER_TA1_1);
        break;
    case TA1IV_TA1CCR2:
        //if (rt_ta1[2] != 0) {
        //    rt_ta1[2]();   // call the callback function    
        //}
        RTIMER_TA1_CALLBACK(RTIMER_TA1_2);
        break;
    case TA1IV_TA1IFG:
        // overflow of timer A1: increment its software extension
        ta1_sw_ext++;
        //PIN_TOGGLE_DIRECT(2,6);
        break;
    default: break;
    }

    ENERGEST_OFF(ENERGEST_TYPE_CPU);

}


clock_time_t clock_time(void) {
    clock_time_t t1, t2;
    do {
        t1 = count;
        t2 = count;
    } while(t1 != t2);
    return t1;
}
