
#ifndef __WATCHDOG_H__
#define __WATCHDOG_H__



static inline void watchdog_stop(void) {
    WDTCTL = WDTPW + WDTHOLD;
}



#endif /* __WATCHDOG_H__ */