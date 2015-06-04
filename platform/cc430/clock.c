#include "contiki.h"

// TODO: periodically re-calibrate the DCO if not exited from LPM for a while

void clock_init(void) {

    // set clock sources:
    // FLLREFCLK <- XT1 (low-frequency crystal, 32,768 Hz) -> NOTE: no crystal installed on PCB
    // SMCLK     <- XT2 (high-frequency crystal, 26 MHz / 8 = 3.25 MHz)
    // ACLK      <- XT1 (low-frequency crystal, 32768 Hz)
    // MCLK      <- XT2 (high-frequency crystal, 26 MHz / 2 = 13 MHz)
    
    // set the supply voltage to the maximum
    SetVCore(PMMCOREV_3);

    SFRIE1 &= ~OFIE;
    
    // enable XT2 and XT1 (ensures that they stay always on)
    ENABLE_XT2();
    ENABLE_XT1(); 
        
    P5SEL   |= BIT0 + BIT1;     // set pins 5.0 and 5.1 to analog (XT1 operation)
    UCSCTL6 |= XCAP_3;          // set internal load capacitor for XT1

    // initially, use the internal REFO and DCODIV clock sources
    UCSCTL3 = SELREF__REFOCLK | FLLREFDIV_0;
    UCSCTL4 = SELA__DCOCLKDIV | SELS__REFOCLK | SELM__DCOCLKDIV;
    // wait until XT1, XT2, and DCO stabilize
    do {
        // clear XT1, XT2, and DCO fault flags
        UCSCTL7 &= ~(XT1LFOFFG + XT2OFFG + DCOFFG);
        // clear oscillator fault flag
        SFRIFG1 &= ~OFIFG;
    } while (SFRIFG1 & OFIFG);
        
    // enable oscillator fault interrupt (NMI)
    SFRIE1 = OFIE;
        
    // XT1 is now stable: reduce its drive strength to save power
    UCSCTL6 &= ~XT1DRIVE_3;

    // set the DCO frequency to 3.25 MHz
    // disable the FLL control loop
    DISABLE_FLL();
    
#ifdef USE_FLL
    // set the lowest possible DCOx, MODx
    UCSCTL0 = 0;
    // select the suitable DCO frequency range
    UCSCTL1 = DCORSEL_6;
    // set the FLL loop divider to 2 and
    // set the FLL loop multiplier N such that (N + 1) * f_FLLREF = f_DCO --> N = 396
    UCSCTL2 = FLLD_0 + 396;
    // enable the FLL control loop
    ENABLE_FLL();
    // wait until the DCO stabilizes
    // (up to 1 x 32 x 32 x 13 MHz / 32,768 Hz = 406,250 DCO cycles)
    __delay_cycles(406250);
#endif
    
    // finally, use the desired clock sources and speeds
    UCSCTL3 = SELREF__XT1CLK | FLLREFDIV_0;
    UCSCTL4 = SELA | SELS | SELM;
    UCSCTL5 = DIVA | DIVS | DIVM;

}
