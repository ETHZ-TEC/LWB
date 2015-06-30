#ifndef __CLOCK_H__
#define __CLOCK_H__


// FLL is only required if one of the following clock sources is used: DCOCLK, DCOCLKDIV, FLLREFCLK
//#define USE_FLL

// speed of XT1 (low-frequency crystal)
#define XT1CLK_SPEED    32768

// speed of XT2 (high-frequency crystal)
#define XT2CLK_SPEED    26000000LU

// source and speed of the Master Clock MCLK
#define SELM            SELM__XT2CLK
#define DIVM            DIVM__2
#define MCLK_SPEED      (XT2CLK_SPEED / 2)

// source and speed of the Auxiliary Clock ACLK
#define SELA            SELA__XT1CLK
#define DIVA            DIVA__1
#define ACLK_SPEED      (XT1CLK_SPEED / 1)

// source and speed of the Sub-System Master Clock SMCLK
#define SELS            SELS__XT2CLK
#define DIVS            DIVS__8
#define SMCLK_SPEED     (XT2CLK_SPEED / 8)      // 3.25 MHz

// check whether the high-frequency crystal XT2 is permanently enabled
#define IS_XT2_ENABLED() (~(UCSCTL6 & XT2OFF))

// permanently enable the high-frequency crystal XT2 (i.e., even if the radio is in SLEEP mode)
#define ENABLE_XT2()    (UCSCTL6 &= ~XT2OFF)
#define ENABLE_XT1()    (UCSCTL6 &= ~XT1OFF)

// disable the high-frequency crystal XT2 (i.e., active only when the radio is active)
#define DISABLE_XT2()   (UCSCTL6 |= XT2OFF)
#define DISABLE_XT1()   (UCSCTL6 |= XT1OFF)

// disable automatic clock requests for ACLK
#define DISABLE_ACLK()  { UCSCTL8 &= ~ACLKREQEN; }

// disable automatic clock requests for ACLK
#define DISABLE_SMCLK() { UCSCTL8 &= ~SMCLKREQEN; }

// enable the FLL control loop
#define ENABLE_FLL()    (__bic_status_register(SCG0))

// disable the FLL control loop
#define DISABLE_FLL()   (__bis_status_register(SCG0))


typedef uint32_t clock_time_t;

#define CLOCK_SECOND    50      // corresponds to roughly 1.008246 seconds

// initialize the clock system
void clock_init(void);

clock_time_t clock_time(void);


#endif /* __CLOCK_H__ */
