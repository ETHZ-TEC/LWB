#ifndef __LEDS_H__
#define __LEDS_H__


#ifdef USE_LEDS
    #define LED_ON(portandpin)      PIN_SET_DIRECT(portandpin)
    #define LED_OFF(portandpin)     PIN_CLEAR_DIRECT(portandpin)
    #define LED_TOGGLE(portandpin)  PIN_TOGGLE_DIRECT(portandpin)
#else
    #define LED_ON(portandpin)   
    #define LED_OFF(portandpin)  
    #define LED_TOGGLE(portandpin)    
#endif

#ifdef BOARD_COMM_V1
    #define LEDS_ON                 { PIN_SET(LED_0); PIN_SET(LED_1); PIN_SET(LED_2); PIN_SET(LED_3); }
    #define LEDS_OFF                { PIN_CLEAR(LED_0); PIN_CLEAR(LED_1); PIN_CLEAR(LED_2); PIN_CLEAR(LED_3); }
    #define LEDS_TOGGLE             { PIN_TOGGLE(LED_0); PIN_TOGGLE(LED_1); PIN_TOGGLE(LED_2); PIN_TOGGLE(LED_3); }
    #define LEDS_INIT               { LEDS_OFF; PIN_UNSELECT(LED_0); PIN_SET_AS_OUTPUT(LED_0); \
                                      PIN_UNSELECT(LED_1); PIN_SET_AS_OUTPUT(LED_1); \
                                      PIN_UNSELECT(LED_2); PIN_SET_AS_OUTPUT(LED_2); \
                                      PIN_UNSELECT(LED_3); PIN_SET_AS_OUTPUT(LED_3); }
#endif

#ifdef BOARD_EM430F5137RF900
    #define LEDS_ON                 { PIN_SET(LED_GREEN); PIN_SET(LED_RED); }
    #define LEDS_OFF                { PIN_CLEAR(LED_GREEN); PIN_CLEAR(LED_RED); }
    #define LEDS_TOGGLE             { PIN_TOGGLE(LED_GREEN); PIN_TOGGLE(LED_RED); }
    #define LEDS_INIT               { PIN_CLEAR(LED_GREEN); PIN_SET_AS_OUTPUT(LED_GREEN); \
                                      PIN_CLEAR(LED_RED); PIN_SET_AS_OUTPUT(LED_RED); }
#endif

#ifdef BOARD_MSP430CCRF
    #define LEDS_ON                 { PIN_SET(LED_RED); }
    #define LEDS_OFF                { PIN_CLEAR(LED_RED); }
    #define LEDS_TOGGLE             { PIN_TOGGLE(LED_RED);  }
    #define LEDS_INIT               { PIN_CLEAR(LED_RED); PIN_SET_AS_OUTPUT(LED_RED); }
#endif


#endif /* __LEDS_H__ */
