// example code, demonstrates SPI usage and implements the handshake signaling and state machine to use the asynchronous interface

#include "contiki.h"


PROCESS(aiwrite, "async interface test");
AUTOSTART_PROCESSES(&aiwrite);

static const char msg[MESSAGE_SIZE] = "hallo welt! hallo welt!";
static char rcv_buf[MESSAGE_SIZE];


PROCESS_THREAD(aiwrite, ev, data) {              // define the process

	PROCESS_BEGIN();
    
    // configure the pins (handshake signaling)
    __delay_cycles(1000);
    async_int_init();
    SPI_ENABLE;
    LED_ON(LED_GREEN);
           
    while (1) {
    
        if (ASYNC_INT_DATA_AVAILABLE) {
            ASYNC_INT_READ((uint8_t*)rcv_buf);
            rcv_buf[MESSAGE_SIZE - 1] = 0;
            printf("\nreceived message: %s\n", rcv_buf);
        } else {
            ASYNC_INT_WRITE((uint8_t*)msg, 24);
        }
        __delay_cycles(MCLK_SPEED/4);   // wait 0.25 seconds
        LEDS_TOGGLE;
    }    
	PROCESS_END();
}

