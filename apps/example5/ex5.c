// example code, demonstrates the usage of the async interface (DMA-driven data transfer)

#include "contiki.h"


PROCESS(async_int, "async interface test");
AUTOSTART_PROCESSES(&async_int);

static const char msg[MESSAGE_SIZE] = "hallo welt! hallo welt!";
static char rcv_buf[MESSAGE_SIZE];


PROCESS_THREAD(async_int, ev, data) {              // define the process

	PROCESS_BEGIN();
    
    // configure the pins (handshake signaling)
    async_int_init((uint16_t)rcv_buf, (uint16_t)msg);
    LED_ON(LED_GREEN);
           
    while (1) {
    
        if (ASYNC_INT_DATA_AVAILABLE) {
            memset(rcv_buf, 0, MESSAGE_SIZE);   // for debugging purposes
            ASYNC_INT_READ; //((uint8_t*)rcv_buf);
            ASYNC_INT_WAIT_COMPLETED;
            rcv_buf[MESSAGE_SIZE - 1] = 0;
            printf("received message: '%s'\n", rcv_buf);
        } else {
            printf("no data available\n");
            ASYNC_INT_WRITE(24);    // (uint8_t*)msg
        }     
        __delay_cycles(MCLK_SPEED);   // wait 1 second
        LEDS_TOGGLE;
    }   
	PROCESS_END();
}

