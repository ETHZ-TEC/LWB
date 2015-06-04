// example code, demonstrates the use of debug output (UART), LEDs, timers, processes and basic message passing / synchronization

#include "contiki.h"

#define PERIOD (RTIMER_SECOND / 8)

static struct etimer timer;   // variables are declared static to ensure their values are kept each time the kernel switches back to this process
static process_event_t msg;


void toggle_leds(const rtimer_t *rt) {
	LEDS_TOGGLE;
}

uint16_t get_code_size() {
    static uint16_t* curr_addr = (uint16_t*)(FLASH_END - 1);   // 16-bit word alignment
    while (((uint16_t)curr_addr > FLASH_START) && (*curr_addr == 0xffff)) {
        curr_addr--;
    }
    return (uint16_t)curr_addr - FLASH_START + 2;
}


PROCESS(proc_blink, "toggles the LEDs");            // register the process 'proc_blink'
PROCESS(proc_receiver, "receiver process");
AUTOSTART_PROCESSES(&proc_blink, &proc_receiver);   // tell contiki to autostart the processes


PROCESS_THREAD(proc_blink, ev, data) {              // define the process

	PROCESS_BEGIN();
    
    static uint16_t count = 0;
    msg = process_alloc_event();                            // allocate the event
	rtimer_schedule(0, rtimer_now(), PERIOD, (rtimer_callback_t)toggle_leds);  // use a timer to toggle the LEDs
    etimer_set(&timer, CLOCK_SECOND);                       // use another timer to trigger the UART output
        
    // calculate the code size
    printf("Total code size is %d bytes\n", get_code_size());
    
    while (1) {
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));   // blocks the process until the specified event occurs
        // you could also use one of the following options to achieve the desired behavior:
        // > PROCESS_WAIT_EVENT(); if(ev == PROCESS_EVENT_TIMER) { ... }    
        // > PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_TIMER);
        
        printf("Hello World %i\n", count);  // logging (UART output)
        count++;
        
        // send 'count' to the other process
        process_post(&proc_receiver, msg, &count);
        
        etimer_reset(&timer);
    }    
	PROCESS_END();
}


PROCESS_THREAD(proc_receiver, ev, data)
{
    PROCESS_BEGIN();
    
    while (1)
    {
        PROCESS_WAIT_EVENT_UNTIL(ev == msg);
        printf("Message received: %d\n", (*(int*)data));
    }
    
    PROCESS_END();
}

