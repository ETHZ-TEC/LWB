#include "debug-print.h"

// modified by rdaforno


#if DEBUG_PRINT_CONF_ON

char content[DEBUG_PRINT_MAX_LENGTH];   // global buffer, required to compose the messages

#ifdef DEBUG_PRINT_BUFFER_XMEM
    static uint8_t  n_buffered_msg = 0;
    static uint32_t start_addr_msg = XMEM_INVALID_ADDR;
    static debug_print_t msg;
#else
    MEMB(debug_print_memb, debug_print_t, N_DEBUG_MSG);
    LIST(debug_print_list);
#endif // DEBUG_PRINT_BUFFER_XMEM



PROCESS(debug_print_process, "Debug print process");
PROCESS_THREAD(debug_print_process, ev, data) {
    PROCESS_BEGIN();

#ifdef DEBUG_PRINT_BUFFER_XMEM
    static uint32_t next_msg = XMEM_INVALID_ADDR;
    n_buffered_msg = 0;
    start_addr_msg = XMEM_INVALID_ADDR;   // this line is necessary!
    fram_init();        // init if not already done
    start_addr_msg = fram_alloc(N_DEBUG_MSG * sizeof(debug_print_t));
#else
    memb_init(&debug_print_memb);
    list_init(debug_print_list);
#endif // DEBUG_PRINT_BUFFER_XMEM

    while (1) {
        DEBUG_TASK_SUSPENDED;
        PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
        // wait until we are polled by somebody
        DEBUG_TASK_ACTIVE;
  #ifdef DEBUG_PRINT_BUFFER_XMEM
        next_msg = start_addr_msg;
        while (n_buffered_msg > 0) {
            // load the message from the external memory
            fram_read(next_msg, sizeof(debug_print_t), (uint8_t*)&msg);      // load the message
        #ifdef MUX_SEL
            uart0_reinit();    // make sure the USCI_A0 module is in UART mode and select the MUX output accordingly
        #endif
            printf("%3u %7llu %c: %s\r\n", node_id, RTIMER_TO_MS(msg.time), msg.protocol, msg.content);
            next_msg += sizeof(debug_print_t);
            n_buffered_msg--;
            // do not pause process between the print-outs (otherwise a circular buffer / list data structure will be necessary!)
            // only let this task run when no other work is to do
        }
  #else        
    #ifdef MUX_SEL
        uart0_reinit();    // make sure the USCI_A0 module is in UART mode and select the MUX output accordingly
    #endif
        while (list_length(debug_print_list) > 0) {
            DEBUG_TASK_SUSPENDED;
            #ifdef WITH_RADIO
      #ifdef WITH_GLOSSY
            // do not try to print anything over the UART while Glossy is active
            while (glossy_is_active()) {
                PROCESS_PAUSE();
            }
      #else
            // do not try to print anything over the UART while the radio is busy
            while (rf1a_is_busy()) {
                PROCESS_PAUSE();
            }
      #endif /* WITH_GLOSSY */
    #endif /* WITH_RADIO */
            DEBUG_TASK_ACTIVE;
            // print the first message in the queue
            debug_print_t *msg = list_head(debug_print_list);
            printf("%3u %7llu %c: %s\r\n", node_id, RTIMER_TO_MS(msg->time), msg->protocol, msg->content);
            // remove it from the queue
            list_remove(debug_print_list, msg);
            memb_free(&debug_print_memb, msg);
            DEBUG_TASK_SUSPENDED;
            PROCESS_PAUSE();
            DEBUG_TASK_ACTIVE;
        }
  #endif // DEBUG_PRINT_BUFFER_XMEM
    }
    PROCESS_END();
}

void debug_print_init(void) {
    process_start(&debug_print_process, NULL);
}

void debug_process_poll(void) {
    process_poll(&debug_print_process);
}

void debug_print_msg(rtimer_clock_t *time, char protocol, char *content) {
#ifdef DEBUG_PRINT_BUFFER_XMEM
    if (n_buffered_msg < N_DEBUG_MSG && XMEM_INVALID_ADDR != start_addr_msg) {
        // construct the message struct
        if (time == NULL) {
            msg.time = rtimer_now();
        } else {
            msg.time = *time;
        }
        msg.protocol = protocol;
        memcpy(msg.content, content, DEBUG_PRINT_MAX_LENGTH);
        // write to external memory
        fram_write(start_addr_msg + n_buffered_msg * sizeof(debug_print_t), sizeof(debug_print_t), (uint8_t*)&msg);
        n_buffered_msg++;
        
        // do NOT poll the debug print process here!
    }
#else
    debug_print_t *msg = memb_alloc(&debug_print_memb);
    if (msg != NULL) {
        // construct the message struct
        if (time == NULL) {
            msg->time = rtimer_now();
        } else {
            msg->time = *time;
        }
        msg->protocol = protocol;
        memcpy(msg->content, content, DEBUG_PRINT_MAX_LENGTH);
        // add it to the list of messages ready to print
        list_add(debug_print_list, msg);

        // poll the debug print process
        process_poll(&debug_print_process);
    }
#endif // DEBUG_PRINT_BUFFER_XMEM
}

#else

void debug_print_init(void) { }
void debug_print_msg(rtimer_clock_t *time, char *protocol, char *content) { }

#endif /* DEBUG_PRINT_CONF_ON */
