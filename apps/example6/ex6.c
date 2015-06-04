// example code, demonstrates the usage of the external FRAM

#include "contiki.h"


static uint8_t read_buffer[128];

PROCESS(framtest, "FRAM test");
AUTOSTART_PROCESSES(&framtest);


void printh(const uint8_t* byte_array, uint16_t num_bytes)
{
    while (num_bytes) {
        printf("%02x ", *byte_array);
        byte_array++;
        num_bytes--;
    }
    printf("\n");
}


PROCESS_THREAD(framtest, ev, data) {

	PROCESS_BEGIN();
    
    static uint16_t block_id;
                                          
    if (!fram_init()) {
        printf("failed to initialize the external FRAM\n");
    }       
    block_id = falloc(200);    // allocate a block with size 200 bytes!
    printf("\nblock ID is %d\n", block_id);
    fram_write(falloc_address(block_id, 0), 12, (uint8_t*)"hallo welt!");
    //FRAM_WAIT_COMPLETE;   // only necessary when using DMA
    fram_read(falloc_address(block_id, 0), 12, read_buffer);
    printf("\ndata received:\n");
    printf((char*)read_buffer);
    fram_sleep();                   // enter sleep mode
    
    while (1) {
        LEDS_TOGGLE;
        __delay_cycles(MCLK_SPEED);     // wait a second
    }    
	PROCESS_END();
}
