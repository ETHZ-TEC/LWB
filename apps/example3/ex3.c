// example code, demonstrates the usage of the external FRAM

#include "contiki.h"


static char buffer[32];
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
                      
    if (!fram_init()) {
        printf("failed to initialize the external FRAM\n");
    }       
    fram_write(0x10502, 12, (uint8_t*)"hallo welt!");
    while (!PIN_GET_INPUT_BIT(FRAM_CTRL));  // wait until transfer complete
    fram_read(0x10502, 12, read_buffer);
    printf((char*)read_buffer);
    
    printf("\ndevice id: ");   
    printf(fram_get_id(buffer));   
    printf("\n");    
    fram_sleep();                       // enter sleep mode
    __delay_cycles(MCLK_SPEED * 5);     // wait 5 seconds
           
    while (1) {
        fram_write(0x10502, 12, (uint8_t*)"hallo welt!");
        fram_read(0x10502, 12, read_buffer);
        printf((char*)read_buffer);
        printf("\n");
        fram_sleep();                   // enter sleep mode
        LEDS_TOGGLE;
        __delay_cycles(MCLK_SPEED);     // wait a second
    }    
	PROCESS_END();
}
