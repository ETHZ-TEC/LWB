// example code, demonstrates SPI usage and implements the handshake signaling and state machine to use the asynchronous interface

#include "contiki.h"

#define MESSAGE_SIZE    128
#define SPI_CLK_SPEED   250000 

#define DATA_AVAILABLE  PIN_GET_INPUT_BIT(AI_CTRL_IND)


typedef enum {
    STATE_IDLE = 0,
    STATE_PREPREAD = 1,
    STATE_PREPWRITE = 2,
    STATE_READ = 3,
    STATE_WRITE = 4,
} SYSTEM_STATE;

static struct  etimer timer;
static uint8_t async_int_state = STATE_IDLE;
static uint8_t ack_level = 0;

PROCESS(spiwrite, "write to the SPI bus");
AUTOSTART_PROCESSES(&spiwrite);


// initializes all necessary pins and modules for the use of the asynchronous interface
void ai_init() {
    // control signals
    PIN_SET_AS_INPUT(AI_CTRL_IND);
    PIN_CFG_PORT_INT(AI_CTRL_IND);
    PIN_SET_AS_OUTPUT(AI_CTRL_MODE);
    PIN_CLEAR(AI_CTRL_MODE);
    PIN_SET_AS_OUTPUT(AI_CTRL_REQ);
    PIN_CLEAR(AI_CTRL_REQ);
    PIN_SET_AS_INPUT(AI_CTRL_ACK);
    PIN_CFG_PORT_INT(AI_CTRL_ACK);
    
    // SPI
    spi_init(SPI_CLK_SPEED);
    
    printf("initialization complete\n");
}


void ai_req_op() {

    if (STATE_IDLE == async_int_state) {
        // --- 1. set MODE ---
        if (DATA_AVAILABLE) {
            PIN_CLEAR(AI_CTRL_MODE);    // 0 = READ
            async_int_state = STATE_PREPREAD;	
        } else {    
            PIN_SET(AI_CTRL_MODE);      // 1 = WRITE
            async_int_state = STATE_PREPWRITE;	
        }
        // --- 2. set up DMA --- (-> for now, just use polling)
        // ...				
        // --- 3. set a timeout timer ---
        etimer_set(&timer, 1);  // ~17ms
        // --- 4. set REQ = 1 ---
        PIN_SET(AI_CTRL_REQ);
    } else {
        printf("not in idle state, operation skipped\n");
    }
}


void ai_start_op() {

    static uint8_t bytes_to_transfer;
    static char rcv_buffer[MESSAGE_SIZE];
    
    bytes_to_transfer = MESSAGE_SIZE;
    if (STATE_PREPWRITE == async_int_state) {
        async_int_state = STATE_WRITE;      // this is a write operation
        while (bytes_to_transfer > 0) {
            SPI_TRANSMIT_BYTE(0x11);
            bytes_to_transfer--;
        }
    } else if (STATE_PREPREAD == async_int_state) {
        async_int_state = STATE_READ;       // perform a read operation
        // first, clear the RX buffer
        SPI_READ_BYTE;
        while (ack_level && bytes_to_transfer) {
            SPI_TRANSMIT_BYTE(0x00);    // dummy write to generate the clock
            SPI_RECEIVE_BYTE(rcv_buffer[MESSAGE_SIZE - bytes_to_transfer]);
            bytes_to_transfer--;
        }
        rcv_buffer[MESSAGE_SIZE - 1] = 0;
        printf("received message: '%s'\n", rcv_buffer);    
    } else {
        printf("invalid call to startOperation(), system is in state %d\n", async_int_state);
        return;
    }
    // transfer complete!
    printf("transfer complete\n");
    // --- 1. stop DMA ---
	// ...
	// --- 2. wait for BUSY flag ---
	SPI_WAIT_BUSY;
	// --- 3. set REQ = 0 ---
	PIN_CLEAR(AI_CTRL_REQ);
	// --- 4. empty the RX buffer ---
	SPI_READ_BYTE;
    // --- 5. process data ---
    // ...
    // --- 6. wait for ACK to go down ---
	while (PIN_GET_INPUT_BIT(AI_CTRL_ACK));
    async_int_state = STATE_IDLE;
    printf("back in idle state\n");
}


PROCESS_THREAD(spiwrite, ev, data) {              // define the process

	PROCESS_BEGIN();
    
    // configure the pins (handshake signaling)
    ai_init();
    SPI_ENABLE;
    LED_ON(LED_GREEN);
           
    while (1) {
    
        ack_level = 0;
        ai_req_op();
        PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
        
        if (!ack_level) {
            printf("timeout\n");    // timeout!
            async_int_state = STATE_IDLE;
            PIN_CLEAR(AI_CTRL_REQ);
            while (PIN_GET_INPUT_BIT(AI_CTRL_ACK));
            async_int_state = STATE_IDLE;
            printf("back in idle state\n");
        } else
        {
            // start the data transfer
            ai_start_op();
        }
        __delay_cycles(13000000);   // wait a second
    }    
	PROCESS_END();
}


ISR(PORT2, port2_interrupt) {

    ENERGEST_ON(ENERGEST_TYPE_CPU);

    if (PIN_IFG(AI_CTRL_IND)) {
        printf("port 2 interrupt: IND pin (level is %d)\n", PIN_GET_INPUT_BIT(AI_CTRL_IND));
        PIN_IES_TOGGLE(AI_CTRL_IND);
        PIN_CLEAR_IFG(AI_CTRL_IND);

    } else if (PIN_IFG(AI_CTRL_ACK)) {
        printf("port 2 interrupt: ACK pin (level is %d)\n", PIN_GET_INPUT_BIT(AI_CTRL_ACK));
        PIN_IES_TOGGLE(AI_CTRL_ACK);
        PIN_CLEAR_IFG(AI_CTRL_ACK);
        ack_level = PIN_GET_INPUT_BIT(AI_CTRL_ACK);
    } else {
        printf("invalid interrupt on port 2");
    }

    ENERGEST_OFF(ENERGEST_TYPE_CPU);
}
