
#include "contiki.h"

static int (*uart0_input_handler)(unsigned char c);

//static volatile uint8_t transmitting;
static uint32_t prescaler = 0;
static uint32_t mod = 0;


int putchar(int c) {         // this function must be defined (referenced from std lib printf.c)

  // print out a single byte over UART
#if WATCHDOG_CONF_ON
  watchdog_periodic();
#endif /* WATCHDOG_CONF_ON */
  /* Loop until the transmission buffer is available. */
  while((UCA0STAT & UCBUSY));

  /* Transmit the data. */
  UCA0TXBUF = c;
  return c;
}


/**
 * @brief set the input handler for the UART 
 * @param[in] input the function to be called from the UART ISR (RX interrupt)
 */
void uart0_set_input_handler(int (*input)(unsigned char c)) {
  uart0_input_handler = input;
}


/**
 * @brief initialize the USCI_A0 module in UART mode (port RS232)
 * @note set the desired baudrate in hal.h (UART0_BAUDRATE).
 * @remark The UART module is driven by the SMCLK.
 */
void uart0_init(void) {

    PIN_MAP_AS_INPUT(UART_A0_RX, PM_UCA0RXD);    // map UART RX input to port 1.5   (same as SPI A0 SOMI)
    PIN_MAP_AS_OUTPUT(UART_A0_TX, PM_UCA0TXD);   // map UART TX output to port 1.6  (same as SPI A0 SIMO)
#ifdef MUX_SEL
    PIN_SET_AS_OUTPUT(MUX_SEL);
    PIN_SET(MUX_SEL);
#endif
    
    UCA0CTL1 |= UCSWRST;                    // Hold peripheral in reset state 
    UCA0CTL1 &= ~UCSSEL_3;
    UCA0CTL1 |= UCSSEL__SMCLK;              // clock source select: SMCLK 
    UCA0CTL0 &= ~(UCMSB + UCSPB + UCPEN + UCSYNC + UC7BIT + UCMODE_3);   // LSB first is 0, one stop bit is 0, no parity is 0 -> just clear the bits
    
    uint32_t f = XT2CLK_SPEED / SMCLK_SPEED;
    uint32_t ratio = (SMCLK_SPEED * 100) / UART0_BAUDRATE;
    prescaler = ratio / 100;
    mod = ((ratio - prescaler * 100) * f + 50) / 100;       // note: factor of 8 results from 26 MHz / SMCLK_SPEED

    UCA0BRW = (uint16_t)prescaler;  // write the 16-bit clock prescaler
    UCA0MCTL = (uint8_t)mod << 1;   // modulation control register

    // select UART mode
    UCA0CTL0 |= UCMODE_0;   // default mode

    UCA0IE &= ~(UCRXIFG + UCTXIFG);     // clear pending interrupts 
    UCA0CTL1 &= ~UCSWRST;               // initialize USCI state machine **before** enabling interrupts
    //UCA0IE |= UCRXIE;                 // note: DO NOT enable interrupts, they can mess up the timing due to higher priority
}


/**
 * @brief re-initialize the USCI_A0 module in UART mode (when it is configured in SPI mode)
 * @remark The UART module is driven by the SMCLK.
 */
void uart0_reinit(void) {

    while (UART_ACTIVE);    // wait until all ongoing transmissions have terminated
#ifdef MUX_SEL
    PIN_SET(MUX_SEL);
#endif
    UCA0CTL1 |= UCSWRST; 
    UCA0CTL0 &= ~(UCMSB + UCSPB + UCPEN + UCSYNC + UC7BIT + UCMODE_3);
    UCA0BRW = (uint16_t)prescaler;
    //UCA0MCTL = (uint8_t)mod << 1;   // modulation control register
    UCA0CTL0 |= UCMODE_0;
    UCA0CTL1 &= ~UCSWRST;
}


ISR(USCI_A0, uart0_rx_interrupt) {

  //ENERGEST_ON(ENERGEST_TYPE_CPU);

  uint8_t c;

  if (UCA0IV == 2) {
    if (UCA0STAT & UCRXERR) {
      c = UCA0RXBUF;   /* Clear error flags by forcing a dummy read. */
    } else {
      c = UCA0RXBUF;
      if (uart0_input_handler != NULL) {
        if (uart0_input_handler(c)) {
          LPM4_EXIT;
        }
      }
    }
  }

  //ENERGEST_OFF(ENERGEST_TYPE_CPU);
}
