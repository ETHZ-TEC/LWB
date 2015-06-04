
/**
 * @file
 * @brief provides functionality to configure the USCI A0 module in UART mode
 * @author rdaforno
 */

#ifndef __UART0_H__
#define __UART0_H__


/**
 * @brief check if the UART module is active / busy (same as USCI_A0_ACTIVE)
 */
#define UART_ACTIVE         ( UCA0STAT & UCBUSY )

/**
 * @brief check if the USCI A0 module is active / busy (i.e. a transmission is ongoing)
 */
#define USCI_A0_ACTIVE      ( UCA0STAT & UCBUSY )

/**
 * @brief check if the USCI A0 module is configured in SPI mode
 */
#define USCI_A0_SPI_MODE    ( UCA0CTL0 & UCSYNC )


void uart0_set_input_handler(int (*input)(unsigned char c));
void uart0_init(void);
void uart0_reinit(void);


#endif /* __UART0_H__ */
