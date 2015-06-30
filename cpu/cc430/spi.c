  
#include "platform.h"


uint32_t spi_sck_div_a0 = 0;


/**
 * @brief initialize the SPI B0 module
 *
 * This function configures USCI B0 peripheral in SPI mode. The default configuration is used: Master, MSB first, 8-bit, 3 lines.
 *
 * @param[in]  bit_clk_speed the desired serial clock frequency in Hz
 * @remark     clock phase and polarity can be set with the definitions SPI_B0_CPOL and SPI_B0_CPHA in hal.h
 */
void spi_b0_init(uint32_t bit_clk_speed) {

    // GPIO configuration
    PIN_SET_AS_MODULE_FUNC(SPI_B0_SOMI);
    PIN_SET_AS_MODULE_FUNC(SPI_B0_SIMO);
    PIN_SET_AS_MODULE_FUNC(SPI_B0_CLK);
    PIN_SET_AS_INPUT(SPI_B0_SOMI);
    //PIN_RESISTOR_EN(SPI_B0_SOMI);
    PIN_SET_AS_OUTPUT(SPI_B0_SIMO);
    PIN_SET_AS_OUTPUT(SPI_B0_CLK);
    
    while (SPI_B0_ACTIVE);          // busy wait
    
    uint32_t sck_div = SMCLK_SPEED / bit_clk_speed;
    
    UCB0CTL1 |= UCSWRST;            // reset USCI (disable the module)
    // reset OFS_UCBxCTL0 bits
    UCB0CTL0 &= ~(UCCKPH + UCCKPL + UC7BIT + UCMSB + UCMST + UCMODE_3 + UCSYNC);
    // reset OFS_UCBxCTL1 bits
    UCB0CTL1 &= ~(UCSSEL_3);
    // select clock source
    UCB0CTL1 |= UCSSEL__SMCLK;      // use SMCLK
    // bit rate control register (clock prescaler)
    UCB0BRW = sck_div;          
    // UCMST = Master mode, UCSYNC = Synchronous mode, UCMSB = MSB first, UCMODE_0 = 0 = 3-pin SPI
    UCB0CTL0 |= (UCMSB + UCMST + UCSYNC + (SPI_B0_CPOL ? UCCKPL : 0) + (SPI_B0_CPHA ? 0 : UCCKPH));
    // clear pending interrupts
    UCB0IE &= ~(UCRXIE + UCTXIE);   
    //UCB0CTL1 &= ~UCSWRST;         // re-enable the module
}


/**
 * @brief initialize the SPI A0 module
 *
 * This function configures USCI A0 peripheral in SPI mode. The default configuration is used: Master, MSB first, 8-bit, 3lines.
 *
 * @param[in]  bit_clk_speed the desired serial clock frequency in Hz
 * @remark     clock phase and polarity can be set with the definitions SPI_A0_CPOL and SPI_A0_CPHA in hal.h
 */
void spi_a0_init(uint32_t bit_clk_speed) {

    PIN_SET_AS_MODULE_FUNC(SPI_A0_SOMI);
    PIN_SET_AS_MODULE_FUNC(SPI_A0_SIMO);
    PIN_SET_AS_MODULE_FUNC(SPI_A0_CLK);
    PIN_SET_AS_INPUT(SPI_A0_SOMI);
    PIN_SET_AS_OUTPUT(SPI_A0_SIMO);
    PIN_SET_AS_OUTPUT(SPI_A0_CLK);
    P1DS = 0;
    
#ifdef MUX_SEL
    PIN_SET_AS_OUTPUT(MUX_SEL);
    PIN_CLEAR(MUX_SEL);
#endif
    while (SPI_A0_ACTIVE);          // busy wait
    
    spi_sck_div_a0 = SMCLK_SPEED / bit_clk_speed;
               
    // see comments in code above for explanation
    UCA0CTL1 |= UCSWRST; 
    UCA0CTL0 &= ~(UCCKPH + UCCKPL + UC7BIT + UCMSB + UCMST + UCMODE_3 + UCSYNC);
    UCA0CTL1 &= ~(UCSSEL_3);
    UCA0CTL1 |= UCSSEL__SMCLK;
    UCA0BRW   = spi_sck_div_a0;
    UCA0CTL0 |= (UCMSB + UCMST + UCSYNC + (SPI_A0_CPOL ? UCCKPL : 0) + (SPI_A0_CPHA ? 0 : UCCKPH));
    UCA0IE   &= ~(UCRXIFG + UCTXIFG);
}


/**
 * @brief re-initialize the SPI A0 module (when it was configured in UART mode before)
 *
 * This function re-configures USCI A0 peripheral in SPI mode (default config: Master, MSB first, 8-bit, 3lines). Use this function to switch from UART to SPI mode.
 */
void spi_a0_reinit(void) {

    while (SPI_A0_ACTIVE);
#ifdef MUX_SEL
    PIN_CLEAR(MUX_SEL);
#endif
    UCA0CTL1 |= UCSWRST; 
    UCA0CTL0 &= ~(UCCKPH + UCCKPL + UC7BIT + UCMODE_3);
    UCA0BRW   = spi_sck_div_a0;
    UCA0CTL0 |= (UCMSB + UCMST + UCSYNC + (SPI_A0_CPOL ? UCCKPL : 0) + (SPI_A0_CPHA ? 0 : UCCKPH));
}

