/*
 * Copyright (c) 2015, Swiss Federal Institute of Technology (ETH Zurich).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author:  Reto Da Forno
 *          Federico Ferrari
 */

/**
 * @addtogroup  Platform
 * @{
 *
 * @defgroup    gpio GPIO
 * @{
 *
 * @file
 *
 * @brief provides configuration and access functions for the GPIO/port pins
 */

#ifndef __GPIO_H__
#define __GPIO_H__

#define PIN0    0
#define PIN1    1
#define PIN2    2
#define PIN3    3
#define PIN4    4
#define PIN5    5
#define PIN6    6
#define PIN7    7

#define PORT1   1
#define PORT2   2
#define PORT3   3
#define PORT4   4
#define PORT5   5

#define PIN_TO_BIT(pin)                 (1 << pin)

/**
 * @brief access a 16-bit register value (read or write)
 */
#define REGVAL16(x)                     (*((volatile uint16_t *)((uint16_t)x)))
/**
 * @brief access an 8-bit register value (read or write)
 */
#define REGVAL8(x)                      (*((volatile uint8_t *)((uint16_t)x)))

/* note: all following macros ending with '_I' (immediate) can only be used
 * when passing numbers directly (no defines or variables) */
#define PIN_XOR_I(port, pin)            P##port##OUT ^= BIT##pin
#define PIN_SET_I(port, pin)            P##port##OUT |= BIT##pin
#define PIN_CLR_I(port, pin)            P##port##OUT &= ~BIT##pin
#define PIN_SEL_I(port, pin)            P##port##SEL |= BIT##pin
#define PIN_UNSEL_I(port, pin)          P##port##SEL &= ~BIT##pin
#define PIN_CFG_OUT_I(port, pin)        P##port##DIR |= BIT##pin
#define PIN_CFG_IN_I(port, pin)         P##port##DIR &= ~BIT##pin
#define PIN_MAP_I(port, pin, map)       P##port##MAP##pin = map
#define PIN_CLR_IFG_I(port, pin)        P##port##IFG &= ~BIT##pin
/* only has an effect if the pin is configured as input */
#define PIN_RES_EN_I(port, pin)         P##port##REN |= BIT##pin
#define PIN_IES_RISING_I(port, pin)     P##port##IES &= ~BIT##pin
#define PIN_IES_FALLING_I(port, pin)    P##port##IES |= BIT##pin
#define PIN_IES_TOGGLE_I(port, pin)     P##port##IES ^= BIT##pin
#define PIN_INT_EN_I(port, pin)         P##port##IE |= BIT##pin
#define PIN_CFG_INT_I(port, pin)        { \
    PIN_CFG_IN_I(port, pin); \
    PIN_IES_RISING_I(port, pin); \
    PIN_CLR_IFG_I(port, pin); \
    PIN_INT_EN_I(port, pin); \
}
#define PIN_IFG_I(port, pin)            (P##port##IFG & (uint8_t)(BIT##pin))
#define PIN_GET_I(port, pin)            (P##port##IN & (uint8_t)(BIT##pin))
#define PORT_XOR_I(port)                P##port##OUT ^= 0xff
#define PORT_SET_I(port)                P##port##OUT = 0xff
#define PORT_CLR_I(port)                P##port##OUT = 0x00
#define PORT_SEL_I(port)                P##port##SEL = 0xff
#define PORT_UNSEL_I(port)              P##port##SEL = 0x00
#define PORT_RES_EN_I(port)             P##port##REN = 0xff
#define PIN_PULLUP_EN_I(port, pin)      { PIN_RES_EN_I(port, pin); \
                                          PIN_SET_I(port, pin); }
#define PIN_PULLDOWN_EN_I(port, pin)    { PIN_RES_EN_I(port, pin); \
                                          PIN_CLR_I(port, pin); }
#define PORT_CLR_IFG_I(port)            P##port##IFG = 0x00
#define PORT_CFG_OUT_I(port)            P##port##DIR = 0xff
#define PORT_CFG_IN_I(port)             P##port##DIR = 0x00

/* do NOT call this macro from within an ISR! */
#define PIN_MAP_AS_OUTPUT_I(port, pin, map)  { \
    __dint(); __nop(); /* disable interrupts */ \
    /* get write-access to the port mapping control registers (see 9.2.1) */ \
    PMAPKEYID = 0x02D52; \
    /* allow reconfiguration of port mapping */ \
    PMAPCTL |= PMAPRECFG; \
    PIN_SEL_I(port, pin); \
    PIN_CFG_OUT_I(port, pin); \
    PIN_MAP_I(port, pin, map); \
    /* lock write-access to the port mapping control registers (see 9.2.1) */ \
    PMAPKEYID = 0; \
    __eint(); __nop(); /* re-enable interrupts */ \
}
/* do NOT call this macro from within an ISR! */
#define PIN_MAP_AS_INPUT_I(port, pin, map)   { \
    __dint(); __nop(); /* disable interrupts */ \
    /* get write-access to the port mapping control registers (see 9.2.1) */ \
    PMAPKEYID = 0x02D52; \
    /* allow reconfiguration of port mapping */ \
    PMAPCTL |= PMAPRECFG; \
    PIN_SEL_I(port, pin); \
    PIN_CFG_IN_I(port, pin); \
    PIN_MAP_I(port, pin, map); \
    /* lock write-access to the port mapping control registers (see 9.2.1) */ \
    PMAPKEYID = 0; \
    __eint(); __nop(); /* re-enable interrupts */ \
}

/**
 * @brief toggle a (port, pin) output, i.e. set it high when it was low and
 * vice-versa
 */
#define PIN_XOR(p)                      PIN_XOR_I(p)
/**
 * @brief set a (port, pin) output, i.e. push it's output signal high
 */
#define PIN_SET(p)                      PIN_SET_I(p)
/**
 * @brief clear a (port, pin) output, i.e. pull it's output signal low
 */
#define PIN_CLR(p)                      PIN_CLR_I(p)
/**
 * @brief select a (port, pin), i.e. configure it in module function mode
 */
#define PIN_SEL(p)                      PIN_SEL_I(p)
/**
 * @brief unselect a (port, pin), i.e. configure it in port mode (GPIO)
 */
#define PIN_UNSEL(p)                    PIN_UNSEL_I(p)
/**
 * @brief set a (port, pin) as output
 * @note don't forget to unselect a pin if you want to use it in GPIO mode
 */
#define PIN_CFG_OUT(p)                  PIN_CFG_OUT_I(p)
/**
 * @brief set a (port, pin) as input
 */
#define PIN_CFG_IN(p)                   PIN_CFG_IN_I(p)
/**
 * @brief map a (port, pin) as output
 * do NOT call this macro from within an ISR
 */
#define PIN_MAP_AS_OUTPUT(p, map)       PIN_MAP_AS_OUTPUT_I(p, map)
/**
 * @brief map a (port, pin) as input
 * do NOT call this macro from within an ISR
 */
#define PIN_MAP_AS_INPUT(p, map)        PIN_MAP_AS_INPUT_I(p, map)
/**
 * @brief clear the interrupt flag of the specified (port, pin)
 */
#define PIN_CLR_IFG(p)                  PIN_CLR_IFG_I(p)
/**
 * @brief enable the resistor for the specified (port, pin), only if configured
 * as input!
 */
#define PIN_PULLUP_EN(p)                PIN_PULLUP_EN_I(p) 
#define PIN_PULLDOWN_EN(p)              PIN_PULLDOWN_EN_I(p)
/**
 * @brief select the rising edge as interrupt edge for the specified (port,
 * pin)
 */
#define PIN_IES_RISING(p)               PIN_IES_RISING_I(p)
/**
 * @brief select the falling edge as interrupt edge for the specified (port,
 * pin)
 */
#define PIN_IES_FALLING(p)              PIN_IES_FALLING_I(p)
/**
 * @brief toggle the interrupt edge for the specified (port, pin)
 */
#define PIN_IES_TOGGLE(p)               PIN_IES_TOGGLE_I(p)
/**
 * @brief enable the port interrupt for the specified (port, pin)
 */
#define PIN_INT_EN(p)                   PIN_INT_EN_I(p)
/**
 * @brief configure the port interrupt for the specified (port, pin), available
 * for pins on port 1 and 2 only
 */
#define PIN_CFG_INT(p)                  PIN_CFG_INT_I(p)
/**
 * @brief check the port interrupt flag of the specified (port, pin), available
 * for pins on port 1 and 2 only
 */
#define PIN_IFG(portandpin)             PIN_IFG_I(portandpin)
/**
 * @brief read the 1-bit signal level at the specified (port, pin), either 0 or
 * 1
 */
#define PIN_GET(portandpin)             PIN_GET_I(portandpin)


#endif /* __GPIO_H__ */

/**
 * @}
 * @}
 */