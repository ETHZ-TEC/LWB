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
 */

#include "contiki.h"
#include "platform.h"

#if BOLT_CONF_ON
/*---------------------------------------------------------------------------*/
/**
 * @brief all the possible states of the finite state machine that controls the
 *interaction with the asynchronous interface
 */
typedef enum {
  BOLT_STATE_IDLE = 0,
  BOLT_STATE_READ,
  BOLT_STATE_WRITE,
  NUM_OF_STATES
} bolt_state_t;
/*---------------------------------------------------------------------------*/
/* helper macros */
#define BOLT_IS_TIMEREQ_ENABLED         ((TA1CCTL0 & CCIE) > 0)
#define BOLT_TIMEREQ_ENABLE             (TA1CCTL0 |= CCIE)
#define BOLT_TIMEREQ_DISABLE            (TA1CCTL0 &= ~CCIE)
#define BOLT_ACK_STATUS                 PIN_GET_INPUT_BIT(BOLT_PIN_ACK)
#define BOLT_WAIT_TILL_COMPLETED        while(BOLT_STATE_IDLE != bolt_state)
/*---------------------------------------------------------------------------*/
volatile bolt_state_t bolt_state = BOLT_STATE_IDLE;
#if BOLT_CONF_TIMEREQ_ENABLE
static rtimer_clock_t ta1_timestamp = 0;
#endif /* BOLT_CONF_TIMEREQ_ENABLE */
#if BOLT_CONF_USE_DMA
static uint16_t bolt_rx_buffer = 0;
static uint16_t bolt_tx_buffer = 0;
#endif /* BOLT_CONF_TIMEREQ_ENABLE */
/*---------------------------------------------------------------------------*/
void
bolt_init(void)
{
  /* control signals */
  PIN_SET_AS_INPUT(BOLT_PIN_IND);
  PIN_RESISTOR_EN(BOLT_PIN_IND);         /* enable resistor to prevent floating
                                           input */
  /* don't enable the port interrupt for this pin */
  /*PIN_CFG_PORT_INT(BOLT_PIN_IND);*/
  PIN_UNSELECT(BOLT_PIN_MODE);
  PIN_CLEAR(BOLT_PIN_MODE);
  PIN_SET_AS_OUTPUT(BOLT_PIN_MODE);
  PIN_UNSELECT(BOLT_PIN_REQ);
  PIN_CLEAR(BOLT_PIN_REQ);
  PIN_SET_AS_OUTPUT(BOLT_PIN_REQ);
  PIN_SET_AS_INPUT(BOLT_PIN_ACK);
  PIN_RESISTOR_EN(BOLT_PIN_ACK);
  /* don't enable interrupts for this pin, use busy wait (polling) instead! */
  /*PIN_CFG_PORT_INT(BOLT_PIN_ACK);*/

  /* SPI */
  if(BOLT_CONF_SPI == SPI_B0_BASE) {
    spi_b0_init(BOLT_CONF_SCLK_SPEED);
  } else {
    spi_a0_init(BOLT_CONF_SCLK_SPEED);
  }

#if BOLT_CONF_TIMEREQ_ENABLE
  PIN_SET_AS_MODULE_FUNC(BOLT_PIN_TIMEREQ);
  PIN_SET_AS_INPUT(BOLT_PIN_TIMEREQ);
  PIN_RESISTOR_EN(BOLT_PIN_TIMEREQ);
  
  /* configure TA1 CCR0 to capture the timestamp on an edge change on pin 2.1
     (do NOT enable interrupts!) */
  rtimer_wait_for_event(BOLT_CONF_TIMEREQ_TIMERID, 0);              
  /* use the DMA to take a snapshot of the 64-bit sw timer extension */
  dma_init_timer((uint16_t)&ta0_sw_ext, (uint16_t)&ta1_timestamp);     
#endif
}
/*---------------------------------------------------------------------------*/
#if BOLT_CONF_USE_DMA
void
bolt_set_dma_buffers(uint16_t rx_buffer_addr, uint16_t tx_buffer_addr)
{
  if(0 == rx_buffer_addr || 0 == tx_buffer_addr) {
    DEBUG_PRINT_WARNING("invalid parameters for bolt_init");
  }
  bolt_rx_buffer = rx_buffer_addr;
  bolt_tx_buffer = tx_buffer_addr;
  dma_init_spi(BOLT_CONF_SPI, rx_buffer_addr, tx_buffer_addr, bolt_release);
}
#endif
/*---------------------------------------------------------------------------*/
#if BOLT_CONF_TIMEREQ_ENABLE
void
bolt_set_timereq_callback(void (*func)(void))
{
  if (!func) {
    /* remove the callback = switch to polling mode (utilize the DMA) */
    rtimer_wait_for_event(BOLT_CONF_TIMEREQ_TIMERID, 0);
    /* use the DMA to take a snapshot of the 64-bit sw timer extension */
    dma_init_timer((uint16_t)&ta0_sw_ext, (uint16_t)&ta1_timestamp);        
  } else {      
    /* set the rtimer callback function */
    rtimer_wait_for_event(BOLT_CONF_TIMEREQ_TIMERID, (rtimer_callback_t)func);
  }
}
/*---------------------------------------------------------------------------*/
uint8_t
bolt_handle_timereq(uint8_t *out_buffer)
{
  if(DMA2CTL & DMAIFG || TA1CCTL0 & CCIFG) {        /* interrupt flag set? */
    ta1_timestamp = (ta1_timestamp << 16) | TA1CCR0;
    DEBUG_PRINT_INFO("timestamp: %llu (now: %llu)", ta1_timestamp,
                     rtimer_now());
    TA1CCTL0 &= ~(CCIFG + COV);
    DMA2CTL &= ~DMAIFG;
    DMA2CTL |= DMAEN;           /* re-enable the DMA */
    /* send the timestamp to the application processor */
    if(out_buffer) {
      memcpy(out_buffer, &ta1_timestamp, 8);
      ta1_timestamp = 0;
    }
    return 1;
  }
  return 0;
}
#endif /* BOLT_CONF_TIMEREQ_ENABLE */
/*---------------------------------------------------------------------------*/
void
bolt_release(void)
{
  /* --- 1. stop DMA --- */
#ifdef BOLT_USE_DMA
  DMA_DISABLE_TX;
  DMA_DISABLE_RX;
#endif
  /* --- 2. wait for BUSY flag --- */
  SPI_WAIT_BUSY(BOLT_CONF_SPI);
  /* --- 3. set REQ = 0 --- */
  PIN_CLEAR(BOLT_PIN_REQ);
  /* --- 4. empty the RX buffer --- */
  SPI_CLEAR_RXBUF(BOLT_CONF_SPI);
  SPI_DISABLE(BOLT_CONF_SPI);      /* disable SPI (optional) */

#ifdef BOLT_USE_DMA
  if(BOLT_STATE_READ == bolt_state && 0 != bolt_rx_buffer) {
    *(uint8_t *)(bolt_rx_buffer + BOLT_CONF_MAX_MSG_LEN - 1) = 0;
    DEBUG_PRINT_INFO("message received: '%s'", (char *)bolt_rx_buffer);
  }
#endif /* BOLT_USE_DMA */

  /* --- 5. wait for ACK to go down --- */
  while(PIN_GET_INPUT_BIT(BOLT_PIN_ACK));
  bolt_state = BOLT_STATE_IDLE;
  DEBUG_PRINT_VERBOSE("back in idle state");
}
/*---------------------------------------------------------------------------*/
uint8_t
bolt_acquire(bolt_op_mode_t mode)
{
  if(PIN_GET_INPUT_BIT(BOLT_PIN_REQ) || PIN_GET_INPUT_BIT(BOLT_PIN_ACK)) {
    DEBUG_PRINT_ERROR("request failed (REQ or ACK still high)");
    return 0;
  }
  if(BOLT_STATE_IDLE != bolt_state) {
    DEBUG_PRINT_ERROR("not in idle state, operation skipped");
    return 0;
  }
#if BOLT_CONF_USE_DMA
  if (bolt_tx_buffer == 0 || bolt_rx_buffer == 0) {        
    DEBUG_PRINT_ERROR("set the DMA RX and TX buffer first!");
    return 0;
  }  
#endif /* BOLT_CONF_USE_DMA */
  
  /* make sure SPI is enabled */
  SPI_ENABLE(BOLT_CONF_SPI);

  /* --- MODE --- */
  /* READ */
  if(BOLT_OP_READ == mode) {
    if(!BOLT_DATA_AVAILABLE) {
      DEBUG_PRINT_WARNING("no data available, read operation skipped");
      return 0;
    }
    PIN_CLEAR(BOLT_PIN_MODE); /* 0 = READ */
    DEBUG_PRINT_VERBOSE("requesting read operation...");
  /* WRITE */
  } else {
    PIN_SET(BOLT_PIN_MODE); /* 1 = WRITE */
    DEBUG_PRINT_VERBOSE("requesting write operation...");
  }
  /* --- set REQ = 1 --- */
  PIN_SET(BOLT_PIN_REQ);
  /* now wait for a rising edge on the ACK line */
  uint8_t cnt = 0;
  do {
      __delay_cycles(MCLK_SPEED / 100000);       /* wait 10 us */
      cnt++;
  } while (!BOLT_ACK_STATUS && cnt < 10);
  
  if (!BOLT_ACK_STATUS) {
    /* ack is still low -> failed */
    bolt_state = BOLT_STATE_IDLE;
    PIN_CLEAR(BOLT_PIN_REQ);
    DEBUG_PRINT_ERROR("BOLT access not granted");
    return 0;
  }
  bolt_state = (mode == BOLT_OP_READ) ? BOLT_STATE_READ : BOLT_STATE_WRITE;

  return 1;
}
/*---------------------------------------------------------------------------*/
uint8_t
bolt_start(uint8_t *data, uint16_t *num_bytes)
{
  if(0 == num_bytes) {
    return 0;
  }
  DEBUG_PRINT_VERBOSE("starting data transfer... ");

  /* WRITE OPERATION */
  if(BOLT_STATE_WRITE == bolt_state) {
    if(0 == *num_bytes) {
      return 0;
    }
#if BOLT_CONF_USE_DMA
    DMA_ENABLEINTERRUPT_TX;
    /* DMA_SETTXBUF_ADDR(bolt_tx_buffer); */
    DMA_SETTRANSFERSIZE_TX(num_bytes - 1);
    DMA_ENABLE_TX;
    /* write the frist byte to trigger the DMA (TXE) */
    SPI_WRITE_BYTE(BOLT_CONF_SPI, *(uint8_t *)bolt_tx_buffer);  
#else
    uint16_t to_transmit = *num_bytes;
    *num_bytes = 0;
    while((*num_bytes) < to_transmit) {
      SPI_TRANSMIT_BYTE(BOLT_CONF_SPI, *data);
      data++;
      (*num_bytes)++;
      if(!BOLT_ACK_STATUS) {
        /* aborted */
        DEBUG_PRINT_WARNING("transfer aborted by ADI!");
        return 0;
      }
    }
    DEBUG_PRINT_VERBOSE("message written to ADI (%d bytes)", to_transmit);
#endif /* BOLT_CONF_USE_DMA */
  /* READ OPERATION */
  } else if(BOLT_STATE_READ == bolt_state) {
#if BOLT_CONF_USE_DMA
    /* DMA_SETRXBUF_ADDR(bolt_rx_buffer); */
    /* DMA_SETTRANSFERSIZE_RX(BOLT_CONF_MAX_MSG_LEN); */
    DMA_DISABLEINTERRUPT_TX;
    DMA_ENABLEINTERRUPT_RX;
    DMA_SETTRANSFERSIZE_TX(BOLT_CONF_MAX_MSG_LEN - 1);
    DMA_ENABLE_RX;
    DMA_ENABLE_TX;
    /* write the frist byte to trigger the DMA (TXE) */
    SPI_WRITE_BYTE(BOLT_CONF_SPI, *(uint8_t *)bolt_tx_buffer);    
#else
    *num_bytes = 0;
    /* first, clear the RX buffer */
    SPI_CLEAR_RXBUF(BOLT_CONF_SPI);
#if SPI_CONF_FAST_READ
    /* transmit 1 byte ahead for faster read speed (fills RXBUF faster) */
    SPI_TRANSMIT_BYTE(BOLT_CONF_SPI, 0x00);                
#endif
    while((*num_bytes < BOLT_CONF_MAX_MSG_LEN) && BOLT_ACK_STATUS) {
      SPI_TRANSMIT_BYTE(BOLT_CONF_SPI, 0x00);          /* generate the clock */
      SPI_RECEIVE_BYTE(BOLT_CONF_SPI, *data);
      data++;
      (*num_bytes)++;
    }
    /* how many bytes received? */
    DEBUG_PRINT_VERBOSE("message read from ADI (%d bytes)", *num_bytes);
#endif /* BOLT_CONF_USE_DMA */
  }
  return 1;
}
/*---------------------------------------------------------------------------*/
ISR(PORT2, port2_interrupt)
{
  ENERGEST_ON(ENERGEST_TYPE_CPU);

  if(PIN_IFG(BOLT_PIN_IND)) {
    DEBUG_PRINT_VERBOSE("port 2 interrupt: IND pin");
    PIN_IES_TOGGLE(BOLT_PIN_IND);
    PIN_CLEAR_IFG(BOLT_PIN_IND);
  } else if(PIN_IFG(BOLT_PIN_ACK)) {
    DEBUG_PRINT_VERBOSE("port 2 interrupt: ACK pin");
    PIN_CLEAR_IFG(BOLT_PIN_ACK);
    if(BOLT_STATE_IDLE != bolt_state) {
      PIN_IES_TOGGLE(BOLT_PIN_ACK);
#if BOLT_CONF_USE_DMA
      if(!PIN_GET_INPUT_BIT(BOLT_PIN_ACK)) {
        /* this was the falling edge */
        /* abort or transmission complete! */
        if(BOLT_STATE_READ == bolt_state) {
          uint16_t rcv_bytes;          
          if (DMA_REMAINING_BYTES == (BOLT_CONF_MAX_MSG_LEN - 1)) {
              rcv_bytes = BOLT_CONF_MAX_MSG_LEN;
          } else {
              rcv_bytes = (BOLT_CONF_MAX_MSG_LEN - DMA_REMAINING_BYTES);
          }
          bolt_release();
          DEBUG_PRINT_VERBOSE(
            "Async interface transfer complete (%d bytes received)",
            rcv_bytes);
        }
      }
#endif /* BOLT_CONF_USE_DMA */
    } else {
      DEBUG_PRINT_VERBOSE("async interface not in IDLE state");
    }
  }

  ENERGEST_OFF(ENERGEST_TYPE_CPU);
}
/*---------------------------------------------------------------------------*/

#endif /* BOLT_CONF_ON */
