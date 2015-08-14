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
//#include "platform.h"

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
#define BOLT_ACK_STATUS                 PIN_GET(BOLT_CONF_ACK_PIN)
#define BOLT_WAIT_TILL_COMPLETED        while(BOLT_STATE_IDLE != bolt_state)
/*---------------------------------------------------------------------------*/
static volatile bolt_state_t bolt_state = BOLT_STATE_IDLE;
static bolt_callback_t bolt_ind_callback = 0;
#if BOLT_CONF_TIMEREQ_ENABLE
static rtimer_clock_t ta1_timestamp = 0;
#endif /* BOLT_CONF_TIMEREQ_ENABLE */
/*---------------------------------------------------------------------------*/
void
bolt_init(bolt_callback_t IND_line_callback)
{
  /* control signals */
  PIN_CFG_IN(BOLT_CONF_IND_PIN);
  if(IND_line_callback) {
    PIN_CFG_INT(BOLT_CONF_IND_PIN);
    bolt_ind_callback = IND_line_callback;
  }
  /* enable resistor to prevent floating input */
  PIN_PULLDOWN_EN(BOLT_CONF_IND_PIN);
  PIN_UNSEL(BOLT_CONF_MODE_PIN);
  PIN_CLR(BOLT_CONF_MODE_PIN);
  PIN_CFG_OUT(BOLT_CONF_MODE_PIN);
  PIN_UNSEL(BOLT_CONF_REQ_PIN);
  PIN_CLR(BOLT_CONF_REQ_PIN);
  PIN_CFG_OUT(BOLT_CONF_REQ_PIN);
  PIN_CFG_IN(BOLT_CONF_ACK_PIN);
  PIN_PULLDOWN_EN(BOLT_CONF_ACK_PIN);
#ifdef BOLT_CONF_IND_OUT_PIN
  PIN_CFG_IN(BOLT_CONF_IND_OUT_PIN);
  PIN_PULLDOWN_EN(BOLT_CONF_IND_OUT_PIN);
#endif /* BOLT_CONF_IND_OUT_PIN */
#if BOLT_CONF_USE_DMA
  PIN_CFG_PORT_INT(BOLT_CONF_ACK_PIN);
  PIN_IES_FALLING(BOLT_CONF_ACK_PIN);
#endif /* BOLT_CONF_USE_DMA */

  /* SPI */
  if(BOLT_CONF_SPI == SPI_B0_BASE) {
    spi_b0_init(BOLT_CONF_SCLK_SPEED);
  } else {
    spi_a0_init(BOLT_CONF_SCLK_SPEED);
  }

#if BOLT_CONF_TIMEREQ_ENABLE
  PIN_SEL(BOLT_CONF_TIMEREQ_PIN);
  PIN_CFG_IN(BOLT_CONF_TIMEREQ_PIN);
  PIN_RES_EN(BOLT_CONF_TIMEREQ_PIN);
  
  /* configure TA1 CCR0 to capture the timestamp on an edge change on pin 2.1
     (do NOT enable interrupts!) */
  rtimer_wait_for_event(BOLT_CONF_TIMEREQ_TIMERID, 0);              
  /* use the DMA to take a snapshot of the 64-bit sw timer extension */
  dma_config_timer(DMA_TRCSRC_TA1CCR0, (uint16_t)&ta0_sw_ext, 
                   (uint16_t)&ta1_timestamp, 8);     
#endif
  
  bolt_state = BOLT_STATE_IDLE;
  
  DEBUG_PRINT_INFO("BOLT initialized");
}
/*---------------------------------------------------------------------------*/
#if BOLT_CONF_TIMEREQ_ENABLE
void
bolt_set_timereq_callback(void (*func)(void))
{
  if (!func) {
    /* remove the callback = switch to polling mode (utilize the DMA) */
    rtimer_wait_for_event(BOLT_CONF_TIMEREQ_TIMERID, 0);
    /* use the DMA to take a snapshot of the 64-bit sw timer extension */
    dma_config_timer(DMA_TRCSRC_TA1CCR0, (uint16_t)&ta0_sw_ext, 
                     (uint16_t)&ta1_timestamp, 8);
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
    DEBUG_PRINT_VERBOSE("timestamp: %llu (now: %llu)", ta1_timestamp,
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
  /*-> this is done in the DMA lib */
  /* --- 2. wait for BUSY flag --- */
  SPI_WAIT_BUSY(BOLT_CONF_SPI);
  /* --- 3. set REQ = 0 --- */
  PIN_CLR(BOLT_CONF_REQ_PIN);
  /* --- 4. empty the RX buffer --- */
  SPI_CLR_RXBUF(BOLT_CONF_SPI);
  SPI_DISABLE(BOLT_CONF_SPI);      /* disable SPI (optional) */

  /* --- 5. wait for ACK to go down --- */
  while(PIN_GET(BOLT_CONF_ACK_PIN));
  bolt_state = BOLT_STATE_IDLE;
  DEBUG_PRINT_VERBOSE("back in idle state");
}
/*---------------------------------------------------------------------------*/
uint8_t
bolt_acquire(bolt_op_mode_t mode)
{  
  if(PIN_GET(BOLT_CONF_REQ_PIN) || 
     PIN_GET(BOLT_CONF_ACK_PIN)) {
    DEBUG_PRINT_ERROR("request failed (REQ or ACK still high)");
    return 0;
  }
  if(BOLT_STATE_IDLE != bolt_state) {
    DEBUG_PRINT_ERROR("not in idle state, operation skipped");
    return 0;
  } 
  
  /* make sure SPI is enabled */
  SPI_ENABLE(BOLT_CONF_SPI);

  /* --- MODE --- */
  /* READ */
  if(BOLT_OP_READ == mode) {
    if(!BOLT_DATA_AVAILABLE) {
      DEBUG_PRINT_WARNING("no data available, read operation skipped");
      return 0;
    }
    PIN_CLR(BOLT_CONF_MODE_PIN); /* 0 = READ */
    DEBUG_PRINT_VERBOSE("requesting read operation...");
  /* WRITE */
  } else {
    PIN_SET(BOLT_CONF_MODE_PIN); /* 1 = WRITE */
    DEBUG_PRINT_VERBOSE("requesting write operation...");
  }
  /* --- set REQ = 1 --- */
  PIN_SET(BOLT_CONF_REQ_PIN);
  /* now wait for a rising edge on the ACK line */
  uint8_t cnt = 0;
  do {
      __delay_cycles(MCLK_SPEED / 100000);       /* wait 10 us */
      cnt++;
  } while (!BOLT_ACK_STATUS && cnt < 10);
  
  if (!BOLT_ACK_STATUS) {
    /* ack is still low -> failed */
    bolt_state = BOLT_STATE_IDLE;
    PIN_CLR(BOLT_CONF_REQ_PIN);
    DEBUG_PRINT_ERROR("access denied");
    return 0;
  }
  bolt_state = (mode == BOLT_OP_READ) ? BOLT_STATE_READ : BOLT_STATE_WRITE;

  return 1;
}
/*---------------------------------------------------------------------------*/
uint8_t
bolt_start(uint8_t *data, uint16_t num_bytes)
{
#if !(BOLT_CONF_USE_DMA)
  uint16_t count = 0;
#endif /* BOLT_CONF_USE_DMA */
  DEBUG_PRINT_VERBOSE("starting data transfer... ");
  
  if(!data) {
      return 0;
  }

  /* WRITE OPERATION */
  if(BOLT_STATE_WRITE == bolt_state) {
    if(0 == num_bytes) {
      return 0;
    }
#if BOLT_CONF_USE_DMA
    dma_config_spi(BOLT_CONF_SPI, bolt_release);
    dma_start(0, (uint16_t)data, num_bytes);
#else
    while(count < num_bytes) {
      SPI_TRANSMIT_BYTE(BOLT_CONF_SPI, *data);
      data++;
      count++;
      if(!BOLT_ACK_STATUS) {  /* aborted */
        DEBUG_PRINT_ERROR("transfer aborted by BOLT");
        return 0;
      }
    }
    DEBUG_PRINT_VERBOSE("%d bytes transmitted", count);
#endif /* BOLT_CONF_USE_DMA */
    return 1;
  /* READ OPERATION */
  } else if(BOLT_STATE_READ == bolt_state) {
#if BOLT_CONF_USE_DMA
    dma_config_spi(BOLT_CONF_SPI, bolt_release);
    dma_start((uint16_t)data, 0, BOLT_MAX_MSG_LEN);
#else
    /* first, clear the RX buffer */
    SPI_CLR_RXBUF(BOLT_CONF_SPI);
#if SPI_CONF_FAST_READ
    /* transmit 1 byte ahead for faster read speed (fills RXBUF faster) */
    SPI_TRANSMIT_BYTE(BOLT_CONF_SPI, 0x00);
#endif
    while((count < BOLT_CONF_MAX_MSG_LEN) && BOLT_ACK_STATUS) {
      SPI_TRANSMIT_BYTE(BOLT_CONF_SPI, 0x00);          /* generate the clock */
      SPI_RECEIVE_BYTE(BOLT_CONF_SPI, *data);
      data++;
      count++;
    }
    /* how many bytes received? */
    DEBUG_PRINT_VERBOSE("%d bytes received", count);
#endif /* BOLT_CONF_USE_DMA */
    return count;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
void 
bolt_handle_irq(void) 
{
  if(PIN_IFG(BOLT_CONF_IND_PIN)) {
    PIN_IES_TOGGLE(BOLT_CONF_IND_PIN);
    PIN_CLR_IFG(BOLT_CONF_IND_PIN);
    DEBUG_PRINT_VERBOSE("BOLT IND pin interrupt");
    if(bolt_ind_callback) {
      bolt_ind_callback(); 
    }
  } else if(PIN_IFG(BOLT_CONF_ACK_PIN)) {
    PIN_CLR_IFG(BOLT_CONF_ACK_PIN);
#if BOLT_CONF_USE_DMA
    uint16_t rcvd_bytes = DMA_REMAINING_BYTES_RX;
    if(BOLT_STATE_READ == bolt_state) {
      if(rcvd_bytes == (BOLT_CONF_MAX_MSG_LEN - 1)) {
        rcvd_bytes = BOLT_CONF_MAX_MSG_LEN;
      } else {
        rcvd_bytes = (BOLT_CONF_MAX_MSG_LEN - rcvd_bytes);
      }
      bolt_release();
      DEBUG_PRINT_VERBOSE("%d bytes received", rcvd_bytes);
    }
#endif /* BOLT_CONF_USE_DMA */
    DEBUG_PRINT_VERBOSE("port 2 interrupt: ACK pin");
  }    
}
/*---------------------------------------------------------------------------*/

#endif /* BOLT_CONF_ON */
