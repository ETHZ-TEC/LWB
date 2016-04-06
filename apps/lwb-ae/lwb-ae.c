/*
 * Copyright (c) 2016, Swiss Federal Institute of Technology (ETH Zurich).
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
 *          Marco Zimmerling
 */
 
/**
 * @brief Low-Power Wireless Bus Test Application
 * 
 * The AE scheduler is used in this example. It is designed for scenarios
 * where most of the time no data is transmitted, but occasionally (upon an
 * event) a node needs to send several data packets to the host (such as in
 * event driven acoustic emission sensing).
 */

#include "contiki.h"
#include "platform.h"
//#include "bolt_min.h"

#define PAYLOAD_LEN             16

/* glossy header + length byte must be added to payload length to get max.
 * pkt length */
#if (PAYLOAD_LEN + 5) > LWB_CONF_MAX_DATA_PKT_LEN
#error "invalid payload length"
#endif

/*---------------------------------------------------------------------------*/
#ifdef APP_TASK_ACT_PIN
#define TASK_ACTIVE             PIN_SET(APP_TASK_ACT_PIN)
#define TASK_SUSPENDED          PIN_CLR(APP_TASK_ACT_PIN)
#else
#define TASK_ACTIVE
#define TASK_SUSPENDED
#endif /* APP_TASK_ACT_PIN */
#ifdef FLOCKLAB
#warning "----------------- COMPILED FOR FLOCKLAB -----------------"
#endif 
/*---------------------------------------------------------------------------*/
uint16_t slot_node_id = 0;       /* global variable, used for debugging only */
/*---------------------------------------------------------------------------*/
static uint16_t pkt_buffer[(LWB_CONF_MAX_DATA_PKT_LEN + 1) / 2];
static uint16_t pkt_cnt = 0;
/*---------------------------------------------------------------------------*/
#if BOLT_CONF_ON
void
read_message(void)
{
  static uint8_t msg[128];
  uint8_t msg_len = 0;
  if(BOLT_DATA_AVAILABLE) {
    BOLT_READ(msg, msg_len);
    //DEBUG_PRINT_INFO("message received from BOLT (%db)", msg_len);
    lwb_put_data((uint8_t*)pkt_buffer, PAYLOAD_LEN);
    lwb_put_data((uint8_t*)pkt_buffer, PAYLOAD_LEN);
    pkt_cnt += 2;
    //DEBUG_PRINT_INFO("sent=%u", pkt_cnt);
  }
}
#endif /* BOLT_CONF_ON */
/*---------------------------------------------------------------------------*/
PROCESS(app_process, "Application Task");
AUTOSTART_PROCESSES(&app_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data) 
{  
  PROCESS_BEGIN();
          
  SVS_DISABLE;
  /* all other necessary initialization is done in contiki-cc430-main.c */
    
  /* INIT code */
/*#if !defined(FLOCKLAB) && HOST_ID != NODE_ID && defined(DEBUG_SWITCH)
  PIN_CFG_INT(DEBUG_SWITCH);
  PIN_PULLUP_EN(DEBUG_SWITCH); 
#endif*/
  
  /* start the LWB thread */
#if defined(FLOCKLAB) || !BOLT_CONF_ON
  lwb_start(0, &app_process);
#else  /* FLOCKLAB */
  lwb_start(read_message, &app_process);
#endif /* FLOCKLAB */
  
#if LWB_VERSION == 1
  
  static lwb_stream_req_t my_stream;
  my_stream.node_id = node_id;
  my_stream.stream_id = 1;
  my_stream.ipi = 1;
  *my_stream.extra_data = -1;
  
  /* MAIN LOOP of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the LWB task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    TASK_ACTIVE;      /* application task runs now */
        
    if(HOST_ID == node_id) {
      /* HOST node */
      /* print out the received data */
      uint16_t cnt = 0;
      while(1) {
        uint8_t pkt_len = lwb_get_data((uint8_t*)pkt_buffer, 0, 0);
        if(pkt_len) {
          cnt++;
        } else {
          break;
        }
      }
      if(cnt) {
        pkt_cnt += cnt;
        DEBUG_PRINT_INFO("rcvd=%u", pkt_cnt);
      }      
      static uint32_t prev_time = 0;
      static uint8_t exp_pkt = 0;
      if(exp_pkt && (lwb_get_time(0) - prev_time) > 1) {
        /* it's the base period -> reset all tracing pins */
        PIN_CLR(FLOCKLAB_LED1);
        PIN_CLR(FLOCKLAB_INT1);
        PIN_CLR(FLOCKLAB_INT2);
        exp_pkt = 0;
      } else if((lwb_get_time(0) - prev_time) == 1) {
        exp_pkt = 1;
      }
      prev_time = lwb_get_time(0);
      
    } else {
      /* generate an event every 5th round */
      static uint16_t round_cnt = 0;
      if(((round_cnt % 5) == 0) && (node_id == 6)) {
        /* stream with ID 1 active? */
        if(lwb_stream_get_state(1) == LWB_STREAM_STATE_INACTIVE) {
          /* request a new stream with ID 1 and IPI 1 */
          if(!lwb_request_stream(&my_stream, 0)) {
            DEBUG_PRINT_ERROR("stream request failed");
          }
          /* generate random data */
          uint8_t i;
          for(i = 0; i < PAYLOAD_LEN; i++) {
            pkt_buffer[i] = (uint8_t)random_rand();  
          }
          /* generate an event (2 packets) */
          lwb_put_data(0, 1, (uint8_t*)pkt_buffer, PAYLOAD_LEN);
          lwb_put_data(0, 1, (uint8_t*)pkt_buffer, PAYLOAD_LEN);
          pkt_cnt += 2;
          DEBUG_PRINT_INFO("sent=%u", pkt_cnt);
        } else {
          DEBUG_PRINT_WARNING("stream still active");
        }
      }
      if(lwb_stream_get_state(1) == LWB_STREAM_STATE_ACTIVE
         && !lwb_get_send_buffer_state()) {
        /* remove the stream (silently drop!) */
        lwb_stream_drop(1);
      } 
      round_cnt++;
    }
    TASK_SUSPENDED;
  }
     
#else /* LWB_VERSION */
  
  if(node_id == 6 || node_id == 28 || node_id == 22) {
    /* generate a dummy packet to 'register' this node at the host */
    lwb_put_data((uint8_t*)&node_id, 2);
    pkt_cnt++;
  }
  
  /* MAIN LOOP of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the LWB task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    TASK_ACTIVE;      /* application task runs now */
        
    if(HOST_ID == node_id) {
      /* HOST node */
      /* print out the received data */
      uint16_t cnt = 0;
      while(1) {
        uint8_t pkt_len = lwb_get_data((uint8_t*)pkt_buffer);
        if(pkt_len) {
          cnt++;
        } else {
          break;
        }
      }
      if(cnt) {
        pkt_cnt += cnt;
        DEBUG_PRINT_INFO("rcvd=%u", pkt_cnt);
      }
      /* make sure the debug pins are in 'idle' state */
      PIN_CLR(FLOCKLAB_LED1);
      PIN_CLR(FLOCKLAB_INT1);
      PIN_CLR(FLOCKLAB_INT2);
      
    } else {
      static uint16_t acks_rcvd = 0;
      /* SOURCE node */
      if(lwb_get_data((uint8_t*)pkt_buffer) && *pkt_buffer == node_id) {
        acks_rcvd++;
        DEBUG_PRINT_INFO("ack=%u", acks_rcvd);
      } 
      //memset(pkt_buffer, 0xf0, PAYLOAD_LEN);
      /* random data */
      uint8_t i;
      for(i = 0; i < PAYLOAD_LEN; i++) {
        pkt_buffer[i] = (uint8_t)random_rand();  
      }
#ifdef FLOCKLAB
      /* initiator nodes start to send data after a certain time */
      if((node_id == 6 || node_id == 28 || node_id == 22) && 
         lwb_get_time(0) > (LWB_CONF_SCHED_PERIOD_IDLE * 4)) {
        /* generate an event */
        lwb_put_data((uint8_t*)pkt_buffer, PAYLOAD_LEN);
        lwb_put_data((uint8_t*)pkt_buffer, PAYLOAD_LEN);
        pkt_cnt += 2;
        DEBUG_PRINT_INFO("sent=%u", pkt_cnt);
      }
#else /* FLOCKLAB */
      //if(lwb_get_time(0) > 20 && lwb_get_time(0) % 15 == 0) {  
      /*
      static uint8_t msg[128];
      uint8_t msg_len = 0;
      if(BOLT_DATA_AVAILABLE) {
        BOLT_READ(msg, msg_len);
        DEBUG_PRINT_INFO("message received from BOLT (%db)", msg_len);
        lwb_put_data((uint8_t*)pkt_buffer, PAYLOAD_LEN);
        lwb_put_data((uint8_t*)pkt_buffer, PAYLOAD_LEN);
        pkt_cnt += 2;
        DEBUG_PRINT_INFO("sent=%u", pkt_cnt);
      }*/
#endif /* FLOCKLAB */
    }
    
    /* IMPORTANT: This process must not run for more than a few hundred
     * milliseconds in order to enable proper operation of the LWB */
    
    /* since this process is only executed at the end of an LWB round, we 
     * can now configure the MCU for minimal power dissipation for the idle
     * period until the next round starts */
  #if FRAM_CONF_ON
    fram_sleep();
  #endif /* FRAM_CONF_ON */
    /* disable all peripherals, reconfigure the GPIOs and disable XT2 */
    TA0CTL &= ~MC_3; /* stop TA0 */
    DISABLE_XT2();
  #ifdef MUX_SEL_PIN
    PIN_CLR(MUX_SEL_PIN);
  #endif /* MUX_SEL_PIN */
    P1SEL = 0; /* reconfigure GPIOs */
    P1DIR = 0xff;
    /* dont clear BIT6 and 7 on olimex board */
    //P1OUT &= ~(BIT2 | BIT3 | BIT4 | BIT5);
    P1OUT = 0;
    /* note: DPP has a pullup on P1.5! */
    P1OUT |= BIT5;
    /* set clock source to DCO */
    UCSCTL4 = SELA__XT1CLK | SELS__DCOCLKDIV | SELM__DCOCLKDIV;
    
    TASK_SUSPENDED;
  }
#endif /* LWB_VERSION */

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/*
ISR(PORT1, port1_interrupt) 
{
  if(PIN_IFG(DEBUG_SWITCH)) {
    if(!lwb_put_data((uint8_t*)&node_id, 2)) {
      DEBUG_PRINT_WARNING("can't queue data");
    }
    pkt_cnt++;
    DEBUG_PRINT_INFO("event triggered, sent=%u", pkt_cnt);
    PIN_CLR_IFG(DEBUG_SWITCH);
  } 
}
*/
/*---------------------------------------------------------------------------*/