/*
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich).
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
 *          Tonio Gsell
 */

/**
 * @brief eLWB Development Application (ETZ Test deployment)
 */

#include "main.h"

/*---------------------------------------------------------------------------*/
#ifdef APP_TASK_ACT_PIN
 #define APP_TASK_ACTIVE    PIN_SET(APP_TASK_ACT_PIN)
 #define APP_TASK_INACTIVE  PIN_CLR(APP_TASK_ACT_PIN)
#else 
 #define APP_TASK_ACTIVE
 #define APP_TASK_INACTIVE
#endif /* APP_TASK_ACT_PIN */
/*---------------------------------------------------------------------------*/
/* global variables */
config_t cfg;
rtimer_clock_t bolt_captured_trq = 0;             /* last captured timestamp */
uint16_t seq_no_lwb  = 0;
uint16_t seq_no_bolt = 0;
uint16_t health_msg_period = HEALTH_MSG_PERIOD;
/*---------------------------------------------------------------------------*/
static uint16_t last_health_pkt = 0;
static dpp_message_t msg_rx;                            /* only used for RX! */
/*---------------------------------------------------------------------------*/
void
capture_timestamp(void)
{ 
  /* simply store the timestamp, do calculations afterwards */
  rtimer_clock_t now = rtimer_now_lf();
  bolt_captured_trq = now - ((uint16_t)(now & 0xffff) - BOLT_CONF_TIMEREQ_CCR);
}
/*---------------------------------------------------------------------------*/
void
update_time(void)
{  
  if(utc_time_updated) {
    /* a UTC timestamp has been received -> update the network time */
    uint32_t elapsed = 0;
    /* adjust the timestamp to align with the next communication round 
    * as closely as possible */
    rtimer_clock_t next_round;
    if(rtimer_next_expiration(ELWB_CONF_LF_RTIMER_ID, &next_round)) {
      elapsed = (next_round - utc_time_rx);          
    } else {
      DEBUG_PRINT_WARNING("invalid timer value");
    }
    /* convert to us */
    elapsed = elapsed * (1000000 / RTIMER_SECOND_LF);
    uint32_t new_time = (utc_time + elapsed) / 1000000;
    uint32_t curr_time = elwb_sched_get_time();
    uint16_t diff = (new_time > curr_time) ? (new_time - curr_time) :
                                             (curr_time - new_time);

    /* only update if the difference is much larger than 1 second */
    if(diff > UTC_TIMESTAMP_MAX_DRIFT) {
      elwb_sched_set_time(new_time);
      DEBUG_PRINT_INFO("timestamp adjusted to %lu", new_time);
      EVENT_INFO(EVENT_CC430_TIME_UPDATED, diff);
    } else {
      DEBUG_PRINT_INFO("timestamp: %lu, drift: %u", new_time, diff);
    }
    utc_time_updated = 0;
  }
}
/*---------------------------------------------------------------------------*/
void
load_config(void)
{
  /* load and restore the config */
  if(nvcfg_load(&cfg)) {
    if(cfg.tx_pwr == 0) {   /* take 0 as invalid -> use value set at compile time */
      cfg.tx_pwr = RF_CONF_TX_POWER;
    } else {
      rf1a_set_tx_power(cfg.tx_pwr);
    }
  #if !IS_HOST
    if(node_id == 0x1122 && cfg.node_id != 0) {         /* still on default value? */
      node_id = cfg.node_id;
      DEBUG_PRINT_MSG_NOW("Node ID %u restored from config", cfg.node_id);
    } else {
      /* save node ID */
      cfg.node_id = node_id;
    }
  #endif /* IS_HOST */
    DEBUG_PRINT_MSG_NOW("Config restored (TX pwr %ddBm, DBG flags 0x%x)",
                        rf1a_tx_power_val[cfg.tx_pwr], cfg.dbg_flags);
  } else {
    EVENT_ERROR(EVENT_CC430_CORRUPTED_CONFIG, 0);
    DEBUG_PRINT_MSG_NOW("WARNING: failed to load config");
  #if !IS_HOST
    /* try to retrieve a valid ID from the unique identifier */
    /*if(node_id == 0x1122) {
      uint64_t my_id = TI_UNIQUE_ID;
      if(my_id == 0x470be0150007001b) {
        node_id = 20054;
      }
    }*/
  #endif /* IS_HOST */
  }
  cfg.rst_cnt++;    /* update reset counter */
  if(!nvcfg_save(&cfg)) {
    EVENT_ERROR(EVENT_CC430_CORRUPTED_CONFIG, 1);
    DEBUG_PRINT_MSG_NOW("Failed to save config");
  }
}
/*---------------------------------------------------------------------------*/
PROCESS(app_pre, "BOLT Task");
PROCESS_THREAD(app_pre, ev, data) 
{  
  PROCESS_BEGIN();
  
  DEBUG_PRINT_MSG_NOW("Process '%s' started", app_pre.name);
  
  while(1) {
    APP_TASK_INACTIVE;
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    APP_TASK_ACTIVE;
    
    AFTER_DEEPSLEEP();    /* restore all clocks */
    
    /* --- read messages from BOLT --- */
    uint16_t read = 0,
             forwarded = 0,
             max_read = 30;
    while(BOLT_DATA_AVAILABLE &&
          (elwb_get_send_buffer_state() < ELWB_CONF_OUT_BUFFER_SIZE) &&
          max_read) {
      if(bolt_read((uint8_t*)&msg_rx)) {
        if(!process_message(&msg_rx, 1)) {
          forwarded++;
        }
        read++;
      } /* else: invalid message received from BOLT */
      max_read--;
    }
    if(read) {
      DEBUG_PRINT_INFO("%u msg read from BOLT, %u forwarded", 
                       read, forwarded);
    }
    
  #if !IS_HOST
    /* --- send the timestamp if one has been requested --- */
    if(bolt_captured_trq) {
      send_timestamp(bolt_captured_trq);
      bolt_captured_trq = 0;
    }
  #else /* !IS_HOST */
    /* note: even though the time is updated here, it will only be used from
     *       the next round since the schedule has already been computed. */
    update_time();
  #endif /* !IS_HOST */
  }
  
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS(app_post, "App Post");
AUTOSTART_PROCESSES(&app_post);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_post, ev, data) 
{
  PROCESS_BEGIN();
  
  /* compile time checks */
  if(sizeof(dpp_message_t) != DPP_MSG_PKT_LEN || 
     sizeof(config_t) != NVCFG_CONF_BLOCK_SIZE) {
    DEBUG_PRINT_FATAL("dpp_message_t or config_t is invalid");
  }
  DEBUG_PRINT_MSG_NOW("Process '%s' started", app_post.name);
  
  /* --- initialization --- */
  
  static uint8_t node_info_sent = 0;
  
#if DEBUG_CONF_P1INT_EN
  P1SEL &= ~BIT5;
  P1IES |= BIT5;
  P1IFG &= ~BIT5;
  P1IE  |= BIT5;
#endif /* DEBUG_CONF_P1INT_EN */
  
  APP_TASK_ACTIVE;
  
  /* init FW updater if required */
#if FW_UPDATE_CONF_ON
  fw_init();
#endif /* FW_UPDATE_CONF_ON */
  
  /* init the ADC */
  adc_init();
  ADC_REFOSC_OFF;     /* shut down REF module to save power */
  
  /* start the preprocess and LWB threads */
  elwb_start(&app_pre, &app_post);
  process_start(&app_pre, NULL);
  
  /* --- load/apply the configuration --- */
  load_config();
  
  /* enable the timestamp request interrupt */
  bolt_set_timereq_callback(capture_timestamp);
  
  /* --- start of application main loop --- */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the LWB task */
    APP_TASK_INACTIVE;
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    APP_TASK_ACTIVE;
    
    /* --- process all packets rcvd from the network (forward to BOLT) --- */
    uint16_t rcvd = 0,
             forwarded = 0;
    while(elwb_rcv_pkt((uint8_t*)&msg_rx)) {
      if(!process_message(&msg_rx, 0)) {
        forwarded++;
      }
      rcvd++;
    }
    if(rcvd) {
      DEBUG_PRINT_INFO("%u msg rcvd from network, %u forwarded", 
                       rcvd, forwarded);
    }
    
    /* generate a node info message if necessary (must be here) */
    if(!node_info_sent) {
  #if !IS_HOST
      if(elwb_get_time(0)) {
  #else /* !IS_HOST */
      if(utc_time) {
        /* register all the nodes defined in ELWB_CONF_SCHED_NODE_LIST */
    #if ELWB_CONF_SCHED_NODE_LIST
        elwb_sched_register_nodes();
    #endif /* ELWB_CONF_SCHED_NODE_LIST */
  #endif /* !IS_HOST */
        send_node_info();
        node_info_sent = 1;
      }
    } else if(health_msg_period) {
      /* only send other messages once the node info msg has been sent! */
      uint16_t div = elwb_get_time(0) / health_msg_period;
      if(div != last_health_pkt) {
        /* using a divider instead of the elapsed time will group the health
        * messages of all nodes together into one round */
        send_node_health();
        send_lwb_health();
        last_health_pkt = div;
      }
    }
    
    /* --- poll the debug task --- */
    debug_print_poll();
    process_poll(&app_post);  /* poll self to run again after the debug task */
    APP_TASK_INACTIVE;
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    APP_TASK_ACTIVE;
    
    /* since this process is only executed at the end of an LWB round, we 
     * can now configure the MCU for minimal power dissipation for the idle
     * period until the next round starts */
    BEFORE_DEEPSLEEP();
  } 
  /* --- end of application main loop --- */

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
