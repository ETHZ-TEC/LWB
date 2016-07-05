/*
 * Copyright (c) 2015, Swiss Federal Institute of Technology (ETH Zurich).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in the
 *  documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *  contributors may be used to endorse or promote products derived
 *  from this software without specific prior written permission.
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
 *          Marco Zimmerling
 */

#include "lwb.h"

/*---------------------------------------------------------------------------*/
static lwb_stream_t streams[LWB_CONF_MAX_N_STREAMS_PER_NODE];
volatile uint32_t lwb_pending_requests = 0;      
volatile uint8_t  lwb_joined_streams_cnt = 0;    /* number of active streams */
/*---------------------------------------------------------------------------*/
void 
lwb_stream_init() 
{
  memset(streams, 0, 
         (LWB_STREAM_INFO_HEADER_LEN + LWB_CONF_STREAM_EXTRA_DATA_LEN) * 
         LWB_CONF_MAX_N_STREAMS_PER_NODE);
  lwb_pending_requests = 0;
  lwb_joined_streams_cnt = 0;
}
/*---------------------------------------------------------------------------*/
uint8_t
lwb_stream_update_state(uint8_t stream_id) 
{
  uint8_t i = 0;
  for(; i < LWB_CONF_MAX_N_STREAMS_PER_NODE; i++) {     /* search the stream */
    if(streams[i].id == stream_id) {
      /* clear the corresponding bit */
      lwb_pending_requests &= ~((uint32_t)1 << i);  
      if(streams[i].ipi) {
        if(streams[i].state != LWB_STREAM_STATE_ACTIVE) {
          streams[i].state = LWB_STREAM_STATE_ACTIVE;
          lwb_joined_streams_cnt++;
        }
        return 1;   /* stream is active */
      } else {
        if(streams[i].state > LWB_STREAM_STATE_INACTIVE) {  
          if(lwb_joined_streams_cnt == 0) {
            DEBUG_PRINT_WARNING("something is wrong: lwb_joined_streams_cnt "
                                "was negative");
          } else {
            lwb_joined_streams_cnt--;
          }
        }        
        streams[i].state = LWB_STREAM_STATE_INACTIVE;
        return 0;   /* stream removed */
      }
    }    
  }
  return 0;  /* stream not found */
}
/*---------------------------------------------------------------------------*/
uint8_t
lwb_stream_add(const lwb_stream_req_t* const stream_info) 
{
  uint8_t i = 0, idx = 0xff;
  if(LWB_INVALID_STREAM_ID == stream_info->stream_id) { return 0; }
  
  for(; i < LWB_CONF_MAX_N_STREAMS_PER_NODE; i++) {
    if(streams[i].id == stream_info->stream_id) {
      /* exists already -> update steam data */
      memcpy((uint8_t*)&streams[i] + 2,     /* skip the first 2 bytes */
             (uint8_t*)stream_info + 4,     /* skip the first 4 bytes */
             LWB_STREAM_REQ_HEADER_LEN - 4 + LWB_CONF_STREAM_EXTRA_DATA_LEN);
      streams[i].state = LWB_STREAM_STATE_WAITING;                 /* rejoin */    
      lwb_pending_requests |= (1 << i);     /* set the 'request pending' bit */
      DEBUG_PRINT_INFO("stream with ID %u updated (IPI %u)", 
                       stream_info->stream_id, 
                       stream_info->ipi);
      return 1;
    }
    if(idx == 0xff && streams[i].state == LWB_STREAM_STATE_INACTIVE) {
      idx = i;  /* this stream is not being used -> take this slot */
    }
  }
  /* add the new stream */
  if(idx != 0xff) {
    /* copy the stream info (skip the first 2 bytes, the node ID) */
    memcpy((uint8_t*)&streams[idx], (uint8_t*)stream_info + 2, 
           (LWB_STREAM_REQ_HEADER_LEN + LWB_CONF_STREAM_EXTRA_DATA_LEN - 2));
    streams[idx].state = LWB_STREAM_STATE_WAITING;
    lwb_pending_requests |= (1 << idx);     /* set the 'request pending' bit */
    DEBUG_PRINT_INFO("stream with ID %u added (IPI %u)", 
                     streams[idx].id, streams[idx].ipi);
    return 1;
  } else {
    DEBUG_PRINT_ERROR("no more space for new streams");
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
void
lwb_stream_drop(uint8_t stream_id)
{
  uint8_t i = 0;
  for(; i < LWB_CONF_MAX_N_STREAMS_PER_NODE; i++) {
    if(streams[i].id == stream_id) {
      /* clear the corresponding bit */
      lwb_pending_requests &= ~((uint32_t)1 << i);  
      streams[i].ipi = 0;
      if(streams[i].state > LWB_STREAM_STATE_INACTIVE &&
         lwb_joined_streams_cnt) {
        lwb_joined_streams_cnt--;
      }
      streams[i].state = LWB_STREAM_STATE_INACTIVE;
      DEBUG_PRINT_INFO("stream with ID %u dropped", stream_id);
    }
  }
}
/*---------------------------------------------------------------------------*/
void 
lwb_stream_rejoin(void) 
{
  uint8_t i = 0;
  /* set 'request pending' bit for all active streams */
  for(; i < LWB_CONF_MAX_N_STREAMS_PER_NODE; i++) {
    if(streams[i].state == LWB_STREAM_STATE_ACTIVE) {
      streams[i].state = LWB_STREAM_STATE_WAITING;
      lwb_pending_requests |= (1 << i);
    }
  }
}
/*---------------------------------------------------------------------------*/
uint8_t
lwb_stream_prepare_req(lwb_stream_req_t* const out_srq_pkt, uint8_t stream_id) 
{
  if(stream_id == LWB_INVALID_STREAM_ID || 
    streams[stream_id].state != LWB_STREAM_STATE_WAITING) {
    /* get the first stream request in the list */
    uint8_t i = 0;
    stream_id = LWB_INVALID_STREAM_ID;
    for(; i < LWB_CONF_MAX_N_STREAMS_PER_NODE; i++) {
      if(streams[i].state == LWB_STREAM_STATE_WAITING) {
        stream_id = i;
        break;
      }
    }
  }
  if(LWB_INVALID_STREAM_ID != stream_id) {
    /* compose the packet */
    out_srq_pkt->id = node_id;
    memcpy((uint8_t*)out_srq_pkt + 3,  /* skip the first 3 bytes */
           (uint8_t*)&streams[stream_id] + 1, 
           LWB_STREAM_REQ_HEADER_LEN - 3 + LWB_CONF_STREAM_EXTRA_DATA_LEN);
    return 1;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
lwb_stream_state_t lwb_stream_get_state(uint8_t stream_id)
{
  uint8_t i = 0;
  /* search the stream */
  for(; i < LWB_CONF_MAX_N_STREAMS_PER_NODE; i++) {
    if(streams[i].id == stream_id) {
      return streams[i].state;
    }
  }
  return LWB_STREAM_STATE_INACTIVE;    
}
/*---------------------------------------------------------------------------*/