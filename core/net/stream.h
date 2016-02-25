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
 *          Marco Zimmerling
 */

/**
 * @addtogroup  lwb
 * @{
 *
 * @defgroup    stream LWB streams
 * @{
 *
 * @file 
 * 
 * @brief   manages streams on the source node
 * 
 * Keeps track of the stream state.
 * lwb_stream_info_t must be defined in the scheduler implementation
 */

#ifndef __STREAM_H__
#define __STREAM_H__

#include "lwb.h"

#define LWB_STREAM_REQ_PENDING          ( lwb_pending_requests != 0 )
#define LWB_STREAMS_ACTIVE              ( lwb_joined_streams_cnt != 0 )

#define LWB_INVALID_STREAM_ID           0xff

/**
 * @brief the different states of a stream
 * active means the host node has accepted the stream request (S-ACK received) 
 *   and allocated bandwidth (data slots) for this stream
 * inactive means the stream request was not yet sent or the host rejected or
 *   dropped the stream (the source node won't get data slots for this stream)
 * waiting means, a stream request was sent and is pending, i.e. the source 
 *   node is awaiting an S-ACK
 */
enum {
  LWB_STREAM_STATE_INACTIVE = 0,
  LWB_STREAM_STATE_WAITING,
  LWB_STREAM_STATE_ACTIVE,
  NUM_LWB_STREAM_STATES
};
/* only way to control the size of an enum type */
typedef uint8_t lwb_stream_state_t; 

/**
 * @brief struct to store information about the active streams on a source node
 */
#define LWB_STREAM_INFO_HEADER_LEN      4
typedef struct {
  lwb_stream_state_t  state;
  uint8_t             id;
  uint16_t            ipi;
#if LWB_CONF_STREAM_EXTRA_DATA_LEN
  uint8_t             extra_data[LWB_CONF_STREAM_EXTRA_DATA_LEN];
#endif /* LWB_CONF_STREAM_EXTRA_DATA_LEN */
} lwb_stream_t;


extern volatile uint32_t lwb_pending_requests;
extern volatile uint8_t  lwb_joined_streams_cnt;

/**
 * @brief initialize the stream list on the source node
 */
void lwb_stream_init(void);

/**
 * @brief update the state of a stream (call this function when an S-ACK was 
 * received)
 * @return one if stream is in the list and still active, zero otherwise
 */
uint8_t lwb_stream_update_state(uint8_t stream_id);

/**
 * @brief add/update a stream to/in the stream list
 * @param[in] stream_info data structure containing the stream info
 * @return 1 if successful, 0 otherwise
 * 
 * The application can call this function to add (and request) a stream.
 * If the stream already exists, it is updated with the new stream information.
 */
uint8_t lwb_stream_add(const lwb_stream_req_t* const stream_info);

/**
 * @brief delete a stream from the stream list without scheduling a stream
 * request (drop silently)
 * @param[in] stream_id ID of the stream
 */
void lwb_stream_drop(uint8_t stream_id);

/**
 * @brief sets all joined streams back to JOINING
 * force all the active streams to re-join, i.e. re-send the stream request
 */
void lwb_stream_rejoin(void);

/**
 * @brief prepare a stream request
 * @param[out] out_srq_pkt the output buffer for the generated stream request
 * packet
 * @param stream_id optional parameter, set this to LWB_INVALID_STREAM_ID if 
 * the function shall deside which stream request to send
 * @return 1 if successful, 0 otherwise
 */
uint8_t 
lwb_stream_prepare_req(lwb_stream_req_t* const out_srq_pkt, uint8_t stream_id);


/**
 * @brief get the state of the stream
 * @return LWB_STREAM_STATE_WAITING if the stream request is pending, 
 * LWB_STREAM_STATE_ACTIVE if the stream is active and LWB_STREAM_STATE_INACTIVE
 * if the stream does not exist or has been disabled.
 */
lwb_stream_state_t lwb_stream_get_state(uint8_t stream_id);


#endif /* __STREAM_H__ */

/**
 * @}
 * @}
 */
