
/**
 * @file
 * @ingroup LWB
 * @brief   manages the streams on the source node
 * @author  rdaforno
 */

#ifndef __STREAM_H__
#define __STREAM_H__

#include "scheduler.h"


#define LWB_STREAM_REQ_PENDING          ( lwb_pending_requests != 0 )
#define LWB_STREAMS_ACTIVE              ( lwb_joined_streams_cnt != 0 )

/**
 * @brief the different joining states of a stream
 */
typedef enum {
    LWB_STREAM_NOT_JOINED = 0,
    LWB_STREAM_JOINING,
    LWB_STREAM_JOINED,
} lwb_stream_joining_state_t;


extern volatile uint32_t lwb_pending_requests;
extern volatile uint8_t  lwb_joined_streams_cnt;


void lwb_stream_init(void);
uint8_t lwb_stream_update(uint8_t stream_id);
uint8_t lwb_stream_add(uint8_t stream_id, uint8_t ipi, int8_t offset);
void lwb_stream_rejoin(void);
uint8_t lwb_stream_prep_req(lwb_stream_req_t * const out_data, uint8_t stream_id);


#endif /* __STREAM_H__ */
