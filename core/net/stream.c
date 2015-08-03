
#include "lwb.h"
#include "stream.h"


/**
 * @brief struct to store information about the active streams on a source node
 */
typedef struct {
    uint16_t                    pkt_cnt;        // number of sent packets
    uint8_t                     id;
    uint8_t                     ipi;    
    int8_t                      ofs;            // offset
    lwb_stream_joining_state_t  joining_state;  // state of this stream
} lwb_stream_t;


static lwb_stream_t streams[LWB_MAX_STREAM_CNT_PER_NODE];
volatile uint32_t lwb_pending_requests = 0;            
volatile uint8_t  lwb_joined_streams_cnt = 0;           // number of active (joined) streams


/**
 * @brief initialize the stream list on the source node
 */
void lwb_stream_init(void) {
    memset(streams, 0, sizeof(lwb_stream_t) * LWB_MAX_STREAM_CNT_PER_NODE);
    uint8_t i = 0;
    for (; i < LWB_MAX_STREAM_CNT_PER_NODE; i++) {
        streams[i].id = LWB_INVALID_STREAM_ID;
    }
}


/**
 * @brief update the status of a stream (call this function when an S-ACK was received)
 * @return one if stream is in the list and still active, zero otherwise
 */
uint8_t lwb_stream_update(uint8_t stream_id) {
    uint8_t i = 0;
    for (; i < LWB_MAX_STREAM_CNT_PER_NODE; i++) {                // search the stream
        if (streams[i].id == stream_id) {
            lwb_pending_requests &= ~((uint32_t)1 << i);    // clear the corresponding bit
            if (streams[i].ipi) {
                if (streams[i].joining_state != LWB_STREAM_JOINED) {
                    streams[i].joining_state = LWB_STREAM_JOINED;
                    lwb_joined_streams_cnt++;
                }
                return 1;   // stream is active
            } else {
                if (streams[i].joining_state > LWB_STREAM_NOT_JOINED) {    // may be JOINING or JOINED
                    if (lwb_joined_streams_cnt == 0) {
                        DEBUG_PRINT_WARNING("something is wrong: lwb_joined_streams_cnt was negative");
                    } else {
                        lwb_joined_streams_cnt--;
                    }
                }                
                streams[i].joining_state = LWB_STREAM_NOT_JOINED;  // mark this stream as deleted
                return 0;   // stream removed
            }
        }        
    }
    return 0;   // stream not found
}


/**
 * @brief add/update a stream to/in the stream list
 * @param[in] stream_id the stream ID
 * @param[in] ipi the new IPI of the stream
 * @param[in] offset the new offset of the stream
 * @return 1 if successful, 0 otherwise
 */
uint8_t lwb_stream_add(uint8_t stream_id, uint8_t ipi, int8_t offset) {
    uint8_t i = 0, idx = 0xff;
    for (; i < LWB_MAX_STREAM_CNT_PER_NODE; i++) {
        if (streams[i].id == stream_id) {
            // already_exists, update ipi
            if (streams[i].ipi == ipi && streams[i].ofs == offset) {
                // no need to update
                return 1;
            }
            streams[i].ipi = ipi;
            streams[i].ofs = offset;
            streams[i].joining_state = LWB_STREAM_JOINING;     // rejoin
            lwb_pending_requests |= (1 << i);           // set the bit
            DEBUG_PRINT_INFO("stream updated (id: %d, ipi: %d)", stream_id, ipi);
            return 1;
        }
        if (idx == 0xff && streams[i].joining_state == LWB_STREAM_NOT_JOINED) {   // unused stream
            idx = i;
        }
    }
    if (ipi == 0) {    
        return 0;    // only add streams with IPI != 0
    }
    // add the new stream
    if (idx != 0xff) {
        streams[idx].id = stream_id;
        streams[idx].ipi = ipi;
        streams[idx].ofs = offset;
        streams[idx].joining_state = LWB_STREAM_JOINING;
        lwb_pending_requests |= (1 << idx);            // set the bit
        DEBUG_PRINT_INFO("stream added (i=%d id=%d ipi=%d ofs=%d)", idx, stream_id, ipi, offset);
        return 1;
    } else {
        DEBUG_PRINT_WARNING("no more space for new streams");
    }
    return 0;
}


/**
 * @brief sets all joined streams back to JOINING
 * force all the active streams to re-join, i.e. re-send the stream request
 */
void lwb_stream_rejoin(void) {
    uint8_t i = 0;
    for (; i < LWB_MAX_STREAM_CNT_PER_NODE; i++) {
        if (streams[i].joining_state == LWB_STREAM_JOINED) {
            streams[i].joining_state = LWB_STREAM_JOINING;
            lwb_pending_requests |= (1 << i);
        }
    }
}


/**
 * @brief prepare a stream request
 * @param[out] out_data the output buffer
 * @param stream_id optional parameter, set this to LWB_INVALID_STREAM_ID if 
 * the function shall deside which stream request to send
 * @return 1 if successful, 0 otherwise
 */
uint8_t lwb_stream_prep_req(lwb_stream_req_t * const out_data, uint8_t stream_id) {
    if (stream_id != LWB_INVALID_STREAM_ID && 
        streams[stream_id].joining_state == LWB_STREAM_JOINING) {
        // create the packet
        out_data->node_id = node_id;
        out_data->stream_id = streams[stream_id].id;
        out_data->ipi = streams[stream_id].ipi;
        out_data->t_offset = streams[stream_id].ofs;
        return 1;
    } else {
        uint8_t i = 0;
        for (; i < LWB_MAX_STREAM_CNT_PER_NODE; i++) {
            if (streams[i].joining_state == LWB_STREAM_JOINING) {
                // create the packet
                out_data->node_id = node_id;
                out_data->stream_id = streams[i].id;
                out_data->ipi = streams[i].ipi;
                out_data->t_offset = streams[i].ofs;
                return 1;
            }
        }
    }
    return 0;
}
