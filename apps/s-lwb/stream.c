
#include "s-lwb.h"


static stream_t streams[N_STREAMS_MAX_SRC];
volatile uint32_t pending_requests = 0;            
volatile uint8_t  n_joined_streams = 0;           // number of active (joined) streams


/**
 * @brief initialize the stream list on the source node
 */
void stream_init(void) {
    memset(streams, 0, sizeof(stream_t) * N_STREAMS_MAX_SRC);
    uint8_t i = 0;
    for (; i < N_STREAMS_MAX_SRC; i++) {
        streams[i].id = INVALID_STREAM_ID;
    }
}


/**
 * @brief update the status of a stream (call this function when an S-ACK was received)
 * @return one if stream is in the list and still active, zero otherwise
 */
uint8_t stream_update(uint8_t stream_id) {
    uint8_t i = 0;
    for (; i < N_STREAMS_MAX_SRC; i++) {                // search the stream
        if (streams[i].id == stream_id) {
            pending_requests &= ~((uint32_t)1 << i);    // clear the corresponding bit
            if (streams[i].ipi) {
                if (streams[i].joining_state != JOINED) {
                    streams[i].joining_state = JOINED;
                    n_joined_streams++;
                }
                return 1;   // stream is active
            } else {
                if (streams[i].joining_state > NOT_JOINED) {    // may be JOINING or JOINED
                    if (n_joined_streams == 0) {
                        DEBUG_PRINT_WARNING("something is wrong: n_joined_streams was negative");
                    } else {
                        n_joined_streams--;
                    }
                }                
                streams[i].joining_state = NOT_JOINED;  // mark this stream as deleted
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
 */
void stream_add(uint8_t stream_id, uint8_t ipi, int8_t offset) {
    uint8_t i = 0, idx = 0xff;
    for (; i < N_STREAMS_MAX_SRC; i++) {
        if (streams[i].id == stream_id) {
            // already_exists, update ipi
            if (streams[i].ipi == ipi && streams[i].ofs == offset) {
                // no need to update
                return;
            }
            streams[i].ipi = ipi;
            streams[i].ofs = offset;
            streams[i].joining_state = JOINING;     // rejoin
            pending_requests |= (1 << i);           // set the bit
            DEBUG_PRINT_INFO("stream updated (id: %d, ipi: %d)", stream_id, ipi);
            return;
        }
        if (idx == 0xff && streams[i].joining_state == NOT_JOINED) {   // unused stream
            idx = i;
        }
    }
    if (ipi == 0) {    
        return;    // only add streams with IPI != 0
    }
    // add the new stream
    if (idx != 0xff) {
        streams[idx].id = stream_id;
        streams[idx].ipi = ipi;
        streams[idx].ofs = offset;
        streams[idx].joining_state = JOINING;
        pending_requests |= (1 << idx);            // set the bit
        DEBUG_PRINT_INFO("stream added (i=%d id=%d ipi=%d ofs=%d)", idx, stream_id, ipi, offset);
    } else {
        DEBUG_PRINT_WARNING("no more space for new streams");
    }
}


/**
 * @brief sets all joined streams back to JOINING
 * force all the active streams to re-join, i.e. re-send the stream request
 */
void stream_rejoin(void) {
    uint8_t i = 0;
    for (; i < N_STREAMS_MAX_SRC; i++) {
        if (streams[i].joining_state == JOINED) {
            streams[i].joining_state = JOINING;
            pending_requests |= (1 << i);
        }
    }
}


/**
 * @brief prepare a stream request
 * @param[out] out_data the output buffer
 */
void stream_prep_req(stream_request_t* out_data) {
    uint8_t i = 0;
    for (; i < N_STREAMS_MAX_SRC; i++) {
        if (streams[i].joining_state == JOINING) {
            // create the packet
            out_data->node_id = node_id;
            out_data->stream_id = streams[i].id;
            out_data->ipi = streams[i].ipi;
            out_data->t_offset = streams[i].ofs;
            break;
        }
    }
}
