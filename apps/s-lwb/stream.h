
/**
 * @file
 * @ingroup LWB
 * @brief   manages the streams on the source node
 * @author  rdaforno
 */

#ifndef STREAM_H
#define STREAM_H



#ifndef N_STREAMS_MAX_SRC
    #define N_STREAMS_MAX_SRC   32
#endif // N_STREAMS_MAX_SRC

#define INVALID_STREAM_ID       0xff

#define STREAM_REQ_PENDING      ( pending_requests != 0 )
#define STREAMS_ACTIVE          ( n_joined_streams != 0 )


/**
 * @brief the different joining states of a stream
 */
typedef enum {
    NOT_JOINED = 0,
    JOINING,
    JOINED,
} joining_state_t;

/**
 * @brief struct to store information about the active streams on a source node
 */
typedef struct {
    uint16_t        pkt_cnt;        // number of sent packets
    uint8_t         id;
    uint8_t         ipi;    
    int8_t          ofs;            // offset
    joining_state_t joining_state;  // state of this stream
} stream_t;


extern volatile uint32_t pending_requests;
extern volatile uint8_t  n_joined_streams;


void stream_init(void);
uint8_t stream_update(uint8_t stream_id);
void stream_add(uint8_t stream_id, uint8_t ipi, int8_t offset);
void stream_rejoin(void);
void stream_prep_req(stream_request_t* out_data);


#endif
