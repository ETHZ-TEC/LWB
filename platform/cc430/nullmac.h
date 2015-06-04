#ifndef __NULLMAC_H__
#define __NULLMAC_H__


void nullmac_init(void);

void unicast_send(void *payload, uint8_t payload_len, addr_t destination);
void broadcast_send(void *payload, uint8_t payload_len);

extern void unicast_received(rtimer_clock_t *timestamp, void *payload, uint8_t payload_len, addr_t source);
extern void broadcast_received(rtimer_clock_t *timestamp, void *payload, uint8_t payload_len, addr_t source);

#endif /* __NULLMAC_H__ */
