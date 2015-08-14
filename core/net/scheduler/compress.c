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
 * @file 
 * @ingroup LWB
 * @brief   compress / uncompress routines for the schedule
 *
 * @remarks
 * - minor modifications made and comments added by rdaforno (2014-08-20)
 * - each slot must be a uint8 variable, i.e. an integer betw. 0 and 255
 * - the node IDs must be sorted in increasing order, otherwise this algorithm will not work
 * - the number of slots must not be higher than 255
 */
 
#include "lwb.h"

 /*---------------------------------------------------------------------------*/
// the number of bits for depth and length are stored in the thirds slot
#define GET_D_BITS()       (compressed_data[2] >> 3)    // 5 bits are reserved to store the number of bits needed for the depth (i.e. 0 to 31 bits)
#define GET_L_BITS()       (compressed_data[2] & 0x07)  // 3 bits are reserved to store the number of bits needed for the length (i.e. 0 to 7 bits)
#define SET_D_L_BITS(d, l) (compressed_data[2] = (d << 3) | (l & 0x07))
#define COMPR_SLOT(a)      (compressed_data[3 + a])
/*---------------------------------------------------------------------------*/
static uint16_t slots_buffer[LWB_CONF_MAX_DATA_SLOTS];
/*---------------------------------------------------------------------------*/
static inline uint8_t 
get_min_bits(uint16_t a) 
{
    uint8_t i;
    for (i = 15; i > 0; i--) {
        if (a & (1 << i)) {
            return i + 1;
        }
    }
    return i + 1;
}
/*---------------------------------------------------------------------------*/
/**
 * @brief compress the schedule
 * @param [in,out] compressed_data the uncompressed schedule will be read from and the compressed schedule will be written to this i/o buffer
 * @param [in] n_slots the number of slots of this schedule
 * @return the size of the compressed schedule
 */
uint16_t 
lwb_sched_compress(uint8_t* compressed_data, uint8_t n_slots) 
{    
    if (n_slots < 2) {  // don't do anything in case there is only 0 or 1 slot
        return n_slots * 2;
    }
        
    memcpy(slots_buffer, compressed_data, n_slots * 2);     // copy the input data into a buffer
    memset(compressed_data + 2, 0, LWB_CONF_MAX_DATA_SLOTS * 2 - 2);    // clear the output data buffer (except for the first slot!)

    // Note: the first slot holds the first node ID
    
    uint8_t  n_runs = 0;        // how many times the delta has changed
    uint16_t  d[n_slots - 1];
    d[n_runs] = slots_buffer[1] - slots_buffer[0];  // delta (step size)
    uint8_t  d_max = d[n_runs];
    uint16_t  l[n_slots - 1];    // length (how many consecutive slots with step size d)
    l[n_runs] = 0;
    uint8_t  l_max = l[n_runs];
    uint8_t  idx;

    for (idx = 1; idx < n_slots - 1; idx++) {
        if (slots_buffer[idx + 1] - slots_buffer[idx] == d[n_runs]) {
            l[n_runs]++;
        } else {
            if (l[n_runs] > l_max) {
                l_max = l[n_runs];  // keep track of the max. num. of conseq. slots with const. delta
            }
            n_runs++; 
            d[n_runs] = slots_buffer[idx + 1] - slots_buffer[idx];  // calc the new delta
            // for debugging only (make sure the node IDs are in increasing order)
            if (slots_buffer[idx + 1] < slots_buffer[idx]) {
                return 0; // node IDs are not sorted
            }
            if (d[n_runs] > d_max) {
                d_max = d[n_runs];  // keep track of the max. delta
            }
            l[n_runs] = 0;
        }
    }
    if (l[n_runs] > l_max) {
        l_max = l[n_runs];
    }
    n_runs++;

    uint8_t d_bits = get_min_bits(d_max);
    uint8_t l_bits = get_min_bits(l_max);
    uint8_t run_bits = d_bits + l_bits; // required bits for each delta + length

    for (idx = 0; idx < n_runs; idx++) {        
        uint16_t offset_bits = run_bits * idx;
        // store the current and the following 3 bytes in a 32-bit variable
        uint32_t tmp = COMPR_SLOT(offset_bits / 8) | ((uint32_t)COMPR_SLOT(offset_bits / 8 + 1) << 8) | ((uint32_t)COMPR_SLOT(offset_bits / 8 + 2) << 16) | ((uint32_t)COMPR_SLOT(offset_bits / 8 + 3) << 24);
        // append the new data (d and l)
        tmp |= ( ( ((uint32_t)d[idx] << l_bits) | l[idx] ) << (offset_bits % 8) );  
        COMPR_SLOT(offset_bits / 8)     = (uint8_t)tmp;
        COMPR_SLOT(offset_bits / 8 + 1) = (uint8_t)(tmp >> 8);
        COMPR_SLOT(offset_bits / 8 + 2) = (uint8_t)(tmp >> 16);
        COMPR_SLOT(offset_bits / 8 + 3) = (uint8_t)(tmp >> 24);
    }
    SET_D_L_BITS(d_bits, l_bits);   // store the number of bits for d and l
        
    return 3 + ((((uint16_t)n_runs * run_bits) + 7) >> 3);      // return the size of the compressed schedule
}
/*---------------------------------------------------------------------------*/
/**
 * @brief uncompress the schedule
 * @param [in,out] compressed_data the compressed schedule will be read from and the uncompressed schedule will be written to this i/o buffer
 * @param [in] n_slots the number of slots of this schedule
 * @return 1 if successful, 0 otherwise
 */
uint8_t
lwb_sched_uncompress(uint8_t* compressed_data, uint8_t n_slots) 
{
    if (n_slots < 2) {  // don't do anything in case there is only 0 or 1 slot
        return 0;
    }
    memcpy(slots_buffer, compressed_data, 2);     // must use memcpy due to pointer alignment problems

    uint8_t d_bits = GET_D_BITS();
    uint8_t l_bits = GET_L_BITS();
    uint8_t run_bits = d_bits + l_bits;
    
    // debugging only: check if the values make sense
    if (d_bits == 0 || d_bits > 8 || l_bits == 0) {
        return 0; // invalid d or l bits
    }
    
    uint8_t slot_idx = 1, idx;
    uint32_t mask = ((1 << run_bits) - 1);
    for (idx = 0; slot_idx < n_slots; idx++) {
        // extract d and l of this run
        uint16_t offset_bits = run_bits * idx;
        uint32_t tmp = COMPR_SLOT(offset_bits / 8) | ((uint32_t)COMPR_SLOT(offset_bits / 8 + 1) << 8) | ((uint32_t)COMPR_SLOT(offset_bits / 8 + 2) << 16) | ((uint32_t)COMPR_SLOT(offset_bits / 8 + 3) << 24);
        uint32_t run_info = ((tmp >> (offset_bits % 8)) & mask);
        uint16_t d = run_info >> l_bits;
        uint16_t l = run_info & ((1 << l_bits) - 1);
        uint8_t i;
        // generate the slots
        for (i = 0; i < l + 1; i++) {
            slots_buffer[slot_idx] = slots_buffer[slot_idx - 1] + d;    // add the offset to the previous slot
            slot_idx++;
        }
    }
    
    memcpy(compressed_data, slots_buffer, n_slots * 2);
    
    return 1;
}
/*---------------------------------------------------------------------------*/