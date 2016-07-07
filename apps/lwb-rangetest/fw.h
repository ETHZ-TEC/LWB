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
 */

/**
 * @addtogroup  Platform
 * @{
 *
 * @defgroup    utils Firmware updater/manager
 * @{
 *
 * @file
 *
 * @brief provides basic functionalities to store, verify and update
 * the firmware
 */

#ifndef __FW_UPDATER_H__
#define __FW_UPDATER_H__


#ifndef FW_CONF_ON
#define FW_CONF_ON                    0
#endif /* FW_CONF_ON */

#ifndef FW_ADDR_XMEM
#define FW_ADDR_XMEM                  2048
#endif /* FW_ADDR_XMEM */

#define FW_MAX_SIZE                   32768    /* do not change */
#define FW_BLOCK_SIZE                 (sizeof(data_t) - 2)
#define FW_NUM_BLOCKS                 ((FW_MAX_SIZE + FW_BLOCK_SIZE - 1) \
                                       / FW_BLOCK_SIZE)
#define FW_BLOCK_INFO_SIZE            ((FW_NUM_BLOCKS + 7) / 8)
#define FW_DATA_START                 (FW_ADDR_XMEM + sizeof(fw_info_t) + \
                                       FW_BLOCK_INFO_SIZE)
#define FW_BACKUP_ADDR_XMEM           (FW_DATA_START + FW_MAX_SIZE)


typedef struct {
  uint16_t         version;
  uint16_t         len;         /* total length in bytes */
  fw_status_type_t status : 8;
  uint8_t          block_size;  /* must be equal to FW_BLOCK_SIZE */
  uint16_t         data_crc;    /* CRC over payload of all data packets */
  uint16_t         crc;         /* CRC over this info block */
} fw_info_t;


/**
 * @brief init the firmware updater (i.e. load the info block and verify it)
 * @return 0 if successful, error code otherwise
 */
uint8_t fw_init(void);

/**
 * @brief clear all existing FW data and overwrite the info struct with a 
 * new one
 * @param fw the new info struct
 * @return 0 if successful, error code otherwise
 */
uint8_t fw_reset(const fw_info_t* new_fw_info);

/**
 * @brief store a new block of FW data in the external memory
 * @param data pointer to the FW data block to store
 */
void fw_store_data(const data_t* data);

/**
 * @brief get a list of data blocks that are still missing (not yet received)
 * @param out_data a buffer to hold the IDs (16-bit values)
 * @param max_num_ids max. number of IDs to store in the buffer, this means the 
 * buffer must be at least max_num_ids * 2 bytes long
 * @return the number of IDs stored in the output buffer
 */
uint16_t fw_get_missing_block_ids(uint16_t* out_data, uint8_t max_num_ids);

/**
 * @brief validate the firmware that is currently in the external memory
 * @return 0 if successful, error code otherwise
 */
uint8_t fw_validate(void);

/**
 * @brief backup the currently installed firmware to the external memory
 * @return 0 if successful, error code otherwise
 */
uint8_t fw_backup(void);


#endif /* __FW_UPDATER_H__ */

/**
 * @}
 * @}
 */
