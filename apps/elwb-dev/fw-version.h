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

#ifndef __FW_VERSION_H__
#define __FW_VERSION_H__

/* current FW version (8 bits for major version, 8 bits for minor) */
#define FW_VERSION      0x0004
#define FW_NAME         "elwb-dev"     /* name of the application (8 bytes) */

/*

TODO:
- bug: host potentially sends a health message at the beginning with an invalid
       timestamp
- inconsistency: network timestamp in schedule will jump, so last - current
                 time will not be equal to the round period

Feature requests:
- 


Revision History
----------------

Version 0.4 (2018-05-02):
- feature: automatically switch RF channel if bootstrap times out
- feature: host prints list of host nodes after a data dissemination round
- change: now only 2 guard times, one for HF and one for LF timer
- change: timestamp is only adjusted if the jump is > 5 seconds
- change: schedule slot, contention slot and guard times increased
- change: LWB_PERIOD_T_DATA is now calculated dynamically in the scheduler

Version 0.3 (2018-04-30):
- bugfix: host would not reset streams to inactive after a data round

Version 0.2 (2018-04-27):
- feature: UTC timestamp now distributed over network (LWB schedule)
- bugfix: node info msg from host sometimes had an invalid timestamp (1970)

Version 0.1 (2018-04-06):
- initial version (based on code in lwb-dev)

*/



#endif /* __FW_VERSION_H__ */
