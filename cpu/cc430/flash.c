
#include "platform.h"



uint16_t 
get_code_size(void) 
{
    uint16_t* curr_addr = (uint16_t*)(FLASH_END - 1);   // 16-bit word alignment
    while (((uint16_t)curr_addr > FLASH_START) && (*curr_addr == 0xffff)) {     // assumption: each unused word in the flash memory has the value 0xffff; flash programming starts at the smallest address
        curr_addr--;
    }
    return (uint16_t)curr_addr - FLASH_START + 2;
}
