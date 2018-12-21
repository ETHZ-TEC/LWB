#ifndef __HAL_PMM_H__
#define __HAL_PMM_H__

/******************************************************************************
 * Function Library for setting the PMM 
 *    File: hal_pmm.h 
 *  
 *    Texas Instruments 
 *  
 *    Version 1.2 
 *    10/17/09 
 *  
 *    V1.0  Initial Version 
 *    V1.1  Adjustment to UG 
 *    V1.2  Added return values 
 *****************************************************************************/

#define PMM_STATUS_OK     0
#define PMM_STATUS_ERROR  1

/* disable the SVS */
#define SVS_DISABLE       { PMMCTL0_H = 0xA5;\
                            SVSMHCTL  = 0; /* high-side ctrl register */\
                            SVSMLCTL  = 0; /* low-side ctrl register */ \
                            PMMCTL0_H = 0x00; }
/* enable SVS high-side normal perf. mode (monitoring and supervisor)
 * set reset/release voltage levels to recommended values (user guide p.84)
 * note: reset is enabled by default in PMMRIE */
#define SVS_ENABLE        { PMMCTL0_H = 0xA5; \
                            SVSMHCTL = SVSHE | SVMHE | SVSHRVL_2 | SVSMHRRL_2;\
                            SVSMLCTL = SVMLE | SVSLE | SVSLRVL_2 | SVSMLRRL_2;\
                            PMMCTL0_H = 0x00; }
/* trigger a reset */
#define PMM_TRIGGER_POR   { PMMCTL0_H  = 0xA5; PMMCTL0_L |= PMMSWPOR; }
#define PMM_TRIGGER_BOR   { PMMCTL0_H  = 0xA5; PMMCTL0_L |= PMMSWBOR; }


/* ==================================================================== */
/**
 * Set the VCore to a new level if it is possible and return a
 * error - value.
 *
 * \param      level       PMM level ID
 * \return    int        1: error / 0: done
 */
uint16_t SetVCore(uint8_t level);

/* ==================================================================== */
/**
 * Set the VCore to a higher level, if it is possible.
 * Return a 1 if voltage at highside (Vcc) is to low
 * for the selected Level (level).
 *
 * \param      level       PMM level ID
 * \return    int        1: error / 0: done
 */
uint16_t SetVCoreUp(uint8_t level);

/* ==================================================================== */
/**
 * Set the VCore to a lower level.
 * Return a 1 if voltage at highside (Vcc) is still to low
 * for the selected Level (level).
 *
 * \param      level       PMM level ID
 * \return    int        1: done with error / 0: done without error
 */
uint16_t SetVCoreDown(uint8_t level);

#endif /* __HAL_PMM_H__ */
