/* Sync word qualifier mode = 30/32 sync word bits detected */
/* CRC autoflush = false */
/* Channel spacing = 199.951172 (default value, MDMCFG2[0:1]) */
/* Data format = Normal mode */
/* Data rate = 249.939 */
/* RX filter BW = 541.666667 */
/* PA ramping = false */
/* Preamble count = 4 (default value, MDMCFG1[4:6]) */
/* Whitening = true (enabled by default, PKTCTRL0[6]) */
/* Address config = No address check */
/* Carrier frequency = 867.999939 */
/* Device address = 0 */
/* TX power = 0 */
/* Manchester enable = false */
/* CRC enable = true (enabled by default, PKTCTRL0[2]) */
/* Deviation = 126.953125 */
/* Packet length mode = Variable packet length mode. Packet length */
/*                      configured by the first byte after sync word */
/* Packet length = 255 */
/* Modulation format = 2-GFSK (MDMCFG2[6:4] = 1) */
/* Base frequency = 867.999939 */
/* Modulated = true */
/* Channel number = 0 */
/***************************************************************
 *  SmartRF Studio(tm) Export
 *
 *  Radio register settings specifed with C-code
 *  compatible #define statements.
 *
 *  RF device: CC430
 *
 ***************************************************************/

#ifndef __SMARTRF_CC430_H__
#define __SMARTRF_CC430_H__

#define SMARTRF_IOCFG0     0x06 /* gdo0 output configuration */
#define SMARTRF_FSCTRL1    0x0C /* frequency synthesizer control */
// note: better RF performance when whitening is enabled! leave default value
//#define SMARTRF_PKTCTRL0   0x05 /* variable pkt len, CRC en, no whitening */
#define SMARTRF_FREQ2      0x21 /* frequency control word, high byte */
#define SMARTRF_FREQ1      0x62 /* frequency control word, middle byte */
#define SMARTRF_FREQ0      0x76 /* frequency control word, low byte */
#define SMARTRF_MDMCFG4    0x2D /* modem configuration */
#define SMARTRF_MDMCFG3    0x3B /* modem configuration */
#define SMARTRF_MDMCFG2    0x13 /* modem configuration (smartrf default: 0x93) */
#define SMARTRF_DEVIATN    0x62 /* modem deviation setting */
#define SMARTRF_MCSM0      0x10 /* main radio control state machine config */
#define SMARTRF_FOCCFG     0x1D /* frequency offset compensation config */
#define SMARTRF_BSCFG      0x1C /* bit synchronization configuration */
#define SMARTRF_AGCCTRL2   0xC7 /* agc control */
#define SMARTRF_AGCCTRL1   0x00 /* agc control */
#define SMARTRF_AGCCTRL0   0xB0 /* agc control */
#define SMARTRF_WORCTRL    0xFB /* wake on radio control */
#define SMARTRF_FREND1     0xB6 /* front end rx configuration */
#define SMARTRF_FSCAL3     0xEA /* frequency synthesizer calibration */
#define SMARTRF_FSCAL2     0x2A /* frequency synthesizer calibration */
#define SMARTRF_FSCAL1     0x00 /* frequency synthesizer calibration */
#define SMARTRF_FSCAL0     0x1F /* frequency synthesizer calibration */
//0x09 means VCO selection calibration stage is disabled (i.e. FSCAL2.VCO_CORE_H_EN is used)
#define SMARTRF_TEST0      0x09 /* various test settings (reset value is 0x0b) */

/* values for Glossy (measured with the logic analyzer, in ns) */
/* TAU1: beginning of tx to beginning of rx (rising edges of GDO pins) */
//#define TAU1               (rtimer_clock_t)13540  /* 13.54 us */
#define TAU1               13600  /* changed by rdaforno */
//varies from 13.54 to 13.76
/* TAU2: end of tx to end of rx (falling edges of GDO pins) */
#define TAU2               (rtimer_clock_t)11860  /* 11.86 us */
//varies from 11.84 to 12.05
/* tx to rx switching (falling to rising edge of GDO), varies from 312210 to 312400 */
//#define T2R                (rtimer_clock_t)302100 /* 302.1 us */
#define T2R                312310  /* changed by rdaforno */
/* rx to tx switching (falling to rising edge of GDO), quite stable */
//#define R2T                (rtimer_clock_t)276700 /* 276.7 us */
#define R2T                (rtimer_clock_t)286760  /* changed by rdaforno */
/* time to transmit one byte @250kBaud */
//#define T_TX_BYTE          (rtimer_clock_t)32010 /* 32.01 us */
#define T_TX_BYTE          32010 /* changed by rdaforno */
//#define T_TX_OFFSET        (rtimer_clock_t)4100  /* 4.100 us */
#define T_TX_OFFSET        4100  /* changed by rdaforno */

#endif /* __SMARTRF_CC430_H__ */

