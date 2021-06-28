## General notes

The code is based on a modified version of Contiki 2.7 and runs on the CC430 only (DPP1 and DPP2).  

To install the required MSP430 toolchain, run:

    sudo apt install gcc-msp430

## Apps

There are several apps in this repository:
*  `apps/archive`: obsolete stuff
*  `apps/hello-world`:  minimal example
*  `apps/glossy`:  a Glossy test application
*  `apps/lwb`: a basic LWB example
*  `apps/elwb-dev`: modified version of the eLWB, used in our deployment


## Glossy test

To run a simple Glossy link test on FlockLab, adjust the parameters in `config.h` and then run `make` and `make flocklab_xml` in the directory `apps/glossy`. The most important parameters in `config.h` are:
```
#define HOST_ID             1                         // ID of the host node (initiator)
#define RF_CONF_TX_CH       5                         // radio channel (5 = 869 MHz)
#define RF_CONF_TX_POWER    RF1A_TX_POWER_PLUS_10_dBm // TX power
#define GLOSSY_PERIOD       (RTIMER_SECOND_HF / 10)   // duration of a round in ticks (RTIMER_SECOND_HF / 10 = 0.1s)
#define GLOSSY_T_SLOT       (RTIMER_SECOND_HF / 20)   // duration of a slot in ticks (RTIMER_SECOND_HF / 20 = 0.05s)
#define GLOSSY_N_TX         3                         // number of retransmissions per flood
#define GLOSSY_PAYLOAD_LEN  8                         // packet payload length in bytes
```
Note that Glossy adds a 2 byte header to the packet and the radio itself adds another 3 bytes (length and CRC). The total transmission time for one packet or one flood can be calculated as follows:  
```
T_PACKET = (3 + 2 + GLOSSY_PAYLOAD_LEN) * 32us + 300us

T_FLOOD  = (N_HOPS + (2 * GLOSSY_N_TX) - 2) * T_PACKET
```
The 300us are added to account for the preamble (4 bytes), the sync word (4 bytes) and some constant radio overhead. The modulation used is 2-GFSK.  

