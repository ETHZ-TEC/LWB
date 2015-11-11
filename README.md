# Low-Power Wireless Bus (LWB)

Low-Power Wireless Bus (LWB) is a communication protocol for low-power wireless embedded systems.
LWB lets nodes communicate as if they were connected to a shared bus, where all nodes can receive all packets, although the underlying multi-hop wireless topology may be very complex.
LWB provides an API through which an application can dynamically adjust its traffic demands at runtime.
To serve the current traffic demands in the network, a scheduler computes online a global communication schedule.
All nodes follow this schedule and communicate in a time-triggered fashion.
Thus, LWB's operation is conceptually similar to that of wired busses used in the avionics and automotive industries, such as [CAN](https://en.wikipedia.org/wiki/CAN_bus), [FlexRay](https://en.wikipedia.org/wiki/FlexRay), or [TTP](https://en.wikipedia.org/wiki/Time-Triggered_Protocol).

LWB supports multiple communication patterns, including one-to-many, many-to-one, and many-to-many.
It quickly adapts to changes in the application's traffic demands and is highly resilient to network dynamics.
This entails in particular that LWB's performance and reliability are remarkably unaffected by, for example, the presence of mobile nodes and wireless interference.
Experiments on large multi-hop networks with more than 100 nodes show that LWB's end-to-end packet reliability typically ranges above 99.9%, with energy consumption on par or below state-of-the-art solutions.

## Glossy

LWB uses Glossy as underlying communication and time-synchronization primitive.
Glossy provides two services that are fundamental to LWB's operation: one-to-all network flooding and network-wide time synchronization.
That is, in multi-hop wireless networks Glossy can send a packet from one node to all others within a few milliseconds and at a reliability close to 100%, while synchronizing all nodes to within microsecond accuracy.
Unlike most wireless protocols, Glossy takes advantage of packet collisions rather than fighting against them.
It deliberately forces multiple nodes to send the same packet at nearly the same time, thereby taking advantage of the [capture effect](https://en.wikipedia.org/wiki/Capture_effect) and [constructive interference](https://en.wikipedia.org/wiki/Interference_(wave_propagation)) to harness different forms of diversity.

## Further Reading and Documentation

If you want to learn more about LWB and Glossy, please have a look at our [IPSN'11](https://github.com/ETHZ-TEC/LWB/blob/master/doc/papers/GlossyIPSN11.pdf) and [SenSys'12](https://github.com/ETHZ-TEC/LWB/blob/master/doc/papers/LWBSenSys12.pdf) papers, which include a high-level description of their designs and detailed performance evaluations. Our [MASCOTS'13](https://github.com/ETHZ-TEC/LWB/blob/master/doc/papers/ModelingMASCOTS13.pdf) paper provides further details on LWB's operation and how this can be accurately modeled.
Finally, the code is fairly well documented.

To get a glimpse of what LWB can be used for, you may also check out our work on:

- [Virtus](https://github.com/ETHZ-TEC/LWB/blob/master/doc/papers/VirtusSRDS13.pdf), a protocol that provides [virtual synchrony](https://en.wikipedia.org/wiki/Virtual_synchrony) in multi-hop low-power wireless networks (previously thought [impossible](http://www1.cse.wustl.edu/~lu/papers/pieee03.pdf)), thus enabling dependable applications through replication;
- [predictability](https://github.com/ETHZ-TEC/LWB/blob/master/doc/papers/ModelingMASCOTS13.pdf) of LWB's end-to-end reliability and energy consumption by exploiting the fact that, unlike link-based transmissions, packet losses in Glossy are largely statistically independent;
- [building and deploying](https://github.com/ETHZ-TEC/LWB/blob/master/doc/papers/DeploymentSenSys13.pdf) a wireless nurse call system based on LWB for two weeks during a summer camp for teenagers with [Duchenne muscular dystrophy](https://en.wikipedia.org/wiki/Duchenne_muscular_dystrophy).

## Code

*Disclaimer: Although we tested the code extensively, LWB and Glossy are research prototypes that likely contain bugs. We take no responsibility for and give no warranties in respect of using the code.*

### Status

We currently provide a revised implementation of LWB and the scheduler as described in the original [SenSys'12](https://github.com/ETHZ-TEC/LWB/blob/master/doc/papers/LWBSenSys12.pdf) paper.
We ported the underlying implementation of Glossy to the [CC430 SoC](http://www.ti.com/lsds/ti/microcontrollers_16-bit_32-bit/wireless_mcus/cc430/overview.page), a state-of-the-art platform that integrates an 868/915 MHz transceiver and an ultra-low power MSP430 microcontroller on one chip.
You may readily run a LWB demo application we provide on the [FlockLab](https://www.flocklab.ethz.ch/wiki/) testbed by following the instructions below.

### Layout

LWB and Glossy are implemented based on a minimum subset of the [Contiki](http://www.contiki-os.org/) operating system.
We try to adhere to Contiki's source tree structure and coding conventions.

`apps/lwb` LWB demo application and FlockLab testbed configuration file

`core/net` LWB header and source files, Glossy header file 

`core/net/scheduler` LWB schedulers

`mcu/cc430/` Glossy implementation for CC430

### Future

We intend to provide here also the original Glossy port for the old but still widely used TelosB platform, which features an MSP430F1611 microcontroller and a CC2420 radio, so you can run LWB also on other public testbeds and in the [Cooja/MSPSim](http://www.contiki-os.org/start.html#simulation) simulator. For now, the TelosB port of Glossy is available [here](http://sourceforge.net/p/contikiprojects/code/HEAD/tree/ethz.ch/glossy/).
More generally, we would like to invite the community to help us collect here bug fixes, enhancements, ports to other platforms, etc. related to LWB and Glossy. Please contact us if you have any comments, suggestions, or would like to get involved.

## LWB Demo Application

### Overview

The demo aims to exemplify how an application might use LWB.
To simplify the setup, the demo uses a static LWB scheduler that schedules communication rounds with a fixed round period and every round contains a contention slot.
After joining the bus operation by synchronizing with the host, each source node request one stream such that it sends one data packet per round to the LWB host, which also acts as the sink in this specific setup.

The application code comprises two source files located in `apps/lwb`: `lwb-test.c` and `config.h`.
The latter contains all application-specific parameters, which you may adjust to your needs.
The application logic in `lwb-test.c` runs within its own Contiki process.
The process first starts LWB and then enters its main loop.
LWB controls when the application process continues its execution: 

`PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);`

Once LWB has polled the application process, it may run for no more than a few
hundred milliseconds (i.e., until the beginning of the next LWB round) so it does not interfere with the scheduled execution of LWB and Glossy.

For concurrent, fully decoupled execution of application and communication tasks, you may use a dual-processor platform, where one processor is dedicated to application processing and the other to handling communication tasks while the two asynchronously exchange messages (e.g., data packets) through the [Bolt](https://github.com/ETHZ-TEC/LWB/blob/master/doc/papers/BoltSenSys15.pdf) processor interconnect. More information on Bolt and how to construct your own customized dual-processor platform is available at [http://www.bolt.ethz.ch](http://www.bolt.ethz.ch).

### Building and Flashing the Demo App

The demo application runs on the [Olimex MSP430-CCRF](https://www.olimex.com/Products/MSP430/Starter/MSP430-CCRF/), which features a CC430F5137 SoC.

1. Install the MSP430 toolchain. You may need to enable 'universe'
    (community-maintained free and open-source software) for this step.

    ```
    sudo apt-get install msp430-libc binutils-msp430 gcc-msp430 msp430mcu
    ```  
2. Go to the directory `apps/lwb` and run `make`.

3. Run `flash.sh` to flash the application onto the CC430F5137. This script uses
    the `tilib` driver, which requires `libmsp430.so`, a shared object that is included, for example, in Code Composer Studio 6. You may also need to adjust the path
    to the `libmsp430.so`.
    Alternatively, you can use any other flash tool, such as TI Uniflash.

### Running the Demo App on FlockLab

FlockLab is a public testbed hosted by the [Computer Engineering Group](http://www.tec.ethz.ch/) at ETH Zurich.
Besides Olimex MSP430-CCRF, FlockLab currently supports the TelosB, OpenMote, TinyNode, Opal, and Iris platforms. Please visit [http://www.flocklab.ethz.ch](http://www.flocklab.ethz.ch) to create an account and start using FlockLab. To concretely run the LWB demo application on FlockLab, you should

1. Compile the code and embed the program image into `apps/lwb/locklab-cc430.xml`:

    ```
    make
    make flocklab
    ```

2. Adjust the FlockLab configuration file `flocklab-cc430.xml`. You can set 
    various parameters for your test. The most important ones are:
    
    ```
    <durationSecs>       the duration of your test in seconds
    <obsIds>             the IDs of the involved nodes (observers)
    ```
    
    See [https://www.flocklab.ethz.ch/wiki/wiki/Public/Man/XmlConfig](https://www.flocklab.ethz.ch/wiki/wiki/Public/Man/XmlConfig) for more information. 
    

3. Go to [https://www.flocklab.ethz.ch/user/login.php](https://www.flocklab.ethz.ch/user/login.php) and log in with your username and password. Select 'Validate XML Test
    Config' in the menu on the right. Choose the file `flocklab-cc430.xml` and
    validate it. Unless there are errors, click on 'Create Test' to schedule your 
    test. Go to 'Manage Tests' to get an overview of all your tests. Once 
    your test has finished, you can download or preview the results.
    
## Research

Glossy and LWB were developed at the [Computer Engineering Group](http://www.tec.ethz.ch/) at [ETH Zurich](https://www.ethz.ch/en.html). The following people have contributed to their design and implementation: [Federico Ferrari](https://ch.linkedin.com/in/fferrari), [Marco Zimmerling](http://www.tik.ee.ethz.ch/~marcoz/), [Reto Da Forno](http://ch.linkedin.com/in/rdaforno), [Luca Mottola](http://home.deib.polimi.it/mottola/), [Lothar Thiele](http://www.tik.ee.ethz.ch/~thiele/pmwiki/pmwiki.php/Site/Home), and [Tonio Gsell](https://github.com/tgsell).
