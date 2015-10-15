# Low-power Wireless Bus (LWB)

Low-power Wireless Bus (LWB) is an efficient and adaptive communication protocol for wireless embedded systems, where nodes communicate as if they were conntected to a shared bus. It provides a high-level API that allows an application to dynamically submit, update, and relinquish communication demands in the form of packet *streams*. An exchangeable *scheduler* orchestrates communication over the wireless bus based on the current communication demands. Unlike other multi-hop protocols, LWB supports multiple diverse traffic patterns (e.g., one-to-many, many-to-one, and many-to-many) and seamlessly adapts to unpredictable network dynamics (e.g., mobile nodes and link fluctuations). At the same time, communication in LWB is highly reliable, energy-efficient, and predictable in terms of end-to-end packet delivery and energy costs.

## Glossy

LWB uses Glossy as underlying communication and time-synchronization primitive. Glossy is a fast and reliable flooding protocol that allows a node to send a packet to all other nodes in a multi-hop wireless network. Optionally, Glossy can synchronize all nodes to within microsecond accuracy at nearly zero additional cost. Unlike most wireless protocols, Glossy takes advantage of packet collisions rather than fighting against them. It deliberately lets nodes send the same packet at the same time, thereby harnessing different forms of diversity.

## Further Reading and Documentation

If you want to learn more about LWB and Glossy, please have a look at the original research papers published at [IPSN'11](ftp://ftp.tik.ee.ethz.ch/pub/people/ferrarif/FZTS2011.pdf) and [SenSys'12](ftp://ftp.tik.ee.ethz.ch/pub/people/ferrarif/FZMT2012.pdf), which also provide a high-level overview of their designs and detailed performance evaluations. For further details on LWB's design and protocol parameters, you may also check out our [MASCOTS'13 paper](ftp://ftp.tik.ee.ethz.ch/pub/people/marcoz/ZFMT2013.pdf) and the [Blink technical report](ftp://ftp.tik.ee.ethz.ch/pub/publications/TIK-Report-356.pdf). The code itself is fairly well documented using [Doxygen](http://www.stack.nl/~dimitri/doxygen/).

Other research artifacts we have built based on LWB include:
- [Virtus](ftp://ftp.tik.ee.ethz.ch/pub/people/ferrarif/FZMT2013.pdf), a protocol that provides [virtual-synchrony guarantees](https://en.wikipedia.org/wiki/Virtual_synchrony) in multi-hop low-power wireless networks, enabling dependable applications;
- [Blink](ftp://ftp.tik.ee.ethz.ch/pub/publications/TIK-Report-356.pdf), a protocol that provides energy-efficient real-time communication in multi-hop low-power wireless networks to support applications with real-time requirements, such as [Cyber-physical Systems (CPS)](https://www.google.de/url?sa=t&rct=j&q=&esrc=s&source=web&cd=4&cad=rja&uact=8&ved=0CDIQFjADahUKEwjd17qXx5nIAhXmjHIKHd5UD6k&url=https%3A%2F%2Fen.wikipedia.org%2Fwiki%2FCyber-physical_system&usg=AFQjCNHEv3w1SdMsXfsWwK3ALj4gJG_rEg&sig2=vpyxY-vJ2VMgg_AhMD_G1Q); and
- [work](ftp://ftp.tik.ee.ethz.ch/pub/people/marcoz/ZFMT2013.pdf) demonstrating that end-to-end packet delivery and energy consumption are highly predictable in LWB, which is also due to the empirical finding that, different from traditional link-based multi-hop protocols, packet receptions and losses in Glossy are largely statistically independent.

We also built a [wireless nurse call system](ftp://ftp.tik.ee.ethz.ch/pub/people/marcoz/ZFLSSDSW2013.pdf) based on LWB, and deployed it for two weeks during a summer camp for teenagers with [Duchenne muscular dystrophy](https://en.wikipedia.org/wiki/Duchenne_muscular_dystrophy).

## Code

### Disclaimer

Glossy and LWB are research prototypes. We provide the code 'as is,' no warranties are given. 
Use it at your own risk. 

### Layout

Both LWB and Glossy are based on the [Contiki](http://www.contiki-os.org/) operating system, and we try to adhere to Contiki's source tree structure and coding conventions.

`apps/lwb` LWB demo application and FlockLab testbed configuration file

`core/net` LWB header and source files, Glossy header file 

`core/net/scheduler` LWB schedulers

`mcu/cc430/` Glossy implementation for CC430

### Status

Currently, we provide an implementation of Glossy for the CC430 SoC platform from Texas Instruments, as well as a completely revised implementation of LWB and the scheduler presented in the original [SenSys'12 paper](ftp://ftp.tik.ee.ethz.ch/pub/people/ferrarif/FZMT2012.pdf). You may run the code on the [FlockLab](https://www.flocklab.ethz.ch/wiki/) testbed by following the instructions below.
We intend to adapt and include the Glossy port for the MSP430F1611 microcontroller and the CC2420 radio (currently available [here](http://sourceforge.net/p/contikiprojects/code/HEAD/tree/ethz.ch/glossy/)), in the near future, so you can run LWB also on other testbeds featuring TelosB nodes and in the Cooja simulator. We are also working toward a release of the Virtus and Blink code.

Please feel free to contact us if you have any comments, would like to submit bug reports, suggest improvements, or contribute your own code.

## LWB Demo Application

### Overview

The demo application exemplifies how one could use LWB. The default scheduler for this demo is a static one, that, the round 
period is fixed and there is one contention slot in each round. Each source
node request one stream and sends one data packet per round to the host node, which also serves as sink in this setup. The application code comprises two source files located in `apps/lwb`, `lwb-test.c` and `config.h`. The latter contains all application-specific parameters and you may adjust it to your needs. The application code 
itself in `lwb-test.c` is straightforward. A contiki process (i.e., application task) is defined,
which starts LWB and then enters its main loop. An important thing to 
note is that LWB controls the entire program flow, that is, the application
task may only run if LWB permits it. This supervision function is achieved 
through the following:

`PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);`

Once LWB has polled the application task, it may run for no more than a few
hundred milliseconds, to ensure proper operation of LWB. 

### Building the Demo App

The provided demo application runs on the Olimex MSP430-CCRF only.

1. Install the msp430 toolchain. Note that you may need to enable 'universe'
    (community-maintained free & open-source software) for this step:
    
    ```
    sudo apt-get install msp430-libc binutils-msp430 gcc-msp430 msp430mcu
    ```
    
1. Go to the directory `apps/lwb` and run `make`.

2. To flash the application onto the CC430F5137, run `flash.sh`. This script uses
    the tilib driver, which requires `libmsp430.so`, a shared object that is included, for example, in Code Composer Studio 6. You may also need to adjust the path
    to the `libmsp430.so`.
    Alternatively, you can use any other flash tool (e.g., TI Uniflash) to 
    program the target.


### Running the Demo App on FlockLab

1. Compile the code and embed the program image into `apps/lwb/locklab-cc430.xml`:

    ```
    make
    make flocklab
    ```

2. Adjust the FlockLab configuration file `flocklab-cc430.xml`. You can set 
    various parameters for your test. The most important parameters are:
    
    ```
    <durationSecs>       the duration of your test in seconds
    <obsIds>             the IDs of the involved nodes (observers)
    ```
    
    Visit www.flocklab.ethz.ch/wiki/wiki/Public/Man/XmlConfig for more info. 
    If you don't set a start time for your test, it will be scheduled ASAP. 
    IMPORTANT: Since only one user can run a test on FlockLab at a time, the 
    system relies on 'fair play'. Do not run long tests (> 1h) during daytime 
    (UTC 06:00 - 18:00) to enable other users to run their tests.

3. Go to www.flocklab.ethz.ch and select 'Use FlockLab'. If you are not yet 
    familiar with FlockLab, you should at least go through the tutorials and 
    the quick start guide. If you don't have a login yet, you can register a 
    new account on this page: www.user.flocklab.ethz.ch. Once your ready to
    schedule your test, login with your username and select 'Validate XML Test
    Config' in the menu on the right. Choose the file "flocklab-cc430.xml" and
    validate it. If no errors pop up, click 'Create Test' to schedule your 
    test. Go to 'Manage Tests' to get an overview of all your tests. Once 
    your test has finished, you can download or preview the results.
    
## Research

Glossy and LWB were developed at the [Computer Engineering Group](http://www.tec.ethz.ch/) at [ETH Zurich](https://www.ethz.ch/en.html). The following people have contributed to their design and implementation: [Federico Ferrari](https://ch.linkedin.com/in/fferrari), [Marco Zimmerling](http://www.tik.ee.ethz.ch/~marcoz/), [Reto Da Forno](http://ch.linkedin.com/in/rdaforno), [Luca Mottola](http://home.deib.polimi.it/mottola/), [Lothar Thiele](http://www.tik.ee.ethz.ch/~thiele/pmwiki/pmwiki.php/Site/Home), and [Tonio Gsell](https://github.com/tgsell).
