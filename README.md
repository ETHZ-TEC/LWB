# DPP - Communication Processor [TODO: change name]

Software based on the Contiki OS for the communication processor of the dual 
processor platform. [TODO: adequate intro text / project description]


## Demo Application Description

The demo application shows how to use the Low-Power Wireless Bus, a protocol
based on Glossy. The scheduler used in this demo is static, i.e. the round 
period is constant and there is one contention slot in each round. Each source
node allocates one stream and sends one data packet per round to the host node.
The application code is located in the directory "apps/lwb". There are two 
source files: "lwb-test.c" and "config.h". The latter contains all application
specific parameters and you may adjust it to your needs. The program code 
itself is straight forward: A contiki process ('application task') is defined,
which starts the LWB and then enters its 'main loop'. An important thing to 
notice is the fact that the LWB controls the program flow, i.e. the application
task may only run if the LWB allows it. This supervision function is achieved 
with the following line of code:
PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
Once the LWB has polled the application task, it may run for no more than a few
hundred milliseconds to ensure proper operation of the LWB. 


## Compile and Flash the Demo Application

To use the commands below you need to install Code Composer Studio v6 from TI.
Note: The provided demo application runs on the Olimex MSP430-CCRF only.

1. Install the msp430 toolchain. Note that you may need to enable 'universe'
    (community-maintained free & open-source software) for this step:
    
    ```
    sudo apt-get install msp430-libc binutils-msp430 gcc-msp430 msp430mcu
    ```
    
1. Go to the directory 'apps/lwb' and run make.

2. To flash the application onto the CC430F5137, run flash.sh. This script uses
    the tilib driver, which requires libmsp430.so, a shared object that is e.g.
    included in Code Composer Studio 6. You may also need to adjust the path
    to the libmsp430.so.
    Alternatively, you can use any other flash tool (e.g. TI Uniflash) to 
    program the target.


## Run the Demo Application on FlockLab

1. Compile the code and embed the proram image into "flocklab-cc430.xml":

    ```
    make
    make flocklab
    ```

2. Adjust the FlockLab configuration file "flocklab-cc430.xml". You can set 
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


## Disclaimer

The software is provided 'as is', no warranties are given. 
Use at your own risk. 
Note that the software has not yet been thoroughly tested.
