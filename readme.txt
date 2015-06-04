Notes on the code:

- Whenever a new source file is added, it must be referenced in "Makefile.include" (PLATSRCS).
- Each header file must be included from the main header file ("core/contiki.h").
- Each source code file only includes the main header file "contiki.h".
- The main function and program entry point along with the initialization procedure is defined in "contiki-cc430-main.c".
- All high-level definitions are in "contiki-conf.h".
- All hardware-related definitions are in the file "hal.h" in the MCU specific subfolder in the "platform" directory.


Notes on the folder structure:

- All files specific to the MCU and board are in the folder "platform". Thus, a seperate subfolder is necessary for each supported MCU.
- All platform-idendependent system files are in the "core".
- The application itself should be placed into a subfolder in the "apps" directory.