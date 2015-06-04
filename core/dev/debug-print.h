#ifndef __DEBUG_PRINT_H__
#define __DEBUG_PRINT_H__

#include "contiki.h"

// CONFIG

#ifndef MAX_PROTOCOL_LENGTH
#define MAX_PROTOCOL_LENGTH 5
#endif
#ifndef MAX_CONTENT_LENGTH
#define MAX_CONTENT_LENGTH  80
#endif
#ifndef N_DEBUG_MSG
#define N_DEBUG_MSG 8
#endif


#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL                 DEBUG_LVL_VERBOSE
#endif // DEBUG_LEVEL

// set debugging level for each module (0 = no debug prints)
#define DEBUG_PRINT(level, time, title, ...)     if (DEBUG_LEVEL >= level) { DEBUG_PRINT_MSG(time, title, __VA_ARGS__); }
#ifdef LED_STATUS
    #define DEBUG_PRINT_ERROR(...)      if (DEBUG_LEVEL >= DEBUG_LVL_ERROR) { DEBUG_PRINT_MSG(0, 'E', __VA_ARGS__); LED_OFF(LED_STATUS); }
#else
    #define DEBUG_PRINT_ERROR(...)      if (DEBUG_LEVEL >= DEBUG_LVL_ERROR) { DEBUG_PRINT_MSG(0, 'E', __VA_ARGS__); }
#endif // LED_STATUS
#define DEBUG_PRINT_WARNING(...)    if (DEBUG_LEVEL >= DEBUG_LVL_WARNING) { DEBUG_PRINT_MSG(0, 'W', __VA_ARGS__); }
#define DEBUG_PRINT_INFO(...)       if (DEBUG_LEVEL >= DEBUG_LVL_INFO) { DEBUG_PRINT_MSG(0, 'I', __VA_ARGS__); }
#define DEBUG_PRINT_VERBOSE(...)    if (DEBUG_LEVEL >= DEBUG_LVL_VERBOSE) { DEBUG_PRINT_MSG(0, 'V', __VA_ARGS__); }

// prints immediately to the UART interface, no formatting (and no newline) or debug level check
#ifdef MUX_SEL
    #define DEBUG_PRINT_NOW(...)    { if (USCI_A0_SPI_MODE) { uart0_reinit(); } printf(__VA_ARGS__); }
#else
    #define DEBUG_PRINT_NOW(...)    printf(__VA_ARGS__)
#endif

#ifdef DEBUG_TASK_ACT_PIN
    #define DEBUG_TASK_ACTIVE       LED_ON(DEBUG_TASK_ACT_PIN)
    #define DEBUG_TASK_SUSPENDED    LED_OFF(DEBUG_TASK_ACT_PIN)
#else
    #define DEBUG_TASK_ACTIVE       
    #define DEBUG_TASK_SUSPENDED    
#endif


// debug levels
enum {
    DEBUG_LVL_OFF = 0,  
    DEBUG_LVL_ERROR = 1,
    DEBUG_LVL_WARNING = 2,
    DEBUG_LVL_INFO = 3,
    DEBUG_LVL_VERBOSE = 4,
    NUM_OF_DEBUG_LEVELS
} debug_level_t;

extern char content[DEBUG_PRINT_MAX_LENGTH];

typedef struct debug_print_t {
	struct debug_print_t *next;
	rtimer_clock_t time;
	char protocol;
	char content[DEBUG_PRINT_MAX_LENGTH];
} debug_print_t;

#if DEBUG_PRINT_CONF_ON
#define DEBUG_PRINT_MSG(t, p, ...) \
	do { \
		snprintf(content, MAX_CONTENT_LENGTH + 1, ##__VA_ARGS__); \
		debug_print_msg(t, p, content); \
	} while (0)
#else
#define DEBUG_PRINT_MSG(t, p, ...)
#endif /* DEBUG_PRINT_CONF_ON */

void debug_print_init(void);
void debug_process_poll(void);
void debug_print_msg(rtimer_clock_t *time, char protocol, char *content);

#endif /* __DEBUG_PRINT_H__ */
