#ifndef __DEBUG_PRINT_H__
#define __DEBUG_PRINT_H__


// CONFIG

#ifndef MAX_PROTOCOL_LENGTH
#define MAX_PROTOCOL_LENGTH             5
#endif

#ifndef DEBUG_PRINT_CONF_BUFFER_SIZE
#define DEBUG_PRINT_CONF_BUFFER_SIZE    8       /* number of messages to store */
#endif

#ifndef DEBUG_PRINT_CONF_MAX_LENGTH
#define DEBUG_PRINT_CONF_MAX_LENGTH     80      /* number of chars per message */
#endif


#ifndef DEBUG_PRINT_CONF_LEVEL
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_VERBOSE
#endif // DEBUG_PRINT_CONF_LEVEL

// set debugging level for each module (0 = no debug prints)
#define DEBUG_PRINT(level, time, title, ...)     if (DEBUG_PRINT_CONF_LEVEL >= level) { DEBUG_PRINT_MSG(time, title, __VA_ARGS__); }
#ifdef LED_STATUS
    #define DEBUG_PRINT_ERROR(...)      if (DEBUG_PRINT_CONF_LEVEL >= DEBUG_PRINT_LVL_ERROR) { DEBUG_PRINT_MSG(0, 'E', __VA_ARGS__); LED_OFF(LED_STATUS); }
#else
    #define DEBUG_PRINT_ERROR(...)      if (DEBUG_PRINT_CONF_LEVEL >= DEBUG_PRINT_LVL_ERROR) { DEBUG_PRINT_MSG(0, 'E', __VA_ARGS__); }
#endif // LED_STATUS
#define DEBUG_PRINT_WARNING(...)    if (DEBUG_PRINT_CONF_LEVEL >= DEBUG_PRINT_LVL_WARNING) { DEBUG_PRINT_MSG(0, 'W', __VA_ARGS__); }
#define DEBUG_PRINT_INFO(...)       if (DEBUG_PRINT_CONF_LEVEL >= DEBUG_PRINT_LVL_INFO) { DEBUG_PRINT_MSG(0, 'I', __VA_ARGS__); }
#define DEBUG_PRINT_VERBOSE(...)    if (DEBUG_PRINT_CONF_LEVEL >= DEBUG_PRINT_LVL_VERBOSE) { DEBUG_PRINT_MSG(0, 'V', __VA_ARGS__); }

// prints immediately to the UART interface, no formatting (and no newline) or debug level check
#ifdef MUX_SEL
    #define DEBUG_PRINT_NOW(...)    { if (USCI_A0_SPI_MODE) { uart0_reinit(); } printf(__VA_ARGS__); }
#else
    #define DEBUG_PRINT_NOW(...)    printf(__VA_ARGS__)
#endif

#ifdef DEBUG_PRINT_TASK_ACT_PIN
    #define DEBUG_PRINT_TASK_ACTIVE       LED_ON(DEBUG_PRINT_TASK_ACT_PIN)
    #define DEBUG_PRINT_TASK_SUSPENDED    LED_OFF(DEBUG_PRINT_TASK_ACT_PIN)
#else
    #define DEBUG_PRINT_TASK_ACTIVE       
    #define DEBUG_PRINT_TASK_SUSPENDED    
#endif


// debug levels (severity level)
enum {
    DEBUG_PRINT_LVL_OFF = 0,  
    DEBUG_PRINT_LVL_ERROR = 1,
    DEBUG_PRINT_LVL_WARNING = 2,
    DEBUG_PRINT_LVL_INFO = 3,
    DEBUG_PRINT_LVL_VERBOSE = 4,
    NUM_OF_DEBUG_PRINT_LEVELS
} debug_level_t;

extern char content[DEBUG_PRINT_CONF_MAX_LENGTH + 1];   /* +1 for the trailing \0 character */

#define DEBUG_PRINT_MSG_HEADER_SIZE 11
typedef struct debug_print_t {
    struct debug_print_t *next;
    rtimer_clock_t time;
    char protocol;
    char content[DEBUG_PRINT_CONF_MAX_LENGTH];  /* no need to store the trailing \0 char */ 
} debug_print_t;

#if DEBUG_PRINT_CONF_ON
#define DEBUG_PRINT_MSG(t, p, ...) \
		snprintf(content, DEBUG_PRINT_CONF_MAX_LENGTH + 1, ##__VA_ARGS__); \
		debug_print_msg(t, p, content)
#else
#define DEBUG_PRINT_MSG(t, p, ...)
#endif /* DEBUG_PRINT_CONF_ON */

void debug_print_init(void);
void debug_process_poll(void);
void debug_print_msg(rtimer_clock_t *time, char protocol, char *content);

#endif /* __DEBUG_PRINT_H__ */
