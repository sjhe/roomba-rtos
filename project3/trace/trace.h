#ifndef __TRACE_H__
#define __TRACE_H__

#include <stdio.h>
#include <avr/common.h>

#include "../uart/uart.h"
#include "../src/os.h"

#define MAX_TRACE_LENGTH 64

typedef enum {
	ENTER = 0, EXIT
} trace_event;

typedef struct trace_entry {
	uint16_t process_number;
	trace_event event;
} trace_entry;

// Add a new trace entry
void add_trace(uint16_t process_number, trace_event event);

// Dump the trace to a string
void print_trace(void);

char * get_trace();

void set_error_handler(void (*funct)(void));

#endif
