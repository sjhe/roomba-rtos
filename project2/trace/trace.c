#include "trace.h"

trace_entry trace[MAX_TRACE_LENGTH];
char str_trace[MAX_TRACE_LENGTH * 3 + 1];
uint16_t trace_length = 0;

void add_trace(uint16_t process_number, trace_event event) {
    if (trace_length >= MAX_TRACE_LENGTH) {
        return;
    }

    trace[trace_length].process_number = process_number;
    trace[trace_length].event = event;
    trace_length++;

}

char * get_trace() {
    int i;
    int char_count = 0;
    for (i = 0; i < trace_length; i++) {
        if (trace[i].event == ENTER) {
//          str_trace[3 * i] = '(';
//          str_trace[3 * i + 1] = 48 + trace[i].process_number;
//          str_trace[3 * i + 2] = ',';
            char_count += sprintf(str_trace + char_count, "(%d,", trace[i].process_number);
        } else {
//          str_trace[3 * i] = 48 + trace[i].process_number;
//          str_trace[3 * i + 1] = ')';
//          str_trace[3 * i + 2] = ',';
            char_count += sprintf(str_trace + char_count, "%d),", trace[i].process_number);
        }
    }
    str_trace[3 * i] = '\0';
    return str_trace;
}

// void set_error_handler(void (*funct)(void)) {
//     os_err_handler = funct;
// }

void print_trace(void) {
    uint16_t i;
    for (i = 0; i < trace_length; i++) {
        // Print process number, preceded by "(" if entering or followed by ")" if exiting
        UART_print("%s%d%s,", trace[i].event == ENTER ? "(" : "", trace[i].process_number, trace[i].event == EXIT ? ")" : "");
    }
}
