/**
 * @file   test001.c
 * @author Scott Craig and Justin Tanner
 * @date   Mon Oct 29 16:19:32 2007
 *
 * @brief  Test 001 - sanity test, can we print to UART
 *
 */

#include "../src/kernel.h"
#include "../src/os.h"
#include "../src/led_test.h"
#include "../uart/uart.h"
#include "../trace/trace.h"


CHAN channel_1 = NULL;
CHAN channel_2 = NULL;
CHAN channel_3 = NULL;

CHAN print_channel = NULL;


void test_Recv() {
    int arg = Task_GetArg();
    int value = NULL;

    add_trace(arg, ENTER);
    value = Recv((CHAN)arg);
    add_trace((int)value, EXIT);
    if((int)arg == 3 ){
        Send(print_channel,arg);  
    }
}

void test_Send() {
    int arg = Task_GetArg();
    add_trace(arg+6, ENTER);
    Send((CHAN)arg, (int)arg);
    add_trace(arg+6, EXIT);
}


void test_results() {
    int value = Recv(print_channel);
    if(value > 0){
        char * trace = get_trace();
        char * correct_trace = "(1,(2,(3,(7,7),(8,8),(9,9),1),2),3),";
        UART_print("Trace: %s\n", trace);
        if (strcmp(correct_trace, trace) == 0) {
            UART_print("pass");
        } else {
            UART_print("fail");
        }
    }
  
}

int a_main(void)
{
    /* setup the test */
 
    init_LED_ON_BOARD();
    channel_1 = Chan_Init();
    channel_2 = Chan_Init();
    channel_3 = Chan_Init();

    print_channel = Chan_Init();

    UART_Init0(38400);
    UART_print("\r\nSTART\r\n");
    Task_Create_System(test_results, 0);

    Task_Create_RR(test_Recv , 1);
    Task_Create_RR(test_Recv , 2);
    Task_Create_RR(test_Recv , 3);
    Task_Create_RR(test_Send , 1);
    Task_Create_RR(test_Send , 2);
    Task_Create_RR(test_Send , 3);


    Task_Terminate();
}
