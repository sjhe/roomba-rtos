/**
 * @file   test8.c
 * @author 
 * @date   
 *
 * @brief  Test 8 - Testing the order of the round robin tasks
 *
 */

#include "../src/kernel.h"
#include "../src/os.h"
#include "../src/led_test.h"
#include "../uart/uart.h"
#include "../trace/trace.h"


CHAN channel_id = NULL;

void round_robin(void)
{
    int arg = -1;
    arg = Task_GetArg();
    UART_print("RR %d\n", (int)arg);
    for(;;)
    {   
        add_trace(arg, ENTER);
        if (arg == 5) {
            Send(channel_id, 1 );
        }
        add_trace(arg, EXIT);
        led_toggle(LED_ON_BOARD);
        // add_to_trace(1);s
        _delay_ms(50);
        Task_Next();
    }
}

void test_results() {
    int value = Recv(channel_id);
    if(value > 0){
        char * trace = get_trace();
        char * correct_trace = "(1,1),(2,2),(3,3),(4,4),(5,5),";
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
    channel_id = Chan_Init();
    if(channel_id == 1){
        UART_Init0(38400);
        UART_print("\r\nSTART\r\n");
        Task_Create_System(test_results, 0);
        Task_Create_RR(round_robin , 1);
        Task_Create_RR(round_robin , 2);
        Task_Create_RR(round_robin , 3);
        Task_Create_RR(round_robin , 4);
        Task_Create_RR(round_robin , 5);
    }
    Task_Terminate();
}
