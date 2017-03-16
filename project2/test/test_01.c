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

void round_robin(void)
{
    for(;;)
    {   
        led_toggle(LED_ON_BOARD);
        // add_to_trace(1);s
        UART_print((uint8_t*)"\r\nRR\r\n", 9);

        _delay_ms(100);
        Task_Next();
    }
}

int a_main(void)
{
    /* setup the test */
    UART_Init0(57600);
    UART_print("\ntest begin\n");
    init_LED_ON_BOARD();


    Task_Create_RR(round_robin , 1);
    Task_Create_RR(round_robin , 2);
    Task_Create_RR(round_robin , 3);
    Task_Create_RR(round_robin , 4);
    Task_Create_RR(round_robin , 5);


    // print_trace();

    Task_Terminate();
}
