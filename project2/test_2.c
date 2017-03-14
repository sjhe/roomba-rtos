/**
 * @file   test_2.c
 * @author 
 * @date   
 * 
 * @brief  Test 002 - Send /Recv
 * 
 */
#include "kernel.h"
#include "os.h"
#include "led_test.h"


CHAN channel_id = NULL;

void setup () {
  init_LED_ON_BOARD();
  init_LED_PING();
  init_LED_ISR();
}
/*============
* A Simple Test 
* ============
*/

void Ping() 
{
  unsigned int index = 0;
  for(;;){
    Send(channel_id, index++);
    index = index % 2 ;
    Task_Next();
  }
}
// Pong
void Pong() 
{
  unsigned int index = 0;
  // disable_LEDs();
  for(;;){
    if(index == 0){
      led_toggle(LED_ON_BOARD);
    }else{
      led_toggle(LED_PING);
    }

    index = Recv(channel_id);

    Task_Next();
  }
}

void a_main(void)
{
  setup();
  channel_id = Chan_Init();
  int i = 0 ;

  if(channel_id != NULL){
    enable_LED(LED_PING);
    for(i = 0 ; i < channel_id; i++){
      _delay_ms(1);
    }
    disable_LEDs();

    Task_Create_System( Pong, 1 );
    Task_Create_System( Ping, 2 );
  }

  channel_id = Chan_Init();

  enable_LED(LED_PING);
  for(i = 0 ; i < channel_id; i++){
      _delay_ms(1);
    }
  disable_LEDs();

  

  Task_Terminate();

}
