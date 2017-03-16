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
  unsigned int index = 1;
  for(;;){

    if(channel_id == 1){
      if(index == 1){
        index = 0;
        Send(channel_id, 0);
      }else{
        index = 1;
        Send(channel_id, 1);
      }
    }
      Task_Next();
  }
}
// Pong
void Pong() 
{
  unsigned int index = 0;
  // disable_LEDs();
  for(;;){
    index = Recv(channel_id);
    if(index == 0){
      enable_LED(LED_ON_BOARD);
      _delay_ms(50);
    }

    Task_Next();
  }
}

// Pong
void DisableAll() 
{
  unsigned int index = 0;
  // disable_LEDs();
  for(;;){
    index = Recv(channel_id);
    if(index == 1){
      disable_LEDs();
      _delay_ms(50);
    }
    Task_Next();
  }
}

void a_main(void)
{
  setup();
  channel_id = Chan_Init();
  int i = 0 ;

  if(channel_id != NULL){
    Task_Create_System( Pong, 1 );
    Task_Create_System( Ping, 2 );
    Task_Create_System( DisableAll, 3 );
  }

  // channel_id = Chan_Init();

  // enable_LED(LED_PING);
  // for(i = 0 ; i < channel_id; i++){
  //     _delay_ms(1);
  //   }
  // disable_LEDs();

  

  Task_Terminate();

}
