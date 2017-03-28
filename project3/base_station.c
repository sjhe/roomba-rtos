#include "src/os.h"
#include "src/led_test.h"
#include <avr/io.h>
#include "uart/uart.h"
#include <string.h>
#include <stdio.h>

#define JOYSTICK_S_X_PIN 0		// analog 0
#define JOYSTICK_S_Y_PIN 1		// analog 1
#define JOYSTICK_R_X_PIN 2		// analog 2
#define JOYSTICK_R_Y_PIN 3		// analog 3

#define JOYSTICK_S_BUTTON_PIN 4
#define JOYSTICK_R_BUTTON_PIN 36

void setup_controllers()
{
    /* We use a single ADC onboard the package, using an
       onbaord multiplexer to select one of the 16 available
       analog inputs. Joystck channels are connected as follows
         0 -  3 Controller 1.  0/ 1 Left X/Y,  2/ 3 Right X/Y.  
         4 -  7 Controller 2.  4/ 5 Left X/Y,  6/ 7 Right X/Y.  
         8 - 11 Controller 3.  8/ 9 Left X/Y, 10/11 Right X/Y.  
        12 - 15 Controller 4. 12/13 Left X/Y, 14/15 Right X/Y.
       Each Controller also features a single push button connected
       to digital inputs as follows, all pins are on PORTC bits 0 to 3
        37 - PORTC0 - Controller 1 : Button A
        36 - PORTC1 - Controller 2 : Button A
        35 - PORTC2 - Controller 3 : Button A
        34 - PORTC3 - Controller 4 : Button A
    */
	
	/* Configure PORTC to received digital inputs for bits 0 to 3 */
	DDRC = (DDRC & 0xF0);
	
	/* Configure Analog Inputs using ADC */
    ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Set ADC prescalar to 128 - 125KHz sample rate @ 16MHz

    ADMUX |= (1 << REFS0); // Set ADC reference to AVCC
    ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading

    ADCSRA |= (1 << ADEN);  // Enable ADC
	ADCSRA |= (1 << ADSC); //Start a conversion to warmup the ADC.
	
	//We're using the ADC in single Conversion mode, so this is all the setup we need. 
}

/**
 * Read an analog value from a given channel. 
 * On the AT mega2560, there are 16 available channels, thus
 * channel can be any value 0 to 15, which will correspond
 * to the analog input on the arduino board. 
 */
int read_analog(uint8_t channel)
{
	/* We're using Single Ended input for our ADC readings, this requires some
	 * work to correctly set the mux values between the ADMUX and ADCSRB registers. 
	 * ADMUX contains the four LSB of the multiplexer, while the fifth bit is kept
	 * within the ADCSRB register. Given the specifications, we want to keep the 
	 * three least significant bits as is, and check to see if the fourth bit is set, if it
	 * is, we need to set the mux5 pin. 
	 */
	
	/* Set the three LSB of the Mux value. */
	/* Caution modifying this line, we want MUX4 to be set to zero, always */
    ADMUX = (ADMUX & 0xF0 ) | (0x07 & channel); 
	/* We set the MUX5 value based on the fourth bit of the channel, see page 292 of the 
	 * ATmega2560 data sheet for detailed information */
	ADCSRB = (ADCSRB & 0xF7) | (channel & (1 << MUX5));
	
    /* We now set the Start Conversion bit to trigger a fresh sample. */
    ADCSRA |= (1 << ADSC);
    /* We wait on the ADC to complete the operation, when it completes, the hardware
       will set the ADSC bit to 0. */
    while ((ADCSRA & (1 << ADSC)));
    /* We setup the ADC to shift input to left, so we simply return the High register. */
    return ADCH;
}

void test_periodic(){
	for(;;){
		led_toggle(LED_ON_BOARD);
		Task_Next();
	}
}

void test_ping(){
	for(;;){
		led_toggle(LED_PING);
		Task_Next();
	}
}

int readAnalog(unsigned int pin_mask)
{

	return 0;
}

void createRoombaCommand(char* dest, char* inputCommand, int rx, int ry)
{
		strcat(dest, inputCommand);

		char speedBuffer[4] = "";
		strcat(dest, ",");
		sprintf(speedBuffer, "%d", rx);
		strcat(dest, speedBuffer);

		speedBuffer[0] = '\0';
		strcat(dest, ",");
		sprintf(speedBuffer, "%d", ry);
		strcat(dest, speedBuffer);

		strcat(dest, "*");  
}

void roombaTask()
{
	char bt_command[16] = "";
	char bt_last_command[16] = "";
	for(;;){
		int joystick_rx = read_analog(JOYSTICK_R_X_PIN);
		int joystick_ry = read_analog(JOYSTICK_R_Y_PIN);
		
		bt_command[0] = '\0';
		createRoombaCommand(bt_command, "r", joystick_rx, joystick_ry);

		if (strcmp(bt_last_command, bt_command) != 0)
		{
			UART_print("%s\n", bt_command);
			Bluetooth_Send_String(bt_command);
		}
		
		strcpy(bt_last_command, bt_command);

		led_toggle(LED_PING);
		Task_Next();
	}
}

void createServoCommand(char* dest, char* inputCommand, int sx, int sy, int laserState)
{
		strcat(dest, inputCommand);

		char speedBuffer[4] = "";
		strcat(dest, ",");
		sprintf(speedBuffer, "%d", sx);
		strcat(dest, speedBuffer);

		speedBuffer[0] = '\0';
		strcat(dest, ",");
		sprintf(speedBuffer, "%d", sy);
		strcat(dest, speedBuffer);
	
		speedBuffer[0] = '\0';
		strcat(dest, ",");
		sprintf(speedBuffer, "%d", laserState);
		strcat(dest, speedBuffer);

		strcat(dest, "*");  
}

void servoTask()
{
	char bt_command[16] = "";
	char bt_last_command[16] = "";
	for (;;)
	{
		int joystick_sx = read_analog(JOYSTICK_S_X_PIN);
		int joystick_sy = read_analog(JOYSTICK_S_Y_PIN);
		int joystick_button = read_analog(JOYSTICK_S_BUTTON_PIN) > 0 ? 1 : 0;

		bt_command[0] = '\0';
		createServoCommand(bt_command, "s", joystick_sx, joystick_sy, joystick_button);

		if (strcmp(bt_last_command, bt_command) != 0)
		{
			UART_print("%s\n", bt_command);
			Bluetooth_Send_String(bt_command);
		}
		
		strcpy(bt_last_command, bt_command);

		Task_Next();
	}
}

void a_main()
{
//	init_pins();
	Bluetooth_UART_Init(); 
	UART_Init0(9600);
    UART_print("\r\nSTART\r\n");

	setup_controllers();
	init_LED_ON_BOARD();
	init_LED_PING();

	Task_Create_Period(servoTask, 0, 15, 8, 0);
//	Task_Create_Period(test_periodic, 0, 15, 4, 0);
//	Task_Create_Period(test_ping, 0, 15, 1, 5);
	Task_Create_Period(roombaTask, 0, 15, 7, 8);

}
