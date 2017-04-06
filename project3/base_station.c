#include "src/os.h"
#include "src/led_test.h"
#include <avr/io.h>
#include "uart/uart.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

#define JOYSTICK_S_X_PIN 0		// analog 0
#define JOYSTICK_S_Y_PIN 1		// analog 1
#define JOYSTICK_R_X_PIN 2		// analog 2
#define JOYSTICK_R_Y_PIN 3		// analog 3

#define JOYSTICK_S_BUTTON_PIN PC0
//#define JOYSTICK_R_BUTTON_PIN 5

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

int read_digital_pinc(uint8_t channel)
{
	return PINC & (1 << channel);
}

int mapVal(float val, float inLowerBound, float inUpperBound, float outLowerBound, float outUpperBound) 
{
	return (val - inLowerBound) * (outUpperBound - outLowerBound) / (inUpperBound - inLowerBound) + outLowerBound;
}

int calculateJoystickVal(int val) {
	int deadzone = 2;
	int retVal = mapVal(val, 0, 255, -10, 10);
	// set deadzone
	if (retVal < deadzone && retVal > -deadzone)
	{
		retVal = 0;
	}

	return retVal;
}

int getJoyStickPercentage(int x, int maxStep, int deadZone) {
  int mapped  = mapVal(x, 0 , 255, -maxStep, maxStep);
  if (-deadZone <= mapped && mapped <= deadZone) {
    mapped = 0;
  }

  return mapped;
}

void createCommand(char* dest, char* inputCommand, int* values, int values_size) {
	strcat(dest, inputCommand);

	char speedBuffer[8] = "";
	int i = 0;
	for (i = 0; i < values_size; i++) {
		strcat(dest, ",");
		sprintf(speedBuffer, "%d", values[i]);
		strcat(dest, speedBuffer);
		speedBuffer[0] = '\0';
	}
		
	strcat(dest, "*");  
}

int j = 0;
char command[32];
void debugRoomba() 
{
//	char command[32];
	uint8_t bytes_received = uart_bytes_received();
	uint8_t receive_byte;
	int i = 0;
	for (i = 0; i < bytes_received; i++)
	{
		receive_byte = uart_get_byte(i);
//		UART_print("%c", receive_byte);
		command[j++] = receive_byte;
		if (receive_byte == '*') 
		{
			command[j] = '\0';
			UART_print("Command : %s\n", command);

			j = 0;
			memset(command, '\0', 32);
		}
	}

	uart_reset_receive();

}

void roombaTask()
{
	char bt_command[32] = "";
	char bt_last_command[32] = "";
	const int BUFFER_SIZE = 2;
	int command_values[BUFFER_SIZE];

	const int minSpeed = 100;
	const int maxSpeed = 500;
	const int minTurnRadius = 100;
	const int maxTurnRadius = 2000;
	for(;;){
		int joystickX = read_analog(JOYSTICK_R_X_PIN);
		int joystickY = read_analog(JOYSTICK_R_Y_PIN);

		float mx = getJoyStickPercentage(joystickX, maxSpeed, minSpeed);
		float my = getJoyStickPercentage(joystickY, maxSpeed, minSpeed);

		int drivePower = sqrt(mx * mx + my * my);
		if (drivePower > maxSpeed) drivePower = maxSpeed;
		if (my > 0) drivePower *= -1;

		int angle = 2000;

		if (joystickX < 127) angle = mapVal(joystickX, 0, 127, minTurnRadius, maxTurnRadius);
		else angle = mapVal(joystickX, 127, 255, -maxTurnRadius, -minTurnRadius);

		if (angle > 1700) angle = 8000;
		else if (angle < -1700) angle = -8000;

		if (joystickX < 30 && my == 0) angle = 1;
		else if (joystickX > 230 && my == 0) angle = -1;

		command_values[1] = drivePower;
		command_values[0] = angle;
		
		bt_command[0] = '\0';
		createCommand(bt_command, "r", command_values, BUFFER_SIZE);

		if (strcmp(bt_last_command, bt_command) != 0)
		{
		//	UART_print("%s\n", bt_command);
			Bluetooth_Send_String(bt_command);
		}
		
		strcpy(bt_last_command, bt_command);

		//debugRoomba();
	
		Task_Next();
	}
}

void servoTask()
{
	char bt_command[16] = "";
	char bt_last_command[16] = "";
	const int BUFFER_SIZE = 3;
	int command_values[BUFFER_SIZE];
	for (;;)
	{
		int joystick_sx = read_analog(JOYSTICK_S_X_PIN);
		int joystick_sy = read_analog(JOYSTICK_S_Y_PIN);
		int joystick_button = read_digital_pinc(JOYSTICK_S_BUTTON_PIN);//read_analog(JOYSTICK_S_BUTTON_PIN);// > 10 ? 1 : 0;

//		command_values[0] = calculateJoystickVal(joystick_sx);
//		command_values[1] = calculateJoystickVal(joystick_sy);
		command_values[0] = getJoyStickPercentage(joystick_sx, 5, 1);
		command_values[1] = -getJoyStickPercentage(joystick_sy, 5, 1);
		command_values[2] = joystick_button;

		bt_command[0] = '\0';
		createCommand(bt_command, "s", command_values, BUFFER_SIZE);

		if (strcmp(bt_last_command, bt_command) != 0)
		{
			//UART_print("%s\n", bt_command);
			Bluetooth_Send_String(bt_command);
		}

		strcpy(bt_last_command, bt_command);

		Task_Next();
	}
}

void a_main()
{
	Bluetooth_UART_Init(); 
	UART_Init0(9600);
    UART_print("\r\nSTART\r\n");

	setup_controllers();
	init_LED_ON_BOARD();
	init_LED_PING();


	Task_Create_Period(servoTask, 0, 10, 5, 0);
	Task_Create_Period(roombaTask, 0, 10, 5, 6);
}
