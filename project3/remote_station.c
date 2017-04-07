#define F_CPU 16000000


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "./roomba/roomba.h"
#include "./src/os.h"
#include "./uart/uart.h"
#include "./src/led_test.h"

#include "./uart/uart.h"

uint8_t RoombaTestPID;
uint8_t RoombaTaskPID;
uint8_t BluetoothSendPID;
uint8_t BluetoothReceivePID;
uint8_t LaserTaskPID;
uint8_t ServoTaskPID;
uint8_t LightSensorTaskPID;
uint8_t GetSensorDataTaskPID;

int laserState;

char roombaState;
int volatile wallState;
int volatile bumpState;

int AUTO;

typedef enum laser_states {
    ON = 0,
    OFF
} LASER_STATES;

typedef enum servo_num {
	X,
	Y
} SERVO_NUM;

typedef enum servo_states {
    FULL_BACK = 0,
    HALF_BACK,
    DEAD_ZONE,
    HALF_FORWARD,
    FULL_FORWARD
} SERVO_STATES;

// Channels
CHAN laserChannel;
CHAN servoChannel;
CHAN sensorChannel;

// Servo State Struct
struct servoStruct {
	int x; 
	int y;
	int laserState;
} servoBuffer ;

// Roomba State Struct
struct roombaStruct {
	int radius; 
	int speed;
	int wall;
	int bump;
	char state;
} roombaBuffer ;


// ------------------------------ TOGGLE PORTL6 ------------------------------ /
void enablePORTL6() {
	PORTL |= _BV(PORTL6);
}
void disablePORTL6() {
	PORTL &= ~_BV(PORTL6);
}
// ------------------------------ TOGGLE PORTL2 ------------------------------ /
void enablePORTL2() {
	PORTL |= _BV(PORTL2);
}
void disablePORTL2() {
	PORTL &= ~_BV(PORTL2);
}

// ------------------------------ TOGGLE PORTL5 ------------------------------ /
void init_PL5() {
	DDRL |= _BV(PL5);		//Set LED to output (pins 4 and 5)
	PORTL = 0x00;		//Initialize port to LOW (turn off LEDs)
}
void disablePORTL5() {
	PORTL &= ~_BV(PL5);
}
void togglePORTL(unsigned int mask){
	PORTL ^= _BV(mask) ;		//Initialize port to high
}
void enablePORTL(unsigned int mask){
	PORTL |= _BV(mask) ;		//Initialize port to high
}

// ------------------------------ TOGGLE PORTH3 ------------------------------ /
void enablePORTH3() {
	PORTL |= _BV(PORTH3);
}
void disablePORTH3() {
	PORTL &= ~_BV(PORTH3);
}

// 

int lerp(int a, int b, float t)
{
	if (t > 1) t = 1.0f;

	return a + (b - a) * t;
}

/**
 * Read an analog value from a given channel. 
 * On the AT mega2560, there are 16 available channels, thus
 * channel can be any value 0 to 15, which will correspond
 * to the analog input on the arduino board. 
 */

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

  ADMUX  |= (1 << REFS0); // Set ADC reference to AVCC
  ADMUX  |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading

  ADCSRA |= (1 << ADEN);  // Enable ADC
	ADCSRA |= (1 << ADSC); //Start a conversion to warmup the ADC.
	
	//We're using the ADC in single Conversion mode, so this is all the setup we need. 
}
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
// ******************************************************************* //
// ****************************** TASKS ****************************** //
// ******************************************************************* //

void Servo_Init() {
	// Setup ports and timers
  DDRH  |= _BV(PH3) | _BV(PH4); // All output
  PORTH = 0;

  // Configure timer/counter1 as phase and frequency PWM mode
  TCNT4  = 0;
  TCCR4A = (1<<COM4A1) | (1<<COM4B1) | (1<<WGM41);  //NON Inverted PWM
  TCCR4B |= (1<<WGM43) | (1<<WGM42) | (1<<CS41) | (1<<CS40); //PRESCALER=64 MODE 14 (FAST PWM)
  ICR4 = 4999;

  OCR4A = 375; // 90 Degrees  Servo X PWM 6
  OCR4B = 375; // 90 Degrees  Servo Y PWM 7
}

// ------------------------------ LASER TASK ------------------------------ //
void update_laser(){
	if(servoBuffer.laserState == 0){
		enablePORTL(PL5);
	}else{
		disablePORTL5();
	}
}

// Update the servo state given the input
int update_ServoState(int lastServoState, int angle, int servo_num ){
	// Update X
	if(servo_num == X){
		int maxX = 525;
		int minX = 175;
		if ( ( lastServoState + angle) <= maxX && (lastServoState + angle) >= minX )  {
			lastServoState += angle;
		}else if( (lastServoState + angle ) < minX ) {
			lastServoState = minX;
		}else if( (lastServoState + angle ) > maxX ){
			lastServoState = maxX;
		}
		// Update
		OCR4A = lastServoState;	
		// _delay_ms(5);
	}
	// Update Y
	if(servo_num == Y){
		int maxY = 450;
		int minY = 250;
		// UART_print("lastServoState %d\n", lastServoState);
		if ( ( lastServoState + angle) <= maxY && (lastServoState + angle) >= minY )  {
			lastServoState += angle;
		}else if( (lastServoState + angle ) < minY ) {
			lastServoState = minY;
		}else if( (lastServoState + angle ) > maxY ){
			lastServoState = maxY;
		}

		OCR4B = lastServoState;	
		// _delay_ms(5);
		// OCR5A = lastServoState;	
	}		
	return lastServoState;
}

// ------------------------------ SERVO TASK ------------------------------ //
void Servo_Task() {
	int lastServoStateX = 375;
	int lastServoStateY = 375;
	int servoStateX = 0;
	int servoStateY = 0;

	for(;;) {
		servoStateX = servoBuffer.x;
		servoStateY = servoBuffer.y;
		// UART_print("x : %d\n", servoStateX);
		// Check X
		lastServoStateX = update_ServoState(lastServoStateX, servoStateX, X);
		// Check Y
		lastServoStateY = update_ServoState(lastServoStateY, servoStateY, Y);
		Task_Next();
	}
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


void send_to_base(char* bt_command, char* bt_last_command, int* command_values){
	createCommand(bt_command, "i", command_values, 3);

	if (strcmp(bt_last_command, bt_command) != 0)
	{
		UART_print("%s\n", bt_command);
		Bluetooth_Send_String(bt_command);	
	}
}

// ------------------------------ GET SENSOR DATA ------------------------------ //
void Get_Sensor_Data() {
	for(;;) {
		Roomba_QueryList(7, 13);
		bumpState   = Roomba_Receive_Byte();
		wallState   = Roomba_Receive_Byte();
		// lightBumper = Roomba_Receive_Byte();
		// UART_print("b:%d w:%d l:%d\n", bumpState, wallState, lightBumper);

		if(bumpState >=  1 || wallState == 1 ){
			unsigned int cur = Now() + 1000;
			roombaState = 'X';
			roombaBuffer.radius = DRIVE_STRAIGHT;
			roombaBuffer.speed = -ROOMBA_SPEED;
			AUTO = 1;
			while( (int)(cur - Now()) > 0  ){				
				Task_Next();
			}

			roombaBuffer.speed  = 0;
			roombaBuffer.radius = 0;
			bumpState   = 0;
			wallState   = 0;
			AUTO = 0;
		}

		Task_Next();
	}
}

// ------------------------------ REVERSE ------------------------------ //
void Reverse() {
	if(roombaState == 'F') {
		Roomba_Drive(ROOMBA_SPEED,TURN_RADIUS); // Forward-Left
	}
	else if(roombaState == 'G') {
		Roomba_Drive(ROOMBA_SPEED,DRIVE_STRAIGHT); // Forward
	}
	else if(roombaState == 'H') {
		Roomba_Drive(ROOMBA_SPEED,-TURN_RADIUS); // Forward-Right
	}
	else if(roombaState == 'E') {
		Roomba_Drive(ROOMBA_TURN,IN_PLACE_CCW); // Left
	}
	else if(roombaState == 'D') {
		Roomba_Drive(ROOMBA_TURN,IN_PLACE_CW); // Right
	}
	else if(roombaState == 'A') {
		Roomba_Drive(-ROOMBA_SPEED,TURN_RADIUS); // Backward-left
	}
	else if(roombaState == 'B') {
		Roomba_Drive(-ROOMBA_SPEED,DRIVE_STRAIGHT); // Backward
	}
	else if(roombaState == 'C') {
		Roomba_Drive(-ROOMBA_SPEED,-TURN_RADIUS); // Backward-right
	}
	else if(roombaState == 'X') {
		Roomba_Drive(-ROOMBA_SPEED,DRIVE_STRAIGHT); // Backward
	}
}

// ------------------------------ BUMP BACK ------------------------------ //
void Bump_Back() {
	Roomba_Drive(-ROOMBA_SPEED,DRIVE_STRAIGHT); // Backward
}

// ------------------------------ MANUAL DRIVE ------------------------------ //
void Manual_Drive(float* lerpTime, int* currentSpeed ) {
	int radius = roombaBuffer.radius;
	int speed  = roombaBuffer.speed;

	*lerpTime += (roombaBuffer.speed == 0 ? 0.1f : 0.05f);
	if (roombaBuffer.speed == *currentSpeed)
	{
		*lerpTime = 0.0f;
	}

	*currentSpeed = lerp(*currentSpeed, roombaBuffer.speed , *lerpTime);
//	UART_print("roomba : %d, %d\n", speed, radius);
	Roomba_Drive(*currentSpeed,radius); // Forward-Left
}

// ------------------------------ ROOMBA TASK ------------------------------ //
void Roomba_Task() {
	roombaBuffer.radius = 0;
	roombaBuffer.speed = 0;
	int currentSpeed = 0;
	float lerpTime = 0.0f;

	for(;;) {
		if(AUTO == 0){
			Manual_Drive(&lerpTime, &currentSpeed);	
		}else{
			Roomba_Drive(roombaBuffer.speed , roombaBuffer.radius);
		}
		Task_Next();
	}
}
// ------------------------------ BLUETOOTH RECEIVE TASK ------------------------------ //
void Bluetooth_Receive() {
	int i;
	char command[32];
	int j = 0 ; 
	memset(command, '\0' , 32);
	servoBuffer.laserState = 1;

	for (;;) // Loop forever
	{
		// unsigned char ReceivedByte ;
		uint8_t bytes_received =  uart_bytes_received();

		uint8_t receive_byte;

		for(i = 0; i < bytes_received; i++){
			receive_byte = uart_get_byte(i);
			command[j++] = receive_byte;
			if(receive_byte == '*'){
				// uint8_t first_byte = uart_get_byte(0);
				command[j] = '\0';

				int count = 0;
				char* token = strtok(command, ",");

				if( token[0] == 's' ){
			  	// UDR0 = 's';
			  	while (token != NULL  )
			  	{
			  		if(count == 1 ){
			  			servoBuffer.x = atoi(token);
			  		}else if (count == 2){
			  			servoBuffer.y = atoi(token);
			  		}else if (count == 3){
			  			servoBuffer.laserState = atoi(token);
			  		}else {
			  			// ignore
			  		}
			  		token = strtok(NULL, ",");
			  		count++;
			  	}
			  	// Clear the command and index 
			  	j = 0; 
			  	memset(command, '\0' , 32);
			  }else if (token[0] == 'r' ){
			  	// UDR0 = 'r';
			  	while (token != NULL  )
			  	{
			  		if(count == 1 ){
			  			roombaBuffer.radius = atoi(token);
			  		}else if (count == 2){
			  			roombaBuffer.speed = atoi(token);
			  		}else {
			  			// ignore
			  		}
			  		token = strtok(NULL, ",");
			  		count++;
			  	}
			  	// Clear the command and index 
					j = 0;
			  	memset(command, '\0' , 32);
			  }else{
			  	UDR0 = 'i';
			  	// UART_print("%s\n",command);
			  	//Ignore
			  	j = 0; 
			  	memset(command, '\0' , 32);
			  	break;
			  }
			}
		}
		update_laser();  // update the laser

		uart_reset_receive();  //Reset the index 

		// send_to_base(bt_command, bt_last_command, command_values);

		Task_Next();  // Yield
	}
}

// Application level main function
// Creates the required tasks and then terminates
void a_main() {
	// Initialize Blocking Channels
	// laserChannel  = Chan_Init();
	// servoChannel  = Chan_Init();
	// sensorChannel = Chan_Init();

	// Initialize Bluetooth and Roomba UART
	Bluetooth_UART_Init();
	// Init Pins
	init_LED_ON_BOARD();
	init_PL5();

	// Roomba_UART_Init();
	// ADC_init();
	Servo_Init();

	Roomba_UART_Init();

	Roomba_Init();

	// setup_controllers();
	AUTO = 0;

	unsigned int currentTick = Now() / 10 ; 
	// Initialize Values
	wallState   = 0;  
	bumpState   = 0;

	UART_Init0(9600);

	// Create Tasks
	BluetoothReceivePID  = Task_Create_Period(Bluetooth_Receive, 2, 9, 2, currentTick );

	ServoTaskPID 				 = Task_Create_Period(Servo_Task, 3, 9, 2,  currentTick + 2 ); // Periodic

	GetSensorDataTaskPID = Task_Create_Period(Get_Sensor_Data, 5, 9, 3, currentTick + 4);

	RoombaTaskPID 		   = Task_Create_Period(Roomba_Task, 4, 9, 2, currentTick + 7); // Periodic


	Task_Terminate();
}
