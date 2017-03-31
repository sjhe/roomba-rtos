#define F_CPU 16000000


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "./roomba/roomba.h"
#include "./src/os.h"
#include "./uart/uart.h"
#include "./src/led_test.h"

#include "./uart/uart.h"



uint8_t LASER = 0;
uint8_t SERVO = 1;
uint8_t PHOTO = 2;
uint8_t SCREEN = 3;
uint8_t ROOMBA = 4;
uint8_t MODE = 5;

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
int wallState;
int bumpState;

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

// Queue globals
#define QSize 	10

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

	if ( ( lastServoState + angle) <= 585 && (lastServoState + angle) >= 150 )  {
		lastServoState += angle;
	}else if( (lastServoState + angle ) < 150 ) {
		lastServoState = 150;
	}else if( (lastServoState + angle ) > 585 ){
		lastServoState = 585;
		// Ignore
	}

		// Update X
	if(servo_num == X){
		OCR4A = lastServoState;	
	}
	// Update Y
	if(servo_num == Y){
		// UART_print("lastServoState %d\n", lastServoState);
		OCR4B = lastServoState;	
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

// ------------------------------ GET SENSOR DATA ------------------------------ //
void Get_Sensor_Data() {
	for(;;) {
		Roomba_QueryList(7, 13);

		bumpState = Roomba_Receive_Byte();

		wallState = Roomba_Receive_Byte();
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
void Manual_Drive() {
	int radius = roombaBuffer.radius;
	int speed  = roombaBuffer.speed;

//	UART_print("roomba : %d, %d\n", speed, radius);

	Roomba_Drive(speed,radius); // Forward-Left
}
// ------------------------------ AUTO DRIVE ------------------------------ //
void Auto_Drive() {
	Roomba_Drive(ROOMBA_SPEED,DRIVE_STRAIGHT);
}

// ------------------------------ ROOMBA TASK ------------------------------ //
void Roomba_Task() {

	// ????????????? WTF??????
//	roombaBuffer.radius = 0;
//	roombaBuffer.speed = 0;
	for(;;) {
		Manual_Drive();
		Task_Next();
	}
}

// ------------------------------ BLUETOOTH RECEIVE TASK ------------------------------ //
void Bluetooth_Receive() {
	int i;
	char command[32];
	int j = 0 ; 
	memset(command, '\0' , 32);
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
			  	// UDR0 = 'i';
			  	UART_print("%s\n",command);
			  	//Ignore
			  }
			}
		}
		update_laser();  // update the laser

		uart_reset_receive();  //Reset the index 

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

	unsigned int currentTick = Now() / 10 ; 
	// Initialize Values
	wallState = 0;  
	bumpState = 0;
	// roombaState = 'X';
	// AUTO = 0;
	UART_Init0(9600);

	// Create Tasks
	BluetoothReceivePID = Task_Create_Period(Bluetooth_Receive, 2, 10, 5, currentTick + 0 );
	ServoTaskPID 				= Task_Create_Period(Servo_Task, 3, 10, 5,  currentTick + 1 ); // Periodic
	RoombaTaskPID 		  = Task_Create_Period(Roomba_Task, 4, 10, 8, currentTick + 2); // Periodic

	Task_Terminate();
}
