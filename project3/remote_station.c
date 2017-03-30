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

uint8_t IdlePID;
uint8_t RoombaTestPID;
uint8_t RoombaTaskPID;
uint8_t BluetoothSendPID;
uint8_t BluetoothReceivePID;
uint8_t LaserTaskPID;
uint8_t ServoTaskPID;
uint8_t LightSensorTaskPID;
uint8_t GetSensorDataTaskPID;

int laserState;
int servoState;
int lastServoState;
char roombaState;
int wallState;
int bumpState;

uint16_t photocellReading;
uint16_t photoThreshold;

int AUTO;

// An idle task that runs when there is nothing else to do
// Could be changed later to put CPU into low power state

typedef enum laser_states {
    OFF = 0,
    ON
} LASER_STATES;

typedef enum servo_states {
    FULL_BACK = 0,
    HALF_BACK,
    DEAD_ZONE,
    HALF_FORWARD,
    FULL_FORWARD
} SERVO_STATES;

// Queue globals
#define QSize 	10

int servoQueue[QSize];
int servoFront;
int servoRear;

int laserQueue[QSize];
int laserFront;
int laserRear;

int roombaQueue[QSize];
int roombaFront;
int roombaRear;

// Channels
CHAN laserChannel;
CHAN servoChannel;
CHAN sensorChannel;

// ------------------------------ IS FULL ------------------------------ //
int buffer_isFull(int *front, int *rear) {
  return (*rear == (*front - 1) % QSize);
}

// ------------------------------ IS EMPTY ------------------------------ //
int buffer_isEmpty(int *front, int *rear) {
  return (*rear == *front);
}

// ------------------------------ ENQUEUE ------------------------------ //
void buffer_enqueue(int val, int *queue, int *front, int *rear) {
  if (buffer_isFull(front, rear)) {
    return;
  }
  queue[*rear] = val;
  *rear = (*rear + 1) % QSize;
}

// ------------------------------ DEQUEUE ------------------------------ //
int buffer_dequeue(int *queue, int *front, int *rear){
  if (buffer_isEmpty(front, rear)) {
    return -1;
  }
  int result = queue[*front];
  *front = (*front + 1) % QSize;
  return result;
}

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


// ------------------------------ TOGGLE PORTH3 ------------------------------ /
void enablePORTH3() {
	PORTL |= _BV(PORTH3);
}
void disablePORTH3() {
	PORTL &= ~_BV(PORTH3);
}

// ------------------------------ ROOMBA TEST ------------------------------ //
void Roomba_Test() {
	for(;;) {
		Roomba_Drive(75,DRIVE_STRAIGHT);
		_delay_ms(5000);
		Roomba_Drive(-75,DRIVE_STRAIGHT);
		_delay_ms(5000);
	}
}


// ******************************************************************* //
// ****************************** TASKS ****************************** //
// ******************************************************************* //

void ADC_init() {
	ADMUX |= (1<<REFS0);
	ADCSRA |= (1<<ADEN) | (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2);
}

void Servo_Init() {
	// Setup ports and timers
    DDRA = 0xFF; // All output
    PORTA = 0;

    // Configure timer/counter1 as phase and frequency PWM mode
    TCNT4 = 0;
    TCCR4A = (1<<COM4A1) | (1<<COM4B1) | (1<<WGM41);  //NON Inverted PWM
    TCCR4B |= (1<<WGM43) | (1<<WGM42) | (1<<CS41) | (1<<CS40); //PRESCALER=64 MODE 14 (FAST PWM)
    ICR4 = 4999;

    OCR4A = 375; // 90 Degrees
}

// ------------------------------ IDLE TASK ------------------------------ //
void Idle() {
	for(;;) {
		continue;
	}
}

// ------------------------------ LASER TASK ------------------------------ //
void Laser_Task() {
	for(;;) {
		// Send(laserChannel, 1);
		togglePORTL(PL5);
		Task_Next();
		// if(!buffer_isEmpty(&laserFront, &laserRear)) {
			// laserState = buffer_dequeue(laserQueue, &laserFront, &laserRear);
			// if (laserState == ON) {
			// 	enablePORTL5();
			// 	laserState = OFF;
			// }
			// else {
			// 	disablePORTL5();
			// 	laserState = ON;
			// }
		// }

		// Recv(laserChannel);
		// Task_Sleep(10);
	}
}

// ------------------------------ SERVO TASK ------------------------------ //
void Servo_Task() {
	for(;;) {
		Send(servoChannel, 1);

		if(!buffer_isEmpty(&servoFront, &servoRear)) {
			servoState = buffer_dequeue(servoQueue, &servoFront, &servoRear);
		}

		if (servoState > 380 && (lastServoState <= 610)) {
			if (servoState > 550) {
					lastServoState += 3;
			}
			lastServoState += 1;
			OCR4A = lastServoState;
		}
		else if (servoState < 370) {
			if (servoState < 1) {
				lastServoState -= 3;
			}
			lastServoState -= 1;
			OCR4A = lastServoState;
		}

		Recv(servoChannel);
		// Task_Sleep(3);
	}
}

// ------------------------------ SET PHOTOCELL THRESHOLD ------------------------------ //
void Set_Photocell_Threshold() {
	uint16_t photo1;
	uint16_t photo2;
	uint16_t photo3;

	ADMUX = (ADMUX & 0xE0);

	ADMUX = (ADMUX | 0x07); // Channel 7

	ADCSRB |= (0<<MUX5);

	ADCSRA |= (1<<ADSC); // Start conversion

	while((ADCSRA)&(1<<ADSC));    //WAIT UNTIL CONVERSION IS COMPLETE

	photo1 = ADC;

	ADCSRA |= (1<<ADSC); // Start conversion

	while((ADCSRA)&(1<<ADSC));    //WAIT UNTIL CONVERSION IS COMPLETE

	photo2 = ADC;

	ADCSRA |= (1<<ADSC); // Start conversion

	while((ADCSRA)&(1<<ADSC));    //WAIT UNTIL CONVERSION IS COMPLETE

	photo3 = ADC;

	photoThreshold = (photo1 + photo2 + photo3) / 3;
}

// ------------------------------ LIGHT SENSOR TASK ------------------------------ //
void LightSensor_Task() {
	int i = 0;

	for(;;) {
		// Read photocell
		ADMUX = (ADMUX & 0xE0);

		ADMUX = (ADMUX | 0x07); // Channel 7

		ADCSRB |= (0<<MUX5);

		ADCSRA |= (1<<ADSC); // Start conversion

		while((ADCSRA)&(1<<ADSC));    //WAIT UNTIL CONVERSION IS COMPLETE

		photocellReading = ADC;

		if (photocellReading >= (photoThreshold + 50)) {
			enablePORTL2();
			Roomba_Play(0);
			disablePORTL5();
			Roomba_Drive(0,0);
			OS_Abort(0);
		}

		if(i % 5 == 0) {
			Set_Photocell_Threshold();
			i = 0;
		}
		else {
			i++;
		}

		// Task_Sleep(10);
	}
}

// ------------------------------ GET SENSOR DATA ------------------------------ //
void Get_Sensor_Data() {
	for(;;) {
		Roomba_QueryList(7, 13);

		// while(!(UCSR3A & (1<<RXC3)));
		// Task_Sleep(2);
		bumpState = Roomba_Receive_Byte();
		// while(!(UCSR3A & (1<<RXC3)));
		// Task_Sleep(2);
		wallState = Roomba_Receive_Byte();

		// Task_Sleep(20);
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
	if(roombaState == 'A') {
		Roomba_Drive(ROOMBA_SPEED,TURN_RADIUS); // Forward-Left
	}
	else if(roombaState == 'B') {
		Roomba_Drive(ROOMBA_SPEED,DRIVE_STRAIGHT); // Forward
	}
	else if(roombaState == 'C') {
		Roomba_Drive(ROOMBA_SPEED,-TURN_RADIUS); // Forward-Right
	}
	else if(roombaState == 'D') {
		Roomba_Drive(ROOMBA_TURN,IN_PLACE_CCW); // Left
	}
	else if(roombaState == 'E') {
		Roomba_Drive(ROOMBA_TURN,IN_PLACE_CW); // Right
	}
	else if(roombaState == 'F') {
		Roomba_Drive(-ROOMBA_SPEED,TURN_RADIUS); // Backward-left
	}
	else if(roombaState == 'G') {
		Roomba_Drive(-ROOMBA_SPEED,DRIVE_STRAIGHT); // Backward
	}
	else if(roombaState == 'H') {
		Roomba_Drive(-ROOMBA_SPEED,-TURN_RADIUS); // Backward-right
	}
	else if(roombaState == 'X') {
		Roomba_Drive(0,0); // Stop
	}
}

// ------------------------------ AUTO DRIVE ------------------------------ //
void Auto_Drive() {
	Roomba_Drive(ROOMBA_SPEED,DRIVE_STRAIGHT);
}

// ------------------------------ ROOMBA TASK ------------------------------ //
void Roomba_Task() {
	for(;;) {
		if(wallState) {
			buffer_dequeue(roombaQueue, &roombaFront, &roombaRear);
			Reverse();

			if(AUTO==1) {
				// Task_Sleep(20);
				buffer_dequeue(roombaQueue, &roombaFront, &roombaRear);
				Roomba_Drive(ROOMBA_SPEED*2, IN_PLACE_CCW);
			}
		}
		else if(bumpState >= 1 && bumpState <= 3) {
			buffer_dequeue(roombaQueue, &roombaFront, &roombaRear);
			Bump_Back();

			if(AUTO==1) {
				// Task_Sleep(20);
				buffer_dequeue(roombaQueue, &roombaFront, &roombaRear);
				Roomba_Drive(ROOMBA_SPEED*2, IN_PLACE_CCW);
			}
		}
		else {
			if(AUTO==1) {
				Auto_Drive();
			}
			else {
				roombaState = buffer_dequeue(roombaQueue, &roombaFront, &roombaRear);
				Manual_Drive();
			}
		}

		// Task_Sleep(20);
	}
}

// ------------------------------ BLUETOOTH SEND ------------------------------ //
void Bluetooth_Send() {
	for(;;) {
		// SEND LIGHT SENSOR DATA
		Bluetooth_Send_Byte(PHOTO);
		Bluetooth_Send_Byte(photocellReading>>8);
		Bluetooth_Send_Byte(photocellReading);
		Task_Next();
	}
}

struct servoState {
	int x; 
	int y;
	int laserState;
} servoBuffer ;



// ------------------------------ BLUETOOTH RECIEVE ------------------------------ //
void Bluetooth_Receive() {
	int i;
	char command[32];
	for (;;) // Loop forever
	{
		// // unsigned char ReceivedByte ;
		uint8_t bytes_received =  uart_bytes_received();

		uint8_t receive_byte;

		memset(command, '\0' , 32);

		for(i = 0; i < bytes_received; i++){
			receive_byte = uart_get_byte(i);
			command[i] = receive_byte;
			if(receive_byte == '*'){
				// uint8_t first_byte = uart_get_byte(0);
				command[i] = '\0';

				int count = 0;
				char* token = strtok(command, ",");

				while( token != NULL ) 
			  {
			  	UART_print("%d\n", count);
			  	UART_print("%s\n", token);
			    token = strtok(NULL, ",");
			    count++;
			  } 
			}
				

				// Write(servoChannel, 1);


				// int x;
				// int y;
				// int laserState;
				// if('s' == tmp){
				// 	 UDR0 = 'G';
				// 	 _delay_ms(1);
				// 	 UDR0 = bytes_received;
				// }
    		// Put data into buffer, sends the data
    		// UDR0 = '\n';
	
			// else{
			// 	// UDR0 = receive_byte;
			// 	// UART_print("%c ", receive_byte);
			// }
		}
		// while (( UCSR1A & (1 << RXC1 )) == 0) {}; // Do nothing until data have been received and is
		// ready to be read from UDR
		// ReceivedByte = UDR1 ; // Fetch the received byte value into the variable " ByteReceived "
		// uart_bytes_received
		// UART_print("%c ", ReceivedByte);
		// if(ReceivedByte == '*'){
		// 	UART_print("\n");
		// }
		// while (( UCSRA & (1 << UDRE )) == 0) {}; // Do nothing until UDR is ready for more data to
		// // be written to it
		// UDR = ReceivedByte;
		uart_reset_receive();
		Task_Next();
	}
}

// Application level main function
// Creates the required tasks and then terminates
void a_main() {

	// // Initialize Ports
	// DDRL |= _BV(DDL6);
	// DDRL |= _BV(DDL2);
	// DDRL |= _BV(DDL5);
	// DDRH |= _BV(DDH3);

	// // Initialize Queues
	// laserFront = 0;
	// laserRear = 0;
	// servoFront = 0;
	// servoRear = 0;
	// roombaFront = 0;
	// roombaRear = 0;

	// Initialize Blocking Channels
	// laserChannel  = Chan_Init();
	// servoChannel  = Chan_Init();
	// sensorChannel = Chan_Init();


	// laserState = ON;
	// Initialize Bluetooth and Roomba UART
	Bluetooth_UART_Init();

	init_LED_ON_BOARD();
	// Roomba_UART_Init();
	// ADC_init();
	// Servo_Init();
	// Roomba_Init();

	// Evaluate light
	// Set_Photocell_Threshold();

	// Initialize Values
	// photocellReading = 0;
	// servoState = 375;
	// lastServoState = 375;
	// wallState = 0;
	// bumpState = 0;
	// roombaState = 'X';
	// AUTO = 0;
	UART_Init0(9200);
	// Create Tasks
	// IdlePID 					      = Task_Create_Idle(Idle , 1);
	BluetoothReceivePID  = Task_Create_Period(Bluetooth_Receive, 2, 10, 0, 0 );
	// BluetoothSendPID 			= Task_Create(Bluetooth_Send, 2, 3);


	// LaserTaskPID 		  = Task_Create_System(System_Bluetooth_Receive, 2);
	// init_PL5();
	// LaserTaskPID 		     = Task_Create_Period(Laser_Task, 2, 20, 0, 0 );
	// LightSensorTaskPID 			= Task_Create(LightSensor_Task, 2, 3);
	// ServoTaskPID 				= Task_Create(Servo_Task, 2, 3); // Periodic
	// RoombaTaskPID 				= Task_Create(Roomba_Task, 2, 2); // Periodic
	// GetSensorDataTaskPID 		= Task_Create(Get_Sensor_Data, 2, 2);

	Task_Terminate();
}