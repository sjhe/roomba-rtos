#include <Wire.h>  // Comes with Arduino IDE

#include "Roomba_Driver.h"
#include <Servo.h> // Servo lib
#include "scheduler.h"  // TTS

//Roomba
#define ROOMBA_SERIAL_PIN     2
#define ROOMBA_DIGITA_PIN     30

Roomba r(ROOMBA_SERIAL_PIN, ROOMBA_DIGITA_PIN);

//PWM 
#define TILT_SERVO_PIN        8
#define PAN_SERVO_PIN         9
#define LASER_PIN            10

#define TEST_PIN             45
#define TEST_PIN_2           39
unsigned long next_time = 1;
unsigned long runtime = 0;
bool initialized = true;

char btInput[32] = "";
char inputBuffer[32] = "";


char *token = "";
const char s[2] = ",";
//Read commands from Bluetooth.


Servo panServo;     //PAN SERVO MOTOR (X)
int panAngle = 90;  // Init angle to 90

Servo tiltServo;    //TILT SERVO Motor (Y)

int tiltAngle = 90; // Init angle to 90

const int maxStepAngle = 5;   // init max step angle

char currentState = 'i';   // Current state of the roomba

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial1.print("Starting init");
  r.init();
  pinMode(LASER_PIN, OUTPUT);
  pinMode(TEST_PIN, OUTPUT);
  pinMode(TEST_PIN_2, OUTPUT);
  panServo.attach(PAN_SERVO_PIN);     // Pan servo
  tiltServo.attach(TILT_SERVO_PIN);   // Tilt Servo
  centerServoPosition();
  
  delay(1000); 

  Scheduler_Init();
  Scheduler_StartTask(0, 25, ReceiveInputTask);
  Scheduler_StartTask(0, 100, RoobaTasks);
  Scheduler_StartTask(0, 50, ServoTasks);



}




// INPUT EXAMPLE: "f,100,s*"

char* device = ' ';

char* joystickButton;    // Global value for controlling the laser

//Roomba
char* roombaDirection = "s"; 
char* roombaSpeed     = "0";

//Servo
char* servoName       = " ";
int  servoPanSpeed    = 0;
int  servoTiltSpeed   = 0;




void parseInputStringAndUpdate(){
  int count = 0;
  token = strtok(btInput, s);
  while( token != NULL ) 
  {
    if(count == 0 ){   // First token
      device = token;
    }
    if(count == 1){   // Second Token
      if(*device == 'l'){  
        joystickButton = token;
        Serial.println(joystickButton);
      }else if(*device == 'r'){
        roombaDirection = token;
      }else if(*device == 's'){
        servoPanSpeed = atoi(token);
      }
    }
    if(count == 2){
      if(*device == 'r'){
        roombaSpeed = token;
      }else{
        if(*device == 's'){
        servoTiltSpeed = atoi(token);
        }
      }
      
    }
    
    token = strtok(NULL, s);
    count++;
  } 
  if(count < 2){
     roombaSpeed = "0";
  }else if(count > 2){
     joystickButton = "1";
  }
}

// turns laser on if inputValue is 0 and off otherwise.
void laserState(char* inputValue)
{
//  Serial.print("Laser State :");
//  Serial.println(inputValue);
  if (inputValue[0] == '0')
  {
    digitalWrite(LASER_PIN, HIGH);
  }
  else
  {
    digitalWrite(LASER_PIN, LOW);
  }
}

//ReceiveInputTask 
void ReceiveInputTask() {
  while(Serial1.available()) {
     digitalWrite(TEST_PIN_2, HIGH);
    //Make sure the Roomba is ready to go.
    char command = Serial1.read();
    
    if (command != '*')
    {
      *inputBuffer = command;
      strcat(btInput, inputBuffer);
    } 
    else
    { 
      //execute bt input
      Serial.println(btInput);
      digitalWrite(TEST_PIN, HIGH);
      parseInputStringAndUpdate();
      laserState(joystickButton);
      digitalWrite(TEST_PIN, LOW);
      btInput[0] = '\0';
    }
  }
  digitalWrite(TEST_PIN_2, LOW);
}


//Roomba Tasks
void RoobaTasks() {
  if(!initialized) {
    r.init();
    initialized = true;
  }

  digitalWrite(TEST_PIN_2, HIGH);
  char command     = ' ';
  int  speed = 0;

  if(roombaDirection[0]){
    command = roombaDirection[0];
  }
  
  if(roombaSpeed[0]){
    speed = roombaSpeed[0];
  }
    
//  int  mappedSpeed  = map(roomb, 0 , 32768, -maxStepAngle, maxStepAngle );
  
  switch(command)
    {
      case 'f': 
//        Serial.println("Driving Roomba");
        r.drive(500, 32768);
        currentState = 'f';
        break;
      case 'b':
        r.drive(-500, 32768);
        currentState = 'b';
        break;
      case 'r':
        r.drive(50, -1);
        currentState = 'r';
        break;
      case 'l':
        r.drive(50, 1);
        currentState = 'l';
        break;
      case 's':
//        Serial.println("Stopping Roomba");
        r.drive(0,0);
        currentState = 's';
        break;
      case 'd':
        r.dock();
        currentState = 'd';
        break;
      case 'p':
        r.power_off();
        initialized = false;
        currentState = 'p';
        break;
      default:
        currentState = 'i';
        break;
    }
  digitalWrite(TEST_PIN_2, LOW);
}


// Servo Functions
// get joystickpercentage
int getJoyStickPercentage(int x){
  int mapped  = map(x, 0 , 1023, -maxStepAngle, maxStepAngle );
//  Serial.print(mapped);
  if(  -2 <= mapped && mapped <= 2 ){
    mapped = 0;
  }
  return mapped;
}

void servoMovement(Servo inputServo, int angle, int* servoAngle, int minAngle, int maxAngle){    
   *servoAngle += angle;
   Serial.print("Servo Angle : ");
   Serial.println(*servoAngle);
   
   if( *servoAngle >= maxAngle ){
      *servoAngle = maxAngle;
   }else if( *servoAngle <= minAngle){
      *servoAngle = minAngle;
   }
  // in steps of 1 degree
  inputServo.write(*servoAngle);        // tell servo to go to position in variable 'pos'
  
}
// End Servo Functions

// Center the servo motors
void centerServoPosition(){
  panServo.write(90);
  tiltServo.write(90);
}
// Servo Tasks
void ServoTasks(){
  // ------ Servo Motor data -------

//  Serial.print("Pan Angle : " );
//  Serial.println(servoPanSpeed);
//
//  Serial.print("Tilt Angle : " );
//  Serial.println(servoTiltSpeed);

  servoMovement( panServo,servoPanSpeed, &panAngle   ,45, 135);  
  servoMovement( tiltServo, servoTiltSpeed, &tiltAngle ,60, 135 );  
}



void loop()
{
  uint32_t idle_period = Scheduler_Dispatch();
  if (idle_period)
  {
    delay(idle_period);
  }
}
