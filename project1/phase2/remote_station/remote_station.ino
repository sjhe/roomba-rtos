#include "Roomba_Driver.h"
#include <Servo.h> // Servo lib


//Roomba
#define ROOMBA_ANALOG_PIN     2
#define ROOMBA_DIGITA_PIN     44

Roomba r(ROOMBA_ANALOG_PIN, ROOMBA_DIGITA_PIN);

//PWM 
#define TILT_SERVO_PIN        8
#define PAN_SERVO_PIN         9
#define LASER_PIN            10



unsigned long next_time = 1;
unsigned long runtime = 0;
bool initialized = true;

char btInput[32] = "";
char inputBuffer[32] = "";


char *token = "";
const char s[2] = ",";
//Read commands from Bluetooth.


Servo panServo;     //PAN SERVO MOTOR (X)

int panAngle = 90;

Servo tiltServo;    //TILT SERVO Motor (Y)

int tiltAngle = 90;

const int maxStepAngle = 5;

char currentState = 'i';   // Current state of the roomba

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial1.print("Starting init");
  r.init();
  pinMode(LASER_PIN, OUTPUT);

  delay(1000);


//  centerServoPosition();
//  panServo.attach(PAN_SERVO_PIN);     // Pan servo
//  tiltServo.attach(TILT_SERVO_PIN);   // Tilt Servo
}


// Center the servo motors
void centerServoPosition(){
  panServo.write(90);
  tiltServo.write(90);
}

// INPUT EXAMPLE: "f,100,s*"

char* device = ' ';

int joystickButton = 0;    // Global value for controlling the laser

void parseInputStringAndUpdate(){
  int count = 0;
  token = strtok(btInput, s);
  while( token != NULL ) 
  {
//    Serial.println( token );
    if(count == 0 ){   // First token
      device = token;
//      Serial.print("Device : ");
//      Serial.println(device);
    }
    if(count == 1){   // Second Token
      if(*device == 'l'){  
        laserState(token);
        
      }
    }
    token = strtok(NULL, s);
    count++;
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



void loop() {
  if(Serial1.available()) {
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
      parseInputStringAndUpdate();
        
      laserState(joystickButton);

      btInput[0] = '\0';
    }
    
    if(!initialized) {
      r.init();
      initialized = true;
    }
 
    switch(command)
    {
      case 'f': 
        r.drive(150, 32768);
        currentState = 'f';
        break;
      case 'b':
        r.drive(-150, 32768);
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
  }

  //Watch power levels and run time.
  if(runtime > next_time) {
    next_time += 5000;
    unsigned int power = 0;
    bool success = r.check_power(&power);
//    Serial1.print("Power: ");
//    Serial1.println(power);

    if (success && (power < 2000 || runtime > 3600000u)) {
//      Serial1.println("Shutting down!");
      r.power_off();
      initialized = false;
      runtime = 0;
      next_time = 0;
    }
  }

  delay(25);
  runtime += 25 ;
}
