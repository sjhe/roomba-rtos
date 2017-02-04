#include "scheduler.h"

#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>
#include <Servo.h> // Servo lib

// Analog
#define JOYSTICK_S_X_PIN     A0
#define JOYSTICK_S_Y_PIN     A1
#define JOYSTICK_R_X_PIN     A2
#define JOYSTICK_R_Y_PIN     A3
#define PHOTORESISTOR_PIN    A5

// PWM
#define TILT_SERVO_PIN        8
#define PAN_SERVO_PIN         9
#define LASER_PIN            10

// Digital
#define JOYSTICK_PIN_BUTTON  44

// set the LCD address to 0x27 for a 16 chars 2 line display
// A FEW use address 0x3F
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

Servo panServo;     //PAN SERVO MOTOR (X)
Servo tiltServo;    //TILT SERVO Motor (Y)

int tiltAngle = 90;
int panAngle = 90;
const int maxStepAngle = 5;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(9600);

  lcd.begin(16, 2);  // initialize the lcd for 16 chars 2 lines, turn on backlight
  lcd.setCursor(0, 0);

  Scheduler_Init();

  Scheduler_StartTask(100, 500, update_lcd);
  Scheduler_StartTask(0, 25, roombaTasks);
  Scheduler_StartTask(50, 25, servoTasks);
}

void update_lcd()
{

  lcd.clear();
  
  
  int joystickRX = analogRead(JOYSTICK_R_X_PIN);
  int joystickRY = analogRead(JOYSTICK_R_Y_PIN);
  int joystickSX = analogRead(JOYSTICK_S_X_PIN);
  int joystickSY = analogRead(JOYSTICK_S_Y_PIN);
  int joystickButton = digitalRead(JOYSTICK_PIN_BUTTON);

  int photoResistor = analogRead(PHOTORESISTOR_PIN);

  lcd.setCursor(0, 0);
  lcd.print(joystickRX);
  lcd.print(", " );
  lcd.print(joystickRY);
  lcd.print(" | " );
  lcd.print(photoResistor);
  lcd.print(" | " );
  lcd.print(joystickButton);

  lcd.setCursor(0, 1);
  lcd.print(joystickSX);
  lcd.print(", " );
  lcd.print(joystickSY);

  int mx = getJoyStickPercentage(joystickSX);
  lcd.print(" | " );
  lcd.print(mx);
  int my = getJoyStickPercentage(joystickSY);
  lcd.print(", " );
  lcd.print(my);

    if(Serial.available()){
      Serial1.print((char)Serial.read());
  //    Serial1.print(mx);
    }
    if(Serial1.available()){
      Serial.print((char)Serial1.read());
    }
}

// turns laser on if inputValue is 0 and off otherwise.
void laserState(int inputValue)
{
  if (inputValue == 0)
  {
    digitalWrite(LASER_PIN, HIGH);
  }
  else
  {
    digitalWrite(LASER_PIN, LOW);
  }
}

int getJoyStickPercentage(int x) {
  int mapped  = map(x, 0 , 1000, -maxStepAngle, maxStepAngle );
  if (  -2 <= mapped && mapped <= 2 ) {
    mapped = 0;
  }

  return mapped;
}

void centerServoPosition() {
  panServo.write(90);
  tiltServo.write(90);
}

void servoMovement(Servo theServo, int angle, int* servoAngle, int minAngle, int maxAngle) {
  *servoAngle += angle;
  if (*servoAngle >= maxAngle )
  {
    *servoAngle = maxAngle;
  }
  else if (*servoAngle <= minAngle)
  {
    *servoAngle = minAngle;
  }

  theServo.write(*servoAngle);
}

void parseAndExecuteJoystickInput()
{
  int joystickX = analogRead(JOYSTICK_R_X_PIN);
  int joystickY = analogRead(JOYSTICK_R_Y_PIN);
  int joystickButton = digitalRead(JOYSTICK_PIN_BUTTON);

  laserState(joystickButton);
}

void createServoCommand(char* dest, char* inputCommand, int inputSpeedX, int inputSpeedY)
{
  strcat(dest, inputCommand);

  char speedBuffer[32] = "";
  strcat(dest, ",");
  sprintf(speedBuffer, "%d", inputSpeedX);
  strcat(dest, speedBuffer);

  speedBuffer[0] = '\0';
  strcat(dest, ",");
  sprintf(speedBuffer, "%d", inputSpeedY);
  strcat(dest, speedBuffer);

  strcat(dest, "*");  
}

void createCommand(char* dest, char* inputCommand, int inputSpeed)
{
  strcat(dest, inputCommand);

  if (inputSpeed != 0)
  {
    char speedBuffer[32] = "";
    strcat(dest, ",");
    sprintf(speedBuffer, "%d", inputSpeed);
    strcat(dest, speedBuffer);
  }

  strcat(dest, "*");
}

char btServoCommand[32] = "";
char lastBtServoCommand[32] = "";
int joystickButtonPrevState = 0;
void servoTasks()
{
  int joystickX = analogRead(JOYSTICK_S_X_PIN);
  int joystickY = analogRead(JOYSTICK_S_Y_PIN);
  int joystickButton = digitalRead(JOYSTICK_PIN_BUTTON);

  int mx = -getJoyStickPercentage(joystickX);
  int my = -getJoyStickPercentage(joystickY);

  if (mx != 0 || my != 0) createServoCommand(btServoCommand, "s", mx, my);
  else if (joystickButton == 0) createCommand(btServoCommand, "l,0", 0);
  else if (strcmp(lastBtServoCommand, btServoCommand) != 0 && joystickButton == 1 && joystickButtonPrevState == 0) createCommand(btServoCommand, "l,1", 0);
  else createServoCommand(btServoCommand, "s", 0, 0);
  
  joystickButtonPrevState = joystickButton;

  // if last command does not equal current command then send
  if (strcmp(lastBtServoCommand, btServoCommand) != 0)
  {
    Serial.println(btServoCommand);
    Serial1.print(btServoCommand);
  }

  strcpy(lastBtServoCommand, btServoCommand);
  btServoCommand[0] = '\0';
}

char btCommand[32] = "";
char lastBtCommand[32] = "";
void roombaTasks()
{
  int joystickX = analogRead(JOYSTICK_R_X_PIN);
  int joystickY = analogRead(JOYSTICK_R_Y_PIN);

  int mx = -getJoyStickPercentage(joystickX);
  int my = -getJoyStickPercentage(joystickY);

  if (my > 0) createCommand(btCommand, "r,f", my);
  else if (my < 0) createCommand(btCommand, "r,b", my);
  else if (mx > 0) createCommand(btCommand, "r,r", mx);
  else if (mx < 0) createCommand(btCommand, "r,l", mx);
  else createCommand(btCommand, "r,s", 0);

  // if last command does not equal current command then send
  if (strcmp(lastBtCommand, btCommand) != 0)
  {
    Serial.println(btCommand);
    Serial1.print(btCommand);
  }

  strcpy(lastBtCommand, btCommand);
  btCommand[0] = '\0';
}

void loop()
{
  uint32_t idle_period = Scheduler_Dispatch();
  if (idle_period)
  {
    delay(idle_period);
  }
}

