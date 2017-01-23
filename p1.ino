#include <Wire.h>  // Comes with Arduino IDE
#include <LiquidCrystal_I2C.h>
#include <Servo.h> // Servo lib

#define JOYSTICK_X_PIN       A0
#define JOYSTICK_Y_PIN       A1
#define JOYSTICK_PIN_BUTTON  44
#define LASER_PIN            10
#define PHOTORESISTOR_PIN    A5
#define PAN_SERVO_PIN         9
//#define TILT_SERVO_PIN     10

// set the LCD address to 0x27 for a 16 chars 2 line display
// A FEW use address 0x3F
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

//PAN SERVO MOTOR (X)
Servo panServo; 
int panAngle = 90;

const int maxStepAngle = 5;

// TILE SERVO MOTOR
//Servo tiltServo;
//int tiltAngle = 90;

void lcdTest()
{
  // ------- Quick 3 blinks of backlight  -------------
  for(int i = 0; i < 3; i++)
  {
    lcd.backlight();
    delay(250);
    lcd.noBacklight();
    delay(250);
  }
  lcd.backlight(); // finish with backlight on  


//-------- Write characters on the display ------------------
// NOTE: Cursor Position: (CHAR, LINE) start at 0  
  lcd.setCursor(0,0); //Start at character 4 on line 0
  lcd.print("Hello, world!");
  delay(1000);
  lcd.setCursor(0,1);
  lcd.print("HI!YourDuino.com");
  delay(8000);  

// Wait and then tell user they can start the Serial Monitor and type in characters to
// Display. (Set Serial Monitor option to "No Line Ending")
  lcd.clear();
  lcd.setCursor(0,0); //Start at character 0 on line 0
  lcd.print("Use Serial Mon");
  lcd.setCursor(0,1);
  lcd.print("Type to display");  

}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);     
  
  pinMode(LASER_PIN, OUTPUT);

  lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight
  
  panServo.attach(PAN_SERVO_PIN);
  centerServoPosition();
  delay(500);
}



// turns laser on if inputValue is 0 and off otherwise.
void laserState(int inputValue)
{
  lcd.setCursor(12, 0);
  if (inputValue == 0)
  {
    lcd.print("ON");
    digitalWrite(LASER_PIN, HIGH);
  }
  else
  {
    lcd.print("OFF");
    digitalWrite(LASER_PIN, LOW);
  }
}


int getJoyStickPercentage(int x){
  const int max_X = 1015;
  const int max_Y = 1015;
  int mapped  = map(x, 0 , 1023, -maxStepAngle, maxStepAngle );
//  Serial.print(mapped);
  if(  -2 <= mapped && mapped <= 2 ){
    mapped = 0;
  }

  lcd.setCursor(5,1);

  lcd.print(mapped);

  return mapped;
}

void centerServoPosition(){
  panServo.write(90);
}

void servoMovement(Servo theServo, int angle){
   int maxAngle = 135;
   int minAngle = 45;
   panAngle += angle;
   if( panAngle >= maxAngle ){
      panAngle = maxAngle;
   }else if( panAngle <= minAngle){
      panAngle = minAngle;
   }
  // in steps of 1 degree
  theServo.write(panAngle);        // tell servo to go to position in variable 'pos'

  lcd.setCursor(11,1);
  lcd.print(panAngle);

}

void parseAndExecuteJoystickInput()
{
  int joystickX = analogRead(JOYSTICK_X_PIN);
  int joystickY = analogRead(JOYSTICK_Y_PIN);
  int joystickButton = digitalRead(JOYSTICK_PIN_BUTTON);
//  // print out the value you read:
//  Serial.print(joystickX);
//  Serial.print(", ");
//  Serial.print(joystickY);
//  Serial.print(" | ");
//  Serial.print(joystickButton);
//  Serial.print(" | ");

  lcd.setCursor(0, 0);
  lcd.print(joystickX);
  lcd.setCursor(5, 0);
  lcd.print(",");
  lcd.setCursor(7, 0);
  lcd.print(joystickY);

  laserState(joystickButton);

//  getJoyStickPercentage(joystickX);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  lcd.clear();
  int joystickX = analogRead(JOYSTICK_X_PIN);
  int joystickY = analogRead(JOYSTICK_Y_PIN);
  int joystickButton = digitalRead(JOYSTICK_PIN_BUTTON);

  int theDelay = 50;
  
  parseAndExecuteJoystickInput();
  

  int photoresistor_val = analogRead(PHOTORESISTOR_PIN);
//  Serial.print(photoresistor_val);
//  Serial.println();
  lcd.setCursor(0,1); //Start at character 0 on line 0
  lcd.print(photoresistor_val);

// ------ Servo Motor data -------

  int stepAngleX = getJoyStickPercentage(joystickX);

  servoMovement( panServo, stepAngleX);  
    
  delay(50);        // delay in between reads for stability

}
