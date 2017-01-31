#include "Roomba_Driver.h"

//serial_connector determines which UART the Roomba is connected to (0, 1, etc)
//brc_pin determines where the baud rate change pin is connected.
Roomba::Roomba(int serial_connector, int brc_pin) {
  m_serial_num = serial_connector;
  m_brc_pin = brc_pin;
  pinMode(m_brc_pin, OUTPUT);
  
}

//Checks the remaining power level.
bool Roomba::check_power(unsigned int *power) {
  //Serial.printtln("cp");
  char battery_charge_high = 0;
  char battery_charge_low = 0;
  
  while(read_serial(&battery_charge_high));
  write_serial(SENSORS);
  write_serial(25);
  bool return_value = false;
  delay(50);
  return_value = read_serial(&battery_charge_high);
  read_serial(&battery_charge_low);

  if (return_value) {
    *power = 0;
    *power = (battery_charge_high << 8) | (battery_charge_low);
  }
  
  return return_value;
}


//Checks the total power capacity.
bool Roomba::check_power_capacity(unsigned int *power) {
  //Serial.printtln("cpc");
  char battery_charge_high = 0;
  char battery_charge_low = 0;
  
  while(read_serial(&battery_charge_high));
  write_serial(SENSORS);
  write_serial(26);
  bool return_value = false;
  delay(50);
  return_value = read_serial(&battery_charge_high);
  read_serial(&battery_charge_low);

  if (return_value) {
    *power = 0;
    *power = (battery_charge_high << 8) | (battery_charge_low);
  }
  return return_value;
}

void Roomba::power_off() {
  write_serial(STOP);
}

void Roomba::init()
{
  Serial1.println("Init");
  start_serial(19200U);
  digitalWrite(m_brc_pin, HIGH);
  
  Serial1.println("Setting Baud");
  //Set baud rate by togling the brc pin 3 times.
  delay(2500);
  digitalWrite(m_brc_pin, LOW);
  delay(300);
  digitalWrite(m_brc_pin, HIGH);
  delay(300);
  digitalWrite(m_brc_pin, LOW);
  delay(300);
  digitalWrite(m_brc_pin, HIGH);
  delay(300);
  digitalWrite(m_brc_pin, LOW);
  delay(300);
  digitalWrite(m_brc_pin, HIGH);  

  start_serial(19200U);
  //Power on
  write_serial(START);
  delay(200);
  //Enter safe mode
  write_serial(SAFE);

  /*
  unsigned int power_cap = 0;
  check_power(&power);
  check_power_capacity(&power_cap);
  //Serial.printt("Power: ");
  //Serial.printt(power);
  //Serial.printt(" / ");
  //Serial.printtln(power_cap);
  //Serial.printtln("Done Startup");
  */
}

void Roomba::drive(int velocity, int radius) {
//  Serial1.println("Drive");
  write_serial(DRIVE);
  write_serial(HIGH_BYTE(velocity));
  write_serial(LOW_BYTE(velocity));
  write_serial(HIGH_BYTE(radius));
  write_serial(LOW_BYTE(radius));
}

void Roomba::dock() {
  write_serial(DOCK);
}

void Roomba::get_data() {
  char val;
  while(read_serial(&val));
}

void Roomba::start_serial(long baud) {
  switch(m_serial_num){
	case 0:
	  Serial.end();
	  Serial.begin(baud);
	  break;
	case 1:
	  Serial1.end();
	  Serial1.begin(baud);
	  break;
	case 2:
	  Serial2.end();
	  Serial2.begin(baud);
	  break;
	case 3:
	  Serial3.end();
	  Serial3.begin(baud);
	  break;
  }
}

bool Roomba::read_serial(char *val) {
  switch(m_serial_num){
  case 0:
    if(Serial.available()) {
      *val = Serial.read();
      return true;
    }
    break;
  case 1:
    if(Serial1.available()) {
      *val = Serial1.read();
      return true;
    }
    break;
  case 2:
    if(Serial2.available()) {
      *val = Serial2.read();
      return true;
    }
    break;
  case 3:
    if(Serial3.available()) {
      *val = Serial3.read();
      return true;
    }
    break;
  }
  return false;
}


void Roomba::write_serial(char val) {
  switch(m_serial_num){
	case 0:
	  Serial.write(val);
	  break;
	case 1:
	  Serial1.write(val);
	  break;
	case 2:
	  Serial2.write(val);
	  break;
	case 3:
	  Serial3.write(val);
	  break;
  }
}


