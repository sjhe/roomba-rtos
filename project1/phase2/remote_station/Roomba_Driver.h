#include "Arduino.h"

#define HIGH_BYTE(x) (x>>8)
#define LOW_BYTE(x)  (x&0xFF)

//This is not an exhaustive list of commands.

#define START 128   // start the Roomba's serial command interface
#define BAUD  129   // set the SCI's baudrate (default on full power cycle is 57600
#define CONTROL 130   // enable control via SCI
#define SAFE  131   // enter safe mode
#define FULL  132   // enter full mode
#define POWER 133   // put the Roomba to sleep
#define SPOT  134   // start spot cleaning cycle
#define CLEAN 135   // start normal cleaning cycle
#define MAX   136   // start maximum time cleaning cycle
#define DRIVE 137   // control wheels
#define MOTORS  138   // turn cleaning motors on or off
#define LEDS  139   // activate LEDs
#define SONG  140   // load a song into memory
#define PLAY  141   // play a song that was loaded using SONG
#define SENSORS 142   // retrieve one of the sensor packets
#define DOCK  143   // force the Roomba to seek its dock.
#define STOP 173

/// Arguments to the BAUD command
typedef enum _br {
  ROOMBA_300BPS = 0,
  ROOMBA_600BPS = 1,
  ROOMBA_1200BPS = 2,
  ROOMBA_2400BPS = 3,
  ROOMBA_4800BPS = 4,
  ROOMBA_9600BPS = 5,
  ROOMBA_14400BPS = 6,
  ROOMBA_19200BPS = 7,
  ROOMBA_28800BPS = 8,
  ROOMBA_38400BPS = 9,
  ROOMBA_57600BPS = 10,
  ROOMBA_115200BPS = 11,
} ROOMBA_BITRATE;

class Roomba{
  public:
    Roomba(int serial_connector, int brc_pin);
  
    void init();
    void drive(int velocity, int radius);
    void dock();
    void get_data();
    bool check_power(unsigned int *power);
    bool check_power_capacity(unsigned int *power);
    void power_off();
    
  private:
    int m_serial_num;
    int m_brc_pin;

    void start_serial(long baud);
    void write_serial(char val);
    bool read_serial(char *val);
};


