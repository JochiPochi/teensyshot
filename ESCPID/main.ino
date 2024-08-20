
#include <Arduino.h>
#include "DSHOT.h"
#include "ESCCMD.h"

#define ESCPID_NB_ESC             1                 // Number of ESCs
#define ESCPID_MAX_ESC            6                 // Max number of ESCs

#define ESCPID_COMM_WD_LEVEL      20                // Maximum number of periods without reference refresh

// Globals
uint16_t  ESCPID_comm_wd = 0;
int16_t   ESC_Throttle[ESCPID_NB_ESC] = {};     // Desired throttle for motors -999 to 999

void setup() {
  int i;

  // Initialize USB serial link for DEBUG
  Serial.begin( 115200 );

  // Initialize the CMD subsystem
  ESCCMD_init( ESCPID_NB_ESC ); //disabled telemetry serial

  // Arming ESCs
  ESCCMD_arm_all( );
  
  // Switch 3D mode on
  ESCCMD_3D_off( );

  // Arming ESCs
  ESCCMD_arm_all( );
  
  // Start periodic loop
  ESCCMD_start_timer( );

  // Stop all motors
  for ( i = 0; i < ESCPID_NB_ESC; i++ ) {
    ESCCMD_stop( i );
  }
  //}

  // Reference watchdog is initially triggered
  ESCPID_comm_wd = ESCPID_COMM_WD_LEVEL;

  //Analog Input Poti
  pinMode(A0, INPUT);
}

void print_esc_tlm(uint8_t i){
  uint16_t cmd = 0;
  int16_t rpm = 0;
  uint16_t voltage = 0;
  uint16_t amp = 0;
  uint8_t temp = 0;
  ESCCMD_read_cmd(i, &cmd);
  ESCCMD_read_rpm(i, &rpm);
  ESCCMD_read_volt(i, &voltage);
  ESCCMD_read_amp(i, &amp);
  ESCCMD_read_deg(i, &temp);
  Serial.print("CMD: ");
  Serial.print(cmd);
  Serial.print(" RPM: ");
  Serial.print(rpm);
  Serial.print(" V: ");
  Serial.print(voltage);
  Serial.print(" A: ");
  Serial.print(amp);
  Serial.print(" T: ");
  Serial.println(temp);
}

void loop() {
  static int    i, ret;

  //Serial.println(analogRead(A0));
  int16_t analog_throttle_read = map(analogRead(A0), 400, 1000, 0, 1999);
  analog_throttle_read =  constrain(analog_throttle_read, 0, 1999);
  ESC_Throttle[0] = analog_throttle_read;
  //ESC_Throttle[1] = myRead;


  if(true) { //your condition to keep the motors running safely
    ESCPID_comm_wd = 0;
  }

  // Check for next timer event
  ret = ESCCMD_tic( );  //disabled telemetry serial

  // Process timer event
  if ( ret == ESCCMD_TIC_OCCURED )  {

    // Read all measurements and compute current control signal
    for ( i = 0; i < ESCPID_NB_ESC; i++ ) {

      if ( ESCPID_comm_wd < ESCPID_COMM_WD_LEVEL ) {
        ret = ESCCMD_throttle( i, ESC_Throttle[i] );
        //Serial.print("Throttle ");
        //Serial.print(i);
        //Serial.print(" at ");
        //Serial.println(ESC_Throttle[i]);
      }
      print_esc_tlm(i);
    }
    
    // Update watchdog
    if ( ESCPID_comm_wd < ESCPID_COMM_WD_LEVEL )  {
      ESCPID_comm_wd++;
    }
  } 
}