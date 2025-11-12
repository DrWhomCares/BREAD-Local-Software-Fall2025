#include <Wire.h>
#include "max6675.h"
#include <FastLED.h>
#include <PID_v1.h>

#define I2C_ADR 10

#define ESTOP   2
#define LED_PIN 5
#define CS1     10
#define CS2     11
#define DATA    12
#define CLK     13
#define RELAY1  6
#define RELAY2  7

#define THERMO_UPDATE_TIME_MS   300
#define SERIAL_UPDATE_TIME_MS   1000

long lastThermoRead = 0;
long lastSerialPrint = 0;

//auto tuning variables
int autoCheck = 0;
int checkOsc = 0;
int peak = 0;

float bio_auto[5] = {0,0,0,0,0}; // {maxTemp, prevTemp, prevTime, Peak1, prevDeriv}

//Edit these offsets to calibrate thermocouples as needed
double thermo1_offset = 4.0;
double thermo2_offset = 4.0;

bool E_STOP = false;

CRGB led;
int hue = 0;

struct Timer_t
{
  unsigned long timeStart;
  bool running = false;
  float time(){
    if(running){
      return ((millis() - timeStart) / 1000.0);
    } else {
      return 0;
    }
  }
  void start(){
    timeStart = millis();
    running = true;
  }
  void stop(){
    running = false;
  }
  void restart(){
    timeStart = millis();
  }
}timer;

struct RLHT_t
{
  double heatSetpoint_1 = 0;   // set desired temperature for heater 1 (if using relay as heater actuator)
  double relay1Input;  // input to relay 1 taken from selected thermocouple
  double rOnTime_1;           // time relay is on in ms (0-period)
  int rPeriod_1 = 2000;          // duration of relay 1 cycle in ms (1000ms = 1s)
  double Kp_1 = 0;             // PID proportional gain for heater 1
  double Ki_1 = 0;             // PID integral gain for heater 1
  double Kd_1 = 0;             // PID derivative gain for heater 1 
  double heatSetpoint_2 = 0;
  double relay2Input;
  double rOnTime_2;
  int rPeriod_2 = 2000;
  double Kp_2 = 0;
  double Ki_2 = 0;
  double Kd_2 = 0;
  double thermo1;   // thermocouple 1 measurement
  double thermo2;
  char thermoSelect[2];   // select which thermocouples pair with relays {relay1, relay2}
} RLHT, RLHT_old, RLHT_auto;

union FLOATUNION_t //Define a float that can be broken up and sent via I2C
{
  float number;
  uint8_t bytes[4];
};

// initialize the Thermocouples
MAX6675 CH1(CLK, CS1, DATA);
MAX6675 CH2(CLK, CS2, DATA);

//Specify the links and initial tuning parameters
PID relay1PID(&(RLHT.relay1Input), &(RLHT.rOnTime_1), &(RLHT.heatSetpoint_1), RLHT.Kp_1, RLHT.Ki_1, RLHT.Kd_1, DIRECT);
PID relay2PID(&(RLHT.relay2Input), &(RLHT.rOnTime_2), &(RLHT.heatSetpoint_2), RLHT.Kp_2, RLHT.Ki_2, RLHT.Kd_2, DIRECT);

unsigned long relay1StartTime;
unsigned long relay2StartTime;

void estop()
{
  if(digitalRead(ESTOP) == HIGH)    // when set to HIGH state
  {
    led = CRGB::Red;
    FastLED.show();

    // save states
    RLHT_old.heatSetpoint_1 = RLHT.heatSetpoint_1;
    RLHT_old.heatSetpoint_2 = RLHT.heatSetpoint_2;
    RLHT_old.rOnTime_1 = RLHT.rOnTime_1;
    RLHT_old.rOnTime_2 = RLHT.rOnTime_2;

    // set critical states to zero and turn off relays
    RLHT.heatSetpoint_1 = 0;
    RLHT.heatSetpoint_2 = 0;
    RLHT.rOnTime_1 = 0;
    RLHT.rOnTime_2 = 0;

    RLHT_auto.heatSetpoint_1 = 0;
    RLHT_auto.heatSetpoint_2 = 0;
    RLHT_auto.rOnTime_1 = 0;
    RLHT_auto.rOnTime_2 = 0;

    digitalWrite(RELAY1, LOW);
    digitalWrite(RELAY2, LOW);

    E_STOP = true;
    Serial.println("ESTOP PRESSED!");
  }
  else    // when ESTOP state is LOW
  {
    led = CRGB::Black;
    FastLED.show();

    // reassign old states
    RLHT_old.heatSetpoint_1 = RLHT_old.heatSetpoint_1;
    RLHT_old.heatSetpoint_2 = RLHT_old.heatSetpoint_2;
    RLHT_old.rOnTime_1 = RLHT_old.rOnTime_1;
    RLHT_old.rOnTime_2 = RLHT_old.rOnTime_2;

    E_STOP = false;

    Serial.println("ESTOP RELEASED!");
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  FastLED.addLeds<NEOPIXEL, LED_PIN>(&led, 1);

  RLHT.thermoSelect[0] = 1;
  RLHT.thermoSelect[1] = 2;

  pinMode(ESTOP, INPUT);
  attachInterrupt(digitalPinToInterrupt(ESTOP), estop, CHANGE);

  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);

  Wire.begin(I2C_ADR);
  Wire.onRequest(requestEvent);
  Wire.onReceive(receiveEvent);
  
  //tell the PID to range between 0 and the full window size
  relay1PID.SetOutputLimits(0, RLHT.rPeriod_1);
  relay2PID.SetOutputLimits(0, RLHT.rPeriod_2);

  //turn the PID on
  relay1PID.SetMode(AUTOMATIC);
  relay2PID.SetMode(AUTOMATIC);

  relay1StartTime = millis();
  relay2StartTime = millis();

  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:

  measureThermocouples();
  if(!E_STOP)
  {
    if(autoCheck == 1){
      autoTune();
      actuateRelays();
      printOutput();
      return;
    }


    setPIDTunings();
    // assign thermocouple readings to relay inputs
    switch(RLHT.thermoSelect[0]){
      case 1:   // thermocouple 1 to relay 1 
        RLHT.relay1Input = RLHT.thermo1;
        break;
      case 2:   // thermocouple 2 to relay 1
        RLHT.relay1Input = RLHT.thermo2;
        break;
    }
    if(isnan(RLHT.relay1Input))
      RLHT.rOnTime_1 = 0;
    else
      relay1PID.Compute();

    switch(RLHT.thermoSelect[1]){
      case 1:   // thermocouple 1 to relay 2 
        RLHT.relay2Input = RLHT.thermo1;
        break;
      case 2:   // thermocouple 2 to relay 2
        RLHT.relay2Input = RLHT.thermo2;
        break;
    }
    if(isnan(RLHT.relay2Input))
      RLHT.rOnTime_2 = 0;
    else
      relay2PID.Compute();

    actuateRelays();
    printOutput();
  }
}
void autoTune(){
  //float tmp = 10; not a real value  
  /*Serial.print("Current Kp: ");
  Serial.println(tmp);
  Serial.print("time");
  Serial.println(time);
  Serial.println("temp: ");
  Serial.println(temp);
  Serial.print("setpoint: ");
  Serial.println(setpoint);
  Serial.println("temp bio: ");
  */
  Serial.println("here");
  double t_in = RLHT.thermo1; // default
  if (RLHT.thermoSelect[0] == 2) t_in = RLHT.thermo2;

  RLHT_auto.thermo1 = t_in;                  // for your derivative/peak logic
  RLHT.relay1Input  = t_in;                  // feed the PID input
  RLHT.heatSetpoint_1 = RLHT_auto.heatSetpoint_1;   // make sure PID drives toward the autotune setpoint

  // make sure the PID uses the current (possibly adjusted) Kp during autotune
  relay1PID.SetTunings(RLHT_auto.Kp_1, 0, 0);
  // keep output window limits
  relay1PID.SetOutputLimits(0, RLHT.rPeriod_1);
  // compute new on-time for the current window
  relay1PID.Compute();

  if(bio_auto[2] == 0 && autoCheck == 1)
  {
    bio_auto[2] = timer.time();
  }
  //we could just increase the gain super slowly
  if (autoCheck == 1 && timer.time() - bio_auto[2] > 20)
  {
    //((current T - previous T) / (current time - previous timee))
    double dt = timer.time() - bio_auto[2];
    double driv = (dt > 0.0) ? (RLHT_auto.thermo1 - bio_auto[1]) / dt : 0.0;
    
    Serial.print("currentTemp: ");
    Serial.println(RLHT_auto.thermo1);
    Serial.print("Deriv: ");
    Serial.println(driv);
    Serial.print("prevDeriv: ");
    Serial.println(bio_auto[4]);
    Serial.print("setpoint: ");
    Serial.println(RLHT_auto.heatSetpoint_1);
    Serial.print("Max Temp: ");
    Serial.println(bio_auto[0]);
    
    //if T is less than 2% of Setpoint and the derivative is decreasing and it isn't oscillating...
    if ((RLHT_auto.thermo1 < RLHT_auto.heatSetpoint_1 - (RLHT_auto.heatSetpoint_1 * 0.02)) && driv < 0.02 && checkOsc == 0)
    {
        //double Ku and send new gains
        RLHT_auto.Kp_1 = RLHT_auto.Kp_1 * 2;
        //RLHTCommandPID(address, heater, bio_post_heater_pid[1][1], 0, 0);
        relay1PID.SetTunings(RLHT_auto.Kp_1, 0, 0);
        Serial.print("Kp Change: ");
        Serial.println(RLHT_auto.Kp_1); 
    }
    else if(RLHT_auto.thermo1 > RLHT_auto.heatSetpoint_1 || checkOsc == 1)
    {
      //check for oscillations
      checkOsc = 1;
      //If MaxTemp is less than current temp, set new peak 1 max and continue
      if(bio_auto[0] < RLHT_auto.thermo1)
      {
        bio_auto[0] = RLHT_auto.thermo1;
      }
      //If previous T is smaller than current, set new maximum T
      else if(bio_auto[1] < RLHT_auto.thermo1){
        //increasing to peak 2
        bio_auto[0] = RLHT_auto.thermo1;
      }
      //If max observed T is larger than new and we have not found the first peak, set it as the first peak
      if(bio_auto[0] > RLHT_auto.thermo1 && peak == 0){
        bio_auto[3] = bio_auto[0];
        bio_auto[0] = 0;
        peak = 1;
        Serial.println("first peak");
        timer.restart();
        bio_auto[2] = 0;
        Serial.println("time reset");
      }
      //If max observed T is larger than new and we have found the first peak...
      else if (bio_auto[0] > RLHT_auto.thermo1 && peak == 1){
        //Find the delta between the secondpeak-firstpeak
        float delp = bio_auto[0]-bio_auto[3];
        Serial.print("peak 1 temp: ");
        Serial.println(bio_auto[3]);
        Serial.print("delp: ");
        Serial.println(delp);
        //If the delta is greater than 0, change Ku
        if(delp > 0){
          RLHT_auto.Kp_1 = (RLHT_auto.Kp_1 / 1.5); //or Ku - (Ku/2)
          //RLHTCommandPID(address, heater,bio_post_heater_pid[1][1],0,0);
          relay1PID.SetTunings(RLHT_auto.Kp_1, 0, 0);
        }
        else{
          float time = timer.time();
          //reset Command
          //RLHTCommandPID(address, heater, 0, 0, 0);
          relay1PID.SetTunings(0,0,0);
          //add values to

          //bio_heater_auto_pid_vals[0] = 0.6*bio_post_heater_pid[1][1];
          //bio_heater_auto_pid_vals[1] = (1.2*bio_post_heater_pid[1][1])/time;
          //bio_heater_auto_pid_vals[2] = 0.075*bio_post_heater_pid[1][1]*time;
          RLHT_auto.Kp_1 = 0.6*RLHT_auto.Kp_1;
          RLHT_auto.Ki_1 = (1.2*RLHT_auto.Kp_1)/time;
          RLHT_auto.Kd_1 = (0.075*RLHT_auto.Kp_1)*time;
          RLHT.Kp_1 = RLHT_auto.Kp_1;
          RLHT.Ki_1 = RLHT_auto.Ki_1;
          RLHT.Kd_1 = RLHT_auto.Kd_1;
          relay1PID.SetTunings(RLHT.Kp_1, RLHT.Ki_1, RLHT.Kd_1);


          checkOsc = 0;
          autoCheck = 0;
          timer.stop();
          bio_auto[2] = 0;
          Serial.println("hit");
        }
      }
    }
    //if(bio_auto[2] != 0){
    //  bio_auto[2] = timer.time();
    //}
    //saving old time, temp, and derivative
    
    bio_auto[1] = RLHT_auto.thermo1;
    bio_auto[2] = timer.time();
    bio_auto[4] = driv;
    RLHT.heatSetpoint_1 = RLHT_auto.heatSetpoint_1;
    Serial.println("RLHT set to auto");
  }
}

void setPIDTunings()
{
  relay1PID.SetTunings(RLHT.Kp_1, RLHT.Ki_1, RLHT.Kd_1);
  relay2PID.SetTunings(RLHT.Kp_2, RLHT.Ki_2, RLHT.Kd_2);
}

void printOutput()
{
  if(millis() - lastSerialPrint >= SERIAL_UPDATE_TIME_MS)
  {
    Serial.print("T1: ");
    Serial.print(RLHT.thermo1);
    Serial.print("\tT2: ");
    Serial.print(RLHT.thermo2);

    Serial.print("\tPID1: ");
    Serial.print(RLHT.Kp_1);
    Serial.print(",");
    Serial.print(RLHT.Ki_1);
    Serial.print(",");
    Serial.print(RLHT.Kd_1);

    Serial.print("\tRelay1Input: ");
    Serial.print(RLHT.relay1Input);
    Serial.print("\tSetpoint: ");
    Serial.print(RLHT.heatSetpoint_1);
    Serial.print("\tonTime2: ");
    Serial.print((int)(RLHT.rOnTime_1));

    Serial.print("\tPID2: ");
    Serial.print(RLHT.Kp_2);
    Serial.print(",");
    Serial.print(RLHT.Ki_2);
    Serial.print(",");
    Serial.print(RLHT.Kd_2);

    Serial.print("\tRelay2Input: ");
    Serial.print(RLHT.relay2Input);
    Serial.print("\tSetpoint: ");
    Serial.print(RLHT.heatSetpoint_2);
    Serial.print("\tonTime2: ");
    Serial.println((int)(RLHT.rOnTime_2));

    lastSerialPrint = millis();
  }
}

void measureThermocouples()
{
  if(millis() - lastThermoRead >= THERMO_UPDATE_TIME_MS)
  {
    RLHT.thermo1 = CH1.readCelsius() + thermo1_offset;
    RLHT.thermo2 = CH2.readCelsius() + thermo2_offset;
    lastThermoRead = millis();
  }
}

void actuateRelays()
{
  if(RLHT.heatSetpoint_1 == 0)
    RLHT.rOnTime_1 = 0;
  if(RLHT.heatSetpoint_2 == 0)
    RLHT.rOnTime_2 = 0;
  // Relay 1
  if (millis() - relay1StartTime > RLHT.rPeriod_1)
  { //time to shift the Relay Window
    relay1StartTime += RLHT.rPeriod_1;
  }
  if ((int)(RLHT.rOnTime_1) > millis() - relay1StartTime) digitalWrite(RELAY1, HIGH);
  else digitalWrite(RELAY1, LOW);

  // Relay 2
  if (millis() - relay2StartTime > RLHT.rPeriod_2)
  { //time to shift the Relay Window
    relay2StartTime += RLHT.rPeriod_2;
  }
  if ((int)(RLHT.rOnTime_2) > millis() - relay2StartTime) digitalWrite(RELAY2, HIGH);
  else digitalWrite(RELAY2, LOW);
}

void requestEvent()
{
  FLOATUNION_t t1;
  FLOATUNION_t t2;

  if(isnan(RLHT.thermo1))
    t1.number = 0;
  else
    t1.number = RLHT.thermo1;

  if(isnan(RLHT.thermo2))
    t2.number = 0;
  else
    t2.number = RLHT.thermo2;

  for (int i = 0; i < 4; i++) Wire.write(t1.bytes[i]);
  for (int i = 0; i < 4; i++) Wire.write(t2.bytes[i]);
}

/* Command layout
  Byte 1:     T (want temperature reading)
              H (change heater setpoint)
              P (change PID tuning)
  Byte 2:     1 (first heater/thermocouple)
              2 (second heater/thermocouple)
  Byte 3-6:   Kp or heatSetpoint
  Byte 7-10:  Ki
  Byte 11-14: Kd
*/

void receiveEvent(int howMany)
{
  byte in_char;
  char in_data[20];

  led = CRGB::Green;
  FastLED.show();

  //Serial.println("Receiving: ");
  int i=0;
  while(Wire.available())
  {
    in_char = Wire.read();
    in_data[i] = in_char;
    Serial.print((byte)in_data[i]);
    Serial.print(",");
    i++;
  }
  Serial.println();

  setParametersRLHT(in_data);

  led = CRGB::Black;
  FastLED.show();
}

/*
RLHT Command Format:
  H,1,setpoint,2,0:   temperature setpoint between relay 1 and thermo 2. Enable reverse response = 0 (false)
  P,Kp,Ki,Kd:       PID tuning
*/

void setParametersRLHT(char *in_data)
{
  FLOATUNION_t float1;
  FLOATUNION_t float2;
  FLOATUNION_t float3;

  switch(in_data[0])
  {
    case 'H':
      for(int i=0; i<4; i++) float1.bytes[i] = in_data[i+2];
      if(in_data[1] == 1){
        RLHT.heatSetpoint_1 = float1.number;
        RLHT.thermoSelect[0] = in_data[6];
        relay1PID.SetControllerDirection(in_data[7]);   // Direct = 0, Reverse = 1
      }
      if(in_data[1] == 2){
        RLHT.heatSetpoint_2 = float1.number;
        RLHT.thermoSelect[1] = in_data[6];
        relay2PID.SetControllerDirection(in_data[7]);   // Direct = 0, Reverse = 1
      }
      autoCheck = 0;
      break;
    case 'P':
      for(int i=0; i<4; i++)  // populate variables for PID tuning
      {
        float1.bytes[i] = in_data[i+2];
        float2.bytes[i] = in_data[i+6];
        float3.bytes[i] = in_data[i+10];
      }
      if(in_data[1] == 1)
      {
        RLHT.Kp_1 = (double)float1.number;
        RLHT.Ki_1 = (double)float2.number;
        RLHT.Kd_1 = (double)float3.number;
      }
      if(in_data[1] == 2)
      {
        RLHT.Kp_2 = (double)float1.number;
        RLHT.Ki_2 = (double)float2.number;
        RLHT.Kd_2 = (double)float3.number;
      }
      autoCheck = 0;
      break;
     case 'A':
      Serial.println("set prams");
      for(int i=0; i<4; i++){
        float1.bytes[i] = in_data[i+2];
        float2.bytes[i] = in_data[i+6];
        float3.bytes[i] = in_data[i+10];
      }
      if(in_data[1] == 1){
        RLHT_auto.heatSetpoint_1 = float1.number;
        RLHT_auto.Kp_1 = float2.number;
        
      }
      autoCheck = 1;
      peak = 0; checkOsc = 0;
      bio_auto[0] = bio_auto[1] = bio_auto[2] = bio_auto[3] = bio_auto[4] = 0;

      if(!timer.running){
        timer.start();
      }else{
        timer.restart();
      }
      break;
  }
}
