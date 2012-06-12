// FIRMWARE FOR GIRONA 500 MAIN MONITOR
// ROS COMPATIBLE
// VICOROB (c) University Of Girona
// Programmer: Lluís Magí

#include <ros.h>
#include <std_msgs/String.h>
//#include <safety_g500/DigitalOutputControl.h>
#include <safety_g500/InternalSensors.h>
#include <safety_g500/MissionTimeout.h>  //Mission Timeout service
#include <safety_g500/DigitalOutput.h>  // DO service
#include <std_msgs/Empty.h>   // heart beat
#include <Wire.h>  // FOR i2c LIBRARY
#include <EEPROM.h>  // for internal EEPROM read/write

// ANALOG PINS
#define PRESSURE  0
#define HUMIDITY  1
// DIGITAL PINS
#define PC_ONOFF      36
#define DIGITAL0      30
#define DIGITAL1      28
#define DIGITAL2      24
#define DIGITAL3      26
#define DIGITAL4      22
#define DIGITAL5      29
#define DIGITAL6      23
#define DIGITAL7      27
#define DIGITAL8      31
#define DIGITAL9      25
#define DIGITAL10     33
#define DIGITAL11     35
#define DCDC12V       37
#define FOCO0         32
#define FOCO1         34
#define SYNC_IN       18
#define STROBE_IN     18
#define WATER_DET     53
#define BOARD_LED     13
#define RELEASE_EN_1  40
#define RELEASE_WD_1  39
#define RELEASE_EN_2  42
#define RELEASE_WD_2  41
#define MOTORS_ENABLED 43
#define CAM_TRIGGER   44

#define MAX_HUMIDITY  90
#define MAX_PRESSURE  2000
#define MAX_TEMP      60.0f

// TIME
#define HALPH_SECOND   10
#define FIVE_SECONDS   152
#define ONE_MINUTE     1831

// EEPROM STORAGE
#define ADDRESSL      0
#define ADDRESSH      1
#define CLEAN         0
#define OVERTEMP      1
#define OVERPRESS     2
#define OVERHUMI      3
#define WATER         4
#define BAT_OVERTEMP  5
#define BAT_OVERPRESS 6
#define BAT_OVERHUMI  7
#define BAT_WATER     8
#define PC_HALT       9
#define MISSION_TIMEOUT 10

#define PC_CILINDER  false
#define BAT_CILINDER true

typedef struct
{
  String id;      // PCs or BAT
  int humid;
  int press;
  float temper;
  boolean wat;
} fisicVar;

// Variables
fisicVar internalVar;
fisicVar externalVar;
String StreamExtCOM;
int timerCount;
int timeOutCount;
boolean timeOut;
boolean sendValues;
int totalTimeMision;
int currentTimeMision;
int timerMisionCount;
boolean PCwatchdog;
int PCwatchdogTimer;
boolean enabled;    // all systems enabled
boolean cilinder;
boolean focusState;
boolean focusBehavior;
int timerCountMotors;
boolean rescueON;
boolean sendMotorsData;
char motorCommand[10];

ros::NodeHandle  nh;
safety_g500::InternalSensors internal_sensor_msg;

ros::Publisher pubInternalSensors("/safety_g500/internal_sensors", &internal_sensor_msg);

void digitalOutputSrv(const safety_g500::DigitalOutput::Request & req, safety_g500::DigitalOutput::Response & res){
  if(enabled == true)
  {
    setDO (req.digital_out, req.value);
    res.success = true;
  }
  else
  {
    res.success = false;
  }
  
}
ros::ServiceServer<safety_g500::DigitalOutput::Request, safety_g500::DigitalOutput::Response> digital_output_server("digital_output",&digitalOutputSrv);


void missionTimeoutSrv(const safety_g500::MissionTimeout::Request & req, safety_g500::MissionTimeout::Response & res){
  if (req.start_mission == false)   // mission finalizes
  {
    enabled = false;
    STOP_ALL();
    rescue_robot_stop();
    digitalWrite(BOARD_LED, LOW);
    res.success = true;
  }
  else          // mission starts
  {
    totalTimeMision = req.time_out;
    if (totalTimeMision == 0)
    {
       enabled = false;
       res.success = false;
    }
    else
    {
      PCwatchdogTimer = 0;
      PCwatchdog = false;
      enabled = true;
     // rescue_robot_stop();
      digitalWrite(BOARD_LED, HIGH);
      res.success = true;
    }
    currentTimeMision = 0;
  }
}
ros::ServiceServer<safety_g500::MissionTimeout::Request, safety_g500::MissionTimeout::Response> mission_timeout_server("mission_timeout",&missionTimeoutSrv);


// empty message for PC alive
void messageHeartBeat(const std_msgs::Empty& message)
{
  PCwatchdogTimer = 0;
}
ros::Subscriber<std_msgs::Empty> sub_heart_beat("/safety_g500/heart_beat", &messageHeartBeat);

/// <summary>
/// Interrupt routine for TIMER3
/// </summary>
ISR(TIMER3_OVF_vect) // every (2 MHz --> 16bit counter) --> 32,768 mseg
{
  if (enabled)
  {
    // timer mision
    timerMisionCount++;
    if (timerMisionCount == ONE_MINUTE)
    {
      currentTimeMision++;
      timerMisionCount = 0;
    }
    // PC watchdog
    PCwatchdogTimer++;
    if (PCwatchdogTimer == ONE_MINUTE)
    {
      PCwatchdogTimer = 0;
      PCwatchdog = true;
    }
  }
  else
  {
    PCwatchdog = false;
    PCwatchdogTimer = 0;
    timerMisionCount = 0;
  }
  // parameters send Timer
  timerCount++;
  if(timerCount == FIVE_SECONDS)
  {
    timerCount = 0;
    sendValues = true;
  }
    // input serial data timeout Timer
  timeOutCount++;
  if(timeOutCount == FIVE_SECONDS)
  {
    timeOutCount = 0;
    timeOut = true;
  }
  // robot rescue
  if (rescueON == true)
  {
    timerCountMotors++;
    if(timerCountMotors == HALPH_SECOND)
    {
      timerCountMotors = 0;
      sendMotorsData = true;
    }
  }
}

/// <summary>
/// Main Setup function
/// </summary>
void setup() {
  nh.initNode();        // initialize ROS communication
  nh.advertiseService(digital_output_server);
  nh.advertiseService(mission_timeout_server);
  nh.advertise(pubInternalSensors);
  nh.subscribe(sub_heart_beat);
  
  // initialize board
  Serial2.begin(19200);  // serial port for external monitor
  Serial3.begin(57600);  // serial port for motors
  analogReference(DEFAULT);  // 5V REFERENCE
  pinMode(PC_ONOFF, OUTPUT);
  pinMode(DIGITAL0, OUTPUT);
  pinMode(DIGITAL1, OUTPUT);
  pinMode(DIGITAL2, OUTPUT);
  pinMode(DIGITAL3, OUTPUT);
  pinMode(DIGITAL4, OUTPUT);
  pinMode(DIGITAL5, OUTPUT);
  pinMode(DIGITAL6, OUTPUT);
  pinMode(DIGITAL7, OUTPUT);
  pinMode(DIGITAL8, OUTPUT);
  pinMode(DIGITAL9, OUTPUT);
  pinMode(DIGITAL10, OUTPUT);
  pinMode(DIGITAL11, OUTPUT);
  pinMode(DCDC12V, OUTPUT);
  pinMode(FOCO0, OUTPUT);
  pinMode(FOCO1, OUTPUT);
  pinMode(SYNC_IN, INPUT);
  pinMode(WATER, INPUT);
  pinMode(BOARD_LED, OUTPUT);
  pinMode(CAM_TRIGGER, OUTPUT);
  Wire.begin();    // I2C ENABLE AS A MASTER
  initTemp();     // INITIALIZE Temperature sensor
  timerCount = 0;
  sendValues = false;
  // Timer 2 setup
  TIMSK3=0x01; // enabled global and timer overflow interrupt;
  TCCR3A = 0x00; // normal operation page 148 (mode0);
  TCNT3=0x0000; // 16bit counter register
  TCCR3B = 0x02; // start timer/ set clock to clk(16MHz)/8 (pag 161)
  ///////
  StreamExtCOM = "";
  timeOut = false;
  digitalWrite(RELEASE_EN_1, LOW);
  digitalWrite(RELEASE_EN_2, LOW);
  digitalWrite(RELEASE_WD_1, LOW);
  digitalWrite(RELEASE_WD_2, LOW);
  digitalWrite(MOTORS_ENABLED, HIGH);
  digitalWrite(BOARD_LED, LOW);
  digitalWrite(CAM_TRIGGER, LOW);
  totalTimeMision = 0;
  currentTimeMision = 0;
  timerMisionCount = 0;
  PCwatchdog = false;
  PCwatchdogTimer = 0;
  enabled = false;
  internalVar.id = "PCs";
  externalVar.id = "BAT";
  cilinder = PC_CILINDER;
  externalVar.humid = 0;
  externalVar.press = 0;
  externalVar.temper = 0.0f;
  externalVar.wat = false;
  internalVar.humid = 0;
  internalVar.press = 0;
  internalVar.temper = 0.0f;
  internalVar.wat = false;
  focusState = false;
  focusBehavior = false;
  STOP_ALL();
  digitalWrite(DIGITAL5, HIGH);    // AHRS allways ON
  rescueON = false;
  sendMotorsData = false;
  strcpy(motorCommand, "u47+80\r");
}

/*
String floatToString(double number, uint8_t digits) 
{ 
  String resultString = "";
  // Handle negative numbers
  if (number < 0.0)
  {
     resultString += "-";
     number = -number;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i=0; i<digits; ++i)
    rounding /= 10.0;
  
  number += rounding;

  // Extract the integer part of the number and print it
  unsigned long int_part = (unsigned long)number;
  double remainder = number - (double)int_part;
  resultString += int_part;

  // Print the decimal point, but only if there are digits beyond
  if (digits > 0)
    resultString += "."; 

  // Extract digits from the remainder one at a time
  while (digits-- > 0)
  {
    remainder *= 10.0;
    int toPrint = int(remainder);
    resultString += toPrint;
    remainder -= toPrint; 
  } 
  return resultString;
}
*/

/// <summary>
/// Rescue robot routine. activate vertical trhusters to rescue the robot.
/// </summary>
void rescue_robot()
{
  motors(true);
  rescueON = true;
  digitalWrite(MOTORS_ENABLED, LOW);
}

/// <summary>
/// Rescue robot routine stop. Deactivate vertical trhusters to rescue the robot.
/// </summary>
void rescue_robot_stop()
{
  rescueON = false;
  motors(false);
  digitalWrite(MOTORS_ENABLED, HIGH);
}



/// <summary>
/// Interrupt routine for Focus Sync
/// </summary>

void videoSync()
{
  digitalWrite(FOCO0, !digitalRead(SYNC_IN));
  digitalWrite(FOCO1, !digitalRead(SYNC_IN));
}


/// <summary>
/// STOP ALL
/// </summary>
void STOP_ALL()
{
  enabled = false;
  for (int i = 0; i < 15; i++)
  {
    setDO(i, false);
  }
}

/// <summary>
/// Reads the Relativity humidity
/// </summary>
/// <returns>The Relativity humidity</returns>
int readHumidity ()
{
   float tem;
   int humi = analogRead(HUMIDITY);
   tem = float(humi) * 0.0049f;  // convert ADC value to volts
   // CONVERT TO Relative Humidity
   tem = tem / 5.0f;
   tem = tem - 0.16f;
   if (tem < 0.0f)
   {
      tem = 0.0f;
   }
   tem = tem / 0.0062f;
   
   return (int(tem));
}
/// <summary>
/// Reads the pressure in KPa
/// </summary>
/// <returns>The value in KPa</returns>
int readPressure ()
{
   float tem;
   int humi = analogRead(PRESSURE);
   tem = float(humi) * 0.0049f;  // convert ADC value to volts
   tem = tem - 0.5f;
   tem = tem * 2000.0f;
   tem = tem / 4.0f;
   return (int(tem));
}
/// <summary>
/// Initializes the TCN75 temperature sensor
/// </summary>
void initTemp ()
{
   byte val = 0;
   Wire.beginTransmission(0x48);
   Wire.write(val);
   Wire.endTransmission();
}
/// <summary>
/// Reads TCN75 sensor in degrees
/// </summary>
/// <returns>The value converted</returns>
float readTemperature ()
{
   float ret;
   int temp = 0;
   byte dataL = 0;
   byte dataH = 0;
   unsigned int temperature;
   
   Wire.requestFrom(0x48, 2);    // request 2 bytes from slave device
   while(Wire.available())    // slave may send less than requested
   {
     if(temp == 0)
     {
       dataH = Wire.read();
       temp++;
     }
     else
     {
       dataL = Wire.read();
     }
   }
   temperature = dataH;
   temperature <<= 8;
   temperature &= 0xFF00;
   temperature |= dataL;
   temperature >>= 7;
   temperature &= 0x01FF;
   ret = float(temperature) / 2.0f;
   return (ret);
}
/// <summary>
/// Resets all digital output bits
/// </summary>
void resetDO ()
{
   digitalWrite(DIGITAL0, LOW);
   digitalWrite(DIGITAL1, LOW);
   digitalWrite(DIGITAL2, LOW);
   digitalWrite(DIGITAL3, LOW);
   digitalWrite(DIGITAL4, LOW);
//   digitalWrite(DIGITAL5, LOW);
   digitalWrite(DIGITAL6, LOW);
   digitalWrite(DIGITAL7, LOW);
   digitalWrite(DIGITAL8, LOW);
   digitalWrite(DIGITAL9, LOW);
   digitalWrite(DIGITAL10, LOW);
   digitalWrite(DIGITAL11, LOW);
   digitalWrite(DCDC12V, LOW);    // DC DC 12V OFF
}
/// <summary>
/// Activates an output digital
/// </summary>
/// <param name="channel">The channel between 0 and 11</param>
/// <param name="value">The value (0 or 1) to be set</param>
void setDO (int channel, boolean value)
{
  switch(channel)
  {
    case 0:
      digitalWrite(DIGITAL0, value);
      break;
    case 1:
      digitalWrite(DIGITAL1, value);
      break;
    case 2:
      digitalWrite(DIGITAL2, value);
      break;
    case 3:
      digitalWrite(DIGITAL3, value);
      break;
    case 4:
      digitalWrite(DIGITAL4, value);
      break;
    case 5:
//      digitalWrite(DIGITAL5, value);
      break;
    case 6:
      digitalWrite(DIGITAL6, value);
      break;
    case 7:
      digitalWrite(DIGITAL7, value);
      break;
    case 8:
      digitalWrite(DIGITAL8, value);
      break;
    case 9:
      digitalWrite(DIGITAL9, value);
      break;
    case 10:
      digitalWrite(DIGITAL10, value);
      break;
    case 11:
      digitalWrite(DIGITAL11, value);
      break;
    case 12:    // motors
      motors(value);
      break;
    case 13:    // focus ON / OFF
      if (value == false) // FOCUS OFF
      {
        detachInterrupt(5);
        digitalWrite(FOCO0, LOW);
        digitalWrite(FOCO1, LOW);
        digitalWrite(DCDC12V, LOW);
        focusState = false;
      }
      else    // FOCUS ON
      {
        if (focusBehavior == false)  // FOCUS IN NORMAL STATE
        {
          detachInterrupt(5);
          digitalWrite(FOCO0, HIGH);
          digitalWrite(FOCO1, HIGH);
          digitalWrite(DCDC12V, HIGH);
        }
        else      // FOCUS IN SYNCRO STATE
        {
          attachInterrupt(5, videoSync, CHANGE);
          digitalWrite(FOCO0, LOW);
          digitalWrite(FOCO1, LOW);
          digitalWrite(DCDC12V, HIGH);
        }
        focusState = true;
      }
    break;
    case 14:  // focus normal / syncro
      if (value == true)  // syncro ON
      { 
        if (focusState == true)  // FOCUS ON
        {
          attachInterrupt(5, videoSync, CHANGE);
          digitalWrite(FOCO0, LOW);
          digitalWrite(FOCO1, LOW);
          digitalWrite(DCDC12V, HIGH);
        }
        focusBehavior = true;
      }
      else
      {
        detachInterrupt(5);
        if (focusState == true)  // FOCUS ON
        {
          digitalWrite(FOCO0, HIGH);
          digitalWrite(FOCO1, HIGH);
          digitalWrite(DCDC12V, HIGH);
        }
        else    // FOCUS OFF
        {
          digitalWrite(FOCO0, LOW);
          digitalWrite(FOCO1, LOW);
          digitalWrite(DCDC12V, LOW);
        }
        focusBehavior = false;
      }
      break;
      case 15:      // release control
        release(value);
        break;
      case 16:    // pc on/off control
        PConOff ();
        break;
  }
}
/// <summary>
/// Generates a pulse to turn ON or OFF the PC
/// </summary>
void PConOff ()
{
   digitalWrite(PC_ONOFF, HIGH);
   delay(300);
   digitalWrite(PC_ONOFF, LOW);
}
/// <summary>
/// Manual Release activation 
/// </summary>
/// <param byte = "nr">The desired release to activate '1' or '0'</param>
void release (boolean nr)
{
  if (nr == false)
  {
    digitalWrite(RELEASE_EN_1, HIGH);
    digitalWrite(RELEASE_WD_1, HIGH);
    delay(500);
    digitalWrite(RELEASE_EN_1, LOW);
    digitalWrite(RELEASE_WD_1, LOW);
  }
  else
  {
    digitalWrite(RELEASE_EN_2, HIGH);
    digitalWrite(RELEASE_WD_2, HIGH);
    delay(500);
    digitalWrite(RELEASE_EN_2, LOW);
    digitalWrite(RELEASE_WD_2, LOW);
  }
}
/// <summary>
/// Writes a value into the EEPROM memory
/// </summary>
/// <param byte = "type">The value type to write</param>
void eepromWrite (byte type)
{
  byte actual;
  
  switch(type)
  {
    case CLEAN:
      EEPROM.write(ADDRESSL, 0);
      EEPROM.write(ADDRESSH, 0);
    break;
    case OVERTEMP:
      actual = EEPROM.read(ADDRESSL);
      EEPROM.write(ADDRESSL, (actual | 0x01));
    break;
    case OVERPRESS:
      actual = EEPROM.read(ADDRESSL);
      EEPROM.write(ADDRESSL, (actual | 0x02));
    break;
    case OVERHUMI:
      actual = EEPROM.read(ADDRESSL);
      EEPROM.write(ADDRESSL, (actual | 0x04));
    break;
    case WATER:
      actual = EEPROM.read(ADDRESSL);
      EEPROM.write(ADDRESSL, (actual | 0x08));
    break;
    case BAT_OVERTEMP:
      actual = EEPROM.read(ADDRESSL);
      EEPROM.write(ADDRESSL, (actual | 0x10));
    break;
    case BAT_OVERPRESS:
      actual = EEPROM.read(ADDRESSL);
      EEPROM.write(ADDRESSL, (actual | 0x20));
    break;
    case BAT_OVERHUMI:
      actual = EEPROM.read(ADDRESSL);
      EEPROM.write(ADDRESSL, (actual | 0x40));
    break;
    case BAT_WATER:
      actual = EEPROM.read(ADDRESSL);
      EEPROM.write(ADDRESSL, (actual | 0x80));
    break;
    case PC_HALT:
      actual = EEPROM.read(ADDRESSH);
      EEPROM.write(ADDRESSH, (actual | 0x01));
    break;
    case MISSION_TIMEOUT:
      actual = EEPROM.read(ADDRESSH);
      EEPROM.write(ADDRESSH, (actual | 0x02));
    break;
  }
}

/// <summary>
/// Reads the EEPROM value and send results to PC
/// </summary>
void processEEPROM()
{
  byte count = 0;
  byte val = EEPROM.read(ADDRESSL);
  if (val & 0x01)
  {
//    str_msg.data = "OVER TEMPERATURE IN PCs CYLINDER;";
    count++;
  }
  if (val & 0x02)
  {
//    str_msg.data = "OVER PRESSURE IN PCs CYLINDER;";
    count++;
  }
  if (val & 0x04)
  {
//    str_msg.data = "OVER HUMIDITY IN PCs CYLINDER;";
    count++;
  }
  if (val & 0x08)
  {
//    str_msg.data = "WATER DETECTET IN PCs CYLINDER;";
    count++;
  }
  if (val & 0x10)
  {
//    str_msg.data = "OVER TEMPERATURE IN BATTERY CYLINDER;";
    count++;
  }
  if (val & 0x20)
  {
//    str_msg.data = "OVER PRESSURE IN BATTERY CYLINDER;";
    count++;
  }
  if (val & 0x40)
  {
//    str_msg.data = "OVER HUMIDITY IN BATTERY CYLINDER;";
    count++;
  }
  if (val & 0x80)
  {
//    str_msg.data = "WATER DETECTET IN BATTERY CYLINDER;";
    count++;
  }
  val = EEPROM.read(ADDRESSH);
  if (val & 0x01)
  {
 //   str_msg.data = "PC TIMEOUT;";
    count++;
  }
  if (val & 0x02)
  {
//    str_msg.data = "MISSION TIMEOUT;";
    count++;
  }
  if (count == 0)
  {
//    str_msg.data = "NO ALARMS;";
  }
//  ROSpublisher.publish( &str_msg );
}

/// <summary>
/// Activates or deactivates the motors
/// </summary>
/// <param boolean = "state">The desired vaule 1 or 0</param>
void motors (boolean state)
{
  if (state == true)
  {
    Serial2.write("D");
    delay(100);
    Serial2.write("2");
    delay(100);
    Serial2.write("1");
    delay(100);
    Serial2.write(167);
    delay(100);
    Serial2.write(13);
    delay(300);
    Serial2.write("D");
    delay(100);
    Serial2.write("2");
    delay(100);
    Serial2.write("1");
    delay(100);
    Serial2.write(167);
    delay(100);
    Serial2.write(13);
    delay(100);
  }
  else
  {
    Serial2.write("D");
    delay(100);
    Serial2.write("2");
    delay(100);
    Serial2.write("0");
    delay(100);
    Serial2.write(166);
    delay(100);
    Serial2.write(13);
    delay(300);
    Serial2.write("D");
    delay(100);
    Serial2.write("2");
    delay(100);
    Serial2.write("0");
    delay(100);
    Serial2.write(166);
    delay(100);
    Serial2.write(13);
    delay(100);
  }
}
/// <summary>
/// Process external monitor data
/// </summary>
/// <param string = "data">The string to process</param>
void processData(String data)
{
  char toChar[8];
  String str;
  long int val;
  float temp;
  int pos1 = data.indexOf(';');
  int pos2 = data.indexOf(';', pos1 + 1);
  // data format: M=m;HHHH;PPPP;TT.T;W;CC  = 23 bytes
  //externalVar.id = data;
  
  str = data.substring(pos1+1, pos2);
  val = str.toInt();
  temp = (float)val * 0.0049f;
  temp = temp / 5.0f;
  temp -= 0.16f;
  temp = temp / 0.0062;
  externalVar.humid = (int)temp;
  
  pos1 = data.indexOf(';', pos2 + 1);
  str = data.substring(pos2+1, pos1);
  val = str.toInt();
  temp = (float)val * 0.0049f;
  temp -= 0.5f;
  temp *= 50.0f;
  externalVar.press = (int)temp;
  
  pos2 = data.indexOf(';', pos1 + 1);
  str = data.substring(pos1+1, pos2);
  externalVar.temper = (float)str.toInt();
  
  externalVar.wat = data.charAt(pos2 + 1) - 48;

  if (externalVar.humid >= MAX_HUMIDITY)
  {
    STOP_ALL();
    eepromWrite(BAT_OVERHUMI);
  }
  if (externalVar.press >= MAX_PRESSURE)
  {
    STOP_ALL();
    eepromWrite(BAT_OVERPRESS);
  }
  if (externalVar.temper >= MAX_TEMP)
  {
    STOP_ALL();
    eepromWrite(BAT_OVERTEMP);
  }
  if (externalVar.wat != false)
  {
    STOP_ALL();
    eepromWrite(BAT_WATER);
  }
}
/// <summary>
/// Send Internal and external values 
/// </summary>
void sendValuesF(boolean id)
{
  String data;
//  char da[30];
  
  if (id ==false)  // PCs
  {
    internal_sensor_msg.id = "PCs";
    internal_sensor_msg.humidity = internalVar.humid;
    internal_sensor_msg.pressure = internalVar.press;
    internal_sensor_msg.temperature = internalVar.temper;
    internal_sensor_msg.water_detected = internalVar.wat;
  }
  else
  {
    //externalVar.id.toCharArray(da, 30);
    //internal_sensor_msg.id =  da;
    internal_sensor_msg.id = "BAT";
    internal_sensor_msg.humidity = externalVar.humid;
    internal_sensor_msg.pressure = externalVar.press;
    internal_sensor_msg.temperature = externalVar.temper;
    internal_sensor_msg.water_detected = externalVar.wat;
  }
  pubInternalSensors.publish( &internal_sensor_msg );
}
/// <summary>
/// Main loop function
/// </summary>
void loop() 
{
  int DOchannel;
  int DOVal;
//  boolean firstTime = true;
  
  internalVar.humid = readHumidity();
  internalVar.press = readPressure();
  internalVar.temper = readTemperature();
  internalVar.wat = HIGH - digitalRead(WATER_DET);

/*
    else if (command.charAt(0) == 'E')   // EEPROM CONTROL
    {
      if (command.charAt(1) == 'C')
      {
        eepromWrite(CLEAN);
      }
      else if (command.charAt(1) == 'R')
      {
        processEEPROM();
      }
  */
  // RS232 control
  while (Serial2.available())
  {
    byte dataIn = Serial2.read();
    timeOutCount = 0;
    
    if (dataIn == '\r')
    {
      if (StreamExtCOM.length() > 20)
      {
        processData (StreamExtCOM);
      }
      StreamExtCOM = "";
    }
    else if (StreamExtCOM.length() > 48)
    {
        StreamExtCOM = "";
    }
    else
    {
      StreamExtCOM += (char)dataIn;
    }
  }
  // RS232 from external monitor timeout
  if (timeOut == true)
  {
    timeOut = false;
    externalVar.humid = 0;
    externalVar.press = 0;
    externalVar.temper = 0.0f;
    externalVar.wat = false;
  }
  // "send internal values to PC" Timer and verify parameters
  if (sendValues == true)
  {
    if (enabled)
    {
      sendValuesF(cilinder);
      cilinder = !cilinder;
    }
    if (internalVar.humid > MAX_HUMIDITY)
    {
      STOP_ALL();
      eepromWrite(OVERHUMI);
    }
    if (internalVar.press > MAX_PRESSURE)  // overpressure
    {
      STOP_ALL();
      eepromWrite(OVERPRESS);
    }
    if (internalVar.temper > MAX_TEMP)  // overTemperature
    {
      STOP_ALL();
      eepromWrite(OVERTEMP);
    }
    if (internalVar.wat != false)  // water detected
    {
      STOP_ALL();
      eepromWrite(WATER);
    }
    sendValues = false;
  }

  // WatchDog for PC is alive
  if (( enabled == true ) && (PCwatchdog == true))  // PC timeout
  {
      PCwatchdog = false;
      //STOP_ALL();
      eepromWrite(PC_HALT);
      rescue_robot();
  }

  // mision time controller
  if (( enabled == true ) && (currentTimeMision == totalTimeMision))
  {
    //STOP_ALL();
    eepromWrite(MISSION_TIMEOUT);
    rescue_robot();
  }
  // robot rescue
  if(rescueON == true)
  {
    if (sendMotorsData == true)
    {
      sendMotorsData = false;
      motorCommand[1] = '4';  // motor ID = 4
      Serial3.write(motorCommand);
      delay(50);
      motorCommand[1] = '5';  // motor ID = 5
      Serial3.write(motorCommand);
      delay(50);
    }
  }
  
  nh.spinOnce(); // ROS transactions
}
