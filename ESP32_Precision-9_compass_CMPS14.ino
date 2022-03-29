#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include <EEPROM.h>             // include library to read and write from flash memory
const uint8_t EEPROM_SIZE = 1 + (sizeof(float) * 3 * 4) + 4;

enum EEP_ADDR
{
    EEP_HEADING_OFFSET  = 0x00,
    EEP_HEEL_OFFSET     = 0x01,
    EEP_TRIM_OFFSET     = 0x02,
    EEP_AUTOCALIBRATION = 0x03
};

//Address of the CMPS14 compass on i2c
#define _i2cAddress 0x60

#define CONTROL_Register 0

#define HEADING_Register 2 
#define PITCH_Register 4 
#define PITCH_Register16 26 
#define ROLL_Register 5
#define ROLL_Register16 28

#define MAGNETX_Register  6
#define MAGNETY_Register  8
#define MAGNETZ_Register 10

#define ACCELEROX_Register 12
#define ACCELEROY_Register 14
#define ACCELEROZ_Register 16

#define GYROX_Register 18
#define GYROY_Register 20
#define GYROZ_Register 22

#define CALIBRATION_Register 30

int calibrationStatus[8];

#define ONE_BYTE   1
#define TWO_BYTES  2
#define FOUR_BYTES 4
#define SIX_BYTES  6

float Pi = 3.1415926;

//---------------------------------

byte _byteHigh;
byte _byteLow;
byte Byte;

  // Please note without clear documentation in the technical documenation
  // it is notoriously difficult to get the correct measurement units.
  // I've tried my best, and may revise these numbers.
  
  int nReceived;
  float pitch;
  float roll;

  float magnetX = 0;
  float magnetY = 0;
  float magnetZ = 0;
  float yaw;

  float accelX = 0;
  float accelY = 0;
  float accelZ = 0;
  // The acceleration along the X-axis, presented in mg 
  // See BNO080_Datasheet_v1.3 page 21
  float accelScale = 9.80592991914f/1000.f; // 1 m/s^2
  
  float gyroX = 0;
  float gyroY = 0;
  float gyroZ = 0;
  // 16bit signed integer 32,768
  // Max 2000 degrees per second - page 6
  float gyroScale = 1.0f/16.f; // 1 Dps

float pitchFactoryCalibration = 1.3;
float rollFactoryCalibration = -0.3;
  
#define CAN_RX_PIN GPIO_NUM_34
#define CAN_TX_PIN GPIO_NUM_32
#include "N2kMessages.h"
#include <NMEA2000_esp32.h>

#define LED_BUILTIN 2

// List here messages your device will transmit.
const unsigned long TransmitMessagesCompass[] PROGMEM = { 127250L, 127251L, 127257L , 0 }; //Vessel Heading, Rate of Turn, Attitude

#define DEV_COMPASS 0 // 60-140

tNMEA2000* nmea2000;
tN2kMsg N2kMsg;
tN2kMsg N2kMsgReply;
int DEVICE_ID = 65;
int SID;
bool n2kConnected = false;


// CMPS14 read error states
bool compassError = false;
bool pitchrollError = false;
bool gyroError = false;
bool accelError = false;

// B&G calibration stop/start
bool calibrationStart = false;
bool calibrationStop = false;
bool calibrationFinished = false;
 
// Deviation variables
float Deviation[128];
bool  calibrationRecording = false;
unsigned long t_dev;
int dev_num = 0;

const int numReadings = 250;
double readings[numReadings];     // the readings from the analog input
int readIndex = 0;                // the index of the current reading
int total = 0;                    // the running total
int average = 0;                  // the average

unsigned long t_next;             // Timer

// ----- Arduino pin definitions
byte intPin = 2;                  // Interrupt pin (not used) ... 2 and 3 are the Arduinos ext int pins

bool showOutput = false;
char InputChar;                   // incoming characters stored here

// ----- software timer
unsigned long Timer1 = 500000L;   // 500mS loop ... used when sending data to to Processing
unsigned long Stop1;              // Timer1 stops when micros() exceeds this value

float heading, heading_mag, heading_true, heading_variation = 0,
      heading_offset_rad = 0,
      heel_offset_rad = 0, 
      trim_offset_rad = 0; 
int8_t heading_offset_deg = 0,      
     heel_offset_deg = 0,
     trim_offset_deg = 0;
bool send_heading = true;                           // to do 20 vs 10hz
bool send_heading_true = false;                     // If we have variation, send True
unsigned char compass_autocalibration = 0x00;

void setup()
{
  Wire.begin(16,17);
  Wire.setClock(400000);                            // 400 kbit/sec I2C speed
  Serial.begin(115200);
  while (!Serial);

  // Setup LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  delay(500);
  
  // ----- Display title
  Serial.println("\nCMPS14 Compass emulating B&G Precision-9");

  // showCMPSVersion();

  // turnOffAutoCalibration();
  turnOnAutoCalibration();

  delay(200);

  Serial.println("EEPROM available.");
  if (!EEPROM.begin(EEPROM_SIZE))
  {
    Serial.println("EEPROM start failed");
  }

  // clearCalibration();
  readCalibration();
  
  delay(200);

  // Initialise canbus
  Serial.println("Set up NMEA2000 device");
  nmea2000 = new tNMEA2000_esp32(CAN_TX_PIN, CAN_RX_PIN);

  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);
  
  // nmea2000->SetDeviceCount(1);
  // nmea2000->SetInstallationDescription1("");
  // nmea2000->SetInstallationDescription2("");
  nmea2000->SetProductInformation("107018103", // Manufacturer's Model serial code
                                 13233, // Manufacturer's product code
                                 "Precision-9 Compass",  // Manufacturer's Model ID
                                 "2.0.3-0",  // Manufacturer's Software version code
                                 "", // Manufacturer's Model version
                                 1,  // load equivalency *50ma
                                 0xffff, // NMEA 2000 version - use default
                                 0xff, // Sertification level - use default
                                 DEV_COMPASS
                                );
 
  // Set device information
  nmea2000->SetDeviceInformation(1048678, // Unique number. Use e.g. Serial number.
                                140, // Device function=Temperature See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
                                60, // Device class=Sensor Communication Interface. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20%26%20function%20codes%20v%202.00.pdf
                                275, // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                                4,
                                DEV_COMPASS
                               );
  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, DEVICE_ID);
  nmea2000->EnableForward(false);
  nmea2000->SetForwardStream(&Serial);
  nmea2000->SetForwardType(tNMEA2000::fwdt_Text);
  nmea2000->ExtendTransmitMessages(TransmitMessagesCompass, DEV_COMPASS);
  nmea2000->SetN2kCANMsgBufSize(20);
  nmea2000->SetMsgHandler(HandleNMEA2000Msg);
  nmea2000->Open();

  Serial.println("Press 'o' to enable serial output.");
  t_next = 0;
}

void loop()
{

  if (t_next + 50 <= millis())                  // Set timer for 50ms
  {
    t_next = millis();                        
    // Read the Compass
    ReadCompass();
    // Read the Accelerator
    // ReadAccelerator(); // Not using it now
    // Read the Gyroscope
    ReadGyro();
    calc_heading();
    // Read 16 bit Pitch and roll
    pitch = getPitch16();
    pitch += pitchFactoryCalibration;
    roll = getRoll16();
    roll += rollFactoryCalibration;
    applyOffsetPitchRoll();

    readCalibrationStatus();
    
    printValues();

    if (n2kConnected && send_heading && compassError == false) {
      if (send_heading_true) {
        SetN2kPGN127250(N2kMsg, SID, heading_true * DEG_TO_RAD, N2kDoubleNA, N2kDoubleNA, N2khr_true);
        nmea2000->SendMsg(N2kMsg, DEV_COMPASS);
      } else {
        SetN2kPGN127250(N2kMsg, SID, heading_mag * DEG_TO_RAD, N2kDoubleNA, N2kDoubleNA, N2khr_magnetic);
        nmea2000->SendMsg(N2kMsg, DEV_COMPASS);
      }
      if (gyroError == false) {
        SetN2kRateOfTurn(N2kMsg, SID, gyroZ * DEG_TO_RAD * -1); // radians
        nmea2000->SendMsg(N2kMsg, DEV_COMPASS);
      }
    }
    send_heading = !send_heading;             // Toggle to do 10hz

    if (n2kConnected && pitchrollError == false) {
      SetN2kAttitude(N2kMsg, SID, N2kDoubleNA, pitch * DEG_TO_RAD, roll * DEG_TO_RAD);
      nmea2000->SendMsg(N2kMsg, DEV_COMPASS);
    }
    
    // Send 130824 performance packet
    if (n2kConnected && pitchrollError == false && SetN2kPGN130824(N2kMsg)) {  // pitch roll
      nmea2000->SendMsg(N2kMsg, DEV_COMPASS);
    }

    SID++; if (SID > 253) {
      SID = 1;
    }
      // Check for commands
    if (Serial.available()) {
      InputChar = Serial.read();
      if (InputChar == 'o') {
        showOutput = !showOutput;
      }
      if (InputChar == 'd') {
        Serial.println("Starting deviation calibration. Start turning at a steady 3 degrees/second and press 's' to start.");
        calibrationStart = true;
      }
      if (InputChar == 's' &&  calibrationRecording == false) {
        Serial.println("Calibration recording started. Keep turning at a steady 3 degrees/second for 390 degrees.");
         calibrationRecording = true;
      }
    }
    if (calibrationStart == true && calibrationStop == false) {
        Serial.println("Calibration recording started. Keep turning at a steady 3 degrees/second for 390 degrees.");
        calibrationRecording = true;
    }
    if ( calibrationRecording) {
      recordDeviation();
      t_dev = millis();
    }
    if (calibrationStop == true && calibrationStart == true) {
      calibrationStart = false;
      calibrationStop = false;
       calibrationRecording = false;
      Serial.println("Calibration cancelled.");
    }
    if (calibrationFinished) {
      Serial.println("Calibration finished.");
       calibrationRecording = false;
      calibrationStart = false;
      showDeviationTable();
    }
    ToggleLed();                              // Toggle led
  }
  nmea2000->ParseMessages();

}

void recordDeviation() {
  // Record compass headings to create deviation table
  // With 3 degrees/second we can record the heading every 0.5 seconds in a table of 120 measurements
  if (millis() > t_dev) {
    t_dev += 500; // Next measurement to happen 500ms from now
    if (calibrationRecording) {    
      Deviation[dev_num] = heading;
      Serial.printf("Rate of turn: % 5.2f dps   Deviation[%3d] = %05.2f\n", gyroZ, dev_num, Deviation[dev_num]);
      dev_num++;
      if (dev_num > 120) {
        dev_num = 0;
      }
    } else {
      Serial.printf("Rate of turn: % 3.0f dps   Press 's' to start recording.\n", gyroZ);  
    }
  }
}

void showDeviationTable() {
  Serial.printf("num heading\n");  
  for (dev_num = 0; dev_num <= 120; dev_num++) {
    Serial.printf("%3d, %05.2f\n", dev_num, Deviation[dev_num]);
  }
}

int num_n2k_messages = 0;

void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  // N2kMsg.Print(&Serial);
  // Serial.printf("PGN: %u\n", (unsigned int)N2kMsg.PGN);
  
  n2kConnected = true;
  
  switch (N2kMsg.PGN) {
    case 130850L:
      if (ParseN2kPGN130850(N2kMsg)) {
        Serial.printf("calibrationStart: %s  calibrationStop: %s\n", String(calibrationStart), String(calibrationStop));
      }
      break;
    case 127258L:
      unsigned char SID;
      tN2kMagneticVariation source;
      uint16_t DaysSince1970;
      double variation;
      ParseN2kMagneticVariation(N2kMsg, SID, source, DaysSince1970, variation);
      heading_variation = (float)variation * RAD_TO_DEG;
      send_heading_true = true;
      break;
    case 130845L:
      uint16_t Key, Command, Value;
      if (ParseN2kPGN130845(N2kMsg, Key, Command, Value)) {
        // Serial.printf("Key: %d Command: %d Value: %d\n", Key, Command, Value);
      }
      break;
  }
  num_n2k_messages++;
  // Serial.printf("Message count: %d\n", num_n2k_messages);
  ToggleLed();
}

bool ParseN2kPGN130850(const tN2kMsg &N2kMsg) {
  Serial.println("Entering ParseN2kPGN130850");
  if (N2kMsg.PGN!=130850L) return false;
  int Index=2;
  unsigned char Command1=N2kMsg.GetByte(Index);
  unsigned char Command2=N2kMsg.GetByte(Index);
  unsigned char Command3=N2kMsg.GetByte(Index);
  unsigned char Command4=N2kMsg.GetByte(Index);
  unsigned char CalibrationStopStart=N2kMsg.GetByte(Index);
  //Serial.printf("Command1: %u  Command4: %u  CalibrationStopStart: %u  ", (unsigned int)Command1, (unsigned int)Command4, (unsigned int)CalibrationStopStart);
  if (Command1 == DEVICE_ID && Command4 == 18 && CalibrationStopStart == 0) {
    calibrationStart = true;
    // Send ack
    Send130851Ack(0); 
    return true;
  } else {
    if (Command1 == DEVICE_ID && Command4 == 18 && CalibrationStopStart == 1) {
      calibrationStop = true;
      // Send ack
      Send130851Ack(1);
      return true;
    }
  }
  return false;
}

bool ParseN2kPGN130845(const tN2kMsg &N2kMsg, uint16_t &Key, uint16_t &Command, uint16_t &Value) {
  // Serial.println("Entering ParseN2kPGN130845");
  if (N2kMsg.PGN != 130845L) return false;
  int Index=2;
  unsigned char Target = N2kMsg.GetByte(Index);
  if (Target == 65) {
    N2kMsg.Print(&Serial);
    tN2kMsg N2kMsgReply;
    unsigned char source = N2kMsg.Source;
    Index = 6;
    Key = N2kMsg.Get2ByteUInt(Index);
    Command = N2kMsg.Get2ByteUInt(Index);    
    if (Command == 0x0000) {
      // Get
      if (SetN2kPGN130845(N2kMsgReply, DEVICE_ID, Key, 2))   // 2 = Ack
        nmea2000->SendMsg(N2kMsgReply, DEV_COMPASS);
    }
    if (Command == 0x0100) {
      // Set
      switch (Key) {
        case 0x0000:  // Heading offset
          heading_offset_rad = N2kMsg.Get2ByteDouble(0.0001, Index);
          heading_offset_deg = (int8_t)round(heading_offset_rad * RAD_TO_DEG);
          if (heading_offset_deg > 127) {
             heading_offset_deg = heading_offset_deg - 360;
          }
          Serial.printf("heading_offset_rad: %f  heading_offset_deg: %d, unsigned char: %d\n", heading_offset_rad, heading_offset_deg, (char)heading_offset_deg);
          EEPROM.writeByte(EEP_HEADING_OFFSET, (char)heading_offset_deg);
          EEPROM.commit();
          break;
        case 0x0039:  // Heel offset
          heel_offset_rad = N2kMsg.Get2ByteDouble(0.0001, Index);
          heel_offset_deg = (int8_t)round(heel_offset_rad * RAD_TO_DEG);
          if (heel_offset_deg > 127) {
             heel_offset_deg = heel_offset_deg - 360;
          }
          Serial.printf("heel_offset_rad: %f  heel_offset_deg: %d\n", heel_offset_rad, heel_offset_deg);
          EEPROM.writeByte(EEP_HEEL_OFFSET, (char)heel_offset_deg);
          EEPROM.commit();
          break;
        case 0x0031:  // Trim offset
          trim_offset_rad = N2kMsg.Get2ByteDouble(0.0001, Index);
          trim_offset_deg = (int8_t)round(trim_offset_rad * RAD_TO_DEG);
          if (trim_offset_deg > 127) {
             trim_offset_deg = trim_offset_deg - 360;
          }
          Serial.printf("trim_offset_rad: %f  trim_offset_deg: %d\n", trim_offset_rad, trim_offset_deg);
          EEPROM.writeByte(EEP_TRIM_OFFSET, (char)trim_offset_deg);
          EEPROM.commit();
          break;
        case 0xd200:  // Auto calibration (01=on, 02=auto locked)
          compass_autocalibration = N2kMsg.GetByte(Index);
          if (SetN2kPGN130845(N2kMsgReply, DEVICE_ID, 0xd400, 2))
            nmea2000->SendMsg(N2kMsgReply, DEV_COMPASS);
          EEPROM.writeByte(EEP_AUTOCALIBRATION, (unsigned char)compass_autocalibration);
          EEPROM.commit();
          break;
        case 0xd300:  // Warning
          break;
        case 0xd400:  // Status
          break;      }
        }
    if (SetN2kPGN130845(N2kMsgReply, DEVICE_ID, Key, 2))
      nmea2000->SendMsg(N2kMsgReply, DEV_COMPASS);      
    return true;
  } else {
    // Serial.printf("Skipping this one...\n");
    return false;
  }
  return false;
}

void Send130851Ack(int StopStart) {
  tN2kMsg N2kMsg;
  SetN2k130851Ack(N2kMsg, DEVICE_ID, 18, (unsigned char)StopStart);
  nmea2000->SendMsg(N2kMsg, DEV_COMPASS);
}

bool SetN2kPGN130845(tN2kMsg &N2kMsg, unsigned char DEVICE_ID, uint16_t Key, uint16_t Command) {
  N2kMsg.SetPGN(130845L);
  N2kMsg.Priority=2;
  N2kMsg.AddByte(0x41); // Reserved
  N2kMsg.AddByte(0x9f); // Reserved
  N2kMsg.AddByte((unsigned char)DEVICE_ID);
  N2kMsg.AddByte(0xff); // Reserved
  N2kMsg.AddByte(0xff); // Reserved
  N2kMsg.AddByte(0xff); // Reserved
  N2kMsg.Add2ByteUInt((uint16_t)Key);
  if (Command == 2) {
    N2kMsg.Add2ByteUInt(0x0200);
  }
  switch (Key) {
    case 0x0000:  // Heading offset
      Serial.printf("Adding heading offset: %d", (int8_t)heading_offset_rad);
      N2kMsg.Add2ByteDouble(heading_offset_rad, 0.0001);
      N2kMsg.AddByte(0xff); // Reserved
     break;
    case 0x0039:  // Heel offset
      N2kMsg.Add2ByteDouble(heel_offset_rad, 0.0001);
      break;
    case 0x0031:  // Trim offset
      N2kMsg.Add2ByteDouble(trim_offset_rad, 0.0001);
      break;
    case 0xd200:  // Auto calibration (01=on, 02=auto locked)
      N2kMsg.AddByte(compass_autocalibration); // Reserved
      N2kMsg.AddByte(0xff); // Reserved
      break;
    case 0xd300:  // Warning
      N2kMsg.AddByte(0x00); // Reserved  // 00=No warning, 01=First calibration in progress, 02=Parameters in use are not valid
      N2kMsg.AddByte(0xff); // Reserved
      break;
    case 0xd400:  // Status
      N2kMsg.AddByte(0x00); // Reserved  // 00=Is calibrated, 01=Not calibrated, 02=Is calibrating
      N2kMsg.AddByte(0xff); // Reserved
      break;
    default:
      return false;
  }
  N2kMsg.AddByte(0xff); // Reserved
  N2kMsg.AddByte(0xff); // Reserved
  return true;
}

void SetN2kPGN130851(tN2kMsg &N2kMsg, int DEVICE_ID, unsigned char Command, unsigned char CalibrationStopStart) {
    N2kMsg.SetPGN(130851L);
    N2kMsg.Priority=2;
    N2kMsg.AddByte(0x41); // Reserved
    N2kMsg.AddByte(0x9f); // Reserved
    N2kMsg.AddByte((unsigned char)DEVICE_ID);
    N2kMsg.AddByte(0xff); // Reserved
    N2kMsg.AddByte(0xff); // Reserved
    N2kMsg.AddByte((unsigned char)Command);
    N2kMsg.AddByte((unsigned char)CalibrationStopStart);
    N2kMsg.AddByte(0x00); // Reserved
    N2kMsg.AddByte(0xff); // Reserved
    N2kMsg.AddByte(0xff); // Reserved
    N2kMsg.AddByte(0xff); // Reserved
    N2kMsg.AddByte(0xff); // Reserved
}

void SetN2k130851Ack(tN2kMsg &N2kMsg, int DEVICE_ID, unsigned char Command, unsigned char CalibrationStopStart) {
  SetN2kPGN130851(N2kMsg, DEVICE_ID, Command, CalibrationStopStart);
}

void SetN2k130845(tN2kMsg &N2kMsg, int DEVICE_ID, uint16_t Key, uint16_t Command) {
  SetN2kPGN130845(N2kMsg, DEVICE_ID, Key, Command);
}

bool SetN2kPGN130824(tN2kMsg &N2kMsg) {
  N2kMsg.SetPGN(130824L);
  N2kMsg.Priority=3;
  N2kMsg.AddByte(0x7d); // Reserved
  N2kMsg.AddByte(0x99); // Reserved
  N2kMsg.AddByte(0x9e); // 9e,20 Pitch rate
  N2kMsg.AddByte(0x20); // 9e,20 Pitch rate
  N2kMsg.Add2ByteDouble(pitch * DEG_TO_RAD, 0.0001);    
  N2kMsg.AddByte(0x3c); // 3c,20 Roll rate
  N2kMsg.AddByte(0x20); // 3c,20 Roll rate
  N2kMsg.Add2ByteDouble(roll * DEG_TO_RAD, 0.0001);
  return true;
}

void ToggleLed() {
  static bool led_state = false;
  digitalWrite(LED_BUILTIN, led_state);
  led_state = !led_state;
}


// --------------
// writeByte()
// --------------
void writeByte(byte address, byte subAddress, byte data)
{
  Wire.beginTransmission(address);  // Initialize the Tx buffer
  Wire.write(subAddress);           // Put slave register address in Tx buffer
  Wire.write(data);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
}

byte readByte(byte address, byte subAddress)
{
  byte data; // `data` will store the register data
  Wire.beginTransmission(address);         // Initialize the Tx buffer
  Wire.write(subAddress);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(address, (byte) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return data;                             // Return data read from slave register
}

// --------------
// readBytes()
// --------------
void readBytes(byte address, byte subAddress, byte count, byte * dest)
{
  Wire.beginTransmission(address);   // Initialize the Tx buffer
  Wire.write(subAddress);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  byte i = 0;
  Wire.requestFrom(address, count);  // Read bytes from slave register address
  while (Wire.available()) {
    dest[i++] = Wire.read();
  }         // Put read results in the Rx buffer
}

void applyOffsetPitchRoll()
{
  // Apply offset for pitch and roll
  pitch += trim_offset_deg;
  roll  += heel_offset_deg;
}

void calc_heading () {
  // Apply offset
  heading_mag = heading + heading_offset_deg;
  if (heading_mag < 0) heading_mag += 360.0;
  if (heading_mag >= 360) heading_mag -= 360.0;
  if (send_heading_true) {
    heading_true = heading_mag + heading_variation;
    if (heading_true < 0) heading_true += 360.0;
    if (heading_true >= 360) heading_true -= 360.0;
  }
}

void printValues() {
  // ----- send the results to the Serial Monitor
  if (showOutput) {  
    // Print data to Serial Monitor window  
    if (!compassError) {
      Serial.printf("Heading: % 5.1f", heading);
      Serial.printf(", Pitch: % 5.1f", pitch);
      Serial.printf(", Roll: % 5.1f", roll);
      Serial.print(" deg ");
    } else {
      Serial.print("[COMPASS ERROR] ");
    }
    
    /*
    Serial.print("\t$ACC,");
    Serial.print(accelX,4);
    Serial.print(",");
    Serial.print(accelY,4); 
    Serial.print(",");
    Serial.print(accelZ,4);
    Serial.print(" m/s^2,");
    */
    
    if (!gyroError) {
      Serial.print("  [Gyro]");
      Serial.printf(" %6.1f", gyroZ);
      Serial.print(" deg/s");
    } else {
      Serial.print(" [GYRO ERROR] ");
    }
  
   
    if (!compassError) {
      Serial.printf("  HeadingM % 5.1f", heading_mag);
      if (send_heading_true) {
        Serial.printf(" Variation % 3.1f", heading_variation);
        Serial.printf(" HeadingT % 5.1f", heading_true);      
      }
    }

    Serial.printf("  [Calibration] ");
    for (int z = 0; z <= 7; z++) {
      Serial.printf("%d", calibrationStatus[z]);
    }
    Serial.printf ("n2kConnected: %d\n", n2kConnected);
  }
}


void clearCalibration()
{
    for (size_t i = 0; i < EEPROM_SIZE; ++i) {
      EEPROM.writeByte(i, 0x0);
    }
    EEPROM.commit();
}

void readCalibration() {
    Serial.println("\n[Saved settings]");
    Serial.print("heading_offset_deg : ");
    heading_offset_deg = (int8_t)EEPROM.readByte(EEP_HEADING_OFFSET);
    heading_offset_rad = degToRad(heading_offset_deg);
    Serial.printf("%d %f\n", heading_offset_deg, heading_offset_rad);

    Serial.print("heel_offset_deg    : ");
    heel_offset_deg = (int8_t)(EEPROM.readByte(EEP_HEEL_OFFSET));
    heel_offset_rad = degToRad(heel_offset_deg);
    Serial.printf("%d %f\n", heel_offset_deg, heel_offset_rad);

    Serial.print("trim_offset_deg    : ");
    trim_offset_deg = (int8_t)(EEPROM.readByte(EEP_TRIM_OFFSET));
    trim_offset_rad = degToRad(trim_offset_deg);    
    Serial.printf("%d %f\n", trim_offset_deg, trim_offset_rad);
    Serial.print("autocalibration    : "); 
    Serial.println(EEPROM.readByte(EEP_AUTOCALIBRATION));        
    Serial.println("\n");
}

float degToRad (int deg) {
  float rad = deg * DEG_TO_RAD;
  return rad;
}

void printBytes()
{
    for (size_t i = 0; i < EEPROM_SIZE; ++i)
        Serial.println(EEPROM.readByte(i), HEX);
}


float getPitch16()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(PITCH_Register16);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0) {
    pitchrollError = true;
    return 0;
  } else {
    pitchrollError = false;
  }
  
  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) {
    pitchrollError = true;
    return 0;
  } else {
    pitchrollError = false;
  }
  
  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  float value =  ((_byteHigh<<8) + _byteLow);
  if (value > 32767) { value -= 65535; };
  return (value / 10.0);
}

float getRoll16()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ROLL_Register16);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0) {
    pitchrollError = true;
    return 0;
  } else {
    pitchrollError = false;
  }

  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) {
    pitchrollError = true;
    return 0;
  } else {
    pitchrollError = false;
  }
  
  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  float value =  ((_byteHigh<<8) + _byteLow);
  if (value > 32767) { value -= 65535; };
  return (value / 10.0);
}

int16_t getgyroX()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(GYROX_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;
  
  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate GryoX
  return ((_byteHigh<<8) + _byteLow);
}

int16_t getgyroY()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(GYROY_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;
  
  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate GryoY
  return ((_byteHigh<<8) + _byteLow);
}

int16_t getgyroZ()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(GYROZ_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;
  
  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate GryoZ
  return ((_byteHigh<<8) + _byteLow);
}

int16_t getAcceleroX()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ACCELEROX_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;

  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate Accelerometer
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

int16_t getAcceleroY()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ACCELEROY_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;
  
  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate Accelerometer
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);
}

int16_t getAcceleroZ()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ACCELEROZ_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress , TWO_BYTES);

  // Something has gone wrong
  if (nReceived != TWO_BYTES) return 0;
  
  // Read the values
  _byteHigh = Wire.read(); 
  _byteLow = Wire.read();

  // Calculate Accelerometer
  return (((int16_t)_byteHigh <<8) + (int16_t)_byteLow);

}

int readCalibrationStatus() {
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(CALIBRATION_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return 0;}
  
  // Request 2 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress , ONE_BYTE);

  // Something has gone wrong
  if (nReceived != ONE_BYTE) { return 0; }
  
  // Read the value
  Byte = Wire.read();
  
  for (int i = 0; i < 8; i++)
  {
      bool b = Byte&0x80;
      if (b) {
        calibrationStatus[i] = 1;
      } else {
        calibrationStatus[i] = 0;        
      }
      Byte = Byte << 1;
  }
  return 0;
}

void ReadCompass()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(HEADING_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0) {
    compassError = true;
    // heading = 0; pitch = 0;  roll = 0;
    return;
  } else {
    compassError = false;
  }
  
  // Request 4 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress , FOUR_BYTES);

  // Something has gone wrong
  if (nReceived != FOUR_BYTES) {
    compassError = true;
    // heading = 0; pitch = 0;  roll = 0;
    return;
  } else {
    compassError = false;
  }
  
  // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  heading = ((_byteHigh<<8) + _byteLow) / 10.0;

  // Read the values
  // float value = Wire.read();
  // if (value > 128) { value -= 256; };
  // pitch = value;

  // Read the values
  // value = Wire.read();
  // if (value > 128) { value -= 256; };
  // roll = value;
}


void ReadAccelerator()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(ACCELEROX_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0) {
    accelError = true;
    // accelX = 0; accelY = 0; accelZ = 0;
    return;
  }
  
  // Request 6 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress , SIX_BYTES);

  // Something has gone wrong
  if (nReceived != SIX_BYTES) {
    accelError = true;
    // accelX = 0; accelY = 0; accelZ = 0;
    return;
  }
  
  // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  accelX = (((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * accelScale;

  // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  accelY = (((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * accelScale;

  // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  accelZ = (((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * accelScale;

}

void ReadGyro()
{
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);

  // Tell register you want some data
  Wire.write(GYROX_Register);

  // End the transmission
  int nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0) {
    gyroError = true;
    // gyroX = 0; gyroY = 0; gyroZ = 0
    return;
  }
  
  // Request 6 bytes from CMPS14
  nReceived = Wire.requestFrom(_i2cAddress , SIX_BYTES);

  // Timed out so return
  if (nReceived != SIX_BYTES) {
    gyroError = true;
    //accelX = 0; accelY = 0; accelZ = 0;
    return;
  }
  
  // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  // gyroX = (((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * gyroScale;
  gyroX = (int16_t)(_byteHigh<<8|_byteLow) * gyroScale;

    // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  // gyroY = (((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * gyroScale;
  gyroY = (int16_t)(_byteHigh<<8|_byteLow) * gyroScale;

  // Read the values
  _byteHigh = Wire.read(); _byteLow = Wire.read();
  //gyroZ = (((int16_t)_byteHigh <<8) + (int16_t)_byteLow) * gyroScale;
  gyroZ = (int16_t)(_byteHigh<<8|_byteLow) * gyroScale;
}

void showCMPSVersion () {
  Serial.print("CMPS14 software version ");
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);
  Wire.write(CONTROL_Register);
  byte version = Wire.read();
  Serial.println(version);
}

int turnOffAutoCalibration () {
  Serial.println("Turning off auto calibration");
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);
  Wire.write(CONTROL_Register);
  // Stop calibration
  Wire.write(byte(B10000000));
  int nackCatcher = Wire.endTransmission();
  return nackCatcher;
}

int turnOnAutoCalibration () {
  Serial.println("Turning on auto calibration");
  // Begin communication with CMPS14
  Wire.beginTransmission(_i2cAddress);
  Wire.write(CONTROL_Register);
  // Stop calibration
  Wire.write(byte(B10010001));
  int nackCatcher = Wire.endTransmission();
  return nackCatcher;
}

void changeAddress(byte i2cAddress, byte newi2cAddress)
{
  // Reset the address on the i2c network
  // Ensure that you have only this module connected on the i2c network
  // The 7 bit i2c address must end with a 0. (even numbers please)
  // For example changeAddress(0x60, 0x64)

  // Address 0x60, 1 long flash, 0 short flashes
  // Address 0x62, 1 long flash, 1 short flashes
  // Address 0x64, 1 long flash, 2 short flashes
  // Address 0x66, 1 long flash, 3 short flashes
  // Address 0x68, 1 long flash, 4 short flashes
  // Address 0x6A, 1 long flash, 5 short flashes
  // Address 0x6C, 1 long flash, 6 short flashes
  // Address 0x6E, 1 long flash, 7 short flashes

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROL_Register);
  Wire.write(byte(0xA0));
  
  // End the transmission
  int nackCatcher = Wire.endTransmission();

  //Wait 100ms
  delay(100);

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROL_Register);
  Wire.write(byte(0xAA));

  // End the transmission
  nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return;}

  //Wait 100ms
  delay(100);

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROL_Register);
  Wire.write(byte(0xA5));

  // End the transmission
  nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return;}

  //Wait 100ms
  delay(100);

  // Begin communication
  Wire.beginTransmission(i2cAddress);
  Wire.write(CONTROL_Register);
  Wire.write(newi2cAddress);

  // End the transmission
  nackCatcher = Wire.endTransmission();

  // Return if we have a connection problem 
  if(nackCatcher != 0){return;}

}
