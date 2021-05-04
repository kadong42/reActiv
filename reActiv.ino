// Function prototypes
// - specify default values here
bool takeAnalogReadings(uint16_t* p_numReadings = nullptr, uint16_t** p_analogVals = nullptr, uint16_t** p_IMUVals = nullptr); // not exactly sure of this line
#include <ArduinoBLE.h>
//#include "SparkFunLSM6DS3.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  
Adafruit_BNO055 bno1 = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bno2 = Adafruit_BNO055(55, 0x29);

#include <Adafruit_ADXL345_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
//----------------------------------------------------------------------------------------------------------------------
// BLE UUIDs
//----------------------------------------------------------------------------------------------------------------------

#define BLE_UUID_SENSOR_DATA_SERVICE              "386a83e2-28fa-11eb-adc1-0242ac120002"
#define BLE_UUID_MULTI_SENSOR_DATA                "5212ddd0-29e5-11eb-adc1-0242ac120002"
#define BLE_UUID_IMU                              "b4250402-fb4b-4746-b2b0-93f0e61122c6" //green char


// Establish Sampling with desired 1kHz Sample Rate
#define  SAMPLE_RATE                  1000                            // Hz
#define  UPDATE_INTERVAL              100000                          // time in uS want to send small packets so making this 0.5sec // time in ms so 1 sec before starting new data aquistion HAD TO CONVERT TO US
#define  SAMPLE_INTERVAL              1000000/SAMPLE_RATE             // time in uS between samples //time in ms so 1000 samples per second [time between each sample]
#define  NUMBER_OF_READINGS           UPDATE_INTERVAL/SAMPLE_RATE     // number of samples required for desired sample Rate with given update_interval

union multi_reading_data
{
  struct __attribute__( ( packed ) )
  {
    byte values[NUMBER_OF_READINGS];
  };
  byte bytes[ NUMBER_OF_READINGS * sizeof( byte ) ];
};

union multi_reading_data multiReadingData;
union multi_reading_data IMUData;
//----------------------------------------------------------------------------------------------------------------------
// BLE
//----------------------------------------------------------------------------------------------------------------------

BLEService sensorDataService( BLE_UUID_SENSOR_DATA_SERVICE );
BLECharacteristic multiReadingDataCharacteristic( BLE_UUID_MULTI_SENSOR_DATA, BLERead | BLENotify, sizeof multiReadingData.bytes );
BLECharacteristic multiReadingDataIMUCharacteristic( BLE_UUID_IMU, BLERead | BLENotify, sizeof IMUData.bytes );
//BLEIntCharacteristic angleCharacteristic(angle, BLERead | BLEWrite | BLENotify);

const int BLE_LED_PIN = LED_BUILTIN;


// Make sure these two variables are correct for your setup
int scale = 200; // 3 (±3g) for ADXL337, 200 (±200g) for ADXL377
boolean micro_is_5V = false; // Set to true if using a 5V microcontroller such as the Arduino Uno, false if using a 3.3V microcontroller, this affects the interpretation of the sensor data
//int initialVal = 0, offsetLoop = 1;
//float sumX = 0, sumY = 0, sumZ = 0;
//float offsetX, offsetY, offsetZ;


float ypr[2][3];
int howOftenPrint = 10;
int printCounter = 0;
int kneeAngle;
float netAcceleration;
int kneeAngleInt;


void setup(void)
{
  Serial.begin(115200);
  delay(5000);
  Serial.println("Begin");
  pinMode( BLE_LED_PIN, OUTPUT );
  if ( setupBleMode() )
  {
    digitalWrite( BLE_LED_PIN, HIGH );
  }
  /*
     The original fill arrary non blocking did not have this initialization
     I think that happens with the first line bool takeAnalogReadings
    for ( int i = 0; i < NUMBER_OF_READINGS; i++ )
    {
    multiReadingData.values[i] = 0;
    }
  */
  /* Initialise the sensor */

  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_16_G);

  
  if(!bno1.begin())
  {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }

  if(!bno2.begin())
  {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    //while(1);
  }
  
  //delay(1000);
    
  bno1.setExtCrystalUse(true);
  bno2.setExtCrystalUse(true);
  

  Serial.println("gucci");
  
}

void loop()
{
  //Serial.println("ring around the rossie");
  // listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();
  if ( central )
  {
    while ( central.connected() )
    {
      if ( central.rssi() != 0 )  // there is a connection (I think)
      {
        // This is a way to both take readings *and* read out the values when the buffer is full
        uint16_t numReadings;
        uint16_t* analogVals;
        uint16_t* IMUVals;
        bool readingsDone = takeAnalogReadings(&numReadings, &analogVals, &IMUVals);
        if (readingsDone)
        {
          // Let's convert to bytes
          for (uint16_t i = 0; i < numReadings; i++)
          {
            byte analogValueByte = (byte) map(analogVals[i], 0, 277, 0, 255);    // want to see what it looks like on the app first
            multiReadingData.values[i] = analogValueByte;

            byte IMUValueByte = (byte) IMUVals[i];
            IMUData.values[i] = IMUValueByte;
          }
          // Let's send over bluetooth
          multiReadingDataCharacteristic.writeValue( multiReadingData.bytes, sizeof multiReadingData.bytes );
          multiReadingDataIMUCharacteristic.writeValue( IMUData.values, sizeof IMUData.bytes );
        }
        
      }
    }
  }

}

// Function definitions:

//---------------------------------------------------------------------------------------------------------------------
// Take analog readings to fill up a buffer.
// Once the buffer is full, return true so that the caller can read out the data.
// Optionally pass in a pointer to get access to the internal buffer in order to read out the data from outside
// this function.
//---------------------------------------------------------------------------------------------------------------------
bool takeAnalogReadings(uint16_t* p_numReadings, uint16_t** p_analogVals, uint16_t** p_IMUVals)
{
  static const uint16_t NUM_READINGS = NUMBER_OF_READINGS;
  static uint16_t i = 0; // index
  static uint16_t analogVals[NUM_READINGS];
  static uint16_t IMUVals[NUM_READINGS];

  const uint32_t SAMPLE_PD = SAMPLE_INTERVAL; // us; sample period (how often to take a new sample)
  static uint32_t tStart = micros(); // ms; start time
  bool bufferIsFull = false; // set to true each time NUM_READINGS have been taken

  // Only take a reading once per SAMPLE_PD
  uint32_t tNow = micros(); // ms; time now
  if (tNow - tStart >= SAMPLE_PD)
  {
    //    Serial.print("taking sample num "); Serial.println(i + 1);
    tStart += SAMPLE_PD; // reset start time to take next sample at exactly the correct pd
    
    sensors_event_t event1; 
    bno1.getEvent(&event1);
    sensors_event_t event2; 
    bno2.getEvent(&event2);
    
    int rawX = analogRead(A1);
    int rawY = analogRead(A2);
    int rawZ = analogRead(A3);
    
    //raw IMU event values, IMU 1
    int imuX_1 = event1.orientation.x;
    int imuY_1 = event1.orientation.y;
    int imuZ_1 = event1.orientation.z;

    //raw IMU event values, IMU 2
    int imuX_2 = event2.orientation.x;
    int imuY_2 = event2.orientation.y;
    int imuZ_2 = event2.orientation.z;
    
    // Scale accelerometer ADC readings into common units
    // Scale map depends on if using a 5V or 3.3V microcontroller
    float scaledX, scaledY, scaledZ; // Scaled values for each axis

    if (micro_is_5V) // Microcontroller runs off 5V
    {
      scaledX = mapf(rawX, 0, 675, -scale, scale); // 3.3/5 * 1023 =~ 675
      scaledY = mapf(rawY, 0, 675, -scale, scale);
      scaledZ = mapf(rawZ, 0, 675, -scale, scale);
    }
    else // Microcontroller runs off 3.3V
    {
      scaledX = mapf(rawX, 0, 1023, -scale, scale);
      scaledY = mapf(rawY, 0, 1023, -scale, scale);
      scaledZ = mapf(rawZ, 0, 1023, -scale, scale);
    }

    netAcceleration = sqrt(scaledX * scaledX + scaledY * scaledY + scaledZ * scaledZ);
    int netAccelerationInt = (int) netAcceleration;
    

    //analogVals[i] = analogRead(A1);
    sensors_event_t event; 
    accel.getEvent(&event);
    int netAccel = (int) sqrt(pow(event.acceleration.x,2) + pow(event.acceleration.y,2) + pow(event.acceleration.z,2));
    analogVals[i] = netAccel;

    IMUVals[i] = imuX_1;
    //IMUVals[i] = 2;
    
    i++;
    if (i >= NUM_READINGS)
    {
      bufferIsFull = true;
      i = 0; // reset to beginning of array, so you don't try to save readings outside of the bounds of the array
    }
  }

  // Assign the user-passed-in pointers so that the user can retrieve the data if they so desire to do it this way
  if (p_numReadings != nullptr)
  {
    *p_numReadings = NUM_READINGS;
  }
  if (p_analogVals != nullptr)
  {
    *p_analogVals = analogVals;
  }
  if (p_IMUVals != nullptr)
  {
    *p_IMUVals = IMUVals;
  }

  return bufferIsFull;
}

//----------------------------------------------------------------------------------------------------------------------
// setupBleMode function
//----------------------------------------------------------------------------------------------------------------------

bool setupBleMode()
{
  if ( !BLE.begin() )
  {
    Serial.println("false");
    return false;
  }

  // set advertised local name and service UUID:
  BLE.setDeviceName("Arduino MKR1010");
  BLE.setLocalName("ECG_Device");
  BLE.setAdvertisedService( sensorDataService );

  // BLE add characteristics
  sensorDataService.addCharacteristic( multiReadingDataCharacteristic );
  sensorDataService.addCharacteristic( multiReadingDataIMUCharacteristic );

  // add service
  BLE.addService( sensorDataService );

  // set the initial value for the characeristic:
  multiReadingDataCharacteristic.writeValue( multiReadingData.bytes, sizeof multiReadingData.bytes );
  multiReadingDataIMUCharacteristic.writeValue( IMUData.bytes, sizeof IMUData.bytes );
  //angleCharacteristic.setValue(0);

  // start advertising
  BLE.advertise();

  Serial.println("true");
  return true;
}


// Same functionality as Arduino's standard map function, except using floats
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
