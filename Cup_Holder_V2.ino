TaskHandle_t Task1;

#include "BluetoothSerial.h" //Header File for Serial Bluetooth
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <HardwareSerial.h>
#include <PololuMaestro.h>

HardwareSerial maestroSerial(2);  // Use Serial2 on ESP32

MPU6050 mpu;
MiniMaestro maestro(maestroSerial);

#define OUTPUT_READABLE_YAWPITCHROLL 
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
bool blinkState = false;
 
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
 
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
 
 
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void IRAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

// Check if Bluetooth configs are enabled
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Bluetooth Serial object
BluetoothSerial SerialBT;

// GPIO where LED is connected to
const int hotLed =  25;
const int coldLed =  33;
const int drinkLed =  32;
float temperature = 29.5;

// GPIO where the DS18B20 is connected to
const int oneWireBus = 4;          
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature sensors(&oneWire);

// Handle received and sent messages
String temperatureString = "";

// Timer: auxiliar variables
unsigned long previousMillis = 0;    // Stores last time temperature was published
const long interval = 3000;         // interval at which to publish sensor readings

void setup() {

  maestroSerial.begin(9600, SERIAL_8N1, 3, 1); // Serial2 on pins 3 (RX) and 1 (TX)

  //Serial.begin(115200); 
  Wire.begin(21, 22, 100000); // sda, scl, clock speed
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU−6050)
  Wire.endTransmission(true);
 
  // initialize device
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);
  
  devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-36);
    mpu.setYGyroOffset(-48);
    mpu.setZGyroOffset(18);

    mpu.setXAccelOffset(-731); // 1688 factory default for my test chip
    mpu.setYAccelOffset(1929); 
    mpu.setZAccelOffset(1529); 
 
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
 
        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
 
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;
 
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
  }
 

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Temp_Sense",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */                  

}

//Task1code: Temperature Sensing
void Task1code( void * pvParameters ){
  pinMode(hotLed, OUTPUT);
  pinMode(coldLed, OUTPUT);
  pinMode(drinkLed, OUTPUT);

  // Bluetooth device name
  SerialBT.begin("Smart_Cup_Holder");

  for(;;){
      unsigned long currentMillis = millis();
  // Send temperature readings
  if (currentMillis - previousMillis >= interval){
    previousMillis = currentMillis;
    sensors.requestTemperatures(); 
    temperatureString = String(sensors.getTempCByIndex(0)) + "C  " +  String(sensors.getTempFByIndex(0)) + "F";
    SerialBT.println(temperatureString);
    temperature = float(sensors.getTempCByIndex(0));
  }

    //Too Hot
    if (temperature > 50.0){
      digitalWrite(hotLed, HIGH);
      digitalWrite(coldLed, LOW);
      digitalWrite(drinkLed, LOW);
    }
   //Too Cold
   else if (temperature < 10.0){
     digitalWrite(hotLed, LOW);
     digitalWrite(coldLed, HIGH);
     digitalWrite(drinkLed, LOW);
   }
   else{
     digitalWrite(hotLed, LOW);
     digitalWrite(coldLed, LOW);
     digitalWrite(drinkLed, HIGH);
   }
   delay(20);
   
   vTaskDelay(2); //Very Very Important!
  }
}

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
 
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
 
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
 
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));
 
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
 
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
 
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//            Serial.print("ypr\t");
//            Serial.print(ypr[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(ypr[2] * 180/M_PI);
        #endif
 
        #ifdef  OUTPUT_READABLE_YAWPITCHROLL
 
            // display Euler angles in degrees
 
            mpu.dmpGetQuaternion (& q, fifoBuffer);
            mpu.dmpGetGravity (& gravity, & q);
            mpu.dmpGetYawPitchRoll (ypr, & q, & gravity);
 
//            Serial.print("ypr \t");
//            Serial.print(ypr[0] * 180 / M_PI);  // Print yaw angle in degrees
//            
//            Serial.print("\t");
//            Serial.print(ypr[1] * 180 / M_PI);  // Print pitch angle in degrees
            int servo2Target = map(ypr[1] * 180 / M_PI, -90, 90, 4000, 8000);  // Map pitch angle to target position
            maestro.setTarget(0, servo2Target);  // Set the target for servo2 (channel 0) using Maestro
            
//            Serial.print("\t");
//            Serial.println(ypr[2] * 180 / M_PI);  // Print roll angle in degrees
            int servo3Target = map(ypr[2] * 180 / M_PI,-90, 90, 4000, 8000);  // Map roll angle to target position
            maestro.setTarget(1, servo3Target);  // Set the target for servo3 (channel 1) using Maestro
             
        #endif
    }
}
