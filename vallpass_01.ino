//----PED_DOP HEADER--------------
#include <SPI.h>
#include <LoRa.h>
#include <Ticker.h>

#define RFM95_NSS PA4
#define RFM95_RES PA0
#define RFM95_DIO0 PB9
#define BOT PB12
#define botDelay 200            //delay para debounce
#define LED_tx PA2
//----------------------------------------------

//----BME280 HEADER--------
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;
//-------------------------

//----CAR_DOP HEADER----------------------------
HardwareSerial Serial3(USART3); //PB10(TX) PB11(RX)
//---------------------------------

//----MPU6050 HEADER----------------------------
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h" // not necessary if using MotionApps include file
#include "Wire.h"

#define LED_PIN PC13
#define INTERRUPT_PIN PA8

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL
//-------------------------------------------------------------------------

//----LoRaWAN Header-------------------------------------
#include <lorawan.h>

// OTAA Credentials
const char *devEui = "70B3D57ED0053682";
const char *appEui = "0000000000000000";
const char *appKey = "3AA0E85831CD3F4BDFED3F13A1DC1FD7";

const unsigned long interval = 60000;    // 10 s interval to send message
unsigned long previousMillis = 0;  // will store last time message sent
unsigned int counter = 0;     // message counter

char myStr[50];
char outStr[255];
byte recvStatus = 0;

const sRFM_pins RFM_pins = {
  .CS = PA4,
  .RST = PA0,
  .DIO0 = PB0,
  .DIO1 = PB1,
};
//-----------------------------------------------------------------------

//----CayenneLPP Header--------------------------------------------------
#include <CayenneLPP.h>
CayenneLPP lpp(51); //qual o melhor valor?
//-----------------------------------------------------------------------

//----PED_DOP GLOBALS-----------------------------------------------------
volatile bool botaoPress = 0;
long ultTempo = 0;             //tempo da última interrupção

uint8_t ped_count = 0;
uint8_t car_count = 0;

void onTimer_tx();
Ticker DeslLED_tx(onTimer_tx,5000,1);

void iniciLED(){
  pinMode(LED_tx,OUTPUT);
  digitalWrite(LED_tx,LOW);   //HIGH -- current sinking
}

void ISR(){
  if(!digitalRead(BOT)){    //negado por causa do pull-up
    botaoPress = 1;
  }
  else{
    botaoPress = 0;
  }
}

void iniciBot(){
  pinMode(BOT,INPUT);
  attachInterrupt(BOT,ISR,FALLING);
}

void onTimer_tx(){
  digitalWrite(LED_tx,LOW);
}
//-------------------------------------

//-----CAR_DOP GLOBALS---------------------
String Value;
//-------------------------------------

//----BME280 GLOBALS------
unsigned long delayTime;

void printValues() {
    Serial.print("Temperature = ");
    Serial.print(bme.readTemperature());
    Serial.println(" °C");

    Serial.print("Pressure = ");

    Serial.print(bme.readPressure() / 100.0F);
    Serial.println(" hPa");

    Serial.print("Humidity = ");
    Serial.print(bme.readHumidity());
    Serial.println(" %");

    Serial.print("Approx. Altitude = ");
    Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
    Serial.println(" m");

    Serial.println();
}
//-------------------------

//-------MPU6050 GLOBALS-------------- 
MPU6050 mpu;
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
//float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
//---------------------------------

void setup(){
//-----PED_DOP SETUP--------------
  Serial.begin(115200);
  //Serial.println("Conectando via LoRa");
  delay(500);
  iniciBot();
  iniciLED();
  DeslLED_tx.start();
//---------------------------------------

//----CAR_DOP SETUP---------------------
  Serial.println("Setting up connection to sensor at 19200 baud!");
  Serial3.begin(19200);
//----------------------------------

//----BME280 SETUP---------
  unsigned status;
  // default settings
  status = bme.begin(0x77);

  if (bme.begin()){
    Serial.println("BME280 found");
  }
  else{
    Serial.println("BME280 not found");
  }

  if(!status){
    Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
    Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    //while(1);
    delay(10);
  }
  delayTime = 2000;
  Serial.println();
//------------------------

//-------------MPU6050 SETUP-----------------------
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    //Serial.begin(19200);
    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        //mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        //Serial.println(F(")..."));
        //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    pinMode(LED_PIN, OUTPUT);
//-----------------------------------------------------------------------------


//----LoRaWAN SETUP------------------------------------
  if(!lora.init()){
    Serial.println("RFM95 not detected");
    delay(5000);
    return;
  }

  // Set LoRaWAN Class change CLASS_A or CLASS_C
  lora.setDeviceClass(CLASS_A);

  // Set Data Rate
  lora.setDataRate(0x03);

  // set channel to random
  lora.setChannel(MULTI);
  
  // Put OTAA Key and DevAddress here
  lora.setDevEUI(devEui);
  lora.setAppEUI(appEui);
  lora.setAppKey(appKey);

  // Join procedure
  bool isJoined;
  do{
    Serial.println("Joining...");
    isJoined = lora.join();
    
    //wait for 10s to try again
    delay(10000);
  }
  while(!isJoined);
  Serial.println("Joined to network");
//-----------------------------------------------------
}

void loop(){
//----------PED_DOP LOOP-------------------------------
  DeslLED_tx.update();
  
  if (botaoPress){
    if(millis() - ultTempo > botDelay){      //debounce
      String mensagem = "Pedestre detectado!";
      Serial.println(mensagem);
      ultTempo = millis();
    }
    ped_count++;
    botaoPress = 0;
    digitalWrite(LED_tx,HIGH);
    DeslLED_tx.start();
  }

//--------------------------------------------------------

//-----CAR_DOP LOOP-------------------------------------------
  while(Serial3.available() > 0){
    char inChar = Serial3.read();
    if (inChar!='\n')
    {
      /*String*/ Value+=inChar;
    }
    else
    { 
      Value.replace("\r","");
      Serial.println(Value);
      Value = String("");
      car_count++;
    }
  }
//---------------------------------------------------------

//-----------MPU6050 LOOP---------------------------------
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the latest packet

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            /*Serial.print("Yaw: ");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print("Pitch: ");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.print("Roll: ");
            Serial.println(ypr[2] * 180/M_PI);
            delay(50);*/
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            /*Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
            delay(10000);*/
        #endif

        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
//--------------------------------------------------------------------------

//----CayenneLPP LOOP--------------------------------------------------------
lpp.reset();
lpp.addDigitalInput(1, ped_count);
lpp.addDigitalInput(2, car_count);
lpp.addGyrometer(3, ypr[2] * 180/M_PI , ypr[1] * 180/M_PI, ypr[0] * 180/M_PI);
lpp.addAccelerometer(4, aaWorld.x, aaWorld.y, aaWorld.z);
lpp.addTemperature(4, bme.readTemperature());
lpp.addBarometricPressure(4 , bme.readPressure() / 100.0F);
lpp.addRelativeHumidity(4, bme.readHumidity());
//--------------------------------------------------------------------------

//----LoRaWAN LOOP-----------------------------------------------------------
// Check interval overflow
  if(millis() - previousMillis > interval) {
    previousMillis = millis();   
    Serial.print("Sending: ");   
    lora.sendUplink((char*) lpp.getBuffer(), lpp.getSize(), 0, 1);
  }
  
  recvStatus = lora.readData(outStr);
  if(recvStatus){
  Serial.println(outStr);
  }
  
  // Check Lora RX
  lora.update();
//--------------------------------------------------------------------------
}
