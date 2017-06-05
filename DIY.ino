#define TRIGGER_NOMAL 1      // Trigger state normal
#define TRIGGER_PULLED 2     // Trigger state pulled
#define TRIGGER_PUSHED 3     // Trigger state pushed
#define INTERRUPT_PIN 2      // use pin 2 on Arduino Uno & most boards
#define LED_PIN 9

typedef union {
    float number;
    uint8_t bytes[4];
} FLOATUNION_t;

typedef union {
    double number;
    uint8_t bytes[4];
} DOUBLEUNION_t;

typedef union {
    uint16_t number;
    uint8_t bytes[2];
} INT16UNION_t;

/* ============================================== *
 | Packet Size 34Byte                             |
 |                                                |
 |       0 : [ START ]                            |
 |  1 ~  4 : [ QT_W1 ][ QT_W2 ][ QT_W3 ][ QT_W4 ] |
 |  5 ~  8 : [ QT_X1 ][ QT_X2 ][ QT_X3 ][ QT_X4 ] |
 |  9 ~ 12 : [ QT_Y1 ][ QT_Y2 ][ QT_Y3 ][ QT_Y4 ] |
 | 13 ~ 16 : [ QT_Z1 ][ QT_Z2 ][ QT_Z3 ][ QT_Z4 ] |
 | 17 ~ 20 : [ PS_X1 ][ PS_X2 ][ PS_X3 ][ PS_X4 ] |
 | 21 ~ 24 : [ PS_Y1 ][ PS_Y2 ][ PS_Y3 ][ PS_Y4 ] |
 | 25 ~ 28 : [ PS_Z1 ][ PS_Z2 ][ PS_Z3 ][ PS_Z4 ] |
 | 29 ~ 32 : [ JOY_X ][ JOY_Y ][ JOY_S ][ TRI_S ] |
 |      33 : [  END  ]                            |
 * ============================================== */
 
#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU9250_9Axis_MotionApps41.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

SoftwareSerial BTSerial(10, 11);  // Setting Bluetooth Serial Port 
MPU9250 mpu;

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;                // set true if DMP init was successful
uint8_t mpuIntStatus;                 // holds actual interrupt status byte from MPU
uint8_t devStatus;                    // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;                  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;                   // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;                         // [w, x, y, z]   quaternion container
VectorInt16 aa;                       // [x, y, z]      accel sensor measurements
VectorInt16 aaReal;                   // [x, y, z]      gravity-free accel sensor measurements
VectorInt16 aaWorld;                  // [x, y, z]      world-frame accel sensor measurements
VectorFloat gravity;                  // [x, y, z]      gravity vector

FLOATUNION_t quaternion[4];           // Quaternion axis data (float) to (byte array)
FLOATUNION_t space[3];                // Position data (float) to (byte array)
FLOATUNION_t velocity[3];             // Velocity data (float) to (byte array)
unsigned long timer;                  // Last frame time
float timeTemp;                       // Delta time

uint8_t dataPacket[34] = {'$', 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, '#'};

int state = TRIGGER_NOMAL;
int i = 0;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    BTSerial.begin(38400);  // set the data rate for the BT port
    while (!BTSerial);

    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    
    devStatus = mpu.dmpInitialize();

    velocity[0].number = 0.0f;
    velocity[1].number = 0.0f;
    velocity[2].number = 0.0f;

    space[0].number = 0.0f;
    space[1].number = 0.0f;
    space[2].number = 0.0f;

    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
    pinMode(LED_PIN, OUTPUT);
    timer = micros();
}

void loop() {
    if(!dmpReady) {
        return;
    }
    
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
    }
    
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check overflow and reset FIFO
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();

    // DMP data ready
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) {
            fifoCount = mpu.getFIFOCount();
        }

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        timeTemp = (float)(micros()-timer) / 1000000.0f;
        timer = micros();
        
        fifoCount -= packetSize;
        
        // Get DMP Quaternion and Real accel
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

        velocity[0].number += aaReal.x / 8192.0f * 9.8 * timeTemp;
        velocity[1].number += aaReal.y / 8192.0f * 9.8 * timeTemp;
        velocity[2].number += aaReal.z / 8192.0f * 9.8 * timeTemp;
        
        space[0].number += velocity[0].number * timeTemp;
        space[1].number += velocity[1].number * timeTemp;
        space[2].number += velocity[2].number * timeTemp;
        
        quaternion[0].number = q.w;
        quaternion[1].number = q.x;
        quaternion[2].number = q.y;
        quaternion[3].number = q.z;

        // index 0 is start marker '$'
        dataPacket[ 1] = quaternion[0].bytes[0]; 
        dataPacket[ 2] = quaternion[0].bytes[1]; 
        dataPacket[ 3] = quaternion[0].bytes[2]; 
        dataPacket[ 4] = quaternion[0].bytes[3]; 
        dataPacket[ 5] = quaternion[1].bytes[0]; 
        dataPacket[ 6] = quaternion[1].bytes[1]; 
        dataPacket[ 7] = quaternion[1].bytes[2]; 
        dataPacket[ 8] = quaternion[1].bytes[3];
        dataPacket[ 9] = quaternion[2].bytes[0]; 
        dataPacket[10] = quaternion[2].bytes[1]; 
        dataPacket[11] = quaternion[2].bytes[2]; 
        dataPacket[12] = quaternion[2].bytes[3];
        dataPacket[13] = quaternion[3].bytes[0]; 
        dataPacket[14] = quaternion[3].bytes[1]; 
        dataPacket[15] = quaternion[3].bytes[2]; 
        dataPacket[16] = quaternion[3].bytes[3]; 

        dataPacket[17] = space[0].bytes[0];
        dataPacket[18] = space[0].bytes[1];
        dataPacket[19] = space[0].bytes[2];
        dataPacket[20] = space[0].bytes[3];
        dataPacket[21] = space[1].bytes[0];
        dataPacket[22] = space[1].bytes[1];
        dataPacket[23] = space[1].bytes[2];
        dataPacket[24] = space[1].bytes[3];
        dataPacket[25] = space[2].bytes[0];
        dataPacket[26] = space[2].bytes[1];
        dataPacket[27] = space[2].bytes[2];
        dataPacket[28] = space[2].bytes[3];

        dataPacket[29] = (analogRead(2) >> 2);
        dataPacket[30] = (analogRead(3) >> 2);
        dataPacket[31] = digitalRead(7);
        dataPacket[32] = state;
        // index 33 is end marker '#'
        
        BTSerial.write(dataPacket, 34);
    }
}
