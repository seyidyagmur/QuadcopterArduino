#include <SoftwareSerial.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

#define GUC_BASLANGIC 1200
#define ROLL_CARPANI 3
#define PITCH_CARPANI 3
#define YPR_LIMIT 300
#define DELTA_SPEED 50

#define RxD 8
#define TxD 7

bool blinkState = false;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

float roll;
float pitch;
float startPitch;
float startRoll;
int guc = 0;
int gear = 0;

Servo esc9;
Servo esc10;
Servo esc11;
Servo esc12;

int x=50;
int a=1300;

MPU6050 mpu;

SoftwareSerial BTSerial(RxD, TxD);

void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
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

    
  esc9.attach(9);
  esc10.attach(10);
  esc11.attach(11);
  esc12.attach(12);
  esc9.writeMicroseconds(1000);
  esc10.writeMicroseconds(1000);
  esc11.writeMicroseconds(1000);
  esc12.writeMicroseconds(1000);

  
  // Configuracion del puerto serie por software
  // para comunicar con el modulo HC-05
  BTSerial.begin(9600);
  BTSerial.flush();
  delay(500);
 
  // Configuramos el puerto serie de Arduino para Debug
  Serial.println("Ready");


}

void loop()
{

  // Esperamos ha recibir datos.
    Serial.println("BT available");
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    //while (!mpuInterrupt && fifoCount < packetSize) {
     // Serial.println("wait for mpu interrupt");
    //}

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

             // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        pitch = ypr[1] * 180/M_PI;
        roll = ypr[2] * 180/M_PI;

        /*
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2] * 180/M_PI);
        Serial.print("\t");
        Serial.println(ypr[2]);
        Serial.print("\t");
*/
    }
    
  if (BTSerial.available()){
    
    // La funcion read() devuelve un caracter 
    char command = BTSerial.read();
    BTSerial.flush();
    Serial.println(command);
    
    unsigned long value;
    
    if(command == '6'){
      guc = GUC_BASLANGIC;
      startRoll = roll;
      startPitch = pitch;
    } else if(command == '5'){
      guc = 1000;
      gear = 0;
    } else if(command == '7'){
      gear = gear < 10 ? (gear + 1) : gear;
      guc = GUC_BASLANGIC + (DELTA_SPEED*gear);
    } else if(command == '8'){
      gear = gear > 0 ? (gear - 1) : 0;
      guc = GUC_BASLANGIC + (DELTA_SPEED*gear);
    }
      
  }

    float rollSpeed = (roll - startRoll) * ROLL_CARPANI;
    float pitchSpeed = (pitch - startPitch) * PITCH_CARPANI;
    if (guc == 1000){
      rollSpeed = 0;
      pitchSpeed = 0;
    } else {
      rollSpeed = rollSpeed < -YPR_LIMIT ? -YPR_LIMIT : rollSpeed;
      rollSpeed = rollSpeed > YPR_LIMIT ? YPR_LIMIT : rollSpeed;
  
      pitchSpeed = pitchSpeed > YPR_LIMIT ? YPR_LIMIT : pitchSpeed;
      pitchSpeed = pitchSpeed < -YPR_LIMIT ? -YPR_LIMIT : pitchSpeed;
  
    } 
  
    // SOL ON//on
/*    esc9.writeMicroseconds(guc -pitchSpeed);
    // SOL ARKA//sol
    esc10.writeMicroseconds(guc -rollSpeed);
    */
    // SAG ON//SAG
   esc12.writeMicroseconds(guc  + rollSpeed);
    // SAG
   esc11.writeMicroseconds((guc + pitchSpeed)); 

    Serial.print("guc: ");
    Serial.print(guc);
    Serial.print("   pitch: ");
    Serial.print(pitchSpeed);
    Serial.print("   roll: ");
    Serial.print(rollSpeed);

     /*
    //İleri
    if (command == '1'){
      
      value=millis();
      while(millis()<(value+500))
      { 
        esc9.writeMicroseconds(a-50);
        esc10.writeMicroseconds(a+50);
        esc11.writeMicroseconds(a+50); 
        esc12.writeMicroseconds(a-50);        
      }
      value=millis();
      while(millis()<(value+500))
      {
        esc9.writeMicroseconds(a+50);
        esc10.writeMicroseconds(a-50);
        esc11.writeMicroseconds(a-50); 
        esc12.writeMicroseconds(a+50);        
      }
      
      esc9.writeMicroseconds(a);
      esc10.writeMicroseconds(a);
      esc11.writeMicroseconds(a);
      esc12.writeMicroseconds(a);
    }
    //Geri
     if (command == '2'){
      
      value=millis();
      while(millis()<(value+500))
      { 
        esc9.writeMicroseconds(a+50);
        esc10.writeMicroseconds(a-50);
        esc11.writeMicroseconds(a-50); 
        esc12.writeMicroseconds(a+50);        
      }
      value=millis();
      while(millis()<(value+500))
      {
        esc9.writeMicroseconds(a-50);
        esc10.writeMicroseconds(a+50);
        esc11.writeMicroseconds(a+50); 
        esc12.writeMicroseconds(a-50);        
      }
      
      esc9.writeMicroseconds(a);
      esc10.writeMicroseconds(a);
      esc11.writeMicroseconds(a); 
      esc12.writeMicroseconds(a);
                   
    }
    //Sağ
     if (command == '3'){
      value=millis();
      while(millis()<(value+500))
      {
        esc9.writeMicroseconds(a+50);
        esc10.writeMicroseconds(a+50);
        esc11.writeMicroseconds(a-50); 
        esc12.writeMicroseconds(a-50);        
      }
      value=millis();
      while(millis()<(value+500))
      {
        esc9.writeMicroseconds(a-50);
        esc10.writeMicroseconds(a-50);
        esc11.writeMicroseconds(a+50); 
        esc12.writeMicroseconds(a+50);        
      }
      
      esc9.writeMicroseconds(a);
      esc10.writeMicroseconds(a);
      esc11.writeMicroseconds(a);
      esc12.writeMicroseconds(a);
 
    }
    //Sol
     if (command == '4'){
      
      value=millis();
      while(millis()<(value+500))
      {
        esc9.writeMicroseconds(a-50);
        esc10.writeMicroseconds(a-50);
        esc11.writeMicroseconds(a+50); 
        esc12.writeMicroseconds(a+50);        
      }
      value=millis();
      while(millis()<(value+500))
      {
        esc9.writeMicroseconds(a+50);
        esc10.writeMicroseconds(a+50);
        esc11.writeMicroseconds(a-50); 
        esc12.writeMicroseconds(a-50);        
      }
      
      esc9.writeMicroseconds(a);
      esc10.writeMicroseconds(a);
      esc11.writeMicroseconds(a);
      esc12.writeMicroseconds(a);
  
    }
      //Dur
     if (command == '5'){
      a=1000;
      esc9.writeMicroseconds(a);
      esc10.writeMicroseconds(a);
      esc11.writeMicroseconds(a);
      esc12.writeMicroseconds(a);
              
 
    }
    //Başla
     if (command == '6'){
      a=1300;
      esc9.writeMicroseconds(a);
      esc10.writeMicroseconds(a);
      esc11.writeMicroseconds(a);
      esc12.writeMicroseconds(a);
              
 
    }
    //yüksel
     if (command == '7'){
 
      a+=x;
      esc9.writeMicroseconds(a);
      esc10.writeMicroseconds(a);
      esc11.writeMicroseconds(a); 
      esc12.writeMicroseconds(a);
              
 
    }
    //alçal
     if (command == '8'){
          
      a-=x;
      esc9.writeMicroseconds(a);
      esc10.writeMicroseconds(a);
      esc11.writeMicroseconds(a);
      esc12.writeMicroseconds(a);
              
 
    }
    */
}
