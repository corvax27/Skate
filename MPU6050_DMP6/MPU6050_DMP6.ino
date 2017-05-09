// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif



///Crée un objet de type MPU6050 de la librairy MPU6050_6Axis_MotionApps20.h
MPU6050 mpu;


/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */





// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
#define OUTPUT_TEAPOT



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false; 

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // Conmpteur de la grosseur de fifobuffer
uint8_t fifoBuffer[64]; // Stockage temporaire

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         Contient les quaternions
VectorInt16 aa;         // [x, y, z]            Mesurer l'accélération
VectorInt16 aaReal;     // [x, y, z]            Accéleration réel sans la gravité
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            Vecteur de gravité

//Structure du stockage des valeurs envoyées à processing
uint8_t teapotPacket[24] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0,0,0,0,0,0,0,0,0,0,0, 0x00, 0x00, '\r', '\n'};





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
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    //Initialise les grands capteurs
    digitalWrite(A0, HIGH); //enable pullup resistor
    digitalWrite(A1, HIGH); //enable pullup resistor
    digitalWrite(A2, HIGH); //enable pullup resistor
    digitalWrite(A3, HIGH); //enable pullup resistor
    digitalWrite(A4, HIGH); //enable pullup resistor

    //Moyen et petit capteurs
    //Moyen
    digitalWrite(A5, HIGH); //enable pullup resistor
    digitalWrite(A6, HIGH); //enable pullup resistor
    //Petit
    digitalWrite(A7, HIGH); //enable pullup resistor
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // Initialise la connection au mpu
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // Permet de vérifier la connection au mpu
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    //Attend que l'utilisateur envoi une touche pour commencer la connection
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // Charge et configure la connection au mpu
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    //Permet de calibrer le gyroscope (le zero)
    mpu.setXAccelOffset(-98); // 1688 factory default for my test chip
    mpu.setYAccelOffset(-4936); // 1688 factory default for my test chip
    mpu.setZAccelOffset(1242); // 1688 factory default for my test chip
    mpu.setXGyroOffset(34);
    mpu.setYGyroOffset(14);
    mpu.setZGyroOffset(-51);
   
    // Si toute la connection a fonctionné
    if (devStatus == 0) {
        // Met la connection à true, maintenant que la connection est prête
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // Met le dmp à ready, permet de démarrer la loop
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // S'il a une erreur de connection
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // Si la connection n'a pas fonctionné ne fait pas la loop
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
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
      






        //Tout le code qui est utilisé pour envoyer les données au teapot sont içi.
        #ifdef OUTPUT_TEAPOT

         
            // Permet d'envoyer l'accélération linéaire en z et les quaternions
            // display real acceleration, adjusted to remove gravity
            
            //Utilise la fonction permettant de reçevoir les quaternions du gyroscope
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

           //Convetie aaReal en int.
            int intTemp =(int)aaReal.z;
         
          //Stockage des données de chacun des ports analogues
            int softpotReading[8] = {0,0,0,0,0,0,0,0};
            
          for(int i=0;i<8;i++)
          {
            //Lit les données des softpots (capteur)
            //Si la valeur est plus grande que 1020 cela veut dire qu'il n'a aucune pression sur le capteur.
            //Il ignore c'est données et laisse la variable égual à zero
            if(analogRead(i) < 1020){softpotReading[i]=analogRead(i);} 
            Serial.println(softpotReading[i]); 
          }
                   
          
            //Stocke les quaternions ( 2 bytes chaque) dans le teapotPacket
            //Le teapotPacket est tous les données envoyées à processing.
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];

             //Permet de separer l'acceleration (int) en 2 bytes (low,high)
            teapotPacket[10] = highByte(intTemp);
            teapotPacket[11] = lowByte(intTemp);

             //Envoie les valeurs des softpots
             //Transforme les valeurs en byte 1 to 255
             
             teapotPacket[12]= (byte)(softpotReading[0]/4);
             teapotPacket[13]= (byte)(softpotReading[1]/4);
             teapotPacket[14]= (byte)(softpotReading[2]/4);
             teapotPacket[15]= (byte)(softpotReading[3]/4);
             teapotPacket[16]= (byte)(softpotReading[4]/4);
             teapotPacket[17]= (byte)(softpotReading[5]/4);
             teapotPacket[18]= (byte)(softpotReading[6]/4);
             teapotPacket[19]= (byte)(softpotReading[7]/4);
          
             Serial.write(teapotPacket, 24);
           
           
            teapotPacket[21]++; // packetCount, loops at 0xFF on purpose
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
