// I2C device class (I2Cdev) demonstration Processing sketch for MPU6050 DMP output //<>// //<>// //<>//
// 6/20/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2012-06-20 - initial release

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

import processing.serial.*;
import processing.opengl.*;
import toxi.geom.*;
import toxi.processing.*;

// NOTE: requires ToxicLibs to be installed in order to run properly.
// 1. Download from http://toxiclibs.org/downloads
// 2. Extract into [userdir]/Processing/libraries
//    (location may be different on Mac/Linux)
// 3. Run and bask in awesomeness

ToxiclibsSupport gfx;

Serial port;                         // The serial port
char[] teapotPacket = new char[14];  // InvenSense Teapot packet
int serialCount = 0;                 // current packet byte position
int synced = 0;
int interval = 0;
PShape skate;
PShape skateStable;
float hauteur=0;
float gravite = 9.8;
float vy = 0;
float bounce = -1;



float[] q = new float[4];
Quaternion quat = new Quaternion(1, 0, 0, 0);

float[] gravity = new float[3];
float[] euler = new float[3];
float[] ypr = new float[3];

//Saving
String[] qString= new String[4];
String[] LoadData;
StringList recordData;
boolean save= true;
PrintWriter output;
int BufferSize= 600;
boolean rIsActive= false;
boolean wIsActive= false;
boolean pIsActive= false;
int counter=0;
int timer=0;

// Menu
boolean game= true;
PImage bg;

void setup() {
  // 800px square viewport using OpenGL rendering
  size(1280, 720, OPENGL);
  gfx = new ToxiclibsSupport(this);


  // setup lights and antialiasing
  lights();
  smooth();

  // display serial port list for debugging/clarity
  //println(Serial.list());

  // get the first available port (use EITHER this OR the specific port code below)
  //String portName = Serial.list()[0];

  // get a specific serial port (use EITHER this OR the first-available code above)
  String portName = "COM4";

  // open the serial port
  port = new Serial(this, portName, 115200);

  // send single character to trigger DMP init/start
  // (expected by MPU6050_DMP6 example Arduino sketch)
  port.write('r');
  skate = loadShape("skate.obj");
  bg = loadImage("Overlay(s).png");
  output= createWriter("save.txt");
  recordData = new StringList();
  skate = loadShape("skate.obj");
  skateStable = loadShape("skate.obj");
}

void draw() {

  if (game) {
    if (millis() - interval > 1000) {
      // resend single character to trigger DMP init/start
      // in case the MPU is halted/reset while applet is running
      port.write('r');
      interval = millis();
    }

    //Interface
    background(bg);




    //Creer une variable d'axe qui inclue tous les axes d'un quaternion.
    float[] axis = quat.toAxisAngle();


    //Matrice du skate en temps réel
    pushMatrix();
    translate(9*width / 32, ((5*height/8)));

    rotate(axis[0], -axis[1], axis[3], axis[2]);
    scale(25);
    shape(skate);
    popMatrix();

    smooth();
  } else
  {

    background(bg);
    textFont(createFont("Courier", 40));
    text("Start", (width/2), 300);
    text("Load Trick", (width/2), 400);
    text("Quit", (width/2), 500);
  }
}



void serialEvent(Serial port) {

  interval = millis();
  while (port.available() > 0) {
    int ch = port.read();

    if (synced == 0 && ch != '$') return;   // initial synchronization - also used to resync/realign if needed
    synced = 1;

    println(ch);

    if ((serialCount == 1 && ch != 2)
      || (serialCount == 12 && ch != '\r')
      || (serialCount == 13 && ch != '\n')) {
      serialCount = 0;
      synced = 0;
      return;
    }

    if (serialCount > 0 || ch == '$') {
      teapotPacket[serialCount++] = (char)ch;
      if (serialCount == 14) {
        serialCount = 0; // restart packet byte position

        // get quaternion from data packet
        q[0] = ((teapotPacket[2] << 8) | teapotPacket[3]) / 16384.0f;
        q[1] = ((teapotPacket[4] << 8) | teapotPacket[5]) / 16384.0f;
        q[2] = ((teapotPacket[6] << 8) | teapotPacket[7]) / 16384.0f;
        q[3] = ((teapotPacket[8] << 8) | teapotPacket[9]) / 16384.0f;
        for (int i = 0; i < 4; i++) if (q[i] >= 2) q[i] = -4 + q[i];

        if (!pIsActive) {
          // set our toxilibs quaternion to new data
          quat.set(q[0], q[1], q[2], q[3]);
          hauteur= q[2];

          qString[0]=str(q[0]);     
          qString[1]=str(q[1]);
          qString[2]=str(q[2]);
          qString[3]=str(q[3]);
        }
        SavingFile();
      }
    }
  }
}

void keyReleased() {

  if (key == 'w') {
    wIsActive =true;
  }

  if (key == 'r' ) {


    rIsActive= true;
  }


  if (key == 'p') {
    pIsActive=true;
  }
}

void SavingFile() {
  if (wIsActive) {

    println("Writing to File");
    for (int n = 0; n < recordData.size(); n++) {

      output.println(recordData.get(n));
    }
    wIsActive = false;
    output.flush();
    output.close();
  }

  if (rIsActive) {
    if (timer == 0) {


      println("Recording in 3 sec");
      delay(1000);
      println("3");
      delay(1000);
      println("2");
      delay(1000);
      println("1");
      delay(1000);               
      timer=millis();
      print("Recording for 10 sec ");
    }



    if (counter < BufferSize && (millis()-timer >= (1000/60)) ) {
      timer = millis();
      print("Recording ");
      println(qString[0]+":"+qString[1]+":"+qString[2]+":"+qString[3]);
      recordData.append(qString[0]+":"+qString[1]+":"+qString[2]+":"+qString[3]);
      counter++;
    }
    if (counter == (BufferSize -1) )
    {
      println("End of Recording");
      rIsActive= false;
      timer =0;
      counter=0;
    }
  }

  if (pIsActive) {

    if (timer==0)
    {
      println("Playing Replay");
      //Load the text file
      LoadData = loadStrings("save.txt");
      timer=millis();
    }


    if (counter < BufferSize && (millis()-timer >= (1000/60)) ) {
      timer = millis();
      q =float(split(LoadData[counter], ':'));
      quat.set(q[0], q[1], q[2], q[3]);
      counter++;
    }

    if (counter >= (BufferSize -1) )
    {
      println("End of replay");
      pIsActive= false;
      counter=0;
    }
  }
}