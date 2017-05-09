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
char[] teapotPacket = new char[24];  // InvenSense Teapot packet
int serialCount = 0;                 // current packet byte position
int synced = 0;
int interval = 0;
PShape skate;





float[] q = new float[4]; // Storage des valeurs de quaternion
float accReely; //Acceleration du skate en y 
Quaternion quat = new Quaternion(1, 0, 0, 0);


//Saut de la planche
float VY_INIT =5.7; //Definie la valeur de la vélocité en y
float vy;
float gravite = 0.1;
float bounce = -1;
float yPos; // Position en y de la planche
boolean isJumping;
boolean isReversing;
int accIgnore = 0; // Ignore les 500 premieres valeurs de l'accéleration
int JUMP_SENSIBILITY = -5000 ; //Détermine la sensibility nécessaire pour faire un saut


//Saving
String[] qString= new String[4];
String[] LoadData;
StringList recordData;
boolean save= true;
PrintWriter output;
int BufferSize= 300; //Nombre de données enregistrées. (x seconde * 60 frame/seconde)
boolean rIsActive= false;
boolean pIsActive= false;
int counter=0;
int timer=0;

// Menu
boolean game= true;
PImage bg;

//Couleur
color red = color(255, 0, 43);
color green = color(0, 255, 0);

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




  isJumping=false;
  isReversing= false;
  vy=VY_INIT;
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

    if (isJumping)
    {
      //Vy représente la vélocité et l'autre la gravité (constante)
      //Lorsque le saut commence la vélocité initiale est définit par la variable VY_INIT.
      // Ensuite la vy est ajoutée à yPos qui est une variable qui change la position de skate dans le translate.
      // Cette vélocité est diminuée jusqu'à ce que la valeur de yPos revienne à zero.
      //Le saut est terminé et la variable vy est remit à l'initial prêt pour une autre utilisation.

      vy -= gravite;
      yPos += vy;
      if (yPos < 0)
      {
        isJumping =false;
        vy= VY_INIT;
      }
    }
    //Matrice des capteurs de pressions
    pushMatrix();

    translate(45*width /64, (7*height/32));
    noStroke();

    for (int i=1; i<51; i++)
    {

      if ( (teapotPacket[12]/5) == i && teapotPacket[12] != 0 )fill(green); 
      else fill(red); 

      rect(i*5, 20, 5, 12);

      if ( (teapotPacket[13]/5) == i && teapotPacket[13] != 0 )fill(green); 
      else fill(red); 
      rect( (i*5)+15, 40, 5, 12);

      if ( (teapotPacket[14]/5) == i && teapotPacket[14] != 0 )fill(green); 
      else fill(red); 
      rect( (i*5)+20, 60, 5, 12);

      if ( (teapotPacket[15]/5) == i && teapotPacket[15] != 0 )fill(green); 
      else fill(red); 
      rect((i*5)+15, 80, 5, 12);

      if ( (teapotPacket[16]/5) == i && teapotPacket[16] != 0 )fill(green); 
      else fill(red); 
      rect(i*5, 100, 5, 12);
    }

    popMatrix();


    //Matrice du skate en temps réel

    pushMatrix();
    translate(9*width / 32, ((5*height/8))-yPos);


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



    if ((serialCount == 1 && ch != 2)
      || (serialCount == 22 && ch != '\r')
      || (serialCount == 23 && ch != '\n')) {
      serialCount = 0;
      synced = 0;
      return;
    }

    if (serialCount > 0 || ch == '$') {
      teapotPacket[serialCount++] = (char)ch;

      if (serialCount == 24) {
        serialCount = 0; // restart packet byte position

        // get quaternion from data packet
        q[0] = ((teapotPacket[2] << 8) | teapotPacket[3]) / 16384.0f;
        q[1] = ((teapotPacket[4] << 8) | teapotPacket[5]) / 16384.0f;
        q[2] = ((teapotPacket[6] << 8) | teapotPacket[7]) / 16384.0f;
        q[3] = ((teapotPacket[8] << 8) | teapotPacket[9]) / 16384.0f;
        accReely= ((teapotPacket[10] << 8) | teapotPacket[11]) /16384.0f;

        //Permet de savoir si la valeur envoyer est negatif ou positif
        // La valeur reçu est de 0 à 65,534
        //Pour le transformer en int16 ,il faut retrouver le negative
        //D'ou pourquoi on divise en 16354
        //Tous les valeurs en haut de deux sont des nombres negatifs
        if (accReely >= 2) accReely= -4 + accReely;
        for (int i = 0; i < 4; i++) if (q[i] >= 2) q[i] = -4 + q[i];
        accReely =accReely * 16384; 

        //Permet de capter les sauts grace à l'accéleration reçu.
        // De plus il ignore les 300 premières lecture qui sont fausse.
        if ((accIgnore++) >300  && accReely <= JUMP_SENSIBILITY  )isJumping = true;

        //Si replay  joue ne pas faire
        if (!pIsActive) {
          // set our toxilibs quaternion to new data
          quat.set(q[0], q[1], q[2], q[3]);


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



  if (key == 'r' ) {


    rIsActive= true;
  }


  if (key == 'p') {
    pIsActive=true;
  }
}

void SavingFile() {


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
      print("Recording for 5 sec ");
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

      println("Writing to File");
      for (int n = 0; n < recordData.size(); n++) {

        output.println(recordData.get(n));
      }
      
      output.flush();
      output.close();

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