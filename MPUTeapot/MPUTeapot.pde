// I2C device class (I2Cdev) demonstration Processing sketch for MPU6050 DMP output //<>// //<>// //<>// //<>// //<>//
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
PShape skate; //Forme 3d du skate
PShape skateBot;





float[] q = new float[4]; // Storage des valeurs de quaternion
float accReely; //Acceleration du skate en y 
Quaternion quat = new Quaternion(1, 0, 0, 0);
Quaternion quatBot = new Quaternion(1, 0, 0, 0);

//Saut de la planche
float VY_INIT =10; //Definit la valeur de la vélocité en y
float vy; //Vélocité
float gravite = 0.3; //constante de gravité 
float bounce = -1;
float yPos; // Position en y de la planche
boolean isJumping; //Saute oui ou non
boolean isReversing; //Monte ou descend
int accIgnore = 0; // Coopteur pour ignorer les premières valeurs  de l'accéleration à ignorer
int NB_IGNORE = 300; //Nb de valeur à ignorer
int JUMP_SENSIBILITY = -5000 ; //Détermine la sensibility nécessaire pour faire un saut


//Saving
int[] teapotBuffer= new int[9];
String[] quatBuffer= new String[4];
float[] saveBuffer= new float[13];
String[] LoadData;
StringList recordData;
PrintWriter output;
int BufferSize= 300; //Nombre de données enregistrées. (x seconde * 60 frame/seconde)
boolean rIsActive= false; //Si enregistre
boolean pIsActive= false; // Si replay
int timer=0;
int counter=0;
String nomFichier; //Nom du fichier choisit par l'utilisateur


//Replay du pro
int[] teapotBufferBot= new int[9];
float[] saveBufferBot= new float[13];
String[] LoadDataBot;
int timerBot=0;
int counterBot=0;
float yPosBot;
float vyBot;
boolean isJumpingBot;
int accIgnoreBot=0; 

// Menu
boolean game= false;
PImage bg;
PImage menu;

//Couleur
color red = color(255, 0, 43);
color green = color(0, 255, 0);
color blue = color(0, 43, 255);
color white = color(255, 255, 255);

PFont f; //Font 
String statut; //Statut des skates

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
  String portName = "COM3";

  // open the serial port
  port = new Serial(this, portName, 115200);

  // send single character to trigger DMP init/start
  // (expected by MPU6050_DMP6 example Arduino sketch)
  port.write('r');

  skate = loadShape("skate.obj");
  bg = loadImage("Overlay(s).png");
  menu = loadImage("Menu.png");

  recordData = new StringList();
  skate = loadShape("skate.obj");
  skateBot= loadShape("skate.obj");
  f = createFont("Arial", 32, true); // STEP 2 Create Font
  statut= "Libre";



  isJumping=false;
  isReversing= false;
  vy=VY_INIT;
  vyBot=VY_INIT;
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

    textFont(f, 32);                  // Spécifier la font à utiliser.
    fill(0);                         // STEP 4 La couleur
    text("Statut: "+ statut, (width/2)-100, 100); //Le message et la position dans l'écran x,y

    //Fonction sauter
    Jumping();

    //Fonction pour déssiner les capteurs
    DrawCapteur();

    //Fonction pour déssiner les skates
    DrawSkate();

    DrawPro();
  } else
  {
    //Déssine menu
    background(menu);
  }
}


//Fonction principal qui tourne toujours en boucle
//Receuille les données envoyées par l'arduino
void serialEvent(Serial port) {

  interval = millis();
  while (port.available() > 0) {
    int ch = port.read();

    if (synced == 0 && ch != '$') return;   // initial synchronization - also used to resync/realign if needed
    synced = 1;


    //Vérification si les données reçu sont bonnes.
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

        //Convertit les valeurs de 2 bytes en int
        q[0] = ((teapotPacket[2] << 8) | teapotPacket[3]) / 16384.0f;
        q[1] = ((teapotPacket[4] << 8) | teapotPacket[5]) / 16384.0f;
        q[2] = ((teapotPacket[6] << 8) | teapotPacket[7]) / 16384.0f;
        q[3] = ((teapotPacket[8] << 8) | teapotPacket[9]) / 16384.0f;
        accReely= ((teapotPacket[10] << 8) | teapotPacket[11]) /16384.0f;

        //Permet de savoir si la valeur envoyer est negatif ou positif
        //La valeur reçu est de 0 à 65,534
        //Pour le transformer en int16 ,il faut retrouver le negative
        //D'ou pourquoi on divise en 16354
        //Tous les valeurs en haut de deux sont des nombres negatifs
        if (accReely >= 2) accReely= -4 + accReely;
        for (int i = 0; i < 4; i++) if (q[i] >= 2) q[i] = -4 + q[i];
        accReely =accReely * 16384; 

        //Permet de capter les sauts grace à l'accéleration reçu.
        // De plus il ignore les 300 premières lecture qui sont fausse.


        //Si replay joue ne pas faire
        if (!pIsActive) {
          // Set les quaternions à la bonne valeur (update)
          quat.set(q[0], q[1], q[2], q[3]);

          //Regarde si le skate est en train de sauter
          //Ignore les premières valeurs défectueuses
          // Si le teapotBuffer[0] (accélération est plus grand que la sensibilité définie, il fait sauter le skate.
          if ((accIgnore++) >NB_IGNORE  && teapotBuffer[0] <= JUMP_SENSIBILITY  )isJumping = true;

          //Stocke tous les valeurs dans un buffer
          quatBuffer[0]=str(q[0]);     
          quatBuffer[1]=str(q[1]);
          quatBuffer[2]=str(q[2]);
          quatBuffer[3]=str(q[3]);

          teapotBuffer[0]=(int)accReely;
          teapotBuffer[1]=teapotPacket[12];
          teapotBuffer[2]=teapotPacket[13];
          teapotBuffer[3]=teapotPacket[14];
          teapotBuffer[4]=teapotPacket[15];
          teapotBuffer[5]=teapotPacket[16];
          teapotBuffer[6]=teapotPacket[17];
          teapotBuffer[7]=teapotPacket[18];
          teapotBuffer[8]=teapotPacket[19];
        }
        //Appele la fonction pour update le Skate Pro
        SetBot();
        //Appele la fonction qui permet d'enregistrer et de replay
        SavingFile();
      }
    }
  }
}




void keyReleased() {

  //Permet de determiner quelle touche est appuyée.
  //Si la bonne toujours en appuyer fait une action.
  if (game) {
    //Active l'enregistrement
    if (key == 'r' && pIsActive==false ) rIsActive= true;


    //Active le replay
    if (key == 'p' && rIsActive==false)  pIsActive= true;

    //Retourne au menu
    if (key == 'm')  game=false;
  }
  if (!game)
  {
    int k = key-48;

    //Permet de faire le choix de la figure à pratiquer selon la touche appuyée
    switch(k) {
    case 1:
      nomFichier="Figures/Ollie.txt";
      game = true;
      break;

    case 2:
      nomFichier="Figures/FS_Shovit.txt";
      game = true;
      break;
    case 3:
      nomFichier="Figures/BS_Shovit.txt";
      game = true;
      break;
    case 4:
      nomFichier="Figures/Kickflip.txt";
      game = true;
      break;
    case 5:
      nomFichier="Figures/Heelflip.txt";
      game = true;
      break;
    case 6:
      nomFichier="Figures/Varial_kick.txt";
      game = true;
      break;
    case 7:
      nomFichier="Figures/Hard_flip.txt";
      game = true;
      break;
    case 8:
      nomFichier="Figures/360_flip.txt";
      game = true;
      break;
    }
  }
}

void Jumping() {
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
}


void SavingFile() {


  if (rIsActive) {


    if (timer == 0) {

      //Message d'introduction de l'enregistrement
      //Initialise l'output
      output= createWriter("save.txt");
      statut = "Enregistre dans 3 secondes";
      delay(1000);
      statut = ("3 secondes restante");
      delay(1000);
      statut = ("2 secondes restante");
      delay(1000);
      statut = ("1 seconde restante");
      delay(1000);               
      timer=millis();
      statut = ("5 secondes d'enregistrement... ");
    }


    //Fait le if tant qu'il ne dépasse pas le buffer Size
    //Permet de calculer le nombre de saisi
    //Fais une saisi à toute les milliseconde choisit
    //Dans ce que si nous voulons 60 images par seconde
    //Donc à chaque 1000/60 milliseconde il faut saisir une donnée
    if (counter < BufferSize && (millis()-timer >= (1000/60)) ) {
      //Remet le timer au temps réel
      timer = millis();

      //Entrepose les données dans cette variable.
      //Chaque saisi prend une ligne
      //Il y aura BufferSize saisi
      recordData.append(quatBuffer[0]+":"+quatBuffer[1]+":"+quatBuffer[2]+":"+quatBuffer[3]+":"+teapotBuffer[0]+":"+teapotBuffer[1]+":"+teapotBuffer[2]+":"+teapotBuffer[3]+":"+teapotBuffer[4]+":"+teapotBuffer[5]+":"+teapotBuffer[6]+":"+teapotBuffer[7]+":"+teapotBuffer[8]);
      counter++;
    }
    //S'il a fini de prendre les saisis
    if (counter >= (BufferSize -1)  )
    {

      //Ecrit ligne par ligne les données saisis
      //Il y aura BufferSize ligne
      for (int n = 0; n < recordData.size(); n++) {

        output.println(recordData.get(n));
      }

      //Reset tous les valeurs pourn la prochaine utilisation
      output.flush();
      output.close();


      rIsActive= false;
      timer = 0;
      counter=0;
      recordData.clear();
      statut = ("Libre");
    }
  }




  //Replay
  if (pIsActive) {

    float[] saveBuffer= new float[13];

    //Première fois
    if (timer==0)
    {
      //Change le statut
      statut = ("Replay en cours");
      //Charge le fichier texte
      LoadData = loadStrings("save.txt");
      //Part le timer
      timer=millis();
    }

    //Même chose que l'autre en haut dans enregistrement
    if (counter < BufferSize && (millis()-timer >= (1000/60)) ) {
      timer = millis();

      //Prend une ligne et la split à chaque fois qu'elle rencontre un : 
      //Chaque valeur est stockée individuellement saveBuffer[0]= première valeur ,saveBuffer[1]= deuxième valeur ...
      saveBuffer =float(split(LoadData[counter], ':'));
      //Set les quaternions
      quat.set(saveBuffer[0], saveBuffer[1], saveBuffer[2], saveBuffer[3]);
      //Determine si le skate saute
      if ((accIgnore++) >NB_IGNORE  && (int)saveBuffer[4] <= JUMP_SENSIBILITY  )isJumping = true;

      //Stocke le tout dans un buffer
      //Accélération
      teapotBuffer[0]=(int)saveBuffer[4];
      //Grands capteurs
      teapotBuffer[1]=(int)saveBuffer[5];
      teapotBuffer[2]=(int)saveBuffer[6];
      teapotBuffer[3]=(int)saveBuffer[7];
      teapotBuffer[4]=(int)saveBuffer[8];
      teapotBuffer[5]=(int)saveBuffer[9];
      //Moyens capteurs
      teapotBuffer[6]=(int)saveBuffer[10];
      teapotBuffer[7]=(int)saveBuffer[11];
      //Petit capteur
      teapotBuffer[8]=(int)saveBuffer[12];

      counter++;
    }
    //Si dernier
    if (counter >= (BufferSize -1) )
    {
      //Seset les valeurs pour prochaine utilisation
      pIsActive= false;
      statut = ("Libre");
      output.flush();
      output.close();
      counter=0;
      timer = 0;
    }
  }
}












void SetBot()
{


  if (timerBot==0)
  {

    //Charge le fichier texte choisit par l'utilisateur (selon la figure)
    LoadDataBot = loadStrings(nomFichier);
    //Commence le timer
    timerBot=millis();
  }

  //Même timer que les autres
  if (counterBot < BufferSize && (millis()-timerBot >= (1000/60)) ) {
    timerBot = millis();

    //Même que replay
    saveBufferBot =float(split(LoadDataBot[counterBot], ':'));

    quatBot.set(saveBufferBot[0], saveBufferBot[1], saveBufferBot[2], saveBufferBot[3]);

    //Si le personnage saute
    if ((int)saveBufferBot[4] <= JUMP_SENSIBILITY  || isJumpingBot == true )
    {
      isJumpingBot = true;

      //À chaque fois qu'une boucle est fait la vélocité est décrementée par la gravite choisi
      vyBot -= gravite;
      //Change la position du skate selon la vélocité
      yPosBot += vyBot;

      if (yPosBot < 0)
      {
        //Saut terminée
        //Remet la vélocité initial
        vyBot= VY_INIT;
        isJumpingBot = false;
      }
    }

    //Accélération
    teapotBufferBot[0]=(int)saveBufferBot[4];
    //Grands capteurs
    teapotBufferBot[1]=(int)saveBufferBot[5];
    teapotBufferBot[2]=(int)saveBufferBot[6];
    teapotBufferBot[3]=(int)saveBufferBot[7];
    teapotBufferBot[4]=(int)saveBufferBot[8];           
    teapotBufferBot[5]=(int)saveBufferBot[9];
    //Moyens capteurs
    teapotBufferBot[6]=(int)saveBufferBot[10];
    teapotBufferBot[7]=(int)saveBufferBot[11];
    //Petit capteur
    teapotBufferBot[8]=(int)saveBufferBot[12];

    counterBot++;
  }
  //Si terminé
  if (counterBot >= (BufferSize -1) )
  {
    timerBot=0;
    accIgnoreBot=0;
    counterBot=0;
  }
}

void DrawSkate() {

  //Retourne les angles de quaternion selon les valeurs de quaternion envoyées.
  float[] axis = quat.toAxisAngle();  

  //Déssine la matrice du skate en temps réel
  pushMatrix();
  translate(9*width / 32, ((5*height/8))-yPos);
  //Change la rotation selon les angles reçues
  rotate(axis[0], -axis[1], axis[3], axis[2]);

  //La taille
  scale(25);
  //Le skate 3d
  shape(skate);

  popMatrix();

  smooth();
}





void DrawPro() {


  //Même chose que l'autre, sauf quaternion reçu du fichier texte
  float[] axis = quatBot.toAxisAngle();  

  //Déssine la matrice du skate Pro
  pushMatrix();
  translate(6*width / 8, ((55*height/64))-yPosBot);


  rotate(axis[0], -axis[1], axis[3], axis[2]);

  scale(20);
  shape(skateBot);

  popMatrix();

  smooth();
}


void DrawCapteur() {
  //Matrice des capteurs de pressions
  pushMatrix();

  translate(45*width /64, (7*height/32));
  noStroke();


  //Grande bandelette
  for (int i=1; i<51; i++)
  {
    //Dessine les bandelettes à l'aide de rectangle.
    //Les rectangles sont déssinées un à coté de l'autre pour former la bandelette.
    //La couleur de chaque rectangle pour être changée si le if est vrai

    //Il recoit dans le if la valeur du teapot. 
    //S'il est égual à 0 cela veut dire qu'il n'y a pas de pression sur la bandelette.
    //S'il recoit une valeur , cela indique la position que l'utilisateur à appuyé.
    //Donc si l'utilisateur appuie sur la position 151 de 255 , Si nous avions 255 carré le 151 changerait à la couleur désiré.
    //Ici j'ai mis 51 carré ce qui était amplement nécessaire et permettait de voir la zone plus facilement.
    //D'où le divisé par 5 (255/5 = 51)
    //Les autres fonctionnent de la même façon, mais avec moins de carré.

    for (int j=1; j<6; j++)
    {

      if ( teapotBuffer[j]/5 == i || teapotBufferBot[j]/5 == i)
      {
        if ( teapotBuffer[j]/5 == i && teapotBufferBot[j]/5 == i)fill(white);
        else if ( teapotBuffer[j]/5 == i)fill(green);
        else fill(blue);
      } else fill(red); 

      //Déssine chaque rectangle un après l'autre avec la bonne couleur
      if (j==1)rect(i*5, 20, 5, 12);
      if (j==2)rect((i*5)+15, 40, 5, 12);
      if (j==3)rect( (i*5)+20, 60, 5, 12);
      if (j==4)rect((i*5)+15, 80, 5, 12);
      if (j==5)rect(i*5, 100, 5, 12);
    }
  }   

  //Moyennes et petite bandelettes
  //Même chose que les autres bandelettes ,
  //Sauf que l'on vient tournée les carrés pour les mettres à la bonne position et avoir 13 carrés à la place de 51
  for (int i=1; i<13; i++) {

    for (int j=6; j<9; j++)
    {

      if ( teapotBuffer[j]/20 == i || teapotBufferBot[j]/20 == i)
      {
        if ( teapotBuffer[j]/20 == i && teapotBufferBot[j]/20 == i)fill(white);
        else if ( teapotBuffer[j]/20 == i)fill(green);
        else fill(blue);
      } else fill(red); 

      if (j==6) {
        pushMatrix();
        //Permet de faire tourner les rectangles
        rotate(PI/3);
        rect((i*4)-65, 130, 4, 10);
        popMatrix();
      }
      if (j==7) {
        pushMatrix();
        rotate(5*PI/3);
        rect((i*4)-185, -75, 4, 10);
        popMatrix();
      }
      if (j==8) {
        pushMatrix();
        rotate(PI/2);
        rect((i*3)+50, 145, 3, 10);
        popMatrix();
      }
    }
  }
  popMatrix();
}