#include <SPI.h>
#include <SD.h>

#include <Adafruit_GFX.h>
#include <Wire.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_STMPE610.h>  

// calibration data for raw touch data to screen coordinates
#define TS_MINX 150
#define TS_MINY 130
#define TS_MAXX 3800
#define TS_MAXY 4000

#define STMPE_CS 8
Adafruit_STMPE610 ts = Adafruit_STMPE610(STMPE_CS);
#define TFT_CS 10
#define TFT_DC 9
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

#include <String.h>
#include <math.h>
#include <ArduinoMatrixMathDouble.h>

//-------------CHANGE PARMS: FROM HERE------------------------
int analogChans [] = {5,7,9,11};
int rSampling = 240; // in Hz
double filterCoeffs [] = {0.003916126660550, 0, -0.007832253321099, 0, 0.003916126660550,
                1.000000000000000, -3.679974110957249, 5.207535578685201, -3.353977015222062, 0.831005589346756,
                
                0.037340318340050, 0, -0.074680636680099, 0, 0.037340318340050,
                1.000000000000000, -2.934200819228941, 3.577223872555188, -2.124757696983410, 0.533251648515028}; //8-13 and 13-30 for 240 Hz
double VoltScale = 1 / 1240.9; // set for Arduino Due
int FES_PIN = 24; // feedback output pin

//TIME PARAMETERS
int nTrainingObs = 50; // total number of observations in training
int tPP = 250; //duration of epoch in MILLIS for posterior probability calculation
int CueDur = 6000; // duration of offline AND online cues in MILLIS
int AnaDur = 750; //total duration of sliding analysis window in MILLIS;
int PosDur = 1500; //total duration of smoothing window for posterior probabilities in MILLIS;
int tBSM = 120;// total time for calculating the binary state machine thresholds IN SECONDS
int online_run_dur = 120;// duration of each online run IN SECONDS

//----------------------CHANGE PARMS: TO HERE--------------------------------

//--ALL THE FOLLOWING ARE AUTOMATICALLY GENERATED FROM THE ABOVE PARAMETERS--
int nC = sizeof(analogChans)/sizeof(int); // number of channels
int nF = sizeof(filterCoeffs)/(10*sizeof(double)); // number of frequency bands
int nD = nC*nF; //number of dimensions = nchan * nfreqs
unsigned long tSampling = (unsigned long) round(1000000.0 / (double)rSampling); //sampling period IN MICROS, 240 Hz: 4167, 256 Hz: 3906
int nTr = (int) round(((double)(CueDur-1000) * (double)rSampling) / 1000.0); //number of sample points per training cue
int nBSM = (int) round(1000 * (double)tBSM / (double)tPP); //number of points used when finding the binary state machine thresholds
int nits_online = (int) round((double)tPP * (double)rSampling / 1000.0); //number of new points added each iteration to the sliding analysis window
int nAnaDur = (int) round((double)AnaDur / (double)tPP);// total points in the sliding analysis window divided by nits_online
int nPosDur = (int) round((double)PosDur / (double)tPP);//total number of  in the sliding posterior probability window divided by nits_online
int nCueDur = (int) round((double)CueDur / (double)tPP);// number of points per cue, cue_dur / tPP
int online_run_its = rSampling * online_run_dur;// IN ITERATIONS Fs * 120 s - @240 Hz = 28800
int online_pp_its = (int) round(1000.0 * (double)online_run_dur / (double)tPP);

byte Toggle_State = 0;
int nSJ = 0, nSS = 0;
String sSJ = "000", sSS = "000";

TS_Point p;
char command;

// variables for manual selection of BSM thresholds
int lowbsm_tenths = 0;
int lowbsm_hundredths = 0;
int highbsm_tenths = 0;
int highbsm_hundredths = 0;
int lowlast_bsm_tenths = 0;
int lowlast_bsm_hundredths = 0;
int highlast_bsm_tenths = 0;
int highlast_bsm_hundredths = 0;
float lowthres = 0;
float highthres = 0;
int bsm_option = 0; //whether we are viewing HIGH or LOW histogram
boolean bsm_select = false; //whether we want to select bsm thresholds
boolean bsm_updated = false;


void setup() {
  Serial.begin(115200);
  Serial.print("Initializing hardware...");
  
  //pinMode(10, OUTPUT);//10 for DUE, 53 for MEGA
  tft.begin();
  if (!ts.begin()) {
    Serial.println("touchscreen:failed...ABORTING");
    return;
  } else { 
    Serial.print("touchscreen:passed..."); 
  }
  if (!SD.begin(4)) {
    Serial.println("SDreader:failed...ABORTING");
    return;
  } else {
    Serial.println("SDreader:passed...DONE");
  }
  
  //obtain most recent subject name and session number
  File root = SD.open("/DATA/");
  nSJ = getSJ(root);
  root.close();
  sSJ = int2str(nSJ, 3);
  root = SD.open("/DATA/");
  nSS = getSS(root, sSJ);
  root.close();
  sSS = int2str(nSS, 3);
  
  pinMode(28, OUTPUT);//MOSFET SOURCE FOR TESTING
  digitalWrite(28, LOW);//ensure FES is off
  
  pinMode(FES_PIN, OUTPUT);
  digitalWrite(FES_PIN, LOW);//ensure FES is off
  
  randomSeed(analogRead(0));//disconnected channel
  
  clearscreen();
  drawmainmenu();
}


//integer to string conversion with set precision width - for writing to SD card txt files
String int2str(int val, int width) {
  String str = String(val);
  if (val==0) { val = 1; }
  int valwidth = (int) floor(log10((double)val));
  valwidth = width - valwidth - 1;
  for (int i = 0; i < valwidth; i++)
  {
    str = "0" + str;
  }
  return str;
} 


// get the last SUBJECT ID
int getSJ(File dir) {
  int mid_int = 0, id_int = 0;
  String id_str = "000";
  //get sID
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    //if (entry.name()[4] == 'B') //FOR DEBUGGING - set train and z files to 001
    //{
    id_str[0] = entry.name()[1];
    id_str[1] = entry.name()[2];
    id_str[2] = entry.name()[3];
    
    id_int = id_str.toInt();
    if (id_int > mid_int)
    {
      mid_int = id_int;
    }
    //}
    entry.close();
  }
  return mid_int;
}

// get the last SESSION ID for the given SUBJECT ID
int getSS(File dir, String SJ) {
  int ms_int = 0, s_int = 0;
  String s_str = "000", sn = "S";
  //get sess # (S)
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    if (entry.name()[1]==SJ[0] && entry.name()[2]==SJ[1] && entry.name()[3]==SJ[2] && entry.name()[4]==sn[0])
    {
      s_str[0] = entry.name()[5];
      s_str[1] = entry.name()[6];
      s_str[2] = entry.name()[7];
      
      s_int = s_str.toInt();
      if (s_int > ms_int)
      {
        ms_int = s_int;
      }
    }
    entry.close();
  }
  return ms_int;
}
  

void clearscreen() 
{
  tft.fillScreen(ILI9341_BLACK);
}


void drawmainmenu()
{ 
  tft.setCursor(32, 15);
  tft.setTextColor(ILI9341_BLUE);
  tft.setTextSize(3);
  tft.print("BCI SYSTEM");
  tft.setTextColor(ILI9341_BLACK);
  tft.setTextSize(3);
  tft.fillRoundRect(5, 50, 230, 82, 8, ILI9341_DARKGREY);
  tft.drawRoundRect(5, 50, 230, 82, 8, ILI9341_LIGHTGREY);
  tft.setCursor(25, 80);
  tft.println("TEST OUTPUT");
  tft.fillRoundRect(5, 140, 100, 82, 8, ILI9341_DARKGREY);
  tft.drawRoundRect(5, 140, 100, 82, 8, ILI9341_LIGHTGREY);
  tft.setTextSize(2);
  tft.setCursor(26, 175);
  tft.println("TRAIN");
  tft.fillRoundRect(109, 140, 126, 82, 8, ILI9341_DARKGREY);
  tft.drawRoundRect(109, 140, 126, 82, 8, ILI9341_LIGHTGREY);
//  tft.setCursor(135, 162);
//  tft.println("SET BSM");
//  tft.setCursor(125, 187);
//  tft.println("THRESHOLD");
  tft.setCursor(120, 175);
  tft.println("CALIBRATE");
  
  tft.setTextSize(3);
  tft.fillRoundRect(5, 230, 230, 82, 8, ILI9341_DARKGREY);
  tft.drawRoundRect(5, 230, 230, 82, 8, ILI9341_LIGHTGREY);
  tft.setCursor(14, 255);
  tft.println("REALTIME BCI");
}


void exitbutton() 
{
  tft.fillRoundRect(1, 290, 240, 30, 8, ILI9341_LIGHTGREY);
  tft.drawRoundRect(1, 290, 240, 30, 8, ILI9341_LIGHTGREY);
  tft.setTextColor(ILI9341_RED);
  tft.setTextSize(3);
  tft.setCursor(90, 295);
  tft.println("EXIT");
}


void drawidle() 
{
  tft.setCursor(100, 140);
  tft.setTextColor(ILI9341_BLACK);
  tft.print("GO");
}


void drawmove() 
{
  tft.setCursor(100, 140);
  tft.setTextColor(ILI9341_GREEN);
  tft.print("GO");
}

void drawxvalidating() 
{
  clearscreen();
  tft.setTextColor(ILI9341_GREEN, ILI9341_BLACK);
  tft.setTextSize(3);
  tft.setCursor(5, 100);  
  tft.println("Running");
  tft.setCursor(5, 135); 
  tft.println("Cross");
  tft.setCursor(5, 170); 
  tft.println("Validation");
}

void drawaccuracy(float accuracy) 
{
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextSize(4);
  tft.setTextColor(ILI9341_GREEN);
  tft.setCursor(20, 90);
  tft.print("Accuracy");
  tft.setCursor(23, 140);
  tft.print("   =   ");
  
  String accuracy_str = "";
  accuracy_str += String(accuracy);
  accuracy_str += " %";
  char accuracy_char[8];
  accuracy_str.toCharArray(accuracy_char,8);
  tft.setCursor(23, 190);
  
  if (accuracy < 85)
  {
     tft.setTextColor(ILI9341_RED);
     tft.print(accuracy_char);
  }
  else
  {
    tft.setTextColor(ILI9341_GREEN);
    tft.print(accuracy_char);
  }
}

//GUI:idle histogram bin generation
void drawidlePPs() {

  float idles[20];
  unsigned long pos = 0;
  int sd_i = 0;
  unsigned long pos2 = 0;
  int sd_i2 = 0;
  int state;
  float x;
  
  for (int i = 0; i < 20; i++) {
    idles[i] = 0;
  }

  File pps = SD.open("/DATA/S" + sSJ + "P.txt", O_READ);  
  if (pps) {
    
    for (int i = 0; i < 480; i++) {
      state = SDindex2float(pps, i, 0, 2, pos, sd_i);
      
      if (state == 0) { // idle
        x = SDindex2float(pps, i, 1, 2, pos2, sd_i2);
        
        for (int j = 0; j < 20; j++) { // loops through all histogram bins
          if (x > j*0.05 && x < (j+1)*0.05) {
            idles[j]++;
          }
        }
      }
    }

      float m = 0;
    for (int i = 0; i < 20; i ++) {
      if (m < idles[i]) {
        m = idles[i];
      }
    }
    for (int i = 0; i < 20; i ++) {
      idles[i] /= m;
    }
  
    for (int i = 0; i < 20; i++) {
      tft.fillRect(i*16, 225-idles[i]*225, 16, idles[i]*225, ILI9341_RED);
      tft.drawRect(i*16, 225-idles[i]*225, 16, idles[i]*225, ILI9341_WHITE);
    }
  
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_WHITE);
    
    for (int i = 1; i <= 10; i++) {
      tft.setCursor(i*32-19, 230);
      tft.println((float) i/10,1);
      tft.setCursor(i*32-12, 220);
      tft.println("|");
    }
  
  }
  pps.close();
}

//GUI:move histogram bin generation
void drawmovePPs() {

  float moves[20];
  unsigned long pos = 0;
  int sd_i = 0;
  unsigned long pos2 = 0;
  int sd_i2 = 0;
  int state;
  float x;
  
  for (int i = 0; i < 20; i++) {
    moves[i] = 0;
  }

  File pps = SD.open("/DATA/S" + sSJ + "P.txt", O_READ);  
  if (pps) {
    
    for (int i = 0; i < 480; i++) {
      state = SDindex2float(pps, i, 0, 2, pos, sd_i);
      
    if (state == 1) { // move
      x = SDindex2float(pps, i, 1, 2, pos2, sd_i2);
      
          for (int j = 0; j < 20; j++) { // loops through all histogram bins
            if (x > j*0.05 && x < (j+1)*0.05) {
              moves[j]++;
            }
          }
      }
    }
    
    float m = 0;
    for (int i = 0; i < 20; i ++) {
      if (m < moves[i]) {
        m = moves[i];
      }
    }
    for (int i = 0; i < 20; i ++) {
      moves[i] /= m;
    }
  
    for (int i = 0; i < 20; i++) {
      tft.fillRect(i*16, 225-moves[i]*225, 16, moves[i]*225, ILI9341_BLUE);
      tft.drawRect(i*16, 225-moves[i]*225, 16, moves[i]*225, ILI9341_WHITE);
    }
  
    tft.setTextSize(1);
    tft.setTextColor(ILI9341_WHITE);
    
    for (int i = 1; i <= 10; i++) {
      tft.setCursor(i*32-19, 230);
      tft.println((float) i/10,1);
      tft.setCursor(i*32-12, 220);
      tft.println("|");
    }
  }
  pps.close();
}

//GUI:buttons for histogram
void draw_bsm_gui() {
  tft.setTextSize(2);
  tft.setTextColor(ILI9341_WHITE);
  tft.fillRoundRect(48, 10, 60, 25, 8, ILI9341_DARKGREY); //HIGH
  tft.fillRoundRect(48, 40, 60, 25, 8, ILI9341_DARKGREY); //LOW
  tft.setCursor(55, 15);
  tft.println("HIGH");
  tft.setCursor(60, 45);
  tft.println("LOW");

  tft.setTextSize(3);
  tft.setTextColor(ILI9341_WHITE);
  tft.fillTriangle(150, 25, 165, 10, 180, 25, ILI9341_DARKGREY); //top left arrow
  tft.fillTriangle(190, 25, 205, 10, 220, 25, ILI9341_DARKGREY); //top right arrow
  tft.fillTriangle(150, 65, 165, 80, 180, 65, ILI9341_DARKGREY); //bottom left arrow
  tft.fillTriangle(190, 65, 205, 80, 220, 65, ILI9341_DARKGREY); //bottom right arrow
  tft.drawTriangle(150, 25, 165, 10, 180, 25, ILI9341_WHITE); //top left arrow
  tft.drawTriangle(190, 25, 205, 10, 220, 25, ILI9341_WHITE); //top right arrow
  tft.drawTriangle(150, 65, 165, 80, 180, 65, ILI9341_WHITE); //bottom left arrow
  tft.drawTriangle(190, 65, 205, 80, 220, 65, ILI9341_WHITE); //bottom right arrow
  tft.setCursor(120, 35);
  tft.println("0.");
  tft.setCursor(160, 35);
  if (bsm_option == 1) {
    tft.println(lowbsm_tenths);
  } else if (bsm_option == 2) {
    tft.println(highbsm_tenths);
  }
  tft.setCursor(200, 35);
  if (bsm_option == 1) {
    tft.println(lowbsm_hundredths);
  } else if (bsm_option == 2) {
    tft.println(highbsm_hundredths);
  }
}



// detect finger push for main loop and histogram buttons
void loop() {

  if (ts.touched())
  {
    TS_Point p = ts.getPoint();
    while (!ts.bufferEmpty() || ts.touched())
    {
      p = ts.getPoint();
    }
    p.x = map(p.x, TS_MINX, TS_MAXX, 0, 240);
    p.y = map(p.y, TS_MINY, TS_MAXY, 0, 320);
    

  if (bsm_select == false) { //main menu gui navigation
    if (p.y > 50 && p.y < 132) {
        command = 'a';
      }
      if (p.y > 140 && p.y < 222 && p.x < 120) {
        command = 'd';
      }
      if (p.y > 140 && p.y < 222 && p.x > 120) {
        command = 'e';
      }
      if (p.y > 230 && p.y < 312) {
        command = 'c';
      }
    }
  
  if (bsm_select == true) { //bsm selection gui navigation

      if (p.x > 210 && p.x < 240 && p.y > 40 && p.y < 115) { //view move histogram (high)
        Serial.println("HIGH");
        bsm_option = 2;
        clearscreen();
        drawmovePPs();
        draw_bsm_gui();
        tft.drawRoundRect(48, 10, 60, 25, 8, ILI9341_WHITE);
      }
      if (p.x > 170 && p.x < 200 && p.y > 40 && p.y < 115) { //view idle histogram (low)
        Serial.println("LOW");
        bsm_option = 1;
        clearscreen();
        drawidlePPs();
        draw_bsm_gui();
        tft.drawRoundRect(48, 40, 60, 25, 8, ILI9341_WHITE);
      }

    
    if (bsm_option !=0) {
      if (p.x > 200 && p.x < 240 && p.y > 130 && p.y < 170) { //top left arrow
        if (bsm_option == 2) {
          if (highbsm_tenths < 9) {
            highbsm_tenths++;
          }
        }
        if (bsm_option == 1) {
          if (lowbsm_tenths < 9) {
            lowbsm_tenths++;
          }
        }
        bsm_updated = true;
      }
      if (p.x > 140 && p.x < 180 && p.y > 130 && p.y < 170) { //bottom left arrow
        if (bsm_option == 2) {
          if (highbsm_tenths > 0) {
            highbsm_tenths--;
          }
        }
        if (bsm_option == 1) {
          if (lowbsm_tenths > 0) {
            lowbsm_tenths--;
          }
        }
        bsm_updated = true;
      }
      if (p.x > 200 && p.x < 240 && p.y > 180 && p.y < 220) { //top right arrow
        if (bsm_option == 2) {
          if (highbsm_hundredths < 9) {
            highbsm_hundredths++;
          }
        }
        if (bsm_option == 1) {
          if (lowbsm_hundredths < 9) {
            lowbsm_hundredths++;
          }
        }
        bsm_updated = true;
      }
      if (p.x > 140 && p.x < 180 && p.y > 180 && p.y < 220) { //bottom right arrow
        if (bsm_option == 2) {
          if (highbsm_hundredths > 0) {
            highbsm_hundredths--;
          }
        }
        if (bsm_option == 1) {
          if (lowbsm_hundredths > 0) {
            lowbsm_hundredths--;
          }
        }
        bsm_updated = true;
      }

      if ((lowbsm_hundredths != 0 || lowbsm_tenths != 0) && (highbsm_hundredths != 0 || highbsm_tenths != 0)) {
        tft.setTextSize(2);
        tft.setTextColor(ILI9341_WHITE);
        tft.fillRoundRect(235, 30, 40, 30, 8, ILI9341_DARKGREY); //Enter button
        tft.drawRoundRect(235, 30, 40, 30, 8, ILI9341_WHITE);
        tft.setCursor(245, 37);
        tft.println(">>");
      } else {
        tft.fillRoundRect(235, 30, 40, 30, 8, ILI9341_BLACK); //erase enter button
      }

      if (p.x > 180 && p.x < 210 && p.y > 220 && p.y < 275) { //enter button
        
          lowthres = (float) lowbsm_tenths/10 + (float) lowbsm_hundredths/100;
          highthres = (float) highbsm_tenths/10 + (float) highbsm_hundredths/100;

        
        if (lowthres != 0 && highthres != 0) {
          if (SD.exists("/DATA/S" + sSJ + "B.txt")) {
            SD.remove("/DATA/S" + sSJ + "B.txt");
          }
          
          File bsmthresfile = SD.open("/DATA/S" + sSJ + "B.txt", O_CREAT | O_WRITE);
          String thr_Str = "";
          thr_Str = String(highthres,20);
          thr_Str += ",";
          thr_Str += String(lowthres,20);
          bsmthresfile.println(thr_Str);
          bsmthresfile.close();
          bsm_select = false;
          bsm_option = 0;
          tft.setRotation(0);
          clearscreen();
          drawmainmenu();
        }
      }
    }

      
      if (bsm_option == 1 && bsm_updated == true) { //updating numbers on the gui
        tft.setTextSize(3);
        tft.setTextColor(ILI9341_BLACK);
        tft.setCursor(160, 35);
        tft.println(lowlast_bsm_tenths);
        tft.setCursor(200, 35);
        tft.println(lowlast_bsm_hundredths);
        
        tft.setTextColor(ILI9341_WHITE);
        tft.setCursor(160, 35);
        tft.println(lowbsm_tenths);
        tft.setCursor(200, 35);
        tft.println(lowbsm_hundredths);
        
        lowlast_bsm_tenths = lowbsm_tenths;
        lowlast_bsm_hundredths = lowbsm_hundredths;
        bsm_updated = false;
      }
      if (bsm_option == 2 && bsm_updated == true) { //updating numbers on the gui
        tft.setTextSize(3);
        tft.setTextColor(ILI9341_BLACK);
        tft.setCursor(160, 35);
        tft.println(highlast_bsm_tenths);
        tft.setCursor(200, 35);
        tft.println(highlast_bsm_hundredths);
        
        tft.setTextColor(ILI9341_WHITE);
        tft.setCursor(160, 35);
        tft.println(highbsm_tenths);
        tft.setCursor(200, 35);
        tft.println(highbsm_hundredths);
        
        highlast_bsm_tenths = highbsm_tenths;
        highlast_bsm_hundredths = highbsm_hundredths;
        bsm_updated = false;
      }
   
      delay(200);
    }
    
  switch (command) {
    case 'a':
//      clearscreen();
//      drawaccuracy(96.0);
//      tft.setTextSize(4);
      Serial.println("Toggle Output");
      if (Toggle_State==0) {
        Toggle_State = 1;
        digitalWrite(FES_PIN, HIGH);
        tft.fillRoundRect(5, 50, 230, 82, 8, ILI9341_DARKCYAN); 
      } else {
        Toggle_State = 0;
        digitalWrite(FES_PIN, LOW);
        tft.fillRoundRect(5, 50, 230, 82, 8, ILI9341_DARKGREY);
        tft.drawRoundRect(5, 50, 230, 82, 8, ILI9341_WHITE);
      }
      tft.setTextColor(ILI9341_BLACK);
      tft.setTextSize(3);
      tft.setCursor(25, 80);
      tft.println("TEST OUTPUT");
      delay(200);
//        delay(10000);
      command = ' ';
      break;
      
    case 'd':
      Serial.println("Collect Training Data");
      tft.fillRoundRect(5, 140, 100, 82, 8, ILI9341_DARKCYAN);
      tft.setTextSize(2);
      tft.setCursor(26, 175);
      tft.println("TRAIN");
      delay(500);
      train();
      drawmainmenu();
      delay(200);
      command = ' ';
      break;
      
    case 'e':
      Serial.println("Calibrate BSM");
      tft.fillRoundRect(109, 140, 126, 82, 8, ILI9341_DARKCYAN);
      tft.setTextSize(2);
      tft.setCursor(120, 175);
      tft.println("CALIBRATE");
      delay(500);
      clearscreen();
      
      if (SD.exists("/DATA/S" + sSJ + "B.txt")) {
        File B = SD.open("/DATA/S" + sSJ + "B.txt", O_READ);
        unsigned long pos = 0;
        int sd_i = 0;

        float highbsm = SDindex2float(B, 0, 0, 2, pos, sd_i);
        float lowbsm = SDindex2float(B, 0, 1, 2, pos, sd_i);

        highbsm_tenths = (int) (highbsm * 10);
        highbsm_hundredths = (int) (round(highbsm * 100)) - (10 * highbsm_tenths);

        lowbsm_tenths = (int) (lowbsm * 10);
        lowbsm_hundredths = (int) (round(lowbsm * 100)) - (10 * lowbsm_tenths);

        lowlast_bsm_tenths = lowbsm_tenths;
        lowlast_bsm_hundredths = lowbsm_hundredths;
        highlast_bsm_tenths = highbsm_tenths;
        highlast_bsm_hundredths = highbsm_hundredths;
        
        B.close();
      } else {
        lowbsm_tenths = 0;
        lowbsm_hundredths = 0;
        highbsm_tenths = 0;
        highbsm_hundredths = 0;
      }
      
      if (SD.exists("/DATA/S" + sSJ + "P.txt")) {
        bsm_select = true;
        tft.setRotation(1);
        tft.setTextSize(2);
        tft.setTextColor(ILI9341_WHITE);
        tft.fillRoundRect(48, 10, 60, 25, 8, ILI9341_DARKGREY);
        tft.fillRoundRect(48, 40, 60, 25, 8, ILI9341_DARKGREY);
        tft.setCursor(55, 15);
        tft.println("HIGH");
        tft.setCursor(60, 45);
        tft.println("LOW");
      } else {
        clearscreen();
        tft.setCursor(10,50);
        tft.setTextSize(3);
        tft.setTextColor(ILI9341_RED);
        tft.println("No Posterior");
        tft.setCursor(10,90);
        tft.println("Probability");
        tft.setCursor(10,130);
        tft.println("File Found!");
        delay(2000);
        clearscreen();
        drawmainmenu();
      }
      
      delay(200);
      command = ' ';
      break;
      
    case 'c':
      Serial.println("Real-Time BCI Testing");
      tft.fillRoundRect(5, 230, 230, 82, 8, ILI9341_DARKCYAN);
      tft.setCursor(14, 255);
      tft.println("REALTIME BCI");
      delay(500);            
      online_cue();
      drawmainmenu();
      delay(200);
      command = ' ';
      break;
    }
  }
}



//save last SD index and use it to search locally so we can read SD card much faster
//ASSUMES comma delimited columns and \n delimited rows in txt file
double SDindex2float(File& datafile, int m, int n, int ncols, unsigned long& near_pos, int& i)
{
  int target_i = m*ncols + n;
  char ch;
  String target = "";
  
  datafile.seek(near_pos);
  while (datafile.available() && i < target_i)
  {
    ch = char(datafile.read());
    if (ch == ',' || ch == '\n')
    {
      i++;
    }
  }
  
  while (1)
  {
    ch = char(datafile.read());
    if (ch == ',' || ch == '\n')
    {
      near_pos = datafile.position() - 1;
      break;
    }
    else
    {
      target += ch;
    }
  }
  
  return target.toFloat();
}


//get the parameters for z scoring each channel (mean and standard deviation)
//in time domain using a single cue duration and save to Z file
void get_zscore_parms()
{
  unsigned long tStart, tStop;
  int i, nC_i;
  float X[nTr*nC], x, X_mean[nC], X_std[nC];

  for (nC_i = 0; nC_i < nC; nC_i++)
  {
    X_mean[nC_i] = 0;
    X_std[nC_i] = 0;
  }
  
  for (i=0; i<nTr*nC; i+=nC)
  {
    tStart = micros();
    for (nC_i = 0; nC_i < nC; nC_i++)
    {
      x = VoltScale * (double)analogRead(analogChans[nC_i]);
      X[i+nC_i] = x;
      X_mean[nC_i] += x;
    }
    tStop = micros();
    delayMicroseconds(tSampling + tStart - tStop);
  }

  //Finish calculating MEAN
  for (nC_i = 0; nC_i < nC; nC_i++)
  {
    X_mean[nC_i] /= (double)nTr;
  }
  
  //Calculate STD
  for (i=0; i<nTr*nC; i+=nC)
  {
    for (nC_i = 0; nC_i < nC; nC_i++)
    {
      X_std[nC_i] += pow(X[i+nC_i] - X_mean[nC_i],2);
    }
  }
  
  X_std[0] = sqrt(X_std[0] / (double)nTr);
  String X_mean_str = String(X_mean[0],20);
  String X_std_str = String(X_std[0],20);
  for (nC_i = 1; nC_i < nC; nC_i++)
  {
    X_std[nC_i] = sqrt(X_std[nC_i] / (double)nTr);
    X_mean_str += ",";
    X_mean_str += String(X_mean[nC_i],20);
    X_std_str += ",";
    X_std_str += String(X_std[nC_i],20);
  }

  if (!SD.exists("/DATA/S" + sSJ + "Z.txt"))
  {
    File zsfile = SD.open("/DATA/S" + sSJ + "Z.txt", O_CREAT | O_WRITE);
    zsfile.println(X_mean_str);
    zsfile.println(X_std_str);
    zsfile.close();
  }
}


//obtain training data using above parameters
void train()
{
  clearscreen();
  tft.setTextSize(4);
  unsigned long tStart, tStop, tDelay;
  int state, i, nC_i, nF_j, b_j, z_ij;
  double X_mean[nC], X_std[nC], Z[4*nD], Y[nD], X[nC], y_ij;
  String Y_Str; // for speed

  nSJ++;
  sSJ = int2str(nSJ, 3);
  
  //zero Z and Y
  for (i = 0; i < 4*nD; i++)
  {
    Z[i] = 0;
  }
  for (i = 0; i < nD; i++)
  {
    Y[i] = 0;
  }

  drawidle();
  delay(6000);
  
  drawmove();
  delay(1000);
  get_zscore_parms();
  File zsfile = SD.open("/DATA/S" + sSJ + "Z.txt", O_READ);
  if (zsfile) // get mean and standard deviations from file
  {
    unsigned long pos = 0;
    int sd_i = 0;
    for (i = 0; i < nC; i++)
    {
      X_mean[i] = SDindex2float(zsfile,0,i,nC,pos,sd_i);
    }
    for (i = 0; i < nC; i++)
    {
      X_std[i] = SDindex2float(zsfile,1,i,nC,pos,sd_i);
    }
  }
  else
  {
    Serial.println("No zscore parms file found!");
    return;
  }
  zsfile.close();
  
  if (!SD.exists("/DATA/S" + sSJ + "T.txt"))
  { 
    File trfile = SD.open("/DATA/S" + sSJ + "T.txt", O_CREAT | O_WRITE);
  
    // FIRST IDLE - nothing to store during 1st second so just delay
    drawidle();
    state = 0;
    delay(1000);
    for (i = 0; i < nTr; i++)
    {
      tStart = micros();
      for (nC_i = 0; nC_i < nC; nC_i++)
      {
        X[nC_i] = (VoltScale * (double)analogRead(analogChans[nC_i])) - X_mean[nC_i];
        X[nC_i] /= X_std[nC_i];
      }
      for (nC_i = 0; nC_i < nC; nC_i++)
      {
        for (nF_j = 0; nF_j < nF; nF_j++)
        {
          b_j = nF_j*10;
          z_ij = nC_i*8 + nF_j*4; // 0, 4 ch 0 ,   8, 12 ch 1,    16, 20 ch 2
          y_ij = filterCoeffs[b_j] * X[nC_i] + Z[z_ij];
          Z[z_ij]  = filterCoeffs[b_j+1] * X[nC_i] + Z[z_ij+1]  - filterCoeffs[b_j+6] * y_ij;
          Z[z_ij+1]  = filterCoeffs[b_j+2] * X[nC_i] + Z[z_ij+2]  - filterCoeffs[b_j+7] * y_ij;
          Z[z_ij+2]  = filterCoeffs[b_j+3] * X[nC_i] + Z[z_ij+3]  - filterCoeffs[b_j+8] * y_ij;
          Z[z_ij+3]  = filterCoeffs[b_j+4] * X[nC_i]              - filterCoeffs[b_j+9] * y_ij;
          y_ij *= y_ij;
          Y[nC_i*2 + nF_j] += y_ij;
        }
      }
      tStop = micros();
      delayMicroseconds(tSampling + tStart - tStop);
    }
    Y_Str = String(state);
    for (nC_i = 0; nC_i < nC; nC_i++)
    {
      for (nF_j = 0; nF_j < nF; nF_j++)
      {
        Y_Str += ","; 
        Y_Str += String(Y[nC_i*2 + nF_j] / i, 20);
      }
    }
    
    // REST OF CUES
    for (int trial = 1; trial < nTrainingObs; trial++)
    {
      if (trial % 2 == 0) 
      {
        drawidle();
        state = 0;
      }
      else
      {
        drawmove();
        state = 1;
      }
      
      //Write previous data to SD card - during initial 1 second of cue (transition period that we don't collect data from)
      tStart = micros();
      
      Serial.println(Y_Str);
      trfile.println(Y_Str);
        
      //zero Z and Y
      for (i = 0; i < 4*nD; i++)
      {
        Z[i] = 0;
      }
      for (i = 0; i < nD; i++)
      {
        Y[i] = 0;
      }
      
      tStop = micros();
      tDelay = (unsigned long)round((1000000 + tStart - tStop) / 1000);
      delay(tDelay);
      
      //Filter signals and calculate avg power
      for (i = 0; i < nTr; i++)
      {
        tStart = micros();
        for (nC_i = 0; nC_i < nC; nC_i++)
        {
          X[nC_i] = (VoltScale * (double)analogRead(analogChans[nC_i])) - X_mean[nC_i];
          X[nC_i] /= X_std[nC_i];
        }
        for (nC_i = 0; nC_i < nC; nC_i++)
        {
          for (nF_j = 0; nF_j < nF; nF_j++)
          {
            //filter channel number nC_i to frequency band number nF_j
            b_j = nF_j*10;
            z_ij = nC_i*8 + nF_j*4; // 0, 4 ch 0 ,   8, 12 ch 1,    16, 20 ch 2
            y_ij = filterCoeffs[b_j] * X[nC_i] + Z[z_ij];
            Z[z_ij]  = filterCoeffs[b_j+1] * X[nC_i] + Z[z_ij+1]  - filterCoeffs[b_j+6] * y_ij;
            Z[z_ij+1]  = filterCoeffs[b_j+2] * X[nC_i] + Z[z_ij+2]  - filterCoeffs[b_j+7] * y_ij;
            Z[z_ij+2]  = filterCoeffs[b_j+3] * X[nC_i] + Z[z_ij+3]  - filterCoeffs[b_j+8] * y_ij;
            Z[z_ij+3]  = filterCoeffs[b_j+4] * X[nC_i]              - filterCoeffs[b_j+9] * y_ij;
            y_ij *= y_ij;
            Y[nC_i*2 + nF_j] += y_ij;
          }
        }
        tStop = micros();
        delayMicroseconds(tSampling + tStart - tStop);
      }
      Y_Str = String(state);
      for (nC_i = 0; nC_i < nC; nC_i++)
      {
        for (nF_j = 0; nF_j < nF; nF_j++)
        {
          Y_Str += ","; 
          Y_Str += String(Y[nC_i*2 + nF_j] / i, 20);
        }
      }
    }
  
    Serial.println(Y_Str);
    trfile.println(Y_Str);
    
    trfile.close();
  }

  byte contstatus = xvalidate();

  if (contstatus==1)// if cross validation performance is good enough to continue (>=80%)
  {
    generate_bsm_thres();
  }
}



int generate_classifier(double *w, double &m0, double &v0, double &m1, double &v1, int *tstR)
{ 
  int errors = 0;
  File trfile = SD.open("/DATA/S" + sSJ + "T.txt", O_READ);
  if (trfile)
  {
    unsigned long pos = 0;
    int i, j, k, tstR_i, n0, n1, sd_i;
    double x_ij, mu0[nD], mu1[nD], pt[nD], S0[nD*nD], S1[nD*nD], wx_i, lh0, lh1, pp1;
    
    // initialize to zero
    n0 = 0;
    n1 = 0;
    for (i = 0; i < nD; i++)
    {
      mu0[i] = 0;
      mu1[i] = 0;
      pt[i] = 0;
      for (j = 0; j < nD; j++) 
      { 
        S0[i*nD + j] = 0;
        S1[i*nD + j] = 0;
      }
    }

    
    // GET CLASS MEANS (MU0, MU1)
    tstR_i = 0;
    pos = 0;
    sd_i = 0;
    for (i = 0; i < nTrainingObs; i++)
    {
      if (i == tstR[tstR_i])
      {
        tstR_i++;
        continue;
      }
      else 
      {
        x_ij = SDindex2float(trfile,i,0,nD+1,pos,sd_i);
        if (x_ij == 0)//idle
        { 
          n0 += 1;
          for (j = 0; j < nD; j++)
          {
            mu0[j] += SDindex2float(trfile,i,j+1,nD+1,pos,sd_i);
          }
        }
        else if (x_ij == 1)//move
        {
          n1 += 1;
          for (j = 0; j < nD; j++)
          {
            mu1[j] += SDindex2float(trfile,i,j+1,nD+1,pos,sd_i);
          }
        }
      }
    }
    for (j = 0; j < nD; j++)
    {
      mu0[j] /= (double)n0;
      mu1[j] /= (double)n1;
    }

    
    // GET CLASS COVS (S0, S1) 
    tstR_i = 0;
    pos = 0;
    sd_i = 0;   
    for (i = 0; i < nTrainingObs; i++)
    {
      if (i == tstR[tstR_i])
      {
        tstR_i++;
        continue;
      }
      else
      {
        x_ij = SDindex2float(trfile,i,0,nD+1,pos,sd_i);
        if (x_ij == 0)//idle
        {   
          for (j = 0; j < nD; j++)
          {
            x_ij = SDindex2float(trfile,i,j+1,nD+1,pos,sd_i);
            pt[j] = x_ij - mu0[j];
          }
          
          // update covariance matrix with outer product
          for (j = 0; j < nD; j++)
          {
            for (k=0; k < nD; k++)
            {
              S0[j*nD + k] += pt[j]*pt[k];
            }
          }
        }
        else if (x_ij == 1)//move
        {   
          for (j = 0; j < nD; j++)
          {
            x_ij = SDindex2float(trfile,i,j+1,nD+1,pos,sd_i);
            pt[j] = x_ij - mu1[j];
          }
          
          // update covariance matrix with outer product
          for (j = 0; j < nD; j++)
          {
            for (k=0; k < nD; k++)
            {
              S1[j*nD + k] += pt[j]*pt[k];
            }
          }
        }
      }
    }
    for (i = 0; i < nD; i++)
    {
      for (j = 0; j < nD; j++)
      {
        S0[i*nD + j] /= (double)n0;
        S1[i*nD + j] /= (double)n1;
      }
    }
    
    //RUN LDA - get weighting vector w for dimensionality reduction
    LDA(mu0, mu1, S0, S1, w, n0, n1, nD);
    
    // scale w so that mu * w = 1
    // find s where mu * w = s s.t. mu * (w/s) = 1
    // we do this to ensure we capture the information when saving to SD card
    double s;
    for (i = 0; i < nD; i++)
    {
      s += w[i] * (n0*mu0[i] + n1*mu1[i]) / (double)(n0+n1);
    }
    for (i = 0; i < nD; i++)
    {
      w[i] /= s;
    }
    
    m0 = 0;
    v0 = 0;
    m1 = 0;
    v1 = 0;
    for (i = 0; i < nD; i++)
    {
      m0 += w[i] * mu0[i];
      m1 += w[i] * mu1[i];
      for (j = 0; j < nD; j++)
      {
        v0 += w[i]*S0[i*nD + j]*w[j];
        v1 += w[i]*S1[i*nD + j]*w[j];
      }
    }
    
    //Calculate # of errors for test points
    tstR_i = 0;
    pos = 0;
    sd_i = 0;
    for (i = 0; i < nTrainingObs; i++)
    {
      if (i != tstR[tstR_i])
      {
        continue;
      }
      else 
      {
        x_ij = SDindex2float(trfile,i,0,nD+1,pos,sd_i);
        //Calculate w * x_i
        wx_i = 0.0;
        for (j = 0; j < nD; j++)
        {
          wx_i += SDindex2float(trfile,i,j+1,nD+1,pos,sd_i) * w[j];
        }
        //Predict state using naive bayes with gaussian likelihood and equal priors
        lh0 = exp(-pow(wx_i - m0,2)/(2*v0)) / sqrt(v0);
        lh1 = exp(-pow(wx_i - m1,2)/(2*v1)) / sqrt(v1);
        pp1 = lh1 / (lh0 + lh1);
        if (round(pp1) != x_ij)
        {
          errors++;
        }
        tstR_i++; //move on to next test point
      }
    }
  }
  else
  {
    Serial.println("No training file found.");
  }
  trfile.close();
  return errors;
}



byte xvalidate()    // LEAVE ONE OUT CROSSVALIDATION IS USED
{
  int i, j, r, tstR[2], n0 = 0, n1 = 0, errors;
  double w[nD], m0, v0, m1, v1, error_rate = 0.0;
  
  clearscreen();
  drawxvalidating();
  
  File trfile = SD.open("/DATA/S" + sSJ + "T.txt", O_READ);
  if (trfile)
  {
    unsigned long pos = 0;
    int sd_i = 0;
    double x_ij;
    for (i = 0; i < nTrainingObs; i++)
    {
      x_ij = SDindex2float(trfile,i,0,nD+1,pos,sd_i);
      if (x_ij == 0)//idle
      { 
        n0++;
      }
      else if (x_ij == 1)//move
      {
        n1++;
      }
    }
  }
  else
  {
    Serial.println("No training file found.");
    return 0;
  }
  trfile.close();

  tstR[1] = -1;
  //LEAVE 1 OUT XVALIDATION
  for (i = 0; i < n0+n1; i++)
  {
    tstR[0] = i;
    // generate classifier
    errors = (int)generate_classifier(w, m0, v0, m1, v1, tstR);
    error_rate += (double)errors / (double)(n0 + n1); 
  }
  
  double accuracy = 100.0 * (1.0 - error_rate);
  
  clearscreen();
  drawaccuracy(accuracy);
  tft.setTextSize(4);
  Serial.print("Accuracy = ");
  Serial.println(accuracy);
  Serial.println("Finished Cross Validation.");
  delay(5000);
  
  clearscreen();
  if (accuracy < 80.0)
  {
    return 0;
  }
  
  tstR[0] = -1;
  // generate classifier
  errors = generate_classifier(w, m0, v0, m1, v1, tstR);

  String w_Str;
  
  //create new coeffs file and keep it open - only 1 write here
  if (!SD.exists("/DATA/S" + sSJ + "W.txt"))
  {
    File coeffsfile = SD.open("/DATA/S" + sSJ + "W.txt", O_CREAT | O_WRITE);
    w_Str = String(w[0],20);
    for (i = 1; i < nD; i++)
    {
      w_Str += ",";
      w_Str += String(w[i],20);
    }
    coeffsfile.println(w_Str);
    coeffsfile.close();
  }
  
  if (!SD.exists("/DATA/S" + sSJ + "G.txt"))
  {
    //create new cparms file and keep it open - only 1 write here
    File cparmsfile = SD.open("/DATA/S" + sSJ + "G.txt", O_CREAT | O_WRITE);
    w_Str = String(m0,20);
    w_Str += ",";
    w_Str += String(m1,20);
    w_Str += ",";
    w_Str += String(v0,20);
    w_Str += ",";
    w_Str += String(v1,20);
    cparmsfile.println(w_Str);
    cparmsfile.close();
  }
  return 1;
}


//get data for generating the binary state machine thresholds
void generate_bsm_thres() 
{
  unsigned long pos;
  int i, j, k, sd_i;
  double X_mean[nC], X_std[nC], w[nD], m0, m1, v0, v1;
  
  clearscreen();
  
  File zsfile = SD.open("/DATA/S" + sSJ + "Z.txt", O_READ);
  if (zsfile)
  {
    pos = 0;
    sd_i = 0;
    for (i = 0; i < nC; i++)
    {
      X_mean[i] = SDindex2float(zsfile,0,i,nC,pos,sd_i);
    }
    for (i = 0; i < nC; i++)
    {
      X_std[i] = SDindex2float(zsfile,1,i,nC,pos,sd_i);
    }
  }
  else
  {
    Serial.println("No zscore parms file found!");
    return;
  }
  zsfile.close();
  
  File coeffsfile = SD.open("/DATA/S" + sSJ + "W.txt", O_READ);
  if (coeffsfile)
  {  
    pos = 0;
    sd_i = 0;
    for (i = 0; i < nD; i++)
    {
      w[i] = SDindex2float(coeffsfile,0,i,nD,pos,sd_i);
    }
  }
  else
  {
    Serial.println("No coeffs file found!");
    return;
  }
  coeffsfile.close();
  
  File cparmsfile = SD.open("/DATA/S" + sSJ + "G.txt", O_READ);
  if (cparmsfile)
  {
    pos = 0;
    sd_i = 0;
    
    m0 = SDindex2float(cparmsfile,0,0,4,pos,sd_i);
    m1 = SDindex2float(cparmsfile,0,1,4,pos,sd_i);
    v0 = SDindex2float(cparmsfile,0,2,4,pos,sd_i);
    v1 = SDindex2float(cparmsfile,0,3,4,pos,sd_i);
  }
  else
  {
    Serial.println("No cparms file found!");
    return;
  }
  cparmsfile.close();

  //Collect EEG data and classify serially at 250 ms intervals
  unsigned long tStart, tStop;
  int nC_i, nF_j, b_j, z_ij, inds[nD], it, win_ind = 0, Ypast_ind = 0, pp1past_ind = 0, cue_inst;
  double X[nC], y_ij, Z[4*nD], Y[nD], Ypast[nD*nAnaDur], Ytot[nD], ThH, ThL, pt, lh0, lh1, pp1, pp1past[nPosDur], pp1tot, pps[nBSM];
  byte cue[nBSM], state[nBSM];
  bool new_cue;
        
  //zero Z, Y, Yo
  for (i = 0; i < 4*nD; i++)
  {
    Z[i] = 0;
  }
  for (i = 0; i < nD; i++)
  {
    Y[i] = 0.0;
    Ytot[i] = 0.0;
    for (j = 0; j < nAnaDur; j++)
    {
      Ypast[i*nAnaDur + j] = 0.0;
    }
  }
  for (i = 0; i < nPosDur; i++)
  {
    pp1past[i] = 0.0;
  }

  i = 0;
  inds[0] = i;
  for (j = 1; j < nD; j++)
  {
    inds[j] = i += nits_online;
  }

  // Collect data but don't save for nPosDur + nAnaDur - 1
  // The resulting posterior probabilities for these time points are worthless
  cue_inst = 0;
  new_cue = false;
  drawidle();
  tStart = micros();
  for (it = 0; it < (nPosDur + nAnaDur - 1)*nits_online; it++)
  {
    tStop = micros();
    delayMicroseconds(tSampling + tStart - tStop);
    tStart = micros();
      
    for (nC_i = 0; nC_i < nC; nC_i++)
    {
      X[nC_i] = (VoltScale * (double)analogRead(analogChans[nC_i])) - X_mean[nC_i];
      X[nC_i] /= X_std[nC_i];
    }
    for (nC_i = 0; nC_i < nC; nC_i++)
    {
      for (nF_j = 0; nF_j < nF; nF_j++)
      {
        b_j = nF_j*10;
        z_ij = nC_i*8 + nF_j*4; // 0, 4 ch 0 ,   8, 12 ch 1,    16, 20 ch 2
        y_ij = filterCoeffs[b_j] * X[nC_i] + Z[z_ij];
        Z[z_ij]  = filterCoeffs[b_j+1] * X[nC_i] + Z[z_ij+1]  - filterCoeffs[b_j+6] * y_ij;
        Z[z_ij+1]  = filterCoeffs[b_j+2] * X[nC_i] + Z[z_ij+2]  - filterCoeffs[b_j+7] * y_ij;
        Z[z_ij+2]  = filterCoeffs[b_j+3] * X[nC_i] + Z[z_ij+3]  - filterCoeffs[b_j+8] * y_ij;
        Z[z_ij+3]  = filterCoeffs[b_j+4] * X[nC_i]              - filterCoeffs[b_j+9] * y_ij;
        y_ij *= y_ij;
        y_ij /= nits_online;
        z_ij = nC_i*2 + nF_j;
        Y[z_ij] += y_ij;
        inds[z_ij]++;
      }
    }
    
    if (inds[0] == nits_online)
    {
      for (i = 0; i < nD; i++)
      {
        Y[i] /= nAnaDur;
        Ytot[i] = Ytot[i] - Ypast[i*nAnaDur + Ypast_ind] + Y[i];
        Ypast[i*nAnaDur + Ypast_ind] = Y[i];
        Y[i] = 0.0;
      }
      if (Ypast_ind < nAnaDur-1)
      {
        Ypast_ind++;
      }
      else
      {
        Ypast_ind = 0;
      }
      
      pt = Ytot[0] * w[0];
      for (i = 1; i < nD; i++)
      {
        pt += Ytot[i] * w[i];
      }
      
      //Predict state using naive bayes with gaussian likelihood and equal priors
      lh0 = exp(-pow(pt - m0,2)/(2*v0)) / sqrt(v0);
      lh1 = exp(-pow(pt - m1,2)/(2*v1)) / sqrt(v1);
      pp1 = lh1 / (lh0 + lh1);
      pp1 /= nPosDur;
      pp1tot = pp1tot - pp1past[pp1past_ind] + pp1;
      pp1past[pp1past_ind] = pp1;
      if (pp1past_ind < nPosDur-1)
      {
        pp1past_ind++;
      }
      else
      {
        pp1past_ind = 0;
      }
      
      i = 0;
      inds[0] = i;
      for (j = 1; j < nD; j++)
      {
        inds[j] = i += nits_online;
      }
      
    }
    //tStop = micros();
    //delayMicroseconds(tSampling + tStart - tStop);
  }
  
  // Collect data for a duration of tBSM to use for setting binary state machine thresholds
  while (win_ind < nBSM)
  {
    tStop = micros();
    delayMicroseconds(tSampling + tStart - tStop);
    tStart = micros();
    
    if ((win_ind % nCueDur) == 0 && new_cue)
    {
      new_cue = false;
      if (cue_inst!=0)
      {
        drawidle();
        cue_inst = 0;
      }
      else if (cue_inst!=1)
      {
        drawmove();
        cue_inst = 1;
      }
    }
      
    for (nC_i = 0; nC_i < nC; nC_i++)
    {
      X[nC_i] = (VoltScale * (double)analogRead(analogChans[nC_i])) - X_mean[nC_i];
      X[nC_i] /= X_std[nC_i];
    }
    for (nC_i = 0; nC_i < nC; nC_i++)
    {
      for (nF_j = 0; nF_j < nF; nF_j++)
      {
        b_j = nF_j*10;
        z_ij = nC_i*8 + nF_j*4; // 0, 4 ch 0 ,   8, 12 ch 1,    16, 20 ch 2
        y_ij = filterCoeffs[b_j] * X[nC_i] + Z[z_ij];
        Z[z_ij]  = filterCoeffs[b_j+1] * X[nC_i] + Z[z_ij+1]  - filterCoeffs[b_j+6] * y_ij;
        Z[z_ij+1]  = filterCoeffs[b_j+2] * X[nC_i] + Z[z_ij+2]  - filterCoeffs[b_j+7] * y_ij;
        Z[z_ij+2]  = filterCoeffs[b_j+3] * X[nC_i] + Z[z_ij+3]  - filterCoeffs[b_j+8] * y_ij;
        Z[z_ij+3]  = filterCoeffs[b_j+4] * X[nC_i]              - filterCoeffs[b_j+9] * y_ij;
        y_ij *= y_ij;
        y_ij /= nits_online;
        z_ij = nC_i*2 + nF_j;
        Y[z_ij] += y_ij;
        inds[z_ij]++;
      }
    }
    
    if (inds[0] == nits_online)
    {
      for (i = 0; i < nD; i++)
      {
        Y[i] /= nAnaDur;
        Ytot[i] = Ytot[i] - Ypast[i*nAnaDur + Ypast_ind] + Y[i];
        Ypast[i*nAnaDur + Ypast_ind] = Y[i];
        Y[i] = 0.0;
      }
      if (Ypast_ind < nAnaDur-1)
      {
        Ypast_ind++;
      }
      else
      {
        Ypast_ind = 0;
      }
      
      pt = Ytot[0] * w[0];
      for (i = 1; i < nD; i++)
      {
        pt += Ytot[i] * w[i];
      }
      
      //Predict state using naive bayes with gaussian likelihood and equal priors
      lh0 = exp(-pow(pt - m0,2)/(2*v0)) / sqrt(v0);
      lh1 = exp(-pow(pt - m1,2)/(2*v1)) / sqrt(v1);
      pp1 = lh1 / (lh0 + lh1);
      pp1 /= nPosDur;
      pp1tot = pp1tot - pp1past[pp1past_ind] + pp1;
      pp1past[pp1past_ind] = pp1;
      if (pp1past_ind < nPosDur-1)
      {
        pp1past_ind++;
      }
      else
      {
        pp1past_ind = 0;
      }
      
      pps[win_ind] = pp1tot;
      cue[win_ind] = cue_inst;
      win_ind++;
      
      i = 0;
      inds[0] = i;
      for (j = 1; j < nD; j++)
      {
        inds[j] = i += nits_online;
      }
      
      new_cue = true;
    }
  }

  clearscreen();

  //create new bsmthres file and keep it open - only 1 write here
  if (!SD.exists("/DATA/S" + sSJ + "P.txt"))
  {
    File ppsfile = SD.open("/DATA/S" + sSJ + "P.txt", O_CREAT | O_WRITE);
    for (k=0; k<nBSM; k++)
    {
      String pps_Str = "";
      pps_Str = String(cue[k],20);
      pps_Str += ",";
      pps_Str += String(pps[k],20);
      ppsfile.println(pps_Str);
    }
    ppsfile.close();
  }
  
  // automatically guess best binary state thresholds by maximizing the classification accuracy of the collected tBSM data
  // But as this is usually wrong, we still do manual threshold selection afterwards
  int nsteps = 100, prev_state, err, min_err = nBSM+1;
  double ThH_b = -1, ThL_b = -1;
  for (i=nsteps-1; i>=2; i--)
  {
    for (j=1; j<i; j++)
    {
      ThH = (double)i / double(nsteps);
      ThL = (double)j / double(nsteps);
      err = 0;
      prev_state = 0;
      for (k=0; k<nBSM; k++)
      {
        if (pps[k] > ThH)
        {
          prev_state = 1;
          if (prev_state!=cue[k])
          {
            err++;
          }
        }
        else if (pps[k] < ThL)
        {
          prev_state = 0;
          if (prev_state!=cue[k])
          {
            err++;
          }
        }
        else
        {
          if (prev_state!=cue[k])
          {
            err++;
          }
        }
      }
      if (err < min_err)
      {
        ThH_b = ThH;
        ThL_b = ThL;
        min_err = err;
      }
    }
  }
    
  //create new bsmthres file and keep it open - only 1 write here
  if (!SD.exists("/DATA/S" + sSJ + "B.txt"))
  {
    File bsmthresfile = SD.open("/DATA/S" + sSJ + "B.txt", O_CREAT | O_WRITE);
    String thr_Str = "";
    thr_Str = String(ThH_b,20);
    thr_Str += ",";
    thr_Str += String(ThL_b,20);
    bsmthresfile.println(thr_Str);
    bsmthresfile.close();
  }
}


//Run an Online Session
void online_cue() {
  tft.setTextSize(4);
  unsigned long pos;
  int i, j, sd_i;
  double X_mean[nC], X_std[nC], w[nD], m0, m1, v0, v1, ThH, ThL;

  digitalWrite(FES_PIN, LOW);
  
  nSS++;
  sSS = int2str(nSS, 3);
  
  clearscreen();
  
  //get zscore paramters
  File zsfile = SD.open("/DATA/S" + sSJ + "Z.txt", O_READ);
  if (zsfile)
  {
    pos = 0;
    sd_i = 0;
    for (i = 0; i < nC; i++)
    {
      X_mean[i] = SDindex2float(zsfile,0,i,nC,pos,sd_i);
    }
    for (i = 0; i < nC; i++)
    {
      X_std[i] = SDindex2float(zsfile,1,i,nC,pos,sd_i);
    }
  }
  else
  {
    Serial.println("No zscore parms file found!");
    return;
  }
  zsfile.close();
  
  //get dimensionality reduction (weighting) parameters
  File coeffsfile = SD.open("/DATA/S" + sSJ + "W.txt", O_READ);
  if (coeffsfile)
  {  
    pos = 0;
    sd_i = 0;
    for (i = 0; i < nD; i++)
    {
      w[i] = SDindex2float(coeffsfile,0,i,nD,pos,sd_i);
    }
  }
  else
  {
    Serial.println("No coeffs file found!");
    return;
  }
  coeffsfile.close();
  
  //get gaussian fit parameters
  File cparmsfile = SD.open("/DATA/S" + sSJ + "G.txt", O_READ);
  if (cparmsfile)
  {
    pos = 0;
    sd_i = 0;
    
    m0 = SDindex2float(cparmsfile,0,0,4,pos,sd_i);
    m1 = SDindex2float(cparmsfile,0,1,4,pos,sd_i);
    v0 = SDindex2float(cparmsfile,0,2,4,pos,sd_i);
    v1 = SDindex2float(cparmsfile,0,3,4,pos,sd_i);
  }
  else
  {
    Serial.println("No cparms file found!");
    return;
  }
  cparmsfile.close();
  
  //get binary state thresholds
  File bsmthresfile = SD.open("/DATA/S" + sSJ + "B.txt", O_READ);
  if (bsmthresfile)
  {
    pos = 0;
    sd_i = 0;
    
    ThH = SDindex2float(bsmthresfile,0,0,2,pos,sd_i);
    ThL = SDindex2float(bsmthresfile,0,1,2,pos,sd_i);
    Serial.print("High Threshold: "); Serial.println(ThH);
    Serial.print("Low Threshold: "); Serial.println(ThL);
  }
  else
  {
    Serial.println("No bsm thres file found!");
    return;
  }
  bsmthresfile.close();

  //Collect EEG data and classify serially at 250 ms intervals or tPP
  unsigned long tStart, tStop;
  int inds[nD], it, nC_i, nF_j, b_j, z_ij, Ypast_ind = 0, pp1past_ind = 0, win_ind = 0, cue_inst, state = 0;
  double X[nC], y_ij, Z[4*nD], Y[nD], Ypast[nD*nAnaDur], Ytot[nD], pt, lh0, lh1, pp1, pp1tot = 0, pp1past[nPosDur];
  
  //zero Z, Y, Ypast, Ytot, pp1past
  for (i = 0; i < 4*nD; i++)
  {
    Z[i] = 0.0;
  }
  for (i = 0; i < nD; i++)
  {
    Y[i] = 0.0;
    Ytot[i] = 0.0;
    for (j = 0; j < nAnaDur; j++)
    {
      Ypast[i*nAnaDur + j] = 0.0;
    }
  }
  for (i = 0; i < nPosDur; i++)
  {
    pp1past[i] = 0.0;
  }

  i = 0;
  inds[0] = i;
  for (j = 1; j < nD; j++)
  {
    inds[j] = i += nits_online;
  }

  int cue_it = 0, cue = 0, save_i = 0, CUES[online_pp_its], STATES[online_pp_its];
  bool new_cue = false;
  
  // Collect data for first nPosDur + nAnaDur - 1 seconds
  // As above, posterior probabilities for this time segment are worthless because of unstable smoothing windows
  drawidle();
  tStart = micros();
  for (it = 0; it < (nPosDur + nAnaDur - 1)*nits_online; it++)
  {
    tStop = micros();
    delayMicroseconds(tSampling + tStart - tStop);
    tStart = micros();
      
    for (nC_i = 0; nC_i < nC; nC_i++)
    {
      X[nC_i] = (VoltScale * (double)analogRead(analogChans[nC_i])) - X_mean[nC_i];
      X[nC_i] /= X_std[nC_i];
    }
    for (nC_i = 0; nC_i < nC; nC_i++)
    {
      for (nF_j = 0; nF_j < nF; nF_j++)
      {
        b_j = nF_j*10;
        z_ij = nC_i*8 + nF_j*4; // 0, 4 ch 0 ,   8, 12 ch 1,    16, 20 ch 2
        y_ij = filterCoeffs[b_j] * X[nC_i] + Z[z_ij];
        Z[z_ij]  = filterCoeffs[b_j+1] * X[nC_i] + Z[z_ij+1]  - filterCoeffs[b_j+6] * y_ij;
        Z[z_ij+1]  = filterCoeffs[b_j+2] * X[nC_i] + Z[z_ij+2]  - filterCoeffs[b_j+7] * y_ij;
        Z[z_ij+2]  = filterCoeffs[b_j+3] * X[nC_i] + Z[z_ij+3]  - filterCoeffs[b_j+8] * y_ij;
        Z[z_ij+3]  = filterCoeffs[b_j+4] * X[nC_i]              - filterCoeffs[b_j+9] * y_ij;
        y_ij *= y_ij;
        y_ij /= nits_online;
        z_ij = nC_i*2 + nF_j;
        Y[z_ij] += y_ij;
        inds[z_ij]++;
      }
    }
    
    if (inds[0] == nits_online)
    {
      for (i = 0; i < nD; i++)
      {
        Y[i] /= nAnaDur;
        Ytot[i] = Ytot[i] - Ypast[i*nAnaDur + Ypast_ind] + Y[i];
        Ypast[i*nAnaDur + Ypast_ind] = Y[i];
        Y[i] = 0.0;
      }
      if (Ypast_ind < nAnaDur-1)
      {
        Ypast_ind++;
      }
      else
      {
        Ypast_ind = 0;
      }
      
      pt = Ytot[0] * w[0];
      for (i = 1; i < nD; i++)
      {
        pt += Ytot[i] * w[i];
      }
      
      //Predict state using naive bayes with gaussian likelihood and equal priors
      lh0 = exp(-pow(pt - m0,2)/(2*v0)) / sqrt(v0);
      lh1 = exp(-pow(pt - m1,2)/(2*v1)) / sqrt(v1);
      pp1 = lh1 / (lh0 + lh1);
      pp1 /= nPosDur;
      pp1tot = pp1tot - pp1past[pp1past_ind] + pp1;
      pp1past[pp1past_ind] = pp1;
      if (pp1past_ind < nPosDur-1)
      {
        pp1past_ind++;
      }
      else
      {
        pp1past_ind = 0;
      }
      
      i = 0;
      inds[0] = i;
      for (j = 1; j < nD; j++)
      {
        inds[j] = i += nits_online;
      }
      
    }
    //tStop = micros();
    //delayMicroseconds(tSampling + tStart - tStop);
  }

  // Alternate cues and classify the rest of the data throughout the online session
  for (it = 0; it < online_run_its; it++)
  {
    tStop = micros();
    delayMicroseconds(tSampling + tStart - tStop);
    tStart = micros();
    
    if ((cue_it % nCueDur) == 0 && new_cue)
    {
      new_cue = false;
      cue_it = 0;
      if (cue!=0)
      {
        drawidle();
        cue = 0;
      }
      else
      {
        drawmove();
        cue = 1;
      }
    }
      
    for (nC_i = 0; nC_i < nC; nC_i++)
    {
      X[nC_i] = (VoltScale * (double)analogRead(analogChans[nC_i])) - X_mean[nC_i];
      X[nC_i] /= X_std[nC_i];
    }
    for (nC_i = 0; nC_i < nC; nC_i++)
    {
      for (nF_j = 0; nF_j < nF; nF_j++)
      {
        b_j = nF_j*10;
        z_ij = nC_i*8 + nF_j*4; // 0, 4 ch 0 ,   8, 12 ch 1,    16, 20 ch 2
        y_ij = filterCoeffs[b_j] * X[nC_i] + Z[z_ij];
        Z[z_ij]  = filterCoeffs[b_j+1] * X[nC_i] + Z[z_ij+1]  - filterCoeffs[b_j+6] * y_ij;
        Z[z_ij+1]  = filterCoeffs[b_j+2] * X[nC_i] + Z[z_ij+2]  - filterCoeffs[b_j+7] * y_ij;
        Z[z_ij+2]  = filterCoeffs[b_j+3] * X[nC_i] + Z[z_ij+3]  - filterCoeffs[b_j+8] * y_ij;
        Z[z_ij+3]  = filterCoeffs[b_j+4] * X[nC_i]              - filterCoeffs[b_j+9] * y_ij;
        y_ij *= y_ij;
        y_ij /= nits_online;
        z_ij = nC_i*2 + nF_j;
        Y[z_ij] += y_ij;
        inds[z_ij]++;
      }
    }
    
    if (inds[0] == nits_online)
    {
      for (i = 0; i < nD; i++)
      {
        Y[i] /= nAnaDur;
        Ytot[i] = Ytot[i] - Ypast[i*nAnaDur + Ypast_ind] + Y[i];
        Ypast[i*nAnaDur + Ypast_ind] = Y[i];
        Y[i] = 0.0;
      }
      if (Ypast_ind < nAnaDur-1)
      {
        Ypast_ind++;
      }
      else
      {
        Ypast_ind = 0;
      }
      
      pt = Ytot[0] * w[0];
      for (i = 1; i < nD; i++)
      {
        pt += Ytot[i] * w[i];
      }
      
      //Predict state using naive bayes with gaussian likelihood and equal priors
      lh0 = exp(-pow(pt - m0,2)/(2*v0)) / sqrt(v0);
      lh1 = exp(-pow(pt - m1,2)/(2*v1)) / sqrt(v1);
      pp1 = lh1 / (lh0 + lh1);
      pp1 /= nPosDur;
      pp1tot = pp1tot - pp1past[pp1past_ind] + pp1;
      pp1past[pp1past_ind] = pp1;
      if (pp1past_ind < nPosDur-1)
      {
        pp1past_ind++;
      }
      else
      {
        pp1past_ind = 0;
      }
      if (pp1tot > ThH)
      {
        digitalWrite(FES_PIN, HIGH);
        state = 1;
      }
      else if (pp1tot < ThL)
      {
        digitalWrite(FES_PIN, LOW);
        state = 0;
      }
      //else state just remains its current value
      
      CUES[save_i] = cue;
      STATES[save_i] = state;
      save_i++;
      
      i = 0;
      inds[0] = i;
      for (j = 1; j < nD; j++)
      {
        inds[j] = i += nits_online;
      }
      
      cue_it++;
      new_cue = true;
    }
    //tStop = micros();
    //delayMicroseconds(tSampling + tStart - tStop);
  }
  tStop = micros();
  
  digitalWrite(FES_PIN, LOW);
  clearscreen();
  
  //create new session data file and keep it open - only 1 write here
  //storage is in format - cues,decoded state
  File sessfile = SD.open("/DATA/S" + sSJ + "S" + sSS + ".txt", O_CREAT | O_WRITE);
  String sess_Str = "";
  for (i = 0; i < online_pp_its; i++)
  {
    sess_Str = String(CUES[i]) + "," + String(STATES[i]);
    sessfile.println(sess_Str);
  }
  sessfile.close();
}

