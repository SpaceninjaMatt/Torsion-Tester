#include "HX711.h"  //Amplifier Library
#include <Wire.h> // Library for I2C communication
#include <LiquidCrystal_I2C.h> // Library for LCD

#define SDK_pinT A3
#define SDK_pinB A0
#define LC_Top_pin A2
#define LC_Bottom_pin A1
#define SDA_pin A4
#define SLC_pin A5
#define Step_Signal_pin 9
#define Step_Dir_pin 8
#define Step_Dis_pin 7
#define ESTOP_pin 5
#define Opt_Rot_pin 2
#define Opt_Home_pin 3
#define Door_pin 4
//global varibles + constants
const int stepsPerRevolution = 8000; //200 normally, but half step and 1:20 ratio 
long freq = 600;
long LCT_OFFSET = -472528L; //change every tare, but a decent start
long LCB_OFFSET = 333248L;
double tMoment;
double bMoment;
double last_tM = 0;
double last_bM = 0;
double last_M =0;
double last_diff_T = 0;
double last_diff_B = 0;
int LC_ave_N = 25;
double angle;
double moment;
double maxMoment = 0;
unsigned long steps;
bool broken = false;
bool sample_Loaded = false;
bool LCError;
bool MATLAB = false;
String command;
const int pinOffset = 150; //calibrate this
int dpm = 50; //degree per minute
int delayT = 2700/dpm;

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2); //Screen Object
HX711 scaleT; 
HX711 scaleB;

//stupid error correction
const int ave_len = 5;
double top_Mom[ave_len];
double bot_Mom[ave_len];
int ar_Pos = 0;

void step(){
  digitalWrite(Step_Signal_pin, HIGH);
	delayMicroseconds(int(1000000L/(freq*2)));
	digitalWrite(Step_Signal_pin, LOW);
	delayMicroseconds(int(1000000L/(freq*2)));
  steps++;
}

double bestFit(){
  double best;
  /*
  double t_ave = 0;
  double b_ave = 0;
  for(int i = 0; i<5;i++){
    t_ave += top_Mom[i];
    b_ave += bot_Mom[i];    
  }
  t_ave /= ave_len;
  b_ave /= ave_len;
 
  if(abs((last_tM-t_ave)/t_ave) > abs((last_bM-b_ave)/b_ave)){
    best = bMoment;
  }
  else{
    best = tMoment;
  }
  
  */
  if(abs(last_M - last_tM) > abs(last_M-last_bM)){
    best = last_bM;
  }
  else{
    best = last_tM;    
  }
  
  //Serial.println(String(best)+"T_ave:"+String(t_ave)+"T_read:"+String(tMoment)+"B_ave:"+String(b_ave)+"B_read:"+String(bMoment));
  return best;
  
}

void home(){
  lcd.clear();
  lcd.setCursor(1,0);
  lcd.print("Starting Home");
  lcd.setCursor(2,1);
  lcd.print("Stay Clear!");
  delay(2000);
  freq = 4000; //homes quicker
  while(digitalRead(Opt_Home_pin) == LOW){
    if(digitalRead(Door_pin) == LOW && digitalRead(ESTOP_pin) == LOW){
      step();
    }
    else if(digitalRead(Door_pin) == HIGH){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("CLOSE DOOR");
      while(digitalRead(Door_pin) == HIGH){
        //do nothing idk
      }
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Homing...");
    }
    else if(digitalRead(ESTOP_pin) == HIGH){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("ESTOP PRESSED");
      while(digitalRead(ESTOP_pin) == HIGH){}
      delay(2000);
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Homing...");
    }
  }
  for(int i = 0; i < pinOffset; i++){
    step();
  }
  freq = 600;
  angle = 0;
  steps = 0;
}

double getMoment(){
  
  double moment;
  double first = (scaleT.read()-LCT_OFFSET);
  tMoment = first/79605L; //using values from calibration
  /*
  if(last_tM != 0){
    if(abs((tMoment-last_tM)/last_tM)>1 || tMoment < -.1){
      tMoment = last_tM + last_diff_T;    
    }
    else{
      last_diff_T = tMoment - last_tM;
    }
  }
  */
  if(tMoment > 6 || tMoment < -1){//spikes
    tMoment = last_tM;
  }
  last_tM = tMoment;
  
  double second = (scaleB.read()-LCB_OFFSET);
  bMoment = second/84985L;
  /*
  if(last_bM != 0){ 
    if(abs((bMoment-last_bM)/last_bM)>2 || bMoment < -.1){
      bMoment = last_bM + last_diff_B;    
    }
    else{
      last_diff_B = bMoment - last_bM;
    }
  }
  last_bM = bMoment;
  */
  if(bMoment > 6 || bMoment < -1){
    bMoment = last_bM;
  }
  last_bM = bMoment;
    
  if(abs((bMoment - tMoment)/(tMoment+0.1)) < .5){
    moment = (tMoment+bMoment)/2;
    LCError = false;
    top_Mom[ar_Pos%ave_len]=tMoment;
    bot_Mom[ar_Pos%ave_len]=bMoment;
    ar_Pos++;

  }
  else{
    
    if (!LCError){
      Serial.println("LC");
      LCError = true;
    }
    moment = (tMoment+bMoment)/2;
    //Serial.println(String(moment,3)+"T_mom:"+String(tMoment,3)+"B_mom:"+String(bMoment,3));
    //moment = tMoment;      
  }
  last_M = moment;
  return moment;
}



void setup() {
  Serial.begin(115200); //to set the communication speed for the Serial Monitor at that speed
  pinMode(Step_Signal_pin, OUTPUT);
  pinMode(Step_Dir_pin, OUTPUT);
  pinMode(Step_Dis_pin, OUTPUT);
  digitalWrite(Step_Dir_pin, HIGH);
  digitalWrite(Step_Dis_pin, LOW);  
  lcd.init();
  lcd.backlight();
  scaleT.begin(LC_Top_pin, SDK_pinT); //this tells data DOUT and clock SCK to the code
  scaleB.begin(LC_Bottom_pin, SDK_pinB); //this tells data DOUT and clock SCK to the code
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Reboot Detected");
  delay(500);
  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Connect Via");
  lcd.setCursor(5,1);
  lcd.print("MATLAB");
  while(!MATLAB){
    String input = Serial.readString();
    //Serial.println(input);
    input.trim();
    if(input.equalsIgnoreCase("MATLAB")){
      MATLAB = true;
    }
  }
  lcd.clear();
  lcd.setCursor(3,0);
  lcd.print("Connected");
  delay(2000);
  lcd.clear(); 
  lcd.setCursor(2,0);
  lcd.print("Close Door To");
  lcd.setCursor(6,1); 
  lcd.print("Home");
  while(digitalRead(Door_pin) == HIGH){}
  home();
}

void loop() {
  scaleT.read();
  scaleB.read();
  while(digitalRead(ESTOP_pin) == HIGH){
    lcd.clear(); 
    lcd.setCursor(0,0);
    lcd.print("ESTOP PRESSED");
    delay(500);
  }

  if(broken == false && angle == 0 && !sample_Loaded){
    lcd.clear();
    lcd.setCursor(2,0);
    lcd.print("Load Sample");
    lcd.setCursor(0,1);
    lcd.print("Then Close Door");
    while(digitalRead(Door_pin) == LOW){}
    delay(2000);
    while(digitalRead(Door_pin) == HIGH){}
    delay(1000);
    sample_Loaded = true;
    Serial.println("ready");

  }else if(broken == true && angle > 0 && sample_Loaded){
    lcd.clear();
    lcd.setCursor(3,0);
    lcd.print("TEST ENDED");
    lcd.setCursor(2,0);
    lcd.print("Cleaning Up");
    for(int i = 0; i < 1000; i ++){ //turns 45 degrees to finish any sample off
      if(digitalRead(Door_pin) == HIGH){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("CLOSE DOOR");
      while(digitalRead(Door_pin) == HIGH){}
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("cleaning...");
      }
      step();
    }
    lcd.clear();
    lcd.setCursor(2,0);
    lcd.print("Clear Sample");
    digitalWrite(Step_Dis_pin, HIGH);
    while(digitalRead(Door_pin) == LOW){}
    delay(3000);
    while(digitalRead(Door_pin) == HIGH){}
    delay(2000);
    sample_Loaded = false;
    digitalWrite(Step_Dis_pin, LOW);
  }
  lcd.clear(); 
  lcd.setCursor(0,0);
  lcd.print("Send Command Via");
  lcd.setCursor(5,1);
  lcd.print("MATLAB");
  command = Serial.readString();
  command.trim();
  //Serial.println(command);
  if(command.equalsIgnoreCase("home")){
    home();
  }
  else if(command.equalsIgnoreCase("start")){
    //Serial.println("test");
    if(angle != 0){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Homing Needed");
      delay(1000);
      home();
    }
    broken = false;
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print("Zeroing Cells");
    //Sometimes the average returns a massive number, this ensures it will be withing 5% of the base calibration
    long LCT_OFFSET_temp= scaleT.read_average(LC_ave_N);
    while(abs((LCT_OFFSET_temp-LCT_OFFSET)/LCT_OFFSET) > .05){
      LCT_OFFSET_temp= scaleT.read_average(LC_ave_N);
      //Serial.println("Top fail");
    }
    LCT_OFFSET = LCT_OFFSET_temp; 

    long LCB_OFFSET_temp = scaleB.read_average(LC_ave_N);
    while(abs((LCB_OFFSET_temp-LCB_OFFSET)/LCB_OFFSET) > .05){
      LCB_OFFSET_temp= scaleB.read_average(LC_ave_N);
      //Serial.println("Bottom fail");
    }
    LCB_OFFSET = LCB_OFFSET_temp; 
    
    LCError = false;
    lcd.clear();
    lcd.setCursor(1,0);
    lcd.print("Starting Test"); //out of the loop for speed sake(might have minimal effect)
    lcd.setCursor(2,1);
    lcd.print("Stay Clear!");
    delay(1000);
    long test_time = millis();
    while(!broken)
    {
      if(digitalRead(Door_pin) == LOW && digitalRead(ESTOP_pin) == LOW){
        
        delay(delayT); //20 deg/min //540 for 5deg/min
        step();
        if(steps % 5 == 0){ //steps per reading. * by 0.045 to get degree per sample
          //long x = millis();
          moment = getMoment();
          if(moment>maxMoment){
            maxMoment = moment;
          }
          Serial.println(String(angle,3) + ":"+ String(moment,5)); //IMPORTANT 
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Angle: " + String(angle,2)); //out of the loop for speed sake(might have minimal effect)
          lcd.setCursor(0,1);
          lcd.print("Torque: "+ String(moment,2));
          //Serial.println("Delay"+ String(millis()-x));
          
        }
        angle +=(.045);

        if(angle > 20 && (abs((moment-maxMoment)/maxMoment)>.3)){//tune the 0.1
          broken = true;
          Serial.println("B");
        }
        if(moment > 5){
          Serial.println("EC");
          broken = true;
          digitalWrite(Step_Dis_pin, HIGH);
          lcd.clear();
          lcd.setCursor(5,0);
          lcd.print("ERROR!");
          lcd.setCursor(1,1);
          lcd.print("TORQUE > 5 Nm");
          delay(5000);

        }
      }
      else if(digitalRead(Door_pin) == HIGH){
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("CLOSE DOOR");
        while(digitalRead(Door_pin) == HIGH){}
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Testing...");
      }
      else if(digitalRead(ESTOP_pin) == HIGH){
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("ESTOP PRESSED");
        lcd.setCursor(0,1);
        lcd.print("TEST ENDED");
        delay(5000);
        broken = true;
      }    
    }
    double run_time = (millis()-test_time);
    double run_time_min = run_time/60000; 
    double rate = angle / run_time_min;
    lcd.clear();
    lcd.setCursor(3,0);
    lcd.print("Test Rate");
    lcd.setCursor(0,1);
    lcd.print(String(rate,3)+ " Deg/min");
    delay(5000);

  }
}
