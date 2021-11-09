//including libraries needed for functions:
#include <Braccio.h>
#include <Servo.h>

//activates servos and their variables, safety positions:
Servo base;      //float m1; //M1 - 90 degrees  (0-180)
Servo shoulder;  //float m2; //M2 - 45 degrees  (15-165)
Servo elbow;     //float m3; //M3 - 180 degrees (0-180)
Servo wrist_ver; //float m4; //M4 - 180 degrees (0-180)
Servo wrist_rot; //float m5; //M5 - 90 degrees  (0-180)
Servo gripper;   //float m6; //M6 - 10 degrees  (10-73)
float m1, m2, m3, m4, m5, m6; //defining angle variables for servos

int sb1=map(90, -2, 165, 0, 180); //reversed base due to plate angles reversed
int sb2=map(90, 16, 155, 15, 165);
int sb3=map(90, 7, 179, 0, 180);
int sb4=map(90, -4, 177, 0, 180);

//defining physical parameters:
float ltot=45.0;     //total length of robot arm is 45cm
float ol=12.5;       //total length of forearm (overarm)
float ul=12.5;       //total length of underarm
float hl=20.0;       //total length of wrist (handledd)
float minC1=33.5; //minimum length arm can reach with overarm straight (33,5cm)
float minAbsXh=10.0; //absolute minimum length it can grab something from
float minAbsY=35.0;  //absolute minimum heigth it can grab something within the min length
float arm=17.678;    //length of inner arm when within 33.5cm (sqrt(2)*12,5cm)

//defining variables needed:
int input=13;
bool part;
int flag;  //defines which calculation is appropriate
char in;

//calculation variables:
float h;  //horizontal hypotenus
float ht; //vertical/total hypotenus
float outer;//length of outer hypotenus
float a, b, c; //for M2<15 degree math

//pre-defines XYZ for testing:
float X=20.0;
float Y=40.0;
float Z=25.0;
/* ________________________________________________________________________________________________________
Main program:*/

void setup() {  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("initializing...");
  pinMode(12, OUTPUT);  // you need to set HIGH the pin 12
  pinMode(input, INPUT);
  digitalWrite(12, HIGH);
  Braccio.begin();  //SOFT_START_DISABLED
  Serial.println("ready");
}

//--------------------------------------------------------------
void loop() { 
//put your main code here, to run repeatedly:
    
  /*part=digitalRead(input);
   if (part==true){*/
  in=Serial.peek();
  if (isdigit(in)||in=='-')  {
    ReadInput(); //checks if serial contains a number, if so, reads.
  }
  else{
    Serial.read();//if not, dumps the buffer
  }
  if (part) {
    part = false;
    Serial.println("calculating...");
    //Serial.read(XYZ) //or parse etc.
    prelim();   //checks if item is within reach or not, sets the Flag var.
    if (flag==1){
      IK();     //calculate
    place();//picks and places part after being picked up, returns to start
    Serial.println("job done, returning to start.");
    }
    else{
      if (flag==2){
        inIK(); //does inner IK math
        place();//picks and places part after being picked up, returns to start
        Serial.println("job done, returning to start.");
        Serial.println();
      }
    }
    if (Z<0){ //checks if servos needs reversal
      //rev();
    }
  }
  //reset to standby pos:
  Braccio.ServoMovement(200,         sb1, sb2, sb3, sb4, 90, 40); //180, 90, 90, 90
 delay(1000);//doesnt need to run as fast as possible...
}

/* ________________________________________________________________________________________________________
___________________________________________________________________________________________________________
___________________________________________________________________________________________________________
nested functions:*/

void ReadInput() {
    delay(100);//allows the buffer to fill up first
      X=Serial.parseFloat();//reads XYZ, each seperated by anything excluding numbers and points, and '-'. (not comma)
      Y=Serial.parseFloat();
      Z=Serial.parseFloat();
      Serial.read();//apparently it got stuck on some remaining stuff in the serial bus
      Serial.print("X=");Serial.print(X);Serial.print(", Y=");Serial.print(Y);Serial.print(", Z=");Serial.println(Z);
      if (Z<0){Serial.println("-Z not coded!");}//sry...
      else {part=true;}
    delay(100);
}


//--------------------------------------------------------------

void prelim() {
  Serial.println("prelim() running!");
  if((X==0)&&(Z==0)){
    h=0;
  }
  else{
    h=hypot(X, Z); 
  }
  ht=hypot(h,Y); 
  outer=(ht-ol);//sets the value of the outer side
  if (h<minAbsXh)  {
    if(Y<minAbsY) {
      flag=0;
      Serial.println("too close!");
      }
    }
  else{
    if(ht>ltot){
      flag=0;
    Serial.println("too far!");
    }
  else{
    Serial.print(ht); Serial.print(" is within range...");
    if (ht<33.5){
      flag=2;
      Serial.println("but within inner dome, doing inner-IK.");
    }
    else{
      flag=1;
      Serial.println("and outside inner dome, doing normal IK.");
      }
    }
  }
  Serial.print("h = ");Serial.print(h);Serial.print(", ht = ");Serial.print(ht);Serial.print(", flag = ");Serial.print(flag); Serial.println(".");
}
//--------------------------------------------------------------

void IK() {
  Serial.println("IK() running!");
  //base:
  if (X>=0) {
    m1=180-degrees(asin(Z/h));
  }
  else  {
    m1=degrees(asin(Z/h));
  }
  //shoulder:
  a=degrees(asin(Y/ht));
  if (a>=15) {
    m2=a;
    //elbow:
    m3=degrees(acos((sq(hypot(outer,ul)))-sq(hl))/(2*outer*ul))+90;
    //wrist:
    m4=degrees(acos(((ul*ul)+(hl*hl)-sq(outer))/(2*ul*hl)))-90;
  } 
  else  {
    //shoulder:
    m2=20;
      a=12.5*sin(radians(20));
      b=h-(12.5*cos(radians(20)));
      c=hypot(a,b);
      
    //elbow:  
    m3=75-(atan(radians(a/b)));
    m3=m3+degrees(acos((sq(hypot(c,ul))-sq(hl))/(2*c*ul)));
    //wrist:
    m4=degrees(acos((sq(hypot(ul,hl))-sq(c))/(2*ul*hl)))-90;
  }
  
  //prints out angles for testing:
  Serial.print(m1);Serial.print(", "); Serial.print(m2);Serial.print(", ");
  Serial.print(m3);Serial.print(", "); Serial.print(m4);Serial.println(".");
}
//--------------------------------------------------------------

void inIK() {
  Serial.println("inIK() running!");
  //base:
  if (X>=0) {
    m1=180-degrees(asin(Z/h));
  }
  else  {
    m1=degrees(asin(Z/h));
  }
  //arm:
  m2=degrees(acos((sq(hypot(hl,ul))-sq(hl))/(2*hl*ul)))+degrees(asin(Y/ht));
  m3=0;
  //wrist:
  m4=degrees(acos((sq(hypot(hl,arm))-sq(ht))/(2*hl*arm)))-45;

  //prints out angles for testing:
  Serial.print(m1);Serial.print(", "); Serial.print(m2);Serial.print(", ");
  Serial.print(m3);Serial.print(", "); Serial.print(m4);Serial.println(".");
}
//--------------------------------------------------------------
void rev() { //to figure out base angle/rotation, 
              //and if other servos has to be reversed due to 180* limit.
              //parse serial.read to float(xyz) first
              
  Serial.println("rev() running!");

//checks if outside base angles:
  if (Z<0){    
    //reverse all joints
    m1=m1-180; 
    m2=180-m2;
    m3=180-m3;
    m4=180-m4;
    //gripper does not need reversal
    }
  //m1=acos(
}

//--------------------------------------------------------------
void place()  {
  Serial.println("place() running!");
  //re-scaling due to quirks in servo accuracy:
  //map(value, fromLow, fromHigh, toLow, toHigh)
  //gonna need to do this manually due to map only doing integers...
  
  m1=mapfloat((m1), 0, 180, 15, 180); //reversed base due to plate angles reversed
  m2=mapfloat((m2), 15, 165, 12, 155);
  m3=mapfloat((m3), 0, 180, 7, 180);
  m4=mapfloat((m4), 0, 180, -6, 171);
  // m1=180-m1;
  
  //prints out angles for testing:
  Serial.print(m1);Serial.print(", "); Serial.print(m2);Serial.print(", ");
  Serial.print(m3);Serial.print(", "); Serial.print(m4);Serial.println(".");
                        //(step delay  M1(base), M2(shoulder), M3(elbow), M4(wrist), M5(angle), M6(fingers));
  if (!isnan(m1)&&!isnan(m2)&&!isnan(m3)&&!isnan(m4)){
    Braccio.ServoMovement(1000,         m1, 90, 90, m4, 90, 10);//aim
    delay(500);  
    Braccio.ServoMovement(2000,         m1, m2+20, m3, m4, 90, 10);//go over
    delay(500);
    Braccio.ServoMovement(1000,         m1, m2, m3, m4, 90, 10);//go to
    Braccio.ServoMovement(200,         m1, m2, m3, m4, 90, 50);//grip
    delay(500);
    Braccio.ServoMovement(1000,         m1, m2+20, m3, m4, 90, 50);//pick up
    delay(500);/*
    Braccio.ServoMovement(3000,         90, 90, 180, 90, 90, 50);//transport
    delay(500);
    Braccio.ServoMovement(3000,         90, 90, 180, 90, 90, 10);//drop
    delay(500);*/
  }
  else{ Serial.println("error, not a number!");}
}

//--------------------------------------------------------------

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;}
