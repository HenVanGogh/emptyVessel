#include "MPU6050.h"
#include "Adafruit_VL53L0X.h"
#include "KalmanFilter.h"

KalmanFilter kalmanX1(0.001, 0.003, 0.03);
KalmanFilter kalmanY1(0.001, 0.003, 0.03);

KalmanFilter kalmanX2(0.001, 0.003, 0.03);
KalmanFilter kalmanY2(0.001, 0.003, 0.03);

KalmanFilter kalmanX3(0.001, 0.003, 0.03);
KalmanFilter kalmanY3(0.001, 0.003, 0.03);

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

MPU6050 mpu1;
MPU6050 mpu2;
MPU6050 mpu3;

struct orintationTable {
  double Xgyro;
  double Ygyro;
  double Xaccel;
  double Yaccel;
};

orintationTable Main;
orintationTable Cal1;
orintationTable Cal2;

void enableGyro(char which)
{
  if (which == 1) {
    digitalWrite(8, LOW);
    digitalWrite(9, HIGH);
    digitalWrite(10, HIGH);
  }
  else if (which == 2) {
    digitalWrite(8, HIGH);
    digitalWrite(9, LOW);
    digitalWrite(10, HIGH);
  }
  else if (which == 3) {
    digitalWrite(8, HIGH);
    digitalWrite(9, HIGH);
    digitalWrite(10, LOW);
  }
  else {
    digitalWrite(8, HIGH);
    digitalWrite(9, HIGH);
    digitalWrite(10, HIGH);
  }
  delay(10);
}


void getOrientationMain()
{
  enableGyro(1);
  float accPitch = 0;
  float accRoll = 0;

  float kalPitch = 0;
  float kalRoll = 0;

  Vector acc = mpu1.readNormalizeAccel();
  Vector gyr = mpu1.readNormalizeGyro();

  // Calculate Pitch & Roll from accelerometer (deg)
  Main.Xaccel = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0) / M_PI;
  Main.Yaccel = (atan2(acc.YAxis, acc.ZAxis)*180.0) / M_PI;

  // Kalman filter
  Main.Xgyro = kalmanY1.update(accPitch, gyr.YAxis);
  Main.Ygyro = kalmanX1.update(accRoll, gyr.XAxis);
  //enableGyro(4);

}


void getOrientationCal1()
{
  enableGyro(2);
  float accPitch = 0;
  float accRoll = 0;

  float kalPitch = 0;
  float kalRoll = 0;

  Vector acc = mpu2.readNormalizeAccel();
  Vector gyr = mpu2.readNormalizeGyro();

  // Calculate Pitch & Roll from accelerometer (deg)
  Cal1.Xaccel = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0) / M_PI;
  Cal1.Yaccel = (atan2(acc.YAxis, acc.ZAxis)*180.0) / M_PI;

  // Kalman filter
  Cal1.Xgyro = kalmanY2.update(accPitch, gyr.YAxis);
  Cal1.Ygyro = kalmanX2.update(accRoll, gyr.XAxis);
  //enableGyro(4);

}


void getOrientationCal2()
{
  enableGyro(3);
  float accPitch = 0;
  float accRoll = 0;

  float kalPitch = 0;
  float kalRoll = 0;

  Vector acc = mpu3.readNormalizeAccel();
  Vector gyr = mpu3.readNormalizeGyro();

  // Calculate Pitch & Roll from accelerometer (deg)
  Cal2.Xaccel = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0) / M_PI;
  Cal2.Yaccel = (atan2(acc.YAxis, acc.ZAxis)*180.0) / M_PI;

  // Kalman filter
  Cal2.Xgyro = kalmanY3.update(accPitch, gyr.YAxis);
  Cal2.Ygyro = kalmanX3.update(accRoll, gyr.XAxis);
  //enableGyro(4);

}


short int getMeasurent() {
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    return(measure.RangeMilliMeter);
  }
  else {
    return(-1);
  }

}

void gryroSetup(){

  enableGyro(1);
  while (!mpu1.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    delay(500);
  }
  mpu1.calibrateGyro();
  enableGyro(2);
  while (!mpu2.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    delay(500);
  }
  mpu2.calibrateGyro();
  enableGyro(3);
  while (!mpu3.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    delay(500);
  }
  mpu3.calibrateGyro();
  enableGyro(4);

  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1);
  }
}


#include <EasyTransfer.h>

//create two objects
EasyTransfer ETin, ETout; 

struct SEND_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  orintationTable gyro1;
  orintationTable gyro2;
  orintationTable gyro3;
  short int distance;
  
};

struct RECEIVE_DATA_STRUCTURE{
  //put your variable definitions here for the data you want to receive
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  int poseTable[3][2];
  bool legsOn;
};
RECEIVE_DATA_STRUCTURE rxdata;
SEND_DATA_STRUCTURE txdata;


#define errorAnalysis




  #define P11 0
  #define P12 3
  #define P13 9
  #define P14 10

  #define P21 2
  #define P22 1
  #define P23 5
  #define P24 4

  #define L11 15
  #define L12 14
  #define L13 8
  #define L14 7
  
  #define L21 13
  #define L22 12
  #define L23 6
  #define L24 11

  double linearDistance = 60.357;
  
long posP11;/////P////
long posP12;
long posP13;
long posP21;
long posP22;
long posP23;


long posL11;/////L/////
long posL12;
long posL13;
long posL21;
long posL22;
long posL23;

long posP31;
long posP32;
long posP33;/////
long posL31;
long posL32;
long posL33;/////


double Ytest;
double YposT;
double ZposT;

double Xpos = 0;
double Ypos = 0;
double Zpos = 0;
  
  double Leg0 = 56.3, Leg1 = 167, Leg2 = 136;
  double radian = 57.2958;

#include <math.h>

//SoftwareSerial gyroSerial(10, 11);

#include <Wire.h>

#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

int con = true;
#include <SoftwareSerial.h>// import the serial library
int BluetoothData; // the data given from Computer


double staP11 = 373-150;/////P////
double staP12 = 579;
double staP13 = 691;
double staP21 = 318+150;
double staP22 = 596;
double staP23 = 318;


double staL11 = 455;/////L/////
double staL12 = 566;
double staL13 = 565;
double staL21 = 278;
double staL22 = 325;
double staL23 = 686+260;

double staP31;
double staP32;
double staP33;

double staL31;
double staL32;
double staL33;


int Xp1 = 0; int Yp1 = 500; int Zp1 = 500;
int Xp2 = 0; int Yp2 = 500; int Zp2 = 500;

int Xl1 = 0; int Yl1 = 500; int Zl1 = 500;
int Xl2 = 0; int Yl2 = 500; int Zl2 = 500;

long valP11 = staP11;/////BIS
long valP12 = staP12;    //OFF 30
long valP13 = staP13;//
long valP21 = staP21;   //Change from 45
long valP22 = staP22;//////BIS
long valP23 = staP23;//
long valP31 = staP31;
long valP32 = staP32;    //OFF 30
long valP33 = staP33;//

long valL11 = staL11;
long valL12 = staL12;//////BIS
long valL13 = staL13;//
long valL21 = staL21;
long valL22 = staL22;    //OFF 60
long valL23 = staL23;//BIS
long valL31 = staL31;
long valL32 = staL32;//BIS
long valL33 = staL33;//



void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
//gyroSerial.begin(19200);
 ETin.begin(details(rxdata), &Serial);
 ETout.begin(details(txdata), &Serial);

pwm.begin();
pwm.setPWMFreq(100);
}

class Runner{
    bool Side;
    double pin1, pin2, pin3;
    double St1, St2, St3;
    double LT, L0, L1, L2, L3, L4, L5, L6, L7, X, Y, Z, T1, T2, T3, K1, K2, K3, K4, K5, K6;
    double radian = 57.2958;
    int Lp;
public: 
        Runner(double Pin1, double Pin2, double Pin3, double st1, double st2, double st3, bool side , int lp){
          pin1 =Pin1;
          pin2 =Pin2;
          pin3 = Pin3;
           
          L0 = Leg0;
          L1 = Leg1;
          L2 = Leg2;
           
          St1 = st1;
          St2 = st2;
          St3 = st3;
        
          Side = side;

          Lp = lp;
        }
        void Step (double x, double y, double z)
        {
          if((y > 20) && (z > 17) && (x > 17) && (y < 250) && (z < 350) && (x < 350)){
          bool error = false;
          X = x;
          Y = y;
          Z = z;

          double XZsafeFactor = sqrt(91809-(Y*Y));
          //Serial.print("Safe Factor XZ - ");
          //Serial.println(XZsafeFactor); 

                double sqrtZX = (Z*Z) + (X*X);
                if(sqrt(sqrtZX) > XZsafeFactor){
                  Serial.println("XZ Safe factor ERROR");
                  //Serial.println(sqrtZX);
                  //Serial.println("WTF");
                }
                if(Y > 300){
                  Serial.println("Y Safe factor ERROR");
                  Serial.println("WTF?");
                }
                T1 = atan (Z/X);
                L5 = sqrt(sqrtZX);
                //L5 = LT- L0;
                L4 = sqrt((y*y) + (L5*L5));
                K1 = acos(L5/L4);
                double PowL2L4 , PowL1 , subL2L4L1 ;
                PowL2L4 = ((L2*L2)+(L4*L4));
                PowL1 = L1*L1;
                subL2L4L1 = PowL2L4 - PowL1;
                K2 = acos(subL2L4L1 /(2*L4*L2));
                K3 = K1+K2;
                K5 = acos(((L1*L1)+ (L2*L2)- (L4*L4))/(2*L1*L2));
                K6 = K3;
                T2 = 3.14159-(K5+ K6);
                T3 = 3.14159- K5;
                
                double TD1 = T1*radian*3.333;
                double TD2 = T2*radian*3.333;
                double TD3 = T3*radian*3.333;

                double pos1;
                double pos2;
                double pos3;

                
/*
                if(isnan(pos1) == false || isnan(pos2) == true || isnan(pos3) == true){
                  Serial.print("ERROR LEG"); Serial.print(Lp); Serial.println("IS NAN"); 
                }
*/
                
                

                

                //pos1 = St1 - TD1;    // To jest zamienione na IF
                pos2 = St2 - TD2;
                if( Lp == 2){
                   pos3 = St3 + TD3;
                }else{
                   pos3 = St3 - TD3;
                } 

                if( Lp == 4 || Lp == 1){
                   pos1 = St1 + TD1;
                }else{
                   pos1 = St1 - TD1;
                }
                //pos3 = St3 - TD3;

                
                
                #ifdef errorAnalysis
                
                if(Lp == 1){
                float exportValue1 = float(pos1);
                float exportValue2 = float(pos2);
                float exportValue3 = float(pos3);
                //Serial.println(exportValue1);
                //Serial.println(exportValue2);
                //Serial.println(exportValue3);
                }
                //Serial.print(Lp);
                
                //Serial.println(pos2);
                //Serial.println(pos3);
                if(isnan(pos1) == true){
                  Serial.print("ERROR_LEG 1_"); Serial.print(Lp); Serial.println("_IS NAN");
                  error = true; 
                }
                if(isnan(pos2) == true){
                  Serial.print("ERROR_LEG 2_"); Serial.print(Lp); Serial.println("_IS NAN");
                  error = true;  
                }
                if(isnan(pos3) == true){
                  Serial.print("ERROR_LEG 3_"); Serial.print(Lp); Serial.println("_IS NAN"); 
                  error = true; 
                }
                #endif
               

                
                if(error == false){
                pwm.setPWM(pin1, 0,pos1 );
                pwm.setPWM(pin2, 0,pos2 );
                pwm.setPWM(pin3, 0,pos3 );
                }else{
                  Serial.print("ErrorCode = "); Serial.println(error);
                }
          }else{
            Serial.print("DATA_ERROR LEG_NR :   "); Serial.println(Lp);
            Serial.print("X :   "); Serial.println(x);
            Serial.print("Y :   "); Serial.println(y);
            Serial.print("Z :   "); Serial.println(z);
          }
                }
                
        
        };

       void fullReport(){
      getOrientationMain();
      getOrientationCal1;
      getOrientationCal2;

     txdata.gyro1.Xgyro = Main.Xgyro;
     txdata.gyro1.Ygyro = Main.Ygyro;
     txdata.gyro1.Xaccel = Main.Xaccel;
     txdata.gyro1.Yaccel = Main.Yaccel;

     txdata.gyro2.Xgyro = Cal1.Xgyro;
     txdata.gyro2.Ygyro = Cal1.Ygyro;
     txdata.gyro2.Xaccel = Cal1.Xaccel;
     txdata.gyro2.Yaccel = Cal1.Yaccel;

     txdata.gyro3.Xgyro = Cal2.Xgyro;
     txdata.gyro3.Ygyro = Cal2.Ygyro;
     txdata.gyro3.Xaccel = Cal2.Xaccel;
     txdata.gyro3.Yaccel = Cal2.Yaccel;

     ETout.sendData();
        }

       

Runner RunnerP1(P11,P12,P13,staP11,staP12,staP13,1,1);
Runner RunnerP2(P21,P22,P23,staP21,staP22,staP23,1,2);

Runner RunnerL1(L11,L12,L13,staL11,staL12,staL13,0,3);
Runner RunnerL2(L21,L22,L23,staL21,staL22,staL23,0,4);

void loop() {
    for(int i=0; i<5; i++){
    ETin.receiveData();
    }
if(rxdata.legsOn){
RunnerP1.Step(rxdata.poseTable[0][0] , rxdata.poseTable[0][1] , rxdata.poseTable[0][2]);
RunnerP2.Step(rxdata.poseTable[1][0] , rxdata.poseTable[1][1] , rxdata.poseTable[1][2]);

RunnerL1.Step(rxdata.poseTable[2][0] , rxdata.poseTable[2][1] , rxdata.poseTable[2][2]);
RunnerL2.Step(rxdata.poseTable[3][0] , rxdata.poseTable[3][1] , rxdata.poseTable[3][2]);
}

fullReport();
}
