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
    //while (1);
  }
}
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
  
  double Leg0 = 56.3, Leg1 = 167, Leg2 = 136;
  double radian = 57.2958;

#include <math.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);




double staP11 = 373-150;
double staP12 = 579;
double staP13 = 691;
double staP21 = 318+150;
double staP22 = 596;
double staP23 = 318;


double staL11 = 455;
double staL12 = 566;
double staL13 = 565;
double staL21 = 278;
double staL22 = 325;
double staL23 = 686+260;



void setup() {
Serial.begin(115200);
//gyroSerial.begin(19200);
gryroSetup();

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
                
  void setConstans(double a, double b, double c){
    if(a != 0){
    St1 = a;
    }
    
    if(b !=0){
    St2 = b;
    }
    
    if(c != 0){
    St3 = c;
    }
    
    
      void setCurrentPosition(double a, double b, double c){
    currentPosition[0] = a;
    currentPosition[1] = b;
    currentPosition[2] = c;
  }
  
  int beginningPosition[2];
  int currentPosition[2];
  int desiredPosition[2];
  float speedInAxis[2];
  int endStep;
  int currentstep = 0;
    bool skip;
  
  
  void setOjective(int x,int y,int z,int speed){
    
    beginningPosition[0] = currentPosition[0];
    beginningPosition[1] = currentPosition[1];
    beginningPosition[2] = currentPosition[2];
    
    desiredPosition[0] = x;
    desiredPosition[1] = y;
    desiredPosition[2] = z;
    
    endStep = speed;
    
    speedInAxis[0] = (desiredPosition[0] - beginningPosition[0]) / speed;
    speedInAxis[1] = (desiredPosition[1] - beginningPosition[1]) / speed;
    speedInAxis[2] = (desiredPosition[2] - beginningPosition[2]) / speed;
    
    skip = false;
    currentstep = 0;
  }
    
    
  
  void update(){
    if(!skip){
if(currentstep != endStep){
    currentPosition[0] = currentPosition[0] + speedInAxis[0];
    currentPosition[1] = currentPosition[1] + speedInAxis[1];
    currentPosition[2] = currentPosition[2] + speedInAxis[2];
    currentstep++;
}else{
  currentPosition[0] = desiredPosition[0];
  currentPosition[1] = desiredPosition[1];
  currentPosition[2] = desiredPosition[2]; 
  skip = true;
  }
    }
  }
  
  
        };



       

Runner RunnerP1(P11,P12,P13,staP11,staP12,staP13,1,1);
Runner RunnerP2(P21,P22,P23,staP21,staP22,staP23,1,2);

Runner RunnerL1(L11,L12,L13,staL11,staL12,staL13,0,3);
Runner RunnerL2(L21,L22,L23,staL21,staL22,staL23,0,4);

void loop() {

   
if(legsOn){
RunnerP1.Step(rxdata.poseTable[0][0] , rxdata.poseTable[0][1] , rxdata.poseTable[0][2]);
RunnerP2.Step(rxdata.poseTable[1][0] , rxdata.poseTable[1][1] , rxdata.poseTable[1][2]);

RunnerL1.Step(rxdata.poseTable[2][0] , rxdata.poseTable[2][1] , rxdata.poseTable[2][2]);
RunnerL2.Step(rxdata.poseTable[3][0] , rxdata.poseTable[3][1] , rxdata.poseTable[3][2]);
}


}
