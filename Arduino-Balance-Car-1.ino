/************Pins*************
	Forward Back
MotorA 	D5	D6
MotorB  D10	D9
EncoderA *D2	D7
EncoderB *D3	D8
MPU6050	A4-SDA	A5-SCL
*****************************/
#include <MsTimer2.h>        //定时中断
#include <KalmanFilter.h>    //卡尔曼滤波
#include "I2Cdev.h"        
#include "MPU6050_6Axis_MotionApps20.h"//MPU6050库文件
#include "Wire.h"
/*******Global Various*******/
//#define MIDDLEVALUE 3
#define MAX 180
#define GMAX 100
#define IMAX 2000
float MIDDLEVALUE=-0.15;
int vb=0;
int va=0;
float balance_kp=40,balance_kd=0.73;//0.5//34//1.608//34//1.29//0.87
//float velocity_kp=0.6,velocity_ki=0.0048;//0.6//0.0048
float velocity_kp=0.6,velocity_ki=0.005;//0.6//0.0048
float AngleShow=0;
float Gyro_xShow=0;
int PWMShow=0;
float VelocityIntegralShow=0;
int VelocityB;
int VelocityA;
int VelocityGlobal=0;
int MotorAPWM=0,MotorBPWM=0;
MPU6050 Mpu6050; //实例化一个 MPU6050 对象，对象名称为 Mpu6050
KalmanFilter KalFilter;//实例化一个卡尔曼滤波器对象，对象名称为 KalFilter
int16_t ax, ay, az, gx, gy, gz;  //MPU6050的三轴加速度和三轴陀螺仪数据
float Angle=0;
float Gyro_x=0;
/*************KalmanFilter******/
float K1 = 0.05; // 对加速度计取值的权重
float Q_angle = 0.001, Q_gyro = 0.005;
float R_angle = 0.5 , C_0 = 1;
float dt = 0.005; //注意：dt的取值为滤波器采样时间 5ms
/*****************************/
void ScheduleControl(){
  static float tmpAngle=0;
  float Gyro;
  sei();//全局中断开启
  MotorAPWM=0;
  MotorBPWM=0;
  Mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  //获取MPU6050陀螺仪和加速度计的数据
  //KalFilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);          //通过卡尔曼滤波获取角度
  //Angle=KalFilter.angle;//Angle是一个用于显示的整形变量
  //Gyro_x=KalFilter.Gyro_x;
  //Gyro_x=(Angle-tmpAngle)/0.0049;
  //Gyro_x+=4.9;
  //Balance();
  //Velocity();
  /*
  tmpAngle=Angle;
  if(MotorAPWM>=MAX)MotorAPWM=MAX;
  if(MotorBPWM>=MAX)MotorBPWM=MAX;
  if(MotorAPWM<=-MAX)MotorAPWM=-MAX;
  if(MotorBPWM<=-MAX)MotorBPWM=-MAX;
  */
  Serial.print
  Motor();
}
void Balance(){
	float AngleTmp=Angle-MIDDLEVALUE;
	static float AngleArray[5]={0};
	static float GyroArray[2]={0};

	AngleArray[0]=AngleArray[1];
	AngleArray[1]=AngleArray[2];
  AngleArray[2]=AngleArray[3];
  AngleArray[3]=AngleArray[4];
  AngleArray[4]=AngleTmp;
	AngleTmp=(AngleArray[0]+AngleArray[1]+AngleArray[2]+AngleArray[3]+AngleArray[4])*0.2;

  GyroArray[0]=GyroArray[1];
  GyroArray[1]=Gyro_x;

  Gyro_x=GyroArray[0]*0.3+GyroArray[1]*0.7;

    //if(Gyro_x>=GMAX)Gyro_x=GMAX;
    //if(Gyro_x<=-GMAX)Gyro_x=-GMAX;
    //Gyro_x=Gyro_x*0.2+Gyro_xLast*0.8;
    
   AngleShow=AngleTmp;
   Gyro_xShow=Gyro_x;
	 MotorAPWM =balance_kp * AngleTmp + Gyro_x * balance_kd;
	 MotorBPWM =balance_kp * AngleTmp + Gyro_x * balance_kd;
}
void Velocity(){
    static char vcount=0;
    if(++vcount==8){
    vcount=0;
    va=VelocityA;
    vb=VelocityB;
  static float VelocityLast=0,VelocityIntegral=0;
  VelocityGlobal=(VelocityA+VelocityB)/2-0;
  VelocityGlobal=VelocityLast*0.3+VelocityGlobal*0.7;
  VelocityIntegral+=VelocityGlobal;
  VelocityLast=VelocityGlobal;
  if(VelocityIntegral>=IMAX)VelocityIntegral=IMAX;
  if(VelocityIntegral<=-IMAX)VelocityIntegral=-IMAX;
  /*if(Serial.available()){
  char c=Serial.read();
      if(c=='a'){
    if(Serial.read()=='+')VelocityIntegral+=1000;
    else MIDDLEVALUE-=1000;
  }
  }*/
  VelocityIntegralShow=VelocityIntegral;
  MotorAPWM +=velocity_kp * VelocityGlobal + VelocityIntegral * velocity_ki;
  MotorBPWM +=velocity_kp * VelocityGlobal + VelocityIntegral * velocity_ki;
    VelocityA=0;
    VelocityB=0;
  }

}
/***********MAIN****************/
void setup() {
  // put your setup code here, to run once:
/******Mpu6050Initialize******/
   Wire.begin();
   Mpu6050.initialize();     //初始化MPU6050
   delay(20);
   attachInterrupt(1,EncoderBISR,CHANGE);
   attachInterrupt(0,EncoderAISR,CHANGE);
   MsTimer2::set(5, ScheduleControl);
   MsTimer2::start();
     Serial.begin(9600);
     Serial.println("All Init OK!");
}

void loop(){
	Plot();
  //Debug();
  //SerialDisplay();
}
void Debug(){
 if(Serial.available()){
  char c=Serial.read();
  if(c=='m'){
    if(Serial.read()=='+')MotorAPWM++,MotorBPWM++;
    else MotorAPWM--,MotorBPWM--;
  }
  if(c=='p'){
    if(Serial.read()=='+')balance_kp+=1;
    else balance_kp-=1;
  }
    if(c=='d'){
    if(Serial.read()=='+')balance_kd+=0.1;
    else balance_kd-=0.1;
  }
    if(c=='M'){
    if(Serial.read()=='+')MIDDLEVALUE+=0.1;
    else MIDDLEVALUE-=0.1;
  }
    if(c=='s')MsTimer2::stop();
    if(c=='i')MsTimer2::start();
 } 
}
void Plot(){
  delay(50);
  //Serial.println(AngleShow);
  Serial.println(PWMShow);
  //Serial.println(Gyro_xShow);
  //Serial.println(va+vb);
  //Serial.println(VelocityIntegralShow);
  //Serial.println(vb);
}
void SerialDisplay(){
 delay(500);
	Serial.print("Angle=");Serial.print(Angle);Serial.print(" Gyro_x="),Serial.print(Gyro_x);
        Serial.print(" PWMA=");Serial.print(MotorAPWM);
        Serial.print(" PWMB=");Serial.print(MotorBPWM);
        Serial.print(" MIDDLEVALUE=");Serial.print(MIDDLEVALUE);
        Serial.print(" Bkp=");Serial.print(balance_kp);Serial.print(" Bkd=");Serial.print(balance_kd);
        Serial.print(" Vkp=");Serial.print(velocity_kp);Serial.print(" Vki=");Serial.print(velocity_ki);
        Serial.print(" va=");Serial.print(va);Serial.print(" vb=");Serial.println(vb); 
}
void Motor(){
  PWMShow=MotorBPWM;
   if(MotorAPWM>=0){
     analogWrite(6,0);
     analogWrite(5,int(MotorAPWM));
   }else{
     analogWrite(5,0);
     analogWrite(6,-int(MotorAPWM));
   }
    //int tmppwm=MotorBPWM*1;
   if(MotorBPWM>=0){
    analogWrite(9,0);
    analogWrite(10,int(MotorBPWM)); 
   }else{
     analogWrite(10,0);
     analogWrite(9,-int(MotorBPWM));
   }
}

void EncoderAISR(){
  if (digitalRead(2) == LOW) {     //如果是下降沿触发的中断
    if (digitalRead(7) == LOW)      VelocityA++;  //根据另外一相电平判定方向
    else      VelocityA--;
  }
  else {     //如果是上升沿触发的中断
    if (digitalRead(7) == LOW)      VelocityA--; //根据另外一相电平判定方向
    else     VelocityA++;
  }
}
void EncoderBISR(){
  if (digitalRead(3) == LOW) {     //如果是下降沿触发的中断
    if (digitalRead(8) == LOW)      VelocityB--;  //根据另外一相电平判定方向
    else      VelocityB++;
  }
  else {     //如果是上升沿触发的中断
    if (digitalRead(8) == LOW)      VelocityB++; //根据另外一相电平判定方向
    else     VelocityB--;
  }
}
