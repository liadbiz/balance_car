/****************************************************************************
  亚博智能科技有限公司
  产品名称：Arduino 智能平衡小车
  产品型号：BST-ABC ver2.0
  修改记录：181207 增加电压采集显示liusen
****************************************************************************/


#include "./PinChangeInt.h"
#include "./MsTimer2.h"
//利用测速码盘计数实现速度PID控制
#include "./BalanceCar.h"
#include "./KalmanFilter.h"
//I2Cdev、MPU6050和PID_v1类库需要事先安装在Arduino 类库文件夹下
#include "./I2Cdev.h"
#include "./MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu; //实例化一个 MPU6050 对象，对象名称为 mpu
BalanceCar balancecar;
KalmanFilter kalmanfilter;
int16_t ax, ay, az, gx, gy, gz;
//TB6612FNG驱动模块控制信号
#define IN1M 7
#define IN2M 6
#define IN3M 13
#define IN4M 12
#define PWMA 9
#define PWMB 10
#define STBY 8

#define PinA_left 2  //中断0
#define PinA_right 4 //中断1



//声明自定义变量
float Voltage = 0;
int voltagepin = A0;
byte inByte; //串口接收字节
int num;
double Setpoint;                               //角度DIP设定点，输入，输出
double Setpoints, Outputs = 0;                         //速度DIP设定点，输入，输出
double kp = 38, ki = 0.0, kd = 0.56;                   //需要你修改的参数
double kp_speed = 3.9, ki_speed = 0.12, kd_speed = 0.0;           // 需要你修改的参数
double kp_turn = 28, ki_turn = 0, kd_turn = 0.29;                 //旋转PID设定
const double PID_Original[6] = {38, 0.0, 0.58, 4.0, 0.12, 0.0}; //恢复默认PID参数
//转向PID参数
double setp0 = 0, dpwm = 0, dl = 0; //角度平衡点，PWM差，死区，PWM1，PWM2
float value;

///////////////////自定义的变量//////////////////////////

volatile long pulseSum = 0;  // 总的脉冲数目
int flag = 0;

const int trigPin = A2;

const int echoPin = A3;

long duration;

int distance;  // 超声波检测的距离

int loopTime; // 小车走的次数，相当于做action的次数

// Q-learning 相关参数
float gamma = 0.8;
float alpha = 0.1;

float epsilon;  // 给getAction函数用的，代表选择action的时候贪婪的程度，如果是1那就是选择(state, action)的Q_value最大的组合

const int numStates = 6; // [ 5 | 4 3 2 1 0  => [ 代表墙，| 代表目标点， 数字代表状态点
const int numActions = 2; //只有两个action，分别是前进和后退一定距离, 0代表前进，1代表后退

int stopDistance = 50; // 最终目标，停在离墙这个距离的地方
int startDistance = 200;

int maxStartDistance = 250; // 最远从2.5m的地方开始，放在最远的地方也就是一次走0.4m
int minStartDistance = 150; // 最近从1.5m的地方开始，放在最近的地方就是一次走0.2m
int deltaDistance = 0;  // 每次走的距离，要根据初始化的时候测得的距离算

float Q_value[numStates][numActions];

int state = 0;  // 在做训练的时候每次的状态
int action = 0;  // 在做训练的时候每次的action
int cur_state = 0;

int reward = 0;

float lookAheadValue = 0.0;

///////////////////自定义的变量//////////////////////////

//********************angle data*********************//
float Q;
float Angle_ax; //由加速度计算的倾斜角度
float Angle_ay;
float K1 = 0.05; // 对加速度计取值的权重
float angle0 = 0.00; //机械平衡角
int slong;
//********************angle data*********************//

//***************Kalman_Filter*********************//
float Q_angle = 0.001, Q_gyro = 0.005; //角度数据置信度,角速度数据置信度
float R_angle = 0.5 , C_0 = 1;
float timeChange = 5; //滤波法采样时间间隔毫秒
float dt = timeChange * 0.001; //注意：dt的取值为滤波器采样时间
//***************Kalman_Filter*********************//

//*********************************************
//******************** speed count ************
//*********************************************

volatile long count_right = 0;//使用volatile lon类型是为了外部中断脉冲计数值在其他函数中使用时，确保数值有效
volatile long count_left = 0;//使用volatile lon类型是为了外部中断脉冲计数值在其他函数中使用时，确保数值有效
int speedcc = 0;

//////////////////////脉冲计算/////////////////////////
int lz = 0;
int rz = 0;
int rpluse = 0;
int lpluse = 0;
int sumam;
/////////////////////脉冲计算////////////////////////////

//////////////转向、旋转参数///////////////////////////////
int turncount = 0; //转向介入时间计算
float turnoutput = 0;
//////////////转向、旋转参数///////////////////////////////

//////////////Wifi控制量///////////////////
#define run_car     '1'//按键前
#define back_car    '2'//按键后
#define left_car    '3'//按键左
#define right_car   '4'//按键右
#define stop_car    '0'//按键停
/*小车运行状态枚举*/
enum {
  enSTOP = 0,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,
  enTLEFT,
  enTRIGHT
} enCarState;
int incomingByte = 0;          // 接收到的 data byte
String inputString = "";         // 用来储存接收到的内容
boolean newLineReceived = false; // 前一次数据结束标志
boolean startBit  = false;  //协议开始标志
int g_carstate = enSTOP; //  1前2后3左4右0停止
String returntemp = ""; //存储返回值
boolean g_autoup = false;
int g_uptimes = 5000;

int front = 0;//前进变量
int back = 0;//后退变量
int turnl = 0;//左转标志
int turnr = 0;//右转标志
int spinl = 0;//左旋转标志
int spinr = 0;//右旋转标志
int bluetoothvalue;//蓝牙控制量
//////////////蓝牙控制量///////////////////

//////////////////超声波速度//////////////////

// int chaoshengbo = 0;
// int tingzhi = 0;
// int jishi = 0;

//////////////////超声波速度//////////////////


//////////////////////脉冲计算///////////////////////
void countpluse()
{

  lz = count_left;
  rz = count_right;


  count_left = 0;
  count_right = 0;

  lpluse = lz;
  rpluse = rz;

  if ((balancecar.pwm1 < 0) && (balancecar.pwm2 < 0))                     //小车运动方向判断 后退时（PWM即电机电压为负） 脉冲数为负数
  {
    rpluse = -rpluse;
    lpluse = -lpluse;
  }
  else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 > 0))                 //小车运动方向判断 前进时（PWM即电机电压为正） 脉冲数为负数
  {
    rpluse = rpluse;
    lpluse = lpluse;
  }
  else if ((balancecar.pwm1 < 0) && (balancecar.pwm2 > 0))                 //小车运动方向判断 前进时（PWM即电机电压为正） 脉冲数为负数
  {
    rpluse = rpluse;
    lpluse = -lpluse;
  }
  else if ((balancecar.pwm1 > 0) && (balancecar.pwm2 < 0))               //小车运动方向判断 左旋转 右脉冲数为负数 左脉冲数为正数
  {
    rpluse = -rpluse;
    lpluse = lpluse;
  }


  //提起判断
  balancecar.stopr += rpluse;
  balancecar.stopl += lpluse;

  //每5ms进入中断时，脉冲数叠加
  balancecar.pulseright += rpluse;
  balancecar.pulseleft += lpluse;
  sumam = balancecar.pulseright + balancecar.pulseleft;
}
////////////////////脉冲计算///////////////////////



//////////////////角度PD////////////////////
void angleout()
{
  balancecar.angleoutput = kp * (kalmanfilter.angle + angle0) + kd * kalmanfilter.Gyro_x;//PD 角度环控制
}
//////////////////角度PD////////////////////

//////////////////////////////////////////////////////////
//////////////////中断定时 5ms定时中断////////////////////
/////////////////////////////////////////////////////////
void inter()
{
  sei();
  countpluse();                                     //脉冲叠加子函数
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);     //IIC获取MPU6050六轴数据 ax ay az gx gy gz
  kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);                                   //获取angle 角度和卡曼滤波
  angleout();                                       //角度环 PD控制

  speedcc++;
  if (speedcc >= 8)                                //40ms进入速度环控制
  {
    Outputs = balancecar.speedpiout(kp_speed, ki_speed, kd_speed, front, back, setp0);
    speedcc = 0;
  }
  turncount++;
  if (turncount > 4)                                //40ms进入旋转控制
  {
    turnoutput = balancecar.turnspin(turnl, turnr, spinl, spinr, kp_turn, kd_turn, kalmanfilter.Gyro_z);                              //旋转子函数
    turncount = 0;
  }
  balancecar.posture++;
  balancecar.pwma(Outputs, turnoutput, kalmanfilter.angle, kalmanfilter.angle6, turnl, turnr, spinl, spinr, front, back, kalmanfilter.accelz, IN1M, IN2M, IN3M, IN4M, PWMA, PWMB);                            //小车总PWM输出

}
//////////////////////////////////////////////////////////
//////////////////中断定时 5ms定时中断///////////////////
/////////////////////////////////////////////////////////
void SendAutoUp()
{
  g_uptimes --;
  if ((g_autoup == true) && (g_uptimes == 0))
  {
    //自动上报
    String CSB;//, VT;
    char temp[10] = {0};
    float fgx;
    float fay;
    float leftspeed;
    float rightspeed;

    fgx = gx;  //传给局部变量
    fay = ay;  //传给局部变量
    leftspeed = balancecar.pwm1;
    rightspeed = balancecar.pwm2;

    double Gx = (double)((fgx - 128.1f) / 131.0f);              //角度转换
    double Ay = ((double)fay / 16384.0f) * 9.8f;

    if (leftspeed > 255 || leftspeed < -255)
      return;
    if (rightspeed > 255 || rightspeed < -255)
      return;
    if ((Ay < -20) || (Ay > 20))
      return;
    if ((Gx < -3000) || (Gx > 3000))
      return;

    returntemp = "";

    memset(temp, 0x00, sizeof(temp));
    dtostrf(leftspeed, 3, 1, temp);  // 相當於 %3.2f
    String LV = temp;

    memset(temp, 0x00, sizeof(temp));
    dtostrf(rightspeed, 3, 1, temp);  // 相當於 %3.1f
    String RV = temp;

    memset(temp, 0x00, sizeof(temp));
    dtostrf(Ay, 2, 2, temp);  // 相當於 %2.2f
    String AC = temp;

    memset(temp, 0x00, sizeof(temp));
    dtostrf(Gx, 4, 2, temp);  // 相當於 %4.2f
    String GY = temp;

    memset(temp, 0x00, sizeof(temp));
    dtostrf(Voltage, 2, 1, temp);  // 相當於 %2.1f

    String VT = temp;

    CSB = "0.00";
    returntemp = "$LV" + LV + ",RV" + RV + ",AC" + AC + ",GY" + GY + ",CSB" + CSB + ",VT" + VT + "#";
    Serial.print(returntemp); //返回协议数据包
  }

  if (g_uptimes == 0)
    g_uptimes = 5000;
}


// ===    初始设置     ===
void setup() {
  // TB6612FNGN驱动模块控制信号初始化
  // Serial.begin(9600);
  pinMode(IN1M, OUTPUT);                         //控制电机1的方向，01为正转，10为反转
  pinMode(IN2M, OUTPUT);
  pinMode(IN3M, OUTPUT);                        //控制电机2的方向，01为正转，10为反转
  pinMode(IN4M, OUTPUT);
  pinMode(PWMA, OUTPUT);                        //左电机PWM
  pinMode(PWMB, OUTPUT);                        //右电机PWM
  pinMode(STBY, OUTPUT);                        //TB6612FNG使能

  // 初始化超声波
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  //初始化电机驱动模块
  digitalWrite(IN1M, 0);
  digitalWrite(IN2M, 1);
  digitalWrite(IN3M, 1);
  digitalWrite(IN4M, 0);
  digitalWrite(STBY, 1);
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  pinMode(PinA_left, INPUT);  //测速码盘输入
  pinMode(PinA_right, INPUT);

  // 加入I2C总线
  Wire.begin();                            //加入 I2C 总线序列
  Serial.begin(9600);                       //开启串口，设置波特率为 115200
  delay(1500);
  mpu.initialize();                       //初始化MPU6050
  delay(2);
  balancecar.pwm1 = 0;
  balancecar.pwm2 = 0;
  //5ms定时中断设置  使用timer2    注意：使用timer2会对pin3 pin11的PWM输出有影响，因为PWM使用的是定时器控制占空比，所以在使用timer的时候要注意查看对应timer的pin口。
  MsTimer2::set(5, inter);
  MsTimer2::start();

  delay(8000);
  // 第一次测得距离，算出每次走的距离
  getDistance();
  startDistance = distance;
  deltaDistance = int( (startDistance - 50) / 5 );

}

////////////////////////////////////////turn//////////////////////////////////

void ResetPID()
{
  kp = PID_Original[0];
  ki =  PID_Original[1];
  kd =  PID_Original[2];                     //需要你修改的参数
  kp_speed =  PID_Original[3];
  ki_speed =  PID_Original[4];
  kd_speed =  PID_Original[5];  // 需要你修改的参数
}
void ResetCarState()
{
  turnl = 0;
  turnr = 0;
  front = 0;
  back = 0;
  spinl = 0;
  spinr = 0;
  turnoutput = 0;
}

float voltage_test()
{
  float fVoltage;
  int iVoltage;
  pinMode(voltagepin, INPUT);
  iVoltage = analogRead(A0);                        //读取A0口值
  fVoltage = (iVoltage / 1023.0) * 5 * 4.2 - 0.35;  //换算为电压值
  //Voltage是端口A0采集到的ad值（0-1023），
  //0.35是由于电阻的精度问题所作的调整值。
  return fVoltage;
}

////////////////////////Q-learning 相关函数/////////////////////
void walk()
{
  ResetCarState();
  //while (pulseSum < (deltaDistance / (3.1415926 * 7) ) * 390 )  // 根据轮子周长和单次走的距离算要走多少脉冲，这里是存在误差的。
  while(pulseSum < 5 * 390);
  {
    //Serial.print(pulseSum);
    //Serial.print("\n");
    getDistance();
    Serial.print(distance);
    Serial.print("\n");
    front = 250;
  }

  ResetCarState();

}

// 超声波检测距离
void getDistance()
{
  digitalWrite(trigPin, LOW);

  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds

  digitalWrite(trigPin, HIGH);

  delayMicroseconds(10);

  digitalWrite(trigPin, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds

  duration = pulseIn(echoPin, HIGH);

  // Calculating the distance

  distance = duration * 0.034 / 2;

}

int getAction()
{
  int ac;
  
  if (state == 0)  // 在状态0，只能前进
  {
    return 0;
  }
  else if (state == 5)  // 在状态5 只能后退
  {
    return 1;
  }else
  {
    if (Q_value[state][0] > Q_value[state][1] )
    {
      ac = 0; 
    }
    else
    {
      ac = 1;
    }
  }

  int randVal = float(random(0,100));
  if (randVal < epsilon * 100)  // return action of max Q value with probability epsilon
  {
    return ac;
  }
  else
  {
    return 1 - ac;
  }
  
}

int getReward()
{
  getDistance();
  if (distance > 40 & distance < 60)  // 允许存在10cm的误差
  {
    return 50;
  }
  if (distance > 60 )
  {
    return 0;
  }
  if (distance < 40) 
  {
    return distance  - 50;
  }
}

float getLookAhead()
{
  if (state == 0)
  {
    return Q_value[state][0]; // 如果是状态0，只能前进
  };
  if (state == 5)
  {
    return Q_value[state][1]; // 如果是状态5，只能后退
  };
  return Q_value[state][0] > Q_value[state][1] ? Q_value[state][0] :  Q_value[state][1];  // 其余状况返回最大值即可
}

void printQ(){
  Serial.print("========Q value=============");
  for(int i=0; i<numStates; i++){
    for(int j=0; j<numActions; j++){
      Serial.print(Q_value[i][j]);
      Serial.print(" ");
    }
    Serial.println("\n");
  }
  Serial.println("==============================");
}

void initializeQ(){
  for(int i=0; i<numStates; i++){
    for(int j=0; j<numActions; j++){
      Q_value[i][j] = 0.0;               //Initialize to a positive number to represent optimism over all state-actions
    }
  }
}

///////////////////////////////////////////////////////////////////


// ===       主循环程序体       ===
void loop() {
  Voltage = voltage_test();
  //Serial.print(Voltage,DEC);
  // Serial.print("\r\n");

  String returnstr = "$0,0,0,0,0,0,0,0,0,0,0,0cm,0.0V#";  //默认发送

  //主函数中循环检测及叠加脉冲 测定小车车速  使用电平改变既进入脉冲叠加 增加电机的脉冲数，保证小车的精确度。
  attachInterrupt(0, Code_left, CHANGE);
  attachPinChangeInterrupt(PinA_right, Code_right, CHANGE);


  if (newLineReceived)
  {
    switch (inputString[1])
    {
      case run_car:   g_carstate = enRUN;   break;
      case back_car:  g_carstate = enBACK;  break;
      case left_car:  g_carstate = enLEFT;  break;
      case right_car: g_carstate = enRIGHT; break;
      case stop_car:  g_carstate = enSTOP;  break;
      default: g_carstate = enSTOP; break;
    }
    //判断协议是否有丢包

    if (inputString[3] == '1' && inputString.length() == 21) //左摇
    {
      g_carstate = enTLEFT;
      //Serial.print(returnstr);
    }
    else if (inputString[3] == '2' && inputString.length() == 21) //右摇
    {
      g_carstate = enTRIGHT;
      // Serial.print(returnstr);
    }

    if (inputString[5] == '1') //查询PID
    {
      char charkp[7], charkd[7], charkpspeed[7], charkispeed[7];

      dtostrf(kp, 3, 2, charkp);  // 相當於 %3.2f
      dtostrf(kd, 3, 2, charkd);  // 相當於 %3.2f
      dtostrf(kp_speed, 3, 2, charkpspeed);  // 相當於 %3.2f
      dtostrf(ki_speed, 3, 2, charkispeed);  // 相當於 %3.2f

      String strkp = charkp; String strkd = charkd; String strkpspeed = charkpspeed; String strkispeed = charkispeed;

      returntemp = "$0,0,0,0,0,0,AP" + strkp + ",AD" + strkd + ",VP" + strkpspeed + ",VI" + strkispeed + "#";

      Serial.print(returntemp); //返回协议数据包
    }
    else if (inputString[5] == '2') //恢复PID
    {
      ResetPID();
      Serial.print("$OK#"); //返回协议数据包
    }

    if (inputString[7] == '1') //自动上报
    {
      g_autoup = true;
      Serial.print("$OK#"); //返回协议数据包
    }
    else if (inputString[7] == '2') //停止自动上报
    {
      g_autoup = false;
      Serial.print("$OK#"); //返回协议数据包
    }

    if (inputString[9] == '1') //角度环更新 $0,0,0,0,1,1,AP23.54,AD85.45,VP10.78,VI0.26#
    {
      int i = inputString.indexOf("AP");
      int ii = inputString.indexOf(",", i);
      if (ii > i)
      {
        String m_skp = inputString.substring(i + 2, ii);
        m_skp.replace(".", "");
        int m_kp = m_skp.toInt();
        kp = (double)( (double)m_kp / 100.0f);
      }


      i = inputString.indexOf("AD");
      ii = inputString.indexOf(",", i);
      if (ii > i)
      {
        //ki = inputString.substring(i+2, ii);
        String m_skd = inputString.substring(i + 2, ii);
        m_skd.replace(".", "");
        int m_kd = m_skd.toInt();
        kd = (double)( (double)m_kd / 100.0f);
      }
      Serial.print("$OK#"); //返回协议数据包
    }

    if (inputString[11] == '1') //速度环更新
    {
      int i = inputString.indexOf("VP");
      int ii = inputString.indexOf(",", i);
      if (ii > i)
      {
        String m_svp = inputString.substring(i + 2, ii);
        m_svp.replace(".", "");
        int m_vp = m_svp.toInt();
        kp_speed = (double)( (double)m_vp / 100.0f);
      }


      i = inputString.indexOf("VI");
      ii = inputString.indexOf("#", i);
      if (ii > i)
      {
        String m_svi = inputString.substring(i + 2, ii);
        m_svi.replace(".", "");
        int m_vi = m_svi.toInt();
        ki_speed = (double)( (double)m_vi / 100.0f);
        Serial.print("$OK#"); //返回协议数据包
      }

    }
    //恢复默认
    inputString = "";   // clear the string
    newLineReceived = false;

  }

  // 走一段距离停下
  //if (flag == 0)
  //{
    //delay(3000);
    //walk();
    //flag = 1;
  //}

    getDistance();
    Serial.print(distance);
    Serial.print("\n");

    if (flag == 0)
    {
      delay(3000);
      walk();
      flag = 1;
    }

    
  // Q-learning start

//  loopTime++;
//  action = getAction();
//  cur_state = state;
//  if (action = 0)
//  {
//    state = state + 1;
//  }else
//  {
//    state = state - 1;
//  }
//  // by now, state is next_state actualy
//  walk();
//  reward = getReward();
//  lookAheadValue = getLookAhead();
//  Q_value[cur_state][action] = Q_value[cur_state][action] + alpha * (reward + gamma * lookAheadValue - Q_value[cur_state][action]);  // 更新Q
//  
//  if (loopTime == 30)
//  {
//    printQ();  // 到次数就停止训练，看看Q
//  }
  // Q-learning end
  
  switch (g_carstate)
  {
    case enSTOP: turnl = 0; turnr = 0;  front = 0; back = 0; spinl = 0; spinr = 0; turnoutput = 0; break;
    case enRUN: ResetCarState(); front = 250; break;
    case enLEFT: turnl = 1; break;
    case enRIGHT: turnr = 1; break;
    case enBACK: ResetCarState(); back = -250; break;
    case enTLEFT: spinl = 1; break;
    case enTRIGHT: spinr = 1; break;
    default: front = 0; back = 0; turnl = 0; turnr = 0; spinl = 0; spinr = 0; turnoutput = 0; break;
  }

  //增加自动上报
  SendAutoUp();


}

////////////////////////////////////////pwm///////////////////////////////////



//////////////////////////脉冲中断计算/////////////////////////////////////

void Code_left() {

  count_left ++;
  pulseSum += count_left;

} //左测速码盘计数



void Code_right() {

  count_right ++;

} //右测速码盘计数

//////////////////////////脉冲中断计算/////////////////////////////////////

//serialEvent()是IDE1.0及以后版本新增的功能，不清楚为什么大部份人不愿意用，这个可是相当于中断功能一样的啊!
int num1 = 0;
void serialEvent()
{

  while (Serial.available())
  {
    incomingByte = Serial.read();              //一个字节一个字节地读，下一句是读到的放入字符串数组中组成一个完成的数据包
    if (incomingByte == '$')
    {
      num1 = 0;
      startBit = true;
    }
    if (startBit == true)
    {
      num1++;
      inputString += (char) incomingByte;     // 全双工串口可以不用在下面加延时，半双工则要加的//
    }
    if (startBit == true && incomingByte == '#')
    {
      newLineReceived = true;
      startBit = false;
    }

    if (num1 >= 80)
    {
      num1 = 0;
      startBit = false;
      newLineReceived = false;
      inputString = "";
    }
  }
}
