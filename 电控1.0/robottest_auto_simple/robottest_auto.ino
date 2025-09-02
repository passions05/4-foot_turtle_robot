// Arduino核心库
#include <Arduino.h>
// ESP32系统寄存器库（用于关闭低电压检测、看门狗配置）
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include <rtc_wdt.h> // 看门狗控制（防止程序卡死）
// I2C通信库（如需连接I2C传感器，如加速度计）
#include <Wire.h>
// ESP32舵机控制库（驱动12个舵机）
#include <ESP32Servo.h>
// EEPROM库（存储舵机校准偏移量，掉电不丢失）
#include "EEPROM.h"

// I2C设备地址（如连接I2C传感器，当前代码未使用）
#define Addr 0x1C
// I2C引脚定义（当前代码未实际用于I2C，仅预留）
#define SDA 18
#define SCL 19

// EEPROM存储大小（64字节，足够存储12个舵机的int8_t偏移量）
#define EEPROM_SIZE 64

/* 舵机核心定义 --------------------------------------------------------------------*/
// 舵机对象二维数组：servo[腿序号][舵机序号]，4条腿×每条3个舵机（肩、肘、足）
Servo servo[4][3];
// 舵机引脚二维数组：按“腿序号-舵机功能”定义
// 腿序号：0=右前腿，1=右后腿，2=左前腿，3=左后腿；舵机序号：0=肩部，1=肘部，2=足部
const int servo_pin[4][3] = {{18,  5,  19}, {  2,  4, 15}, {33, 25,  32}, {14,  27, 13}};

// 舵机角度偏移量（校准用，每个舵机的实际角度=计算角度+offset，EEPROM存储）
int8_t offset[4][3] =    {{0, 0, 0}, {0, 0, 0}, { 0, 0, 0}, {0, 0, 0}};

/* 机器人物理尺寸 ---------------------------------------------------------*/
const float length_a = 55;    // 大腿长度（肩部到肘部）
const float length_b = 77.5;  // 小腿长度（肘部到足部）
const float length_c = 27.5;  // 髋关节到肩部的偏移距离（机身安装偏移）
const float length_side = 71; // 机身两侧腿的间距（左右腿横向距离）
const float z_absolute = -28; // 绝对Z坐标（坐下时足部最低位置，参考原点）

/* 运动常量（固定运动参数） ----------------------------------------------------*/
const float z_default = -50;  // 站立时足部Z坐标（负号表示向下，根据原点定义）
const float z_up = -30;       // 抬腿时足部Z坐标（比站立位置高，避免蹭地）
const float z_boot = z_absolute; // 初始化/坐下时Z坐标
const float x_default = 62;   // 站立时足部X坐标（前后方向，远离机身为正）
const float x_offset = 0;     // X方向补偿偏移（微调机身前后平衡）
const float y_start = 0;      // 足部Y坐标起始值（左右方向，机身中线为0）
const float y_step = 40;      // 每步移动的Y方向距离（步长）
const float y_default = x_default; // 站立时足部Y坐标（对称设置）

/* 运动变量（实时变化的运动参数） ----------------------------------------------------*/
volatile float site_now[4][3];    // 每条腿末端的实时坐标（x,y,z），volatile确保多任务可见
volatile float site_expect[4][3]; // 每条腿末端的期望坐标（目标位置）
float temp_speed[4][3];   // 每个轴（x,y,z）的瞬时速度（运动前计算，保证直线运动）
float move_speed = 1.4;   // 基础运动速度（单位：坐标单位/15ms，与Task2周期匹配）
float speed_multiple = 1; // 速度倍数（用于微调整体速度）
const float spot_turn_speed = 4;  // 原地转弯速度（比直线运动快）
const float leg_move_speed = 8;   // 抬腿运动速度（较快，避免停留）
const float body_move_speed = 6;  // 机身平移速度（较慢，保证稳定）
const float stand_seat_speed = 1; // 站立/坐下速度（最慢，保证平稳）
volatile int rest_counter;      // 休息计数器（每0.02秒+1，用于自动休息逻辑，当前未启用）
const float KEEP = 255;         // 坐标保持常量：set_site时用KEEP表示“不改变该轴坐标”
const float pi = 3.1415926;     // 圆周率（角度与弧度转换用）

/* 转弯常量（原地转弯的几何计算参数） --------------------------------------------------------*/
// 转弯时的几何辅助参数（基于机器人尺寸计算，确保转弯时腿部不碰撞）
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
const float turn_x1 = (temp_a - length_side) / 2;  // 转弯时腿1的X目标坐标
const float turn_y1 = y_start + y_step / 2;        // 转弯时腿1的Y目标坐标
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha); // 转弯时腿0的X目标坐标
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side; // 转弯时腿0的Y目标坐标

// 状态与微调变量
// 单个舵机微调变量, 用于在polar_to_servo中进行姿态微调
int FRFoot = 0, FRElbow = 0, FRShdr = 0;
int FLFoot = 0, FLElbow = 0, FLShdr = 0;
int RRFoot = 0, RRElbow = 0, RRShdr = 0;
int RLFoot = 0, RLElbow = 0, RLShdr = 0;

// 超声波避障变量：distance=测量距离（cm），trig_echo=超声波引脚
float       distance;
const int   trig_echo = 23;

// 函数声明
/* 舵机控制函数 ------------------------------------------------------------*/
void servo_celebration(void);  // 舵机初始化（从EEPROM读取偏移量）
void servo_attach(void);       // 挂载所有舵机
void servo_detach(void);       // 卸载所有舵机

/* 存储函数 --------------------------------------------------------------------*/
void writeKeyValue(int8_t key, int8_t value); // 写入EEPROM键值对
int8_t readKeyValue(int8_t key);              // 读取EEPROM键值对

/* 任务与运动函数 --------------------------------------------------------------------*/
void Task1(void *pvParameters); // 任务1 (Core0): 自主运动
void Task2(void *pvParameters); // 任务2（Core1）：舵机服务（实时更新角度）
bool Avoid(void);              // 超声波避障逻辑
bool is_stand(void);           // 判断机器人是否处于站立状态
void sit(void);                // 机器人坐下
void stand(void);              // 机器人站立
void b_init(void);             // 机器人身体初始化
void turn_left(unsigned int step);  // 原地左转
void turn_right(unsigned int step); // 原地右转
void step_forward(unsigned int step); // 前进
void step_back(unsigned int step);    // 后退
void body_left(int i);         // 机身向左平移
void body_right(int i);        // 机身向右平移
void hand_wave(int i);         // 挥手
void hand_shake(int i);        // 握手
void head_up(int i);           // 抬头
void head_down(int i);         // 低头
void body_dance(int i);        // 舞蹈

/* 坐标与角度转换函数 --------------------------------------------------------------------*/
void servo_service(void);      // 舵机核心服务（更新实时坐标→计算舵机角度→写入舵机）
void set_site(int leg, float x, float y, float z); // 设置单腿期望坐标与速度
void wait_reach(int leg);      // 等待单条腿到达期望坐标
void wait_all_reach(void);     // 等待所有腿到达期望坐标
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z); // 直角坐标→舵机角度
void polar_to_servo(int leg, float alpha, float beta, float gamma); // 舵机角度→舵机脉宽（加偏移量后写入）


void setup(){
  /* 看门狗配置：防止程序卡死，超时10秒重启 */
  rtc_wdt_protect_off(); // 关闭看门狗写保护
  rtc_wdt_enable();      // 启用看门狗
  rtc_wdt_feed();        // 喂狗
  rtc_wdt_set_time(RTC_WDT_STAGE0, 1000*10); // 设置超时时间：10秒

  /* 串口初始化：波特率115200，用于状态输出 */
  Serial.begin(115200);
  delay(1000); // 等待串口稳定
  Serial.println("\n四足机器人自主控制系统");
  
  Serial.println("机器人开始初始化...");
  
  /* 初始化腿部默认坐标：开机时所有腿到“坐下”位置 */
  set_site(0, x_default - x_offset, y_start + y_step, z_boot); // 右前腿
  set_site(1, x_default - x_offset, y_start + y_step, z_boot); // 右后腿
  set_site(2, x_default + x_offset, y_start, z_boot);          // 左前腿
  set_site(3, x_default + x_offset, y_start, z_boot);          // 左后腿
  // 初始化实时坐标=期望坐标
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      site_now[i][j] = site_expect[i][j];
    }
  }
  
  /* EEPROM初始化 */
  EEPROM.begin(EEPROM_SIZE);
  
  /* 舵机初始化 */
  Serial.println("舵机服务启动...");
  servo_celebration(); // 从EEPROM读取偏移量
  servo_attach();      // 挂载所有舵机
  Serial.println("舵机初始化完成。");
  Serial.println("机器人初始化完成。");

  /* 关闭低电压检测：避免电池电压波动导致无限重启 */
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  /* 多核心任务创建 */
  // Core0（PRO_CPU）：运行Task1（自主运动）
  xTaskCreatePinnedToCore(Task1, "Task1", 10000, NULL, 1, NULL,  0);
  // Core1（APP_CPU）：运行Task2（舵机服务，高实时性）
  xTaskCreatePinnedToCore(Task2, "Task2", 10000, NULL, 1, NULL,  1);
  
  delay(2000); // 等待任务启动稳定
  Serial.println("\n机器人自主运行中...");
}

//舵机初始化（读取 EEPROM 偏移量）
void servo_celebration(void)
{
  int z = 0; 
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      int8_t val = EEPROM.read(z); 
      offset[i][j] = val;          
      z++;                         
      delay(10); 
    }
  }
}
//挂载所有舵机
void servo_attach(void)
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      servo[i][j].attach(servo_pin[i][j], 500, 2500);
      delay(20);
      servo[i][j].write(90 + offset[i][j]);
      delay(20);
    }
  }
}
//卸载所有舵机
void servo_detach(void)
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      servo[i][j].detach();
      delay(20);
    }
  }
}

//将校准值写入 EEPROM
void writeKeyValue(int8_t key, int8_t value)
{
  EEPROM.write(key, value);
  EEPROM.commit();
}
//从 EEPROM 读取校准值
int8_t readKeyValue(int8_t key)
{
  int8_t value = EEPROM.read(key);
  return value;
}


void loop()
{
  rtc_wdt_feed(); // 喂狗
  vTaskDelay(100); // 主循环延时，降低CPU占用
}


//Task1（Core0）：自主运动逻辑
void Task1(void *pvParameters)
{
  (void) pvParameters;
  // 等待初始化稳定
  vTaskDelay(5000 / portTICK_PERIOD_MS); 
  
  Serial.println("Task1: 开始自主运动...");
  // 首先站立
  stand();
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  while (1) // 任务循环
  {
      step_forward(40);
      delay(500);
      turn_left(2);
      delay(500);
      step_forward(25);
      delay(500);
      turn_right(1);
      delay(500);
      step_forward(15);
      delay(500);
      turn_right(1);
      delay(500);
      step_forward(15);
      delay(500);
      turn_right(1);
      delay(500);
      step_forward(40);
      delay(500);
      
      while(1){
         vTaskDelay(1000 / portTICK_PERIOD_MS); // 任务完成，空闲循环
      }
  }
}

//Task2（Core1）：舵机核心服务（高实时性）
void Task2(void *pvParameters)
{
  while (1) // 任务循环
  {
    rtc_wdt_feed(); // 喂狗
    vTaskDelay(15); // 15ms周期
    servo_service(); // 更新舵机角度
  }
}

//自动避障控制功能
bool Avoid(void)
{
  while (1)
  {
    vTaskDelay(10);
    pinMode(trig_echo, OUTPUT);                      
    digitalWrite(trig_echo, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_echo, LOW);                    
    pinMode(trig_echo, INPUT);                       
    distance  = pulseIn(trig_echo, HIGH);            
    Serial.print(distance);
    distance  = distance * 340 / 2 / 10000;          
    Serial.print("距离: ");
    Serial.print(distance);
    Serial.println("厘米");                            
    pinMode(trig_echo, OUTPUT);  
    return (distance < 40) ? true : false;                   
  }
}

//运动控制核心函数
/*是否站立---------------------------------------------------------------------------*/
bool is_stand(void)
{
  if (site_now[0][2] == z_default)
    return true;
  else
    return false;
}
/*坐下阻塞函数---------------------------------------------------------------------------*/
void sit(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_boot);
  }
  wait_all_reach();
}
/*站立阻塞函数---------------------------------------------------------------------------*/
void stand(void)
{
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++)
  {
    set_site(leg, KEEP, KEEP, z_default);
  }
  wait_all_reach();
}
/*身体初始化阻塞函数---------------------------------------------------------------------------*/
void b_init(void)
{
  set_site(0, x_default, y_default, z_default);
  set_site(1, x_default, y_default, z_default);
  set_site(2, x_default, y_default, z_default);
  set_site(3, x_default, y_default, z_default);
  wait_all_reach();
}
//原地左转
void turn_left(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&1 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();
      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start, z_up);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&2 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();
      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();
      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();
      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}
//原地右转
void turn_right(unsigned int step)
{
  move_speed = spot_turn_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&0 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();
      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();
      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&3 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();
      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();
      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}
//前进
void step_forward(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[2][1] == y_start)
    {
      //leg 2&1 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();
      move_speed = body_move_speed;
      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();
      move_speed = leg_move_speed;
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 0&3 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();
      move_speed = body_move_speed;
      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();
      move_speed = leg_move_speed;
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}
//后退
void step_back(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    if (site_now[3][1] == y_start)
    {
      //leg 3&0 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();
      move_speed = body_move_speed;
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();
      move_speed = leg_move_speed;
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
    else
    {
      //leg 1&2 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();
      move_speed = body_move_speed;
      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
      move_speed = leg_move_speed;
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

//机身左移
void body_left(int i)
{
  set_site(0, site_now[0][0] + i, KEEP, KEEP);
  set_site(1, site_now[1][0] + i, KEEP, KEEP);
  set_site(2, site_now[2][0] - i, KEEP, KEEP);
  set_site(3, site_now[3][0] - i, KEEP, KEEP);
  wait_all_reach();
}
//机身右移
void body_right(int i)
{
  set_site(0, site_now[0][0] - i, KEEP, KEEP);
  set_site(1, site_now[1][0] - i, KEEP, KEEP);
  set_site(2, site_now[2][0] + i, KEEP, KEEP);
  set_site(3, site_now[3][0] + i, KEEP, KEEP);
  wait_all_reach();
}
//挥手
void hand_wave(int i)
{
  float x_tmp, y_tmp, z_tmp;
  move_speed = 1;

  if (site_now[3][1] == y_start)
  {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, turn_x1, turn_y1, 50);
      wait_all_reach(); 
      set_site(2, turn_x0, turn_y0, 50);
      wait_all_reach(); 
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15); 
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(0, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}
//握手
void hand_shake(int i)
{
  float x_tmp, y_tmp, z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start)
  {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(2, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(2, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    body_left(15); 
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++)
    {
      set_site(0, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(0, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}
//抬头
void head_up(int i)
{
  set_site(0, KEEP, KEEP, site_now[0][2] - i);
  set_site(1, KEEP, KEEP, site_now[1][2] + i);
  set_site(2, KEEP, KEEP, site_now[2][2] - i);
  set_site(3, KEEP, KEEP, site_now[3][2] + i);
  wait_all_reach();
}
//低头
void head_down(int i)
{
  set_site(0, KEEP, KEEP, site_now[0][2] + i);
  set_site(1, KEEP, KEEP, site_now[1][2] - i);
  set_site(2, KEEP, KEEP, site_now[2][2] + i);
  set_site(3, KEEP, KEEP, site_now[3][2] - i);
  wait_all_reach();
}
//舞蹈
void body_dance(int i)
{
  float body_dance_speed = 2;
  sit();
  move_speed = 1;
  set_site(0, x_default, y_default, KEEP);
  set_site(1, x_default, y_default, KEEP);
  set_site(2, x_default, y_default, KEEP);
  set_site(3, x_default, y_default, KEEP);
  wait_all_reach();
  set_site(0, x_default, y_default, z_default - 20);
  set_site(1, x_default, y_default, z_default - 20);
  set_site(2, x_default, y_default, z_default - 20);
  set_site(3, x_default, y_default, z_default - 20);
  wait_all_reach();
  move_speed = body_dance_speed;
  head_up(30);
  for (int j = 0; j < i; j++)
  {
    if (j > i / 4)
      move_speed = body_dance_speed * 2;
    if (j > i / 2)
      move_speed = body_dance_speed * 3;

    set_site(0, KEEP, y_default - 20, KEEP); 
    set_site(1, KEEP, y_default + 20, KEEP); 
    set_site(2, KEEP, y_default - 20, KEEP); 
    set_site(3, KEEP, y_default + 20, KEEP); 
    wait_all_reach();
    set_site(0, KEEP, y_default + 20, KEEP); 
    set_site(1, KEEP, y_default - 20, KEEP); 
    set_site(2, KEEP, y_default + 20, KEEP); 
    set_site(3, KEEP, y_default - 20, KEEP); 
    wait_all_reach();
  }
  move_speed = body_dance_speed;
  head_down(30);
}

//舵机核心服务函数
void servo_service(void)
{
  static float alpha, beta, gamma;
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
        site_now[i][j] += temp_speed[i][j];
      else
        site_now[i][j] = site_expect[i][j];
    }
    cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
    polar_to_servo(i, alpha, beta, gamma);
  }
  rest_counter++;
  vTaskDelay(10);
}

//设置单腿期望坐标
void set_site(int leg, float x, float y, float z)
{
  float length_x = 0, length_y = 0, length_z = 0;

  if (x != KEEP)
    length_x = x - site_now[leg][0]; 
  if (y != KEEP)
    length_y = y - site_now[leg][1]; 
  if (z != KEEP)
    length_z = z - site_now[leg][2]; 

  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
  temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
  temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;

  if (x != KEEP)
    site_expect[leg][0] = x;
  if (y != KEEP)
    site_expect[leg][1] = y;
  if (z != KEEP)
    site_expect[leg][2] = z;
}

//等待单腿到达
void wait_reach(int leg)
{
  while (1)
  {
    if (site_now[leg][0] == site_expect[leg][0])
      if (site_now[leg][1] == site_expect[leg][1])
        if (site_now[leg][2] == site_expect[leg][2])
          break; 
    vTaskDelay(1);
  }
}
//等待所有腿到达
void wait_all_reach(void)
{
  for (int i = 0; i < 4; i++)
    wait_reach(i);
}
//直角坐标转舵机角度
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z)
{
  float v, w;
  w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
  v = w - length_c;
  alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
  beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b);
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  alpha = alpha / pi * 180;
  beta = beta / pi * 180;
  gamma = gamma / pi * 180;
}

//舵机角度转舵机脉宽
void polar_to_servo(int leg, float alpha, float beta, float gamma)
{
  if (leg == 0) // 右前腿
  {
    alpha = 90 - alpha - FRElbow; 
    beta = beta - FRFoot;         
    gamma = 90 + gamma + FRShdr;  
  }
  else if (leg == 1) // 右后腿
  {
    alpha += 90 - RRElbow;         
    beta = 180 - beta + RRFoot;   
    gamma = 90 - gamma - RRShdr;  
  }
  else if (leg == 2) // 左前腿
  {
    alpha += 90 + FLElbow;        
    beta = 180 - beta + FLFoot;   
    gamma = 90 - gamma - FLShdr;  
  }
  else if (leg == 3) // 左后腿
  {
    alpha = 90 - alpha - RLElbow; 
    beta = beta - RLFoot;         
    gamma = 90 + gamma + RLShdr;  
  }

  servo[leg][0].write(alpha + offset[leg][0]); 
  servo[leg][1].write(beta + offset[leg][1]);  
  servo[leg][2].write(gamma + offset[leg][2]); 
}
