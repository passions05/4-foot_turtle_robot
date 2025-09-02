//192.168.134.95
// Arduino核心库
#include <Arduino.h>
// ESP32系统寄存器库（用于关闭低电压检测、看门狗配置）
#include <soc/soc.h>
#include <soc/rtc_cntl_reg.h>
#include <rtc_wdt.h> // 看门狗控制（防止程序卡死）
// I2C通信库（如需连接I2C传感器，如加速度计）
#include <Wire.h>
// WiFi相关库（热点创建、TCP服务器）
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h> // 本地域名解析（可选）
// SPI与SD卡库（如需存储日志）
#include <SPI.h>
#include <SD.h>
#include <WiFiAP.h> // WiFi热点模式专用库
// ESP32舵机控制库（驱动12个舵机）
#include <ESP32Servo.h>
// 蓝牙串口库（如需蓝牙控制，当前代码未启用）
#include <BluetoothSerial.h>
// C++容器库（用于键值对存储，当前代码未直接使用）
#include <map>
// EEPROM库（存储舵机校准偏移量，掉电不丢失）
#include "EEPROM.h"



// I2C设备地址（如连接I2C传感器，当前代码未使用）
#define Addr 0x1C
// I2C引脚定义（当前代码未实际用于I2C，仅预留）
#define SDA 18
#define SCL 19

// EEPROM存储大小（64字节，足够存储12个舵机的int8_t偏移量）
#define EEPROM_SIZE 64

// 舵机对象数组（12个舵机，备用定义，实际用二维数组servo[4][3]）
Servo servo1[12];
// 备用舵机引脚定义（与下方二维数组servo_pin对应，二选一）
const int servo1_pin[12] = {18,  5, 19, 2,  4, 15, 33, 25, 32, 14, 27, 13};

/* 舵机核心定义 --------------------------------------------------------------------*/
// 舵机对象二维数组：servo[腿序号][舵机序号]，4条腿×每条3个舵机（肩、肘、足）
Servo servo[4][3];
// 舵机引脚二维数组：按“腿序号-舵机功能”定义
// 腿序号：0=右前腿，1=右后腿，2=左前腿，3=左后腿；舵机序号：0=肩部，1=肘部，2=足部
const int servo_pin[4][3] = {{18,  5,  19}, {  2,  4, 15}, {33, 25,  32}, {14,  27, 13}};

// 舵机角度偏移量（校准用，每个舵机的实际角度=计算角度+offset，EEPROM存储）
int8_t offset[4][3] =    {{0, 0, 0}, {0, 0, 0}, { 0, 0, 0}, {0, 0, 0}};

// 舵机编号映射数组（用于Web校准功能，将Web传来的键值映射到实际舵机索引）
int mapM[16] = {1, 0, 2, 5, 3, 4, 0, 0, 0, 0, 7, 6, 8, 11, 9, 10};




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



//调试与状态变量
// 单个舵机角度变量（备用，实际用offset+计算角度，可用于调试）
int FRFoot = 0;  // 右前腿足部
int FRElbow = 0; // 右前腿肘部
int FRShdr = 0;  // 右前腿肩部
int FLFoot = 0;  // 左前腿足部
int FLElbow = 0; // 左前腿肘部
int FLShdr = 0;  // 左前腿肩部
int RRFoot = 0;  // 右后腿足部
int RRElbow = 0; // 右后腿肘部
int RRShdr = 0;  // 右后腿肩部
int RLFoot = 0;  // 左后腿足部
int RLElbow = 0; // 左后腿肘部
int RLShdr = 0;  // 左后腿肩部

int abc = 0; // 备用调试变量（未使用）

// 调试模式开关：debug_mode=true时启用详细日志，auto_demo=true时自动循环演示动作
bool debug_mode = false;
bool auto_demo = false;

// 时间变量（用于计算周期，未实际使用）
static uint32_t currentTime = 0;
static uint16_t previousTime = 0;
uint16_t cycleTime = 0;

// 超声波避障变量：distance=测量距离（cm），trig_echo=超声波引脚（Trig与Echo复用，需硬件支持）
float       distance;
const int   trig_echo = 23;

// 动作状态与串口输入变量：act=当前动作编号，val=串口接收字符
int act = 1;
char val;

// WiFi服务器变量：header=存储Web请求头，ssid=热点名称，password=热点密码
String header;
const char *ssid = "realme GT5 Pro"; // 自定义WiFi热点名称
const char *password = "b3ypj27m";// 自定义WiFi热点密码

// 任务函数声明（用于多核心分配）
void Task1code( void *pvParameters );
void Task2code( void *pvParameters );
//wifi在core0，其他在core1；1为大核
WiFiServer server(8080);  // 创建TCP服务器，端口8080
WiFiClient client; 




// 函数声明
/* 菜单与调试函数 --------------------------------------------------------------------*/
void printMainMenu();          // 打印串口调试主菜单
void debugLegPosition();       // 调试单条腿的位置（输入X/Y/Z坐标）
void debugLegAngles();         // 调试单条腿的舵机角度（直接输入0-180度）
void showCurrentPositions();   // 显示所有腿的实时/期望坐标
void showServoAngles();        // 显示所有舵机的计算角度与偏移量
void handleSerialInput();      // 处理串口输入指令（执行对应动作）

/* 舵机控制函数 --------------------------------------------------------------------*/
void servo_attach_single_leg(int leg); // 挂载指定单条腿的舵机
void servo_detach_single_leg(int leg); // 卸载指定单条腿的舵机
void servo_detach_all();       // 卸载所有舵机（断电节能）
void servo_attach_all();       // 挂载所有舵机（准备运动）
void servo_celebration(void);  // 舵机初始化（从EEPROM读取偏移量）
void servo_attach(void);       // 挂载所有舵机（基础函数）
void servo_detach(void);       // 卸载所有舵机（基础函数）

/* 存储与校准函数 --------------------------------------------------------------------*/
void writeKeyValue(int8_t key, int8_t value); // 写入EEPROM键值对（存储偏移量）
int8_t readKeyValue(int8_t key);              // 读取EEPROM键值对（读取偏移量）
void handleSave();                           // 处理Web请求的舵机校准值保存

/* 任务与运动函数 --------------------------------------------------------------------*/
void Task1(void *pvParameters); // 任务1（Core0）：自动演示动作循环
void Task2(void *pvParameters); // 任务2（Core1）：舵机服务（实时更新角度）
bool Avoid(void);              // 超声波避障逻辑（距离<40cm时转弯）
bool is_stand(void);           // 判断机器人是否处于站立状态
void sit(void);                // 机器人坐下（阻塞函数，完成后返回）
void stand(void);              // 机器人站立（阻塞函数，完成后返回）
void b_init(void);             // 机器人身体初始化（回归默认坐标）
void turn_left(unsigned int step);  // 原地左转（step=转弯步数）
void turn_right(unsigned int step); // 原地右转（step=转弯步数）
void step_forward(unsigned int step); // 前进（step=步数）
void step_back(unsigned int step);    // 后退（step=步数）
void body_left(int i);         // 机身向左平移（i=平移量）
void body_right(int i);        // 机身向右平移（i=平移量）
void hand_wave(int i);         // 挥手动作（i=挥手次数，用前腿模拟）
void hand_shake(int i);        // 握手动作（i=握手次数，用前腿模拟）
void head_up(int i);           // 头部抬起（模拟，通过调整前后腿高度实现）
void head_down(int i);         // 头部低下（模拟，通过调整前后腿高度实现）
void body_dance(int i);        // 舞蹈动作（i=循环次数，多动作组合）

/* 坐标与角度转换函数 --------------------------------------------------------------------*/
void servo_service(void);      // 舵机核心服务（更新实时坐标→计算舵机角度→写入舵机）
void set_site(int leg, float x, float y, float z); // 设置单腿期望坐标与速度
void wait_reach(int leg);      // 等待单条腿到达期望坐标（阻塞）
void wait_all_reach(void);     // 等待所有腿到达期望坐标（阻塞）
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z); // 直角坐标→极坐标（舵机角度）
void polar_to_servo(int leg, float alpha, float beta, float gamma); // 极坐标→舵机角度（加偏移量后写入）





void setup(){
  /* 看门狗配置：防止程序卡死，超时10秒重启 */
  rtc_wdt_protect_off(); // 关闭看门狗写保护（允许修改配置）
  rtc_wdt_enable();      // 启用看门狗
  rtc_wdt_feed();        // 喂狗（重置超时计时器）
  rtc_wdt_set_time(RTC_WDT_STAGE0, 1000*10); // 设置超时时间：10秒（单位ms）

  /* 串口初始化：波特率115200，用于调试与指令输入 */
  Serial.begin(115200);
  delay(1000); // 等待串口稳定
  Serial.println("\n========================================");
  Serial.println("        四足机器人调试系统");
  Serial.println("========================================");
  
  Serial.println("机器人开始初始化...");
  
  /* 初始化腿部默认坐标：开机时所有腿到“坐下”位置 */
  set_site(0, x_default - x_offset, y_start + y_step, z_boot); // 右前腿
  set_site(1, x_default - x_offset, y_start + y_step, z_boot); // 右后腿
  set_site(2, x_default + x_offset, y_start, z_boot);          // 左前腿
  set_site(3, x_default + x_offset, y_start, z_boot);          // 左后腿
  // 初始化实时坐标=期望坐标（开机无运动偏差）
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      site_now[i][j] = site_expect[i][j];
    }
  }
  
  /* EEPROM初始化：开启64字节存储（用于舵机偏移量） */
  EEPROM.begin(64);
  
  /* 舵机初始化：读取EEPROM偏移量，挂载所有舵机 */
  Serial.println("舵机服务已启动");
  servo_celebration(); // 从EEPROM读取偏移量
  servo_attach();      // 挂载所有舵机并设置初始角度（90+偏移量）
  Serial.println("舵机已初始化");
  Serial.println("机器人初始化完成");

  Serial.print("连接wifi: ");
  Serial.println(ssid); 
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi连接成功！");
  Serial.print("ESP32 IP地址: ");
  Serial.println(WiFi.localIP());  // 打印ESP32的IP（电脑需要用此IP连接）

  // 启动TCP服务器
  server.begin();
  Serial.println("TCP服务器启动，等待客户端连接...");
  Serial.println("可用指令：0-初始化,1-前进,2-后退,3-站立,4-坐下,5-左转,6-右转,9-挥手,a-握手,b-舞蹈");


  for (int i = 0; i < 12; i++)
  {
    int8_t val = EEPROM.read(i);
    Serial.print(val); 
    Serial.print(" ");
  }
  Serial.println();

  /* 关闭低电压检测：避免电池电压波动导致无限重启 */
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  /* 多核心任务创建：ESP32有2个核心，分工提升性能 */
  // xTaskCreatePinnedToCore(任务函数, 任务名, 栈大小, 参数, 优先级, 任务句柄, 核心号)
  // Core0（PRO_CPU）：运行Task1（自动演示，低实时性）
  xTaskCreatePinnedToCore(Task1, "Task1", 10000, NULL, 1, NULL,  0);  
  // Core1（APP_CPU）：运行Task2（舵机服务，高实时性）
  xTaskCreatePinnedToCore(Task2, "Task2", 10000, NULL, 1, NULL,  1);
  
  delay(2000); // 等待任务启动稳定
  printMainMenu(); // 打印调试主菜单

}





//菜单与调试相关函数
//1.打印调试主菜单
void printMainMenu() {
  Serial.println("\n========================================");
  Serial.println("           调试主菜单");
  Serial.println("========================================");
  Serial.println("0 - 初始化/重置机器人");
  Serial.println("1 - 前进 (5步)");
  Serial.println("2 - 后退 (5步)");
  Serial.println("3 - 站立");
  Serial.println("4 - 坐下");
  Serial.println("5 - 左转 (5步)");
  Serial.println("6 - 右转 (5步)");
  Serial.println("7 - 调试单条腿位置 (坐标)");
  Serial.println("8 - 调试单条腿角度 (直接舵机)");
  Serial.println("9 - 挥手");
  Serial.println("a - 握手");
  Serial.println("b - 身体舞蹈");
  Serial.println("c - 显示当前位置");
  Serial.println("d - 显示舵机角度");
  Serial.println("e - 切换自动演示 (当前: " + String(auto_demo ? "开启" : "关闭") + ")");
  Serial.println("f - 舵机校准模式");
  Serial.println("========================================");
  Serial.println("请输入您的选择：");
}
//2.调试单条腿位置（输入 X/Y/Z 坐标）
void debugLegPosition() {
  Serial.println("调试单条腿位置模式");
  Serial.println("正在断开所有舵机...");
  servo_detach_all();
  delay(500);
  
  Serial.println("请选择要调试的腿号 (0-3):");
  Serial.println("0 - 右前腿");
  Serial.println("1 - 右后腿"); 
  Serial.println("2 - 左前腿");
  Serial.println("3 - 左后腿");
  
  while (!Serial.available()) {
    vTaskDelay(10);
  }
  
  int leg = Serial.parseInt();
  Serial.read(); // 清空缓冲区
  
  if (leg < 0 || leg > 3) {
    Serial.println("无效的腿号！返回主菜单。");
    servo_attach_all(); // 重新上电所有舵机
    printMainMenu();
    return;
  }
  
  // 只给选中的腿上电
  servo_attach_single_leg(leg);
  delay(500);
  
  Serial.println("已选择腿号: " + String(leg));
  Serial.println("请输入目标位置 (x,y,z):");
  Serial.println("格式: x y z (用空格分隔)");
  Serial.println("建议范围: x(30-90), y(-40-80), z(-80--20)");
  
  while (Serial.available() < 3) {
    vTaskDelay(10);
  }
  
  float x = Serial.parseFloat();
  float y = Serial.parseFloat();
  float z = Serial.parseFloat();
  
  Serial.println("移动腿 " + String(leg) + " 到位置:");
  Serial.println("X: " + String(x) + ", Y: " + String(y) + ", Z: " + String(z));
  
  // 保存其他腿的当前位置，确保它们保持不动
  for (int i = 0; i < 4; i++) {
    if (i != leg) {
      set_site(i, KEEP, KEEP, KEEP); // 其他腿保持当前位置
    }
  }
  
  // 使用set_site函数设置目标腿的位置
  set_site(leg, x, y, z);
  
  Serial.println("正在移动到目标位置...");
  // 等待所有腿到达期望位置（实际上只有选中的腿会移动）
  wait_all_reach();
  
  Serial.println("位置已到达！");
  Serial.println("腿 " + String(leg) + " 当前状态:");
  Serial.println("  当前位置 - X: " + String(site_now[leg][0]) + ", Y: " + String(site_now[leg][1]) + ", Z: " + String(site_now[leg][2]));
  Serial.println("  期望位置 - X: " + String(site_expect[leg][0]) + ", Y: " + String(site_expect[leg][1]) + ", Z: " + String(site_expect[leg][2]));
  
  // 计算并显示对应的舵机角度（用于参考）
  float alpha, beta, gamma;
  cartesian_to_polar(alpha, beta, gamma, site_now[leg][0], site_now[leg][1], site_now[leg][2]);
  Serial.println("  计算角度 - Alpha: " + String(alpha) + "°, Beta: " + String(beta) + "°, Gamma: " + String(gamma) + "°");
  
  Serial.println("按 'm' 键返回主菜单 (将重新给所有舵机上电)");
  
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'm' || c == 'M') {
        servo_detach_single_leg(leg);
        delay(500);
        servo_attach_all();
        printMainMenu();
        break;
      }
    }
    vTaskDelay(10);
  }
}
//3.调试单条腿位置（输入 X/Y/Z 坐标）
void debugLegAngles() {
  Serial.println("调试单条腿角度模式");
  Serial.println("正在断开所有舵机...");
  servo_detach_all();
  delay(500);
  
  Serial.println("请选择要调试的腿号 (0-3):");
  Serial.println("0 - 右前腿");
  Serial.println("1 - 右后腿");
  Serial.println("2 - 左前腿"); 
  Serial.println("3 - 左后腿");
  
  while (!Serial.available()) {
    vTaskDelay(10);
  }
  
  int leg = Serial.parseInt();
  Serial.read(); // 清空缓冲区
  
  if (leg < 0 || leg > 3) {
    Serial.println("无效的腿号！返回主菜单。");
    servo_attach_all();
    printMainMenu();
    return;
  }
  
  // 只给选中的腿上电
  servo_attach_single_leg(leg);
  delay(500);
  
  Serial.println("已选择腿号: " + String(leg));
  Serial.println("请输入舵机角度 (肩部 肘部 足部):");
  Serial.println("格式: 角度1 角度2 角度3 (用空格分隔)");
  Serial.println("有效范围: 0-180 度");
  
  while (Serial.available() < 3) {
    vTaskDelay(10);
  }
  
  float angle1 = Serial.parseFloat(); // 肩部
  float angle2 = Serial.parseFloat(); // 肘部  
  float angle3 = Serial.parseFloat(); // 足部
  
  // 验证角度
  if (angle1 < 0 || angle1 > 180 || angle2 < 0 || angle2 > 180 || angle3 < 0 || angle3 > 180) {
    Serial.println("无效角度！必须在0-180度范围内。返回主菜单。");
    servo_detach_single_leg(leg);
    servo_attach_all();
    printMainMenu();
    return;
  }
  
  Serial.println("设置腿 " + String(leg) + " 到角度:");
  Serial.println("肩部: " + String(angle1) + "°, 肘部: " + String(angle2) + "°, 足部: " + String(angle3) + "°");
  
  // 直接设置舵机角度
  servo[leg][0].write(angle1 + offset[leg][0]);
  servo[leg][1].write(angle2 + offset[leg][1]);
  servo[leg][2].write(angle3 + offset[leg][2]);
  
  delay(1000); // 等待舵机到达位置
  
  Serial.println("角度已到达！");
  Serial.println("最终舵机位置:");
  Serial.println("  舵机0 (肩部): " + String(angle1 + offset[leg][0]) + "° (偏移: " + String(offset[leg][0]) + ")");
  Serial.println("  舵机1 (肘部): " + String(angle2 + offset[leg][1]) + "° (偏移: " + String(offset[leg][1]) + ")");
  Serial.println("  舵机2 (足部): " + String(angle3 + offset[leg][2]) + "° (偏移: " + String(offset[leg][2]) + ")");
  Serial.println("按 'm' 键返回主菜单 (将重新给所有舵机上电)");
  
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'm' || c == 'M') {
        servo_detach_single_leg(leg);
        delay(500);
        servo_attach_all();
        printMainMenu();
        break;
      }
    }
    vTaskDelay(10);
  }
}
//4.显示当前腿部坐标
void showCurrentPositions() {
  Serial.println("当前腿部位置:");
  Serial.println("========================================");
  for (int i = 0; i < 4; i++) {
    String legName = "";
    switch(i) {
      case 0: legName = "右前腿"; break;
      case 1: legName = "右后腿"; break;
      case 2: legName = "左前腿"; break;
      case 3: legName = "左后腿"; break;
    }
    Serial.println("腿 " + String(i) + " (" + legName + "):");
    Serial.println("  当前: X=" + String(site_now[i][0]) + ", Y=" + String(site_now[i][1]) + ", Z=" + String(site_now[i][2]));
    Serial.println("  期望: X=" + String(site_expect[i][0]) + ", Y=" + String(site_expect[i][1]) + ", Z=" + String(site_expect[i][2]));
  }
  Serial.println("========================================");
}
//5.显示当前腿部坐标
void showServoAngles() {
  Serial.println("当前舵机角度:");
  Serial.println("========================================");
  for (int i = 0; i < 4; i++) {
    String legName = "";
    switch(i) {
      case 0: legName = "右前腿"; break;
      case 1: legName = "右后腿"; break;  
      case 2: legName = "左前腿"; break;
      case 3: legName = "左后腿"; break;
    }
    
    // 从位置计算当前角度
    float alpha, beta, gamma;
    cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
    
    Serial.println("腿 " + String(i) + " (" + legName + "):");
    Serial.println("  原始角度 - Alpha: " + String(alpha) + "°, Beta: " + String(beta) + "°, Gamma: " + String(gamma) + "°");
    Serial.println("  偏移值: " + String(offset[i][0]) + ", " + String(offset[i][1]) + ", " + String(offset[i][2]));
  }
  Serial.println("========================================");
}
//6.处理串口输入指令
void handleSerialInput() {
  if (Serial.available()) {
    String input = Serial.readString();
    input.trim();
    
    if (input == "0") {
      Serial.println("初始化机器人...");
      servo_celebration();
      servo_attach();
      stand();
      Serial.println("机器人已初始化并站立");
      
    } else if (input == "1") {
      Serial.println("前进 (5步)");
      step_forward(5);
      Serial.println("前进运动完成");
      
    } else if (input == "2") {
      Serial.println("后退 (5步)");
      step_back(5);
      Serial.println("后退运动完成");
      
    } else if (input == "3") {
      Serial.println("站立中...");
      stand();
      Serial.println("机器人现在站立");
      
    } else if (input == "4") {
      Serial.println("坐下中...");
      sit();
      Serial.println("机器人现在坐下");
      
    } else if (input == "5") {
      Serial.println("左转 (5步)");
      turn_left(5);
      Serial.println("左转完成");
      
    } else if (input == "6") {
      Serial.println("右转 (5步)");
      turn_right(5);
      Serial.println("右转完成");
      
    } else if (input == "7") {
      debugLegPosition();
      return;
      
    } else if (input == "8") {
      debugLegAngles();
      return;
      
    } else if (input == "9") {
      Serial.println("挥手 (3次)");
      hand_wave(3);
      Serial.println("挥手完成");
      
    } else if (input == "a") {
      Serial.println("握手 (3次)");
      hand_shake(3);
      Serial.println("握手完成");
      
    } else if (input == "b") {
      Serial.println("身体舞蹈 (10个周期)");
      body_dance(10);
      Serial.println("身体舞蹈完成");
      
    } else if (input == "c") {
      showCurrentPositions();
      
    } else if (input == "d") {
      showServoAngles();
      
    } else if (input == "e") {
      auto_demo = !auto_demo;
      Serial.println("自动演示已" + String(auto_demo ? "启用" : "禁用"));
      
    } else if (input == "f") {
      Serial.println("舵机校准模式 - 功能待实现");
      
    } else {
      Serial.println("无效输入！请选择有效选项。");
    }
    
    delay(1000);
    printMainMenu();
  }
}
//7.WIFI建立服务端控制
void handleCommand(String cmd) {
  String response;
  
  if (cmd == "0") {
    response = "初始化机器人...\n";
    client.print(response);
    servo_celebration();
    servo_attach();
    stand();
    response = "机器人已初始化并站立\n";
    
  } else if (cmd == "1") {
    response = "前进 (5步)...\n";
    client.print(response);
    step_forward(5);
    response = "前进运动完成\n";
    
  } else if (cmd == "2") {
    response = "后退 (5步)...\n";
    client.print(response);
    step_back(5);
    response = "后退运动完成\n";
    
  } else if (cmd == "3") {
    response = "站立中...\n";
    client.print(response);
    stand();
    response = "机器人现在站立\n";
    
  } else if (cmd == "4") {
    response = "坐下中...\n";
    client.print(response);
    sit();
    response = "机器人现在坐下\n";
    
  } else if (cmd == "5") {
    response = "左转 (2.5步)...\n";
    client.print(response);
    turn_left(2.5);
    response = "左转完成\n";
    
  } else if (cmd == "6") {
    response = "右转 (2.5步)...\n";
    client.print(response);
    turn_right(2.5);
    response = "右转完成\n";
    
  } else if (cmd == "7") {
    response = "左移 (3次)...\n";
    client.print(response);
    hand_wave(3);
    response = "左移完成\n";
    
  } else if (cmd == "8") {
    response = "右移 (3次)...\n";
    client.print(response);
    body_left(3);
    response = "右移完成\n";
    
  } else if (cmd == "9") {
    response = "挥手 (3次)...\n";
    client.print(response);
    body_right(3);
    response = "挥手完成\n";
    
  } else if (cmd == "a") {
    response = "握手 (3次)...\n";
    client.print(response);
    hand_shake(3);
    response = "握手完成\n";
    
  } else if (cmd == "b") {
    response = "身体舞蹈 (10个周期)...\n";
    client.print(response);
    body_dance(10);
    response = "身体舞蹈完成\n";
    
  } else if (cmd == "e") {
    auto_demo = !auto_demo;
    response = "自动演示已" + String(auto_demo ? "启用" : "禁用") + "\n";
    
  } else {
    response = "无效指令！可用指令：0-初始化,1-前进,2-后退,3-站立,4-坐下,5-左转,6-右转,9-挥手,a-握手,b-舞蹈\n";
  }
  
  client.print(response);
  Serial.print(response);
}




//舵机控制相关函数
//1.舵机初始化（读取 EEPROM 偏移量）
void servo_celebration(void)
{
  int z = 0; // 偏移量存储索引（0-11，对应12个舵机）
  // 遍历4条腿×3个舵机，从EEPROM读取偏移量
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      int8_t val = EEPROM.read(z); // 读取第z字节的偏移量
      offset[i][j] = val;          // 赋值给偏移量数组
      z++;                         // 索引+1
      delay(100); // 小延时，避免EEPROM读取过快
    }
  }
}
//2.挂载所有舵机
void servo_attach(void)
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      servo[i][j].attach(servo_pin[i][j], 500, 2500);
      delay(100);
      servo[i][j].write(90 + offset[i][j]);
      Serial.print("偏移值:");
      Serial.println(offset[i][j]);
      delay(20);
    }
  }
}
//3.卸载所有舵机
void servo_detach(void)
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      servo[i][j].detach();
      delay(100);
    }
  }
}
//4.挂载指定单条腿的舵机
void servo_attach_single_leg(int leg)
{
  for (int j = 0; j < 3; j++)
  {
    servo[leg][j].attach(servo_pin[leg][j], 500, 2500);
    delay(100);
    servo[leg][j].write(90 + offset[leg][j]);
    delay(20);
  }
  Serial.println("腿 " + String(leg) + " 舵机已上电");
}
// 5.卸载指定单条腿的舵机
void servo_detach_single_leg(int leg)
{
  for (int j = 0; j < 3; j++)
  {
    servo[leg][j].detach();
    delay(100);
  }
  Serial.println("腿 " + String(leg) + " 舵机已断电");
}
//6.卸载所有舵机（快捷函数）
void servo_detach_all()
{
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      servo[i][j].detach();
      delay(50);
    }
  }
  Serial.println("所有舵机已断电");
}
//7.挂载所有舵机（快捷函数）
void servo_attach_all()
{
  servo_attach();
  Serial.println("所有舵机已上电");
}





//EEPROM相关函数
//将校准值写入 EEPROM
void writeKeyValue(int8_t key, int8_t value)
{
  // 1. 向EEPROM的“key地址”写入“value（校准值）”
  // EEPROM.write(地址, 数据)：ESP32的EEPROM是“模拟EEPROM”（实际存于Flash），写入后需commit才生效
  EEPROM.write(key, value);
  
  // 2. 提交写入操作（关键！）
  // 模拟EEPROM的写入不会立即生效，必须调用commit()将数据刷入Flash，否则掉电后数据丢失
  EEPROM.commit();
}
//从 EEPROM 读取校准值
int8_t readKeyValue(int8_t key)
{
  // 1. 串口打印调试信息：提示正在读取，以及读取的地址（key）
  Serial.println("读取");
  Serial.println(key);
  
  // 2. 从EEPROM的“key地址”读取数据，并赋值给value
  int8_t value = EEPROM.read(key);
  
  // 3. 返回读取到的校准值
  return value;
}
//从web接收设置的校准值
void handleSave()
{
  // 1. 从HTTP请求头（header）中截取key和value的字符串
  // 说明：HTTP请求头格式类似“GET /save?key=05&value=+03 HTTP/1.1”，需按实际请求格式截取
  // 这里假设key在header的第14~16个字符（ substring(14,16)：左闭右开，取索引14、15的字符）
  String key = header.substring(14, 16);
  // 假设value在header的第23~26个字符（取索引23、24、25的字符，可能包含正负号和数字）
  String value = header.substring(23, 26);

  // 2. 将key字符串转换为整数（keyInt：舵机的原始编号，0~15，对应mapM数组的索引）
  int8_t keyInt = key.toInt();
  int8_t valueInt = 0; // 存储最终的校准值（整数）

  // 3. 处理value字符串：过滤掉非数字字符（如HTTP请求中的换行符、空格、符号等）
  // 从字符串末尾向前遍历，找到最后一个数字字符，截取前面的有效部分
  for (int i = value.length() - 1; i >= 0; i--) {
    // 判断字符是否为数字（ASCII码48=0，57=9）
    if (value.charAt(i) >= 48 && value.charAt(i) <= 57) {
      // 截取从开头到该数字字符的子串（有效数字部分）
      value = value.substring(0, i + 1);
      break;
    }
  }

  // 4. 映射舵机编号：用mapM数组将“Web端显示的编号”转换为“实际舵机索引”
  // 说明：mapM数组是之前定义的舵机编号映射表（如mapM[5]=4），解决Web端编号与实际舵机索引不匹配的问题
  keyInt = mapM[keyInt];
  
  // 5. 将处理后的value字符串转换为整数（校准值）
  valueInt = value.toInt();

  // 打印关键信息：实际舵机索引（keyInt）、校准值（value）、完整请求头（header）
  Serial.print("键:");
  Serial.println(keyInt);
  Serial.print("值:");
  Serial.println(value);
  Serial.print(header);

  // 计算腿序号（row）和腿上的舵机位置（col）：
  // 每条腿有3个舵机（肩、肘、足），所以 keyInt / 3 = 腿序号（0~3），keyInt % 3 = 舵机位置（0=肩，1=肘，2=足）
  int row = keyInt / 3;   // 腿序号（0=右前，1=右后，2=左前，3=左后）
  int col = keyInt % 3;   // 舵机在腿上的位置（0=肩，1=肘，2=足）


  // 立即更新该舵机的角度：90（舵机中位） + 新校准值（valueInt）
  // 说明：舵机默认中位是90度，校准值是对中位的偏移（如+5表示比中位多转5度，-3表示少转3度）
  servo[row][col].write(90 + valueInt);
  
  // 更新偏移量数组（offset）：确保后续运动时使用新的校准值
  offset[row][col] = valueInt;


  // 校验校准值范围（-99~124）：避免超出舵机物理角度范围（0~180度）
  if (valueInt >= -99 && valueInt <= 124) 
  {
    // 调用writeKeyValue，将新校准值保存到EEPROM（永久存储）
    writeKeyValue(keyInt, valueInt); 
  }

  // 重新读取EEPROM中的所有校准值（刷新offset数组，确保一致性）
  servo_celebration();
  
  // 小延时（10ms）：确保数据处理完成，避免后续操作冲突
  vTaskDelay(10);
}








void loop()
{
  rtc_wdt_feed(); // 喂狗（每循环一次重置超时，防止重启）
  handleSerialInput(); // 处理串口输入指令（如“1”=前进，“3”=站立）
  vTaskDelay(100); // 延时100ms，降低CPU占用

  if (!client.connected()) {
    client = server.available();  // 接受新连接
    if (client) {
      Serial.println("新客户端已连接！");
      client.println("可用指令：");
      client.println("0-初始化 1-前进 2-后退 3-站立 4-坐下");
      client.println("5-左转 6-右转 7-左移 8-右移 9-挥手 a-握手 b-舞蹈");
      client.println("已连接，请发送指令");
    }
  } 
  if(client.connected()) {
    // 若客户端已连接，检查是否有数据发送
    if (client.available() > 0) {
      String cmd = client.readStringUntil('\n');
      cmd.trim();
      Serial.print("收到指令: ");
      Serial.println(cmd);
      handleCommand(cmd);  // 处理指令
      }
    }
    delay(50);
}








//Task1（Core0）：自动演示逻辑
void Task1(void *pvParameters)
{
  while (1) // 任务循环（永不退出）
  {
    // 当自动演示模式开启且非调试模式时，循环执行动作
    if (auto_demo && !debug_mode) {
      vTaskDelay(100); // 小延时，保证任务调度
      Serial.println("自动演示: 站立");
      stand();         // 执行站立
      delay(2000);     // 延时2秒，观察动作
      
      Serial.println("自动演示: 前进");
      step_forward(5); // 前进5步
      delay(2000);
      
      Serial.println("自动演示: 后退");
      step_back(5);    // 后退5步
      delay(2000);
      
      Serial.println("自动演示: 左转");
      turn_left(5);    // 左转5步
      delay(2000);
      
      Serial.println("自动演示: 右转");
      turn_right(5);   // 右转5步
      delay(2000);
      
      Serial.println("自动演示: 挥手");
      hand_wave(3);    // 挥手3次
      delay(2000);
      
      Serial.println("自动演示: 握手");
      hand_shake(3);   // 握手3次
      delay(2000);  
      
      Serial.println("自动演示: 身体舞蹈");
      body_dance(10);  // 舞蹈10个周期
      delay(2000);    
      
      Serial.println("自动演示: 坐下");
      sit();           // 执行坐下
      delay(5000);     // 延时5秒，结束一轮演示
    } else {
      // 自动演示关闭时，延长休眠时间（降低CPU占用）
      vTaskDelay(1000); 
    }
  }
}
//Task2（Core1）：舵机核心服务（高实时性）
void Task2(void *pvParameters)
{
  while (1) // 任务循环（永不退出）
  {
    rtc_wdt_feed(); // 喂狗（防止超时重启）
    vTaskDelay(15); // 15ms周期（≈66Hz，舵机更新频率，过高会卡顿）
    servo_service(); // 核心：更新舵机角度（实时坐标→角度计算→写入舵机）
  }
}

//自动避障控制功能
bool Avoid(void)
{
  while (1)
  {
    vTaskDelay(10);
    WiFiClient client = server.available();  //监听连入设备
    pinMode(trig_echo, OUTPUT);                      //设置Trig_RX_SCL_I/O为输出

    digitalWrite(trig_echo, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_echo, LOW);                    //Trig_RX_SCL_I/O脚输出10US高电平脉冲触发信号

    pinMode(trig_echo, INPUT);                       //设置Trig_RX_SCL_I/O为输入，接收模块反馈的距离信号

    distance  = pulseIn(trig_echo, HIGH);            //计数接收到的高电平时间
    Serial.print(distance);
    distance  = distance * 340 / 2 / 10000;          //计算距离 1：声速：340M/S  2：实际距离为1/2声速距离 3：计数时钟为1US//温补公式：c=(331.45+0.61t/℃)m•s-1 (其中331.45是在0度）
    Serial.print("距离: ");
    Serial.print(distance);
    Serial.println("厘米");                            //串口输出距离信号
    pinMode(trig_echo, OUTPUT);                      //设置Trig_RX_SCL_I/O为输出，准备下次测量

    if (client)
    {
      String currentLine = "";
      while (client.connected())
      {
        if (client.available())
        {
          char c = client.read();
          header += c;
          if (c == '\n')
          {
            if (currentLine.length() == 0)
            {
              if (header.indexOf("GET /40/on") >= 0) //结束避障
              {
                return true;
              }
              else
              {
                return false;
              }
            }
          }
        }
      }
    }
    delay(30);                                       //单次测离完成后加30mS的延时再进行下次测量。防止近距离测量时，测量到上次余波，导致测量不准确。

    if (distance <= 40)
    {
      stand();
      turn_left(5);
    }
    else
    {
      step_forward(5);
    }
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
//原地左转（阻塞，按步数）
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
//原地右转（阻塞，按步数）
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
//前进动作（阻塞，按步数）
void step_forward(unsigned int step)
{
  move_speed = leg_move_speed;
  while (step-- > 0)
  {
    Serial.println(step);
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
//后退动作（阻塞，按步数）
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

// add by RegisHsu
// 功能：控制机器人机身整体向左平移（不改变朝向）
// 参数 i：平移幅度（单位：坐标单位，需根据机器人实际尺寸调整，i越大平移越远）
void body_left(int i)
{
  // 右前腿：当前X坐标 + i（向右移，补偿机身左移），Y/Z保持不变（KEEP）
  set_site(0, site_now[0][0] + i, KEEP, KEEP);
  // 右后腿：同右前腿逻辑，X+ i
  set_site(1, site_now[1][0] + i, KEEP, KEEP);
  // 左前腿：当前X坐标 - i（向左移，主动实现机身左移），Y/Z保持不变
  set_site(2, site_now[2][0] - i, KEEP, KEEP);
  // 左后腿：同左前腿逻辑，X- i
  set_site(3, site_now[3][0] - i, KEEP, KEEP);
  // 阻塞等待所有腿到达目标位置，确保机身平稳平移（无倾斜）
  wait_all_reach();
}
// 功能：控制机器人机身整体向右平移（不改变朝向），与body_left逻辑对称
// 参数 i：平移幅度（单位：坐标单位，i越大平移越远）
void body_right(int i)
{
  // 右前腿：X- i（向左移，补偿机身右移），Y/Z不变
  set_site(0, site_now[0][0] - i, KEEP, KEEP);
  // 右后腿：同右前腿逻辑，X- i
  set_site(1, site_now[1][0] - i, KEEP, KEEP);
  // 左前腿：X+ i（向右移，主动实现机身右移），Y/Z不变
  set_site(2, site_now[2][0] + i, KEEP, KEEP);
  // 左后腿：同左前腿逻辑，X+ i
  set_site(3, site_now[3][0] + i, KEEP, KEEP);
  // 等待所有腿到位，确保平稳
  wait_all_reach();
}
// 功能：用前腿模拟“挥手”动作（前后摆动前腿），增强机器人互动性
// 参数 i：挥手次数（i=3表示挥手3次）
void hand_wave(int i)
{
  // 临时变量：保存前腿原始坐标（挥手后恢复原位）
  float x_tmp;
  float y_tmp;
  float z_tmp;
  // 降低运动速度（1），避免挥手动作过快、卡顿
  move_speed = 1;

  // 条件判断：根据左后腿（3号腿）的Y坐标判断机器人当前状态（避免动作冲突）
  // site_now[3][1] == y_start 表示左后腿在“初始Y位置”
  if (site_now[3][1] == y_start)
  {
    // 1. 机身先向右移15单位（让左前腿有足够摆动空间，避免碰机身）
    body_right(15);
    // 2. 保存左前腿（2号腿）当前坐标（后续恢复用）
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    // 3. 提高动作速度（用机身平移速度，保证挥手流畅）
    move_speed = body_move_speed;

    // 4. 循环i次：执行“挥手”（左前腿前后摆动）
    for (int j = 0; j < i; j++)
    {
      // 左前腿移到“前摆位置”（turn_x1/turn_y1是之前定义的转弯辅助坐标，此处复用为摆动终点），Z=50（抬升，避免碰地）
      set_site(2, turn_x1, turn_y1, 50);
      wait_all_reach(); // 等待到位
      // 左前腿移到“后摆位置”（turn_x0/turn_y0），Z保持50
      set_site(2, turn_x0, turn_y0, 50);
      wait_all_reach(); // 等待到位
    }

    // 5. 恢复左前腿到原始坐标（挥手后复位）
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    // 6. 恢复运动速度，机身向左移15单位（回到原始位置）
    move_speed = 1;
    body_left(15);
  }
  else
  {
    // 对称逻辑：若左后腿不在初始Y位置，用右前腿（0号腿）挥手
    body_left(15); // 机身先向左移15单位（给右前腿留空间）
    // 保存右前腿原始坐标
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;

    // 循环i次：右前腿前后摆动（挥手）
    for (int j = 0; j < i; j++)
    {
      set_site(0, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(0, turn_x0, turn_y0, 50);
      wait_all_reach();
    }

    // 恢复右前腿和机身位置
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}
// 功能：用前腿模拟“握手”动作（上下摆动前腿，模拟握住后上下晃动）
// 参数 i：握手次数（i=3表示握手3次）
void hand_shake(int i)
{
  // 临时变量：保存前腿原始坐标（握手后恢复）
  float x_tmp;
  float y_tmp;
  float z_tmp;
  // 降低初始速度，避免动作生硬
  move_speed = 1;

  // 条件判断：根据左后腿Y坐标判断状态，选择左前腿/右前腿执行动作
  if (site_now[3][1] == y_start)
  {
    // 1. 机身右移15单位（给左前腿留空间）
    body_right(15);
    // 2. 保存左前腿原始坐标
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    // 3. 提高动作速度，保证握手流畅
    move_speed = body_move_speed;

    // 4. 循环i次：执行“握手”（左前腿上下摆动）
    for (int j = 0; j < i; j++)
    {
      // 左前腿移到“握起位置”：X=x_default-30（向前伸，靠近人），Y=y_start+2*y_step（固定前后位置），Z=55（抬起）
      set_site(2, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      // 左前腿移到“放下位置”：X/Y不变，Z=10（降低，模拟握手晃动）
      set_site(2, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }

    // 5. 恢复左前腿和机身位置
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  }
  else
  {
    // 对称逻辑：用右前腿握手
    body_left(15); // 机身左移15单位
    // 保存右前腿原始坐标
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;

    // 循环i次：右前腿上下摆动（握手）
    for (int j = 0; j < i; j++)
    {
      set_site(0, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(0, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }

    // 恢复右前腿和机身位置
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}
// 功能：模拟机器人“抬头”姿态（通过调整前后腿高度差实现）
// 参数 i：抬头幅度（i越大，前后腿高度差越大，抬头越明显）
void head_up(int i)
{
  // 右前腿：Z坐标 = 当前Z - i（抬升前腿，让前部变高），X/Y不变
  set_site(0, KEEP, KEEP, site_now[0][2] - i);
  // 右后腿：Z坐标 = 当前Z + i（降低后腿，让后部变低），X/Y不变
  set_site(1, KEEP, KEEP, site_now[1][2] + i);
  // 左前腿：同右前腿，抬升
  set_site(2, KEEP, KEEP, site_now[2][2] - i);
  // 左后腿：同右后腿，降低
  set_site(3, KEEP, KEEP, site_now[3][2] + i);
  // 等待所有腿到位，确保姿态稳定（前高后低=抬头）
  wait_all_reach();
}
// 功能：模拟机器人“低头”姿态（与抬头逻辑对称，前低后高）
// 参数 i：低头幅度（i越大，前后腿高度差越大，低头越明显）
void head_down(int i)
{
  // 右前腿：Z坐标 = 当前Z + i（降低前腿，让前部变低），X/Y不变
  set_site(0, KEEP, KEEP, site_now[0][2] + i);
  // 右后腿：Z坐标 = 当前Z - i（抬升后腿，让后部变高），X/Y不变
  set_site(1, KEEP, KEEP, site_now[1][2] - i);
  // 左前腿：同右前腿，降低
  set_site(2, KEEP, KEEP, site_now[2][2] + i);
  // 左后腿：同右后腿，抬升
  set_site(3, KEEP, KEEP, site_now[3][2] - i);
  // 等待所有腿到位，确保姿态稳定（前低后高=低头）
  wait_all_reach();
}
// 功能：执行组合舞蹈动作（包含坐下、姿态调整、加速摆动，增强观赏性）
// 参数 i：舞蹈循环次数（i越大，舞蹈时间越长）
void body_dance(int i)
{
  // 临时变量：保存坐标（未实际使用，预留扩展）
  float x_tmp;
  float y_tmp;
  float z_tmp;
  // 舞蹈基础速度（初始值2，后续会加速）
  float body_dance_speed = 2;

  // 1. 先让机器人坐下（舞蹈起始姿态）
  sit();
  // 2. 降低初始速度，确保姿态调整平稳
  move_speed = 1;
  // 3. 调整所有腿的X/Y坐标到默认位置（统一姿态），Z保持坐下高度
  set_site(0, x_default, y_default, KEEP);
  set_site(1, x_default, y_default, KEEP);
  set_site(2, x_default, y_default, KEEP);
  set_site(3, x_default, y_default, KEEP);
  wait_all_reach();

  // 4. 调整所有腿的Z坐标到“半站立”高度（z_default-20，比站立低20单位，舞蹈姿态）
  set_site(0, x_default, y_default, z_default - 20);
  set_site(1, x_default, y_default, z_default - 20);
  set_site(2, x_default, y_default, z_default - 20);
  set_site(3, x_default, y_default, z_default - 20);
  wait_all_reach();

  // 5. 设置舞蹈速度，先抬头（舞蹈起始动作）
  move_speed = body_dance_speed;
  head_up(30);

  // 6. 循环i次：核心舞蹈动作（左右摆动腿，逐步加速）
  for (int j = 0; j < i; j++)
  {
    // 舞蹈节奏控制：前1/4循环用基础速度，1/4-1/2循环加速2倍，1/2后加速3倍（越来越快）
    if (j > i / 4)
      move_speed = body_dance_speed * 2;
    if (j > i / 2)
      move_speed = body_dance_speed * 3;

    // 左腿左摆、右腿右摆（姿态1）
    set_site(0, KEEP, y_default - 20, KEEP); // 右前腿Y-20（左移）
    set_site(1, KEEP, y_default + 20, KEEP); // 右后腿Y+20（右移）
    set_site(2, KEEP, y_default - 20, KEEP); // 左前腿Y-20（左移）
    set_site(3, KEEP, y_default + 20, KEEP); // 左后腿Y+20（右移）
    wait_all_reach();

    // 左腿右摆、右腿左摆（姿态2，与姿态1对称，形成摆动）
    set_site(0, KEEP, y_default + 20, KEEP); // 右前腿Y+20（右移）
    set_site(1, KEEP, y_default - 20, KEEP); // 右后腿Y-20（左移）
    set_site(2, KEEP, y_default + 20, KEEP); // 左前腿Y+20（右移）
    set_site(3, KEEP, y_default - 20, KEEP); // 左后腿Y-20（左移）
    wait_all_reach();
  }

  // 7. 舞蹈结束：恢复速度，低头（与起始抬头动作呼应，姿态收尾）
  move_speed = body_dance_speed;
  head_down(30);
}
/*
  功能：舵机核心服务函数（约10ms调用一次，高实时性）
  作用：1. 将腿的“实时坐标”逐步逼近“期望坐标”；2. 将实时坐标转换为舵机角度并写入舵机
  关键逻辑：通过temp_speed控制每步移动距离，确保腿沿直线移动到目标位置
*/
void servo_service(void)
{
  // 静态变量：保存舵机角度（alpha=肩部角度，beta=肘部角度，gamma=足部角度），避免每次调用重新初始化
  static float alpha, beta, gamma;

  // 遍历4条腿，更新每条腿的坐标和舵机角度
  for (int i = 0; i < 4; i++)
  {
    // 遍历X/Y/Z三个轴，更新实时坐标（逐步逼近期望坐标）
    for (int j = 0; j < 3; j++)
    {
      // 若“实时坐标与期望坐标的差值”≥“单步速度”：按速度逐步移动
      if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
        site_now[i][j] += temp_speed[i][j];
      // 若差值＜单步速度：直接将实时坐标设为期望坐标（避免超调）
      else
        site_now[i][j] = site_expect[i][j];
    }

    // 步骤1：将当前腿的“直角坐标（x,y,z）”转换为“极坐标（舵机角度alpha,beta,gamma）”
    cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
    // 步骤2：将极坐标（舵机角度）转换为实际舵机控制值（加偏移量校准），并写入舵机
    polar_to_servo(i, alpha, beta, gamma);
  }

  // 休息计数器+1（每10ms+1，用于后续自动休息逻辑，当前未启用）
  rest_counter++;
  // 延时10ms：控制舵机更新频率（约100Hz，适配舵机响应速度）
  vTaskDelay(10);
}
/*
  功能：设置单条腿的“期望坐标”，并计算“单步速度”（确保腿沿直线移动）
  参数：leg=腿序号（0-3）；x/y/z=目标坐标（KEEP表示不改变该轴坐标）
  关键逻辑：根据“当前坐标到期望坐标的距离”计算速度分量，保证X/Y/Z轴同步到达（直线运动）
*/
void set_site(int leg, float x, float y, float z)
{
  // 临时变量：保存X/Y/Z轴的“总移动距离”
  float length_x = 0, length_y = 0, length_z = 0;

  // 计算各轴总移动距离（仅当目标坐标不是KEEP时）
  if (x != KEEP)
    length_x = x - site_now[leg][0]; // X轴总距离=期望X - 当前X
  if (y != KEEP)
    length_y = y - site_now[leg][1]; // Y轴总距离=期望Y - 当前Y
  if (z != KEEP)
    length_z = z - site_now[leg][2]; // Z轴总距离=期望Z - 当前Z

  // 计算“空间直线总距离”（勾股定理：三维空间中两点距离）
  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  // 计算各轴“单步速度”（速度分量=总距离分量/总距离 × 基础速度 × 速度倍数）
  // 逻辑：总距离分量/总距离 = 该轴的“方向占比”，确保X/Y/Z轴同步到达（直线运动）
  temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
  temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
  temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;

  // 设置该腿的“期望坐标”（仅更新非KEEP的轴）
  if (x != KEEP)
    site_expect[leg][0] = x;
  if (y != KEEP)
    site_expect[leg][1] = y;
  if (z != KEEP)
    site_expect[leg][2] = z;
}
/*
  功能：阻塞等待单条腿的“实时坐标”完全等于“期望坐标”
  参数：leg=腿序号（0-3）
  作用：确保动作按顺序执行（比如“抬腿→移动→放下”需分步阻塞）
*/
void wait_reach(int leg)
{
  // 死循环，直到实时坐标与期望坐标完全一致
  while (1)
  {
    // 检查X/Y/Z三轴是否均到达期望坐标
    if (site_now[leg][0] == site_expect[leg][0])
      if (site_now[leg][1] == site_expect[leg][1])
        if (site_now[leg][2] == site_expect[leg][2])
          break; // 三轴均到位，退出循环

    // 延时1ms：降低CPU占用，避免空循环浪费资源
    vTaskDelay(1);
  }
}
/*
  功能：阻塞等待所有4条腿的“实时坐标”均等于“期望坐标”
  作用：确保机身整体姿态稳定（比如“站立”“平移”需所有腿同步到位）
*/
void wait_all_reach(void)
{
  // 遍历4条腿，逐个等待到位（所有腿都到位才退出）
  for (int i = 0; i < 4; i++)
    wait_reach(i);
}
/*
  功能：将腿末端的“直角坐标（x,y,z）”转换为“舵机极坐标（alpha,beta,gamma）”
  参数：alpha/beta/gamma=输出的舵机角度（引用传递，直接修改外部变量）；x/y/z=输入的直角坐标
  数学模型：基于机器人腿部机械结构（3自由度：肩、肘、足）推导，将三维坐标转换为三个舵机的旋转角度
*/
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z)
{
  // 临时变量：w=X-Y平面内“腿末端到机身中心的距离”；v=调整后的水平距离（减去肩部安装偏移length_c）
  float v, w;

  // 步骤1：计算w（X-Y平面距离，带方向：x≥0时为正，x<0时为负，确保方向正确）
  w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
  // 步骤2：计算v（水平方向实际参与舵机角度计算的距离=w - 肩部安装偏移length_c）
  v = w - length_c;

  // 步骤3：计算alpha（肩部与肘部的夹角，对应肘部舵机角度）
  // 公式意义：结合“反正切（z/v，垂直高度与水平距离的夹角）”和“余弦定理（根据大腿/小腿长度计算关节角）”
  alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
  
  // 步骤4：计算beta（肘部与足部的夹角，对应足部舵机角度）
  // 公式意义：余弦定理（已知大腿length_a、小腿length_b、水平+垂直距离，计算肘部关节角）
  beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b);
  
  // 步骤5：计算gamma（肩部绕Z轴的旋转角，对应肩部舵机角度）
  // 公式意义：反正切（y/x，计算X-Y平面内腿的水平朝向，w<0时反向，确保方向正确）
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);

  // 步骤6：将弧度转换为角度（舵机控制需角度值，1弧度=180/pi度）
  alpha = alpha / pi * 180;
  beta = beta / pi * 180;
  gamma = gamma / pi * 180;
}

///*
//  - trans site from polar to microservos
//  - mathematical model map to fact
//  - the errors saved in eeprom will be add
//   ---------------------------------------------------------------------------*/
//void polar_to_servo(int leg, float alpha, float beta, float gamma)
//{
//  if (leg == 0)
//  {
//    alpha = 90 - alpha;
//    beta = beta;
//    gamma += 90;
//  }
//  else if (leg == 1)
//  {
//    alpha += 90;
//    beta = 180 - beta;
//    gamma = 90 - gamma;
//  }
//  else if (leg == 2)
//  {
//    alpha += 90;
//    beta = 180 - beta;
//    gamma = 90 - gamma;
//  }
//  else if (leg == 3)
//  {
//    alpha = 90 - alpha;
//    beta = beta;
//    gamma += 90;
//  }
//
//  servo[leg][0].write(alpha+offset[leg][0]);
//  servo[leg][1].write(beta+offset[leg][1]);
//  servo[leg][2].write(gamma+offset[leg][2]);
//}

/*
  - trans site from polar to microservos
  - mathematical model map to fact
  - the errors saved in eeprom will be add
   ---------------------------------------------------------------------------*/
/*
  功能：将“极坐标（舵机角度alpha,beta,gamma）”转换为实际舵机控制值（加偏移量校准）
  参数：leg=腿序号（0-3）；alpha/beta/gamma=输入的极坐标角度
  关键逻辑：根据不同腿的安装方向（前/后/左/右）调整角度（机械结构对称导致角度方向不同），并加入偏移量补偿机械误差
*/
void polar_to_servo(int leg, float alpha, float beta, float gamma)
{
  // 根据腿序号调整角度（适配不同腿的安装方向，避免动作反向）
  if (leg == 0) // 右前腿（Front Right）
  {
    alpha = 90 - alpha - FRElbow; // 肘部角度：90-基础值-FRElbow（FRElbow是肘部微调量，-表示向上）
    beta = beta - FRFoot;         // 足部角度：基础值-FRFoot（FRFoot是足部微调量，-表示向上）
    gamma = 90 + gamma + FRShdr;  // 肩部角度：90+基础值+FRShdr（FRShdr是肩部微调量，-表示向左）
  }
  else if (leg == 1) // 右后腿（Rear Right）
  {
    alpha += 90 - RRElbow;         // 肘部角度：90-RRElbow（+表示向上）
    beta = 180 - beta + RRFoot;   // 足部角度：180-基础值+RRFoot（+表示向上）
    gamma = 90 - gamma - RRShdr;  // 肩部角度：90-基础值-RRShdr（+表示向左）
  }
  else if (leg == 2) // 左前腿（Front Left）
  {
    alpha += 90 + FLElbow;        // 肘部角度：基础值+90+FLElbow（+表示向上）
    beta = 180 - beta + FLFoot;   // 足部角度：180-基础值+FLFoot（+表示向上）
    gamma = 90 - gamma - FLShdr;  // 肩部角度：90-基础值-FLShdr（+表示向左）
  }
  else if (leg == 3) // 左后腿（Rear Left）
  {
    alpha = 90 - alpha - RLElbow; // 肘部角度：90-基础值-RLElbow（-表示向上）
    beta = beta - RLFoot;         // 足部角度：基础值-RLFoot（-表示向上）
    gamma = 90 + gamma + RLShdr;  // 肩部角度：90+基础值+RLShdr（-表示向左）
  }

  // 下方注释代码：旧版PWM舵机控制逻辑（用硬件PWM模块，当前代码用ESP32Servo库，故注释）
  //  int AL = ((850/180)*alpha);if (AL > 580) AL=580;if (AL < 125) AL=125;pwm.setPWM(servo_pin[leg][0],0,AL);
  //  int BE = ((850/180)*beta);if (BE > 580) BE=580;if (BE < 125) BE=125;pwm.setPWM(servo_pin[leg][1],0,BE);
  //  int GA = ((580/180)*gamma);if (GA > 580) GA=580;if (GA < 125) GA=125;pwm.setPWM(servo_pin[leg][2],0,GA);

  // 最终步骤：将“调整后的角度+偏移量”写入舵机（offset是EEPROM存储的校准值，补偿机械误差）
  servo[leg][0].write(alpha + offset[leg][0]); // 肩部舵机
  servo[leg][1].write(beta + offset[leg][1]);  // 肘部舵机
  servo[leg][2].write(gamma + offset[leg][2]); // 足部舵机
}
