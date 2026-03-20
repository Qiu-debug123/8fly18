// ============================================================
// 8fly四轴无人机控制系统
// 功能：实现四轴无人机的姿态稳定控制和遥控响应
// 硬件：Arduino + MPU6050 + 4个电机 + 遥控接收器
// 算法：Mahony姿态解算(在AttitudeEstimator.cpp中实现姿态解算) + PID控制 + 电机混控
// ============================================================

#include <Wire.h>                       // I2C通信库，用于与MPU6050传感器通信
#include <EEPROM.h>                     // EEPROM存储库，用于保存校准参数
#include "pid.h"                        // 自定义PID控制器库
#include "AttitudeEstimator.h"          // 自定义姿态解算库（Mahony算法）
#include <Servo.h>                      // 舵机控制库
#include "flow_decode.h"
#include "mtf02.h"
#include "EKF.h"             
#include "uwb.h"                        // UWB数据处理库          
//#include "pid.cpp"

#define EI_ARDUINO_INTERRUPTED_PIN      // 启用中断引脚功能
#include <EnableInterrupt.h>            // 中断处理库，用于遥控器PWM信号接收

//定义一个低通滤波器
#define LPF_1_(hz, t, in, out) ((out) += (1 / (1 + 1 / ((hz) * 3.14f * (t)))) * ((in) - (out)))

int MTF_confidence = 0;
int Mtf02_count = 0;
int Mtf02_count_1 = 0;
float LPF_MTF_dx;
float LPF_MTF_dy;

//定义一个整型限幅器
void constrain_int(int& data, int min, int max) {
    if (data < min) {
        data = min;
    } else if (data > max) {
        data = max;
    }
}
void constrain_float(float& data, float min, float max) {
	if (data < min) {
		data = min;
	} else if (data > max) {
		data = max;
	}
}

// ==============================================
// NLink Frame0 解析示例 - Arduino Mega
// 帧头: 0x01 0x00 0x02
// 每帧后9字节: x,y,z 方向位置 (int24 LE)
// ==============================================
typedef struct {
  uint8_t frame_header[3]; // 0x01 0x00 0x02
  int32_t pos[3];          // x, y, z
} NLink_Frame0;

float real_pos[3];// 用于存储UWB输出位置值

// 你的原始X/Y轴结果结构体，无修改
EKF_UWB_IMU_DATA ekf_X;
EKF_UWB_IMU_DATA ekf_Y;

// 创建X/Y轴滤波实例，使用你的原代码默认参数
EKFFilter ekfX;
EKFFilter ekfY;
int Is_UWB_work = 0; // UWB工作状态标志，1表示正常工作，0表示未工作

NLink_Frame0 current_frame;

void Init_UWB() {// 初始化UWB相关变量
	ekf_X.speed = 0.0f;
	ekf_X.pos = 0.0f;
	ekf_Y.speed = 0.0f;
	ekf_Y.pos = 0.0f;
}

// ---------------- 小端解析函数 ----------------
int32_t readInt24LE(uint8_t* data) {
  int32_t value = data[0] | (data[1] << 8) | (data[2] << 16);
  if (value & 0x800000) value |= 0xFF000000;  // 符号扩展
  return value;
}

// ---------------- 处理数据 ----------------
void processFrameData() {

  real_pos[0] = current_frame.pos[0] / 1000.0f;
  real_pos[1] = current_frame.pos[1] / 1000.0f;
  real_pos[2] = current_frame.pos[2] / 1000.0f;

//   Serial.print("Position(m): ");
//   Serial.print(real_pos[0], 3); Serial.print(", ");
//   Serial.print(real_pos[1], 3); Serial.print(", ");
//   Serial.println(real_pos[2], 3);
}

float pitch_Delay_data_1 = 0;
float pitch_Delay_data_2 = 0;
float pitch_Delay_data_3 = 0;
float pitch_Delay_data_4 = 0;
float pitch_Delay_data_5 = 0;
// float pitch_Delay_data_6 = 0;

// void pitch_Delay_six_times(float now_data, float& six_times_past_data){
// 	pitch_Delay_data_6 = pitch_Delay_data_5;
// 	pitch_Delay_data_5 = pitch_Delay_data_4;
// 	pitch_Delay_data_4 = pitch_Delay_data_3;
// 	pitch_Delay_data_3 = pitch_Delay_data_2;
// 	pitch_Delay_data_2 = pitch_Delay_data_1;
// 	pitch_Delay_data_1 = now_data;
// 	six_times_past_data = pitch_Delay_data_6;
// }

void pitch_Delay_two_point_five_times(float now_data, float& two_point_five_times_past_data){
	pitch_Delay_data_3 = pitch_Delay_data_2;
	pitch_Delay_data_2 = pitch_Delay_data_1;
	pitch_Delay_data_1 = now_data;
	two_point_five_times_past_data = 0.5*pitch_Delay_data_3 + 0.5*pitch_Delay_data_2;
}

void pitch_Delay_three_times(float now_data, float& three_times_past_data){
	pitch_Delay_data_3 = pitch_Delay_data_2;
	pitch_Delay_data_2 = pitch_Delay_data_1;
	pitch_Delay_data_1 = now_data;
	three_times_past_data = pitch_Delay_data_3;
}

void pitch_Delay_two_times(float now_data, float& two_times_past_data){
	pitch_Delay_data_2 = pitch_Delay_data_1;
	pitch_Delay_data_1 = now_data;
	two_times_past_data = pitch_Delay_data_2;
}

void pitch_Delay_one_times(float now_data, float& one_times_past_data){
	pitch_Delay_data_1 = now_data;
	one_times_past_data = pitch_Delay_data_1;
}

float roll_Delay_data_1 = 0;
float roll_Delay_data_2 = 0;
float roll_Delay_data_3 = 0;
float roll_Delay_data_4 = 0;
float roll_Delay_data_5 = 0;
// float roll_Delay_data_6 = 0;

// void roll_Delay_six_times(float now_data, float& six_times_past_data){
// 	roll_Delay_data_6 = roll_Delay_data_5;
// 	roll_Delay_data_5 = roll_Delay_data_4;
// 	roll_Delay_data_4 = roll_Delay_data_3;
// 	roll_Delay_data_3 = roll_Delay_data_2;
// 	roll_Delay_data_2 = roll_Delay_data_1;
// 	roll_Delay_data_1 = now_data;
// 	six_times_past_data = roll_Delay_data_6;
// }

void roll_Delay_two_point_five_times(float now_data, float& two_point_five_times_past_data){
	roll_Delay_data_3 = roll_Delay_data_2;
	roll_Delay_data_2 = roll_Delay_data_1;
	roll_Delay_data_1 = now_data;
	two_point_five_times_past_data = 0.5*roll_Delay_data_3 + 0.5*roll_Delay_data_2;
}

void roll_Delay_three_times(float now_data, float& three_times_past_data){
	roll_Delay_data_3 = roll_Delay_data_2;
	roll_Delay_data_2 = roll_Delay_data_1;
	roll_Delay_data_1 = now_data;
	three_times_past_data = roll_Delay_data_3;
}

void roll_Delay_two_times(float now_data, float& two_times_past_data){
	roll_Delay_data_2 = roll_Delay_data_1;
	roll_Delay_data_1 = now_data;
	two_times_past_data = roll_Delay_data_2;
}

void roll_Delay_one_times(float now_data, float& one_times_past_data){
	roll_Delay_data_1 = now_data;
	one_times_past_data = roll_Delay_data_1;
}

struct _1_ekf_filter Angle_rate_ekf;

float EKF_1_MTF_Raw_rollRate = 0; 
float EKF_1_MTF_Raw_pitchRate = 0;

float MTF_gyro_fix_dx; 
float MTF_gyro_fix_dy;
float Last_MTF_gyro_fix_dx; 
float Last_MTF_gyro_fix_dy;
float target_X_speed;
float target_Y_speed;
float target_X_acceleration;
float target_X_acceleration_lvbo=0;
float target_Y_acceleration_lvbo=0;
float target_Y_acceleration;
float LPF_MTF_gyro_fix_dx;
float LPF_MTF_gyro_fix_dy;
float LPF_MTF_init_dx;
float LPF_MTF_init_dy;
float MTF_Raw_rollRate; // 原始横滚角速度（rad/s）
float MTF_Raw_pitchRate; // 原始俯仰角速度（rad/s）
float Delay_MTF_Raw_rollRate; // 延时后的横滚角速度
float Delay_MTF_Raw_pitchRate; // 延时后的俯仰角速度
float LPF_MTF_Raw_rollRate; // 横滚角速度低通滤波值
float LPF_MTF_Raw_pitchRate; // 俯仰角速度低通滤波值
float LPF_2_MTF_Raw_rollRate;
float LPF_2_MTF_Raw_pitchRate;



float LPF_1_MTF_Raw_rollRate=0; 
float LPF_1_MTF_Raw_pitchRate=0;
float MTF_measured_X_speed;
float MTF_measured_Y_speed;
float New_MTF_measured_X_speed;
float New_MTF_measured_Y_speed;
float New_MTF_measured_X_speed_lvbo;
float New_MTF_measured_Y_speed_lvbo;
float MTF_PosX;
float MTF_PosY;
float Acc_PosX;
float Acc_PosY;
float real_MTF_PosX=0;
float real_MTF_PosY=0;
float MTF_Test_PosX;
float MTF_Test_PosY;
float MTF_init_dx; 
float MTF_init_dy;


int state_check = 1; // 状态机检查变量(0:未起飞, 1:已起飞)

float rcinput_0 = 0;  // 遥控器输入通道0（Roll）
float rcinput_1 = 0;  // 遥控器输入通道1（Pitch）
float rcinput_2 = 0;  // 遥控器输入通道2（Throttle）
float rcinput_3 = 0;  // 遥控器输入通道3（Yaw）

// 在mtf02.c中声明外部全局变量（与.ino文件中的变量对应）
//现转移至优象光流
uint32_t g_distance;
uint8_t g_strength;
int16_t g_flow_vel_x;
int16_t g_flow_vel_y;
uint8_t g_flow_quality; 




// ============================================================
// 遥控器PWM信号接收配置
// 使用中断方式接收4通道遥控器信号：Roll、Pitch、Throttle、Yaw
// ============================================================
byte receiver_pins[4] = {10,11,12,13};  // 4个数字引脚接收PWM信号
//roll-10,pitch-11,油门-12，yaw-13       // 引脚分配：横滚、俯仰、油门、偏航
volatile int receiver_input[4];          // 存储4个通道的PWM脉宽值（1000-2000us）
unsigned long timer_1, timer_2, timer_3, timer_4;  // 用于计算PWM脉宽的计时器


/* ============================================================
 * 四轴无人机电机布局和坐标系定义                    
 * 		 ↶				 ↷                     ↶		↷
 *       3(LU)            4(RU)				+   yaw   -
 *         -  x(roll)  +                    (顺时针为正)
 *                ↑
 *                |
 *                |             +
 *                L--------→ y(pitch)       (前倾为正)
 *                              -
 *       2(LD)            1(RD)
 *       ↻               ↺
 * 
 * 电机编号说明：
 * 1(RD): 右下电机 - 顺时针旋转
 * 2(LD): 左下电机 - 逆时针旋转  
 * 3(LU): 左上电机 - 顺时针旋转
 * 4(RU): 右上电机 - 逆时针旋转
 * ============================================================
 */
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// 
// 根据 roll 和 pitch 对应的不同，修改 ESC_Ctrl
// 
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// ============================================================
// 飞控状态机定义
// 系统通过状态机管理不同的工作模式
// ============================================================
#define STATE_INIT		0	// 初始化状态：系统启动，等待解锁条件
#define STATE_FLYPRE	1	// 飞行准备阶段：MPU校准，电机预热（暂未使用）
#define STATE_FLY   	2	// 飞行状态：正常飞行控制模式
#define STATE_STOP  	3	// 停止状态：紧急停机（暂未使用）
#define STATE_MEAS		4	// 测量模式：校准传感器偏差

int state = STATE_INIT;          // 当前系统状态，初始为初始化状态


int Control_State = 0;      // 状态变量，用于靠近墙体(默认值为0)



float flow_x_lpf_att_i;
float flow_y_lpf_att_i;

/* ============================================================
 * 电调(ESC)控制参数配置
 * ESC通过PWM信号控制电机转速，脉宽范围1000-2000us
 * ============================================================
 */
#define ESC_INIT_PWM	1000    // ESC初始化脉宽值（最低转速）
#define ESC_MIN_PWM		1000    // ESC最小脉宽值（电机停止）
#define ESC_MAX_PWM		2000    // ESC最大脉宽值（最高转速）

int ESC_PWM[4] = { ESC_INIT_PWM, ESC_INIT_PWM, ESC_INIT_PWM, ESC_INIT_PWM };  // 4个电机的PWM值
unsigned long zero_timer, timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;  // PWM输出计时器


/* ============================================================
 * PID控制器参数
 * 实现姿态稳定控制：角度环控制姿态，角速度环控制响应速度
 * ============================================================
 */
PIDController pidController;                                    // PID控制器实例
float targetRoll = 0.0, targetPitch = 0.0, targetYaw = 0.0;   // 目标姿态角度（度）
float targetHigh = 0.0;                                       // 目标高度（厘米）
float target_X_position = 0.0;
float target_Y_position = 0.0;
float roll_level_adjust, pitch_level_adjust;                   // 水平校准调整值

//ESC_Ctrl用变量，用于电调控制
bool ESC_Ctrl_Flag_1 = false;  // 电调控制标志1
bool ESC_Ctrl_Flag_2 = false;  // 电调控制标志2
bool ESC_Ctrl_Flag_3 = false;  // 电调控制标志3
bool ESC_Ctrl_Flag_4 = false;  // 电调控制标志4


/* ============================================================
 * MPU6050传感器数据和姿态解算变量
 * 包含原始传感器数据、滤波数据、姿态角度等
 * ============================================================
 */
// 当前姿态角度（欧拉角，单位：度）
float roll = 0.0, pitch = 0.0, yaw = 0.0,roll_acc = 0.0, pitch_acc = 0.0, gz_input, gy_input=0.0, gx_input=0.0;
float targetRollRate = 0.0, targetPitchRate = 0.0, targetYawRate = 0.0; 
float MTF_height = 0.0;  
float targetHeightRate = 0.0;
float Height_Rate = 0.0;
float LPF_Height_Rate = 0.0;
float last_Height = 0.0;

// 加速度计原始数据和校准参数
long ax, ay, az, acc0_total,acc0_x,acc0_y,acc0_z;
float ax_cali,ay_cali,az_cali;

// 陀螺仪数据（角速度，单位：rad/s）
float gx, gy, gz;

// 磁力计数据（暂未使用）
float mx, my, mz;

// 初始姿态角度（用于校准）
float pitch_0 = 0.0;float roll_0 = 0.0;
// float roll_cal = 0, pitch_cal = 0;

// 陀螺仪积分姿态角度
float roll_gyro = 0.0,pitch_gyro = 0.0;

// 统计变量
float RAve = 0.0, PAve = 0.0;
int Newcount = 0;
const int measureTime = 1000;
int dataCount=0;

// 角度转换常数
float rad2angle = 57.2957795f;  // 弧度转角度系数
float angle2rad = 0.0174533f;   // 角度转弧度系数
//四元数结构体定义（用于姿态表示，暂未在主程序中使用）
typedef volatile struct
{
	float q0;  // 四元数实部
	float q1;  // 四元数虚部i
	float q2;  // 四元数虚部j
	float q3;  // 四元数虚部k
} Quaternion;

// 四元数相关变量（预留，当前使用Mahony算法在AttitudeEstimator中处理）
// Quaternion NumQ = {1, 0, 0, 0};     // 当前姿态四元数
// Quaternion k1 = {0, 0, 0, 0};       // 龙格库塔法中间变量
// Quaternion k2 = {0, 0, 0, 0};       // 龙格库塔法中间变量
// Quaternion q_mid= {0, 0, 0, 0};     // 龙格库塔法中间变量

// PI控制器积分误差（预留）
float error_x_I = 0.0;
float error_y_I = 0.0;
float error_z_I = 0.0;

bool acc_changed = false;  // 加速度变化标志
//低通滤波器数据缓存（预留，当前使用AttitudeEstimator中的滤波）
// float gx_list[2] = {0,0};    // 陀螺仪X轴滤波缓存
// float gy_list[2] = {0,0};    // 陀螺仪Y轴滤波缓存
// float gz_list[2] = {0,0};    // 陀螺仪Z轴滤波缓存
// float accx_list[2] = {0,0};  // 加速度计X轴滤波缓存
// float accy_list[2] = {0,0};  // 加速度计Y轴滤波缓存
// float accz_list[2] = {0,0};  // 加速度计Z轴滤波缓存

// 主循环计时器和控制变量
long loop_timer;             // 用于控制主循环频率（250Hz）
int count1=0;                // 通用计数器

float batteryVoltage = 0;    // 电池电压监测
bool setGyroAngle;           // 陀螺仪角度设置标志

//存放树莓派传输的控制信息（预留接口）
//INDEX=0为pitch Index=1为roll Index=2为yaw Index=3为target_height
int target[4]={0,0,0,0};     // 外部控制目标值数组

void pwmReceive();           // PWM接收中断函数声明

Servo edf;                   // 电调控制对象（备用方案）

/* ============================================================
 * 位置估计相关变量
 * 用于估计无人机在空间中的位置
 * ============================================================
 */
float Pos_lasttime = 0.0;  // 上次位置估计时间
float Pos_nowtime = 0.0;   // 当前时间
float Pos_dt = 0.0;       // 时间间隔
float X_position = 0.0;  // X轴位置cm
float Y_position = 0.0;  // Y轴位置cm
float Last_X_position = 0.0; // 上次X轴位置
float Last_Y_position = 0.0; // 上次Y轴位置
float Output_X = 0.0; // 输出X轴位置
float Output_Y = 0.0; // 输出Y轴位置

/* ============================================================
 * 超声波测距模块配置（当前已注释，预留功能）
 * 用于高度测量和定高功能
 * ============================================================
 */

int loop_count = 0; // 主循环计数变量

int state_dinggao_chaoshengbo = 1; // 定高超声波状态变量 1:退出靠近墙体和定高 2：靠近墙体加定高
int state_dinggao_chaoshengbo_count = 0; // 定高超声波状态计数变量

int Ul_trigPin = 17, Ul_echoPin = 18;     // 定义超声波的引脚
float Ul_duration;                       // 测到的距离
unsigned long Ul_starttime;             // 超声波测距的启动时间
int Ul_startflag = 0;                  // 超声波测距的启动标志位

float Ul_Pos_X = 0.0;
float Ul_Pos_X_lvbo = 0.0;
float Ul_start_Pos_X = 0.0;

float UL_duration_lvbo=0; // 超声波测距低通滤波值
int lvbo_count = 0;

void interrupt_5(){
  Ul_duration = (micros() - Ul_starttime) / 58.82;   // 需要减去trig和echo变化之间的延迟2240us  time/2/1000 * 34cm/ms
  Ul_duration -= 10; //修正量
  if(loop_count <= 10) {
	UL_duration_lvbo = Ul_duration; // 初始化低通滤波值
  }
  if(loop_count >= 60 && loop_count <= 80) {
	Ul_start_Pos_X = UL_duration_lvbo;
  }
  if(loop_count >90) {
	  Ul_Pos_X = -(UL_duration_lvbo - Ul_start_Pos_X);
	  Ul_Pos_X_lvbo = 0.15*Ul_Pos_X + 0.85*Ul_Pos_X_lvbo;
  }
  Ul_startflag = 0;
}

void Ul_setup()
{
//  Serial.begin(115200);   // 设定串口的波特率
  //FlexiTimer2::set(20, 1.0/1000, timer2); // 初始化定时器2
  // 替换原定时器配置，触发信号改为10μs
  // FlexiTimer2::set(100, 1.0/100, timer2);  // 时间基数10ms，间隔1s
  pinMode(Ul_trigPin, OUTPUT);
  pinMode(Ul_echoPin, INPUT);
  digitalWrite( Ul_trigPin , LOW );
  // 初始化外部中断0
  //attachInterrupt(0, interrupt_0, FALLING);  //interrupt为你中断通道编号，function为中断函数，mode为中断触发模式
  //attachInterrupt(5, interrupt_5, FALLING);  //Arduino Mega2560中，数字引脚2对应中断0，数字引脚3对应中断1，数字引脚18对应中断5
  enableInterrupt(18, interrupt_5, FALLING); // 使用EnableInterrupt库配置中断
}

void Ul_update_distance_cm()
{
  if (Ul_startflag == 0){   //启动测距
    Ul_startflag = 1;
    digitalWrite(Ul_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(Ul_trigPin, HIGH);
    delayMicroseconds(10); // 10us 高电平
    //FlexiTimer2::start();
    digitalWrite(Ul_trigPin, LOW);
    Ul_starttime = micros();
    // Serial.print(Ul_duration);
    // Serial.print(",");
    // Serial.print(100);
    // Serial.print(",");
    // Serial.print(50);
    // Serial.println();
  }
}

void Ul_ditonglvbo(){
	if (abs(Ul_duration - UL_duration_lvbo) < 20.0)
	{
		UL_duration_lvbo = 0.15*Ul_duration + 0.85*UL_duration_lvbo;
	}
	else
	{
    ++lvbo_count;
    if(lvbo_count > 2) {
      lvbo_count = 0;
      UL_duration_lvbo = Ul_duration;
	}
	}
}

int start_flag_count = 0;
int lvbo_count2 = 0;
int  Ul2_trigPin = 53, Ul2_echoPin = 52;   // 定义超声波2的引脚
float Ul2_duration;                       // 2测到的距离
float last_Ul2_duration = 0; // 上次测距值
float Ul2_speed = 0;
unsigned long Ul2_starttime;             // 超声波2测距的启动时间
int Ul2_startflag = 0;                  // 超声波2测距的启动标志位
long Ul2_loop_timer;

float UL2_duration_lvbo=0; // 超声波2测距低通滤波值

float Ul2_Pos_Y = 0.0;
float Ul2_Pos_Y_lvbo = 0.0;
float Ul2_start_Pos_Y = 0.0;

void interrupt_6(){
  Ul2_duration = (micros() - Ul2_starttime)/ 58.82;   //time/2/1000 * 34cm/ms
  Ul2_duration -= 38; //修正量
  Ul2_speed = (Ul2_duration - last_Ul2_duration)/((micros() - Ul2_starttime));
  Ul2_speed *= 10000; // 转换为m/s

  if(loop_count <= 10) {
	UL_duration_lvbo = Ul_duration; // 初始化低通滤波值
  }
	if(loop_count >= 60 && loop_count <= 80) {
		Ul2_start_Pos_Y = UL2_duration_lvbo;
	}
	if(loop_count >90) {
		Ul2_Pos_Y = UL2_duration_lvbo - Ul2_start_Pos_Y;
		Ul2_Pos_Y_lvbo = 0.15*Ul2_Pos_Y + 0.85*Ul2_Pos_Y_lvbo;
	}

  Ul2_startflag = 0;
  last_Ul2_duration = Ul2_duration;
}

void Ul2_setup()
{
  Ul2_loop_timer = micros();
  pinMode(Ul2_trigPin, OUTPUT);
  pinMode(Ul2_echoPin, INPUT);
  digitalWrite( Ul2_trigPin , LOW );
  enableInterrupt(Ul2_echoPin, interrupt_6, FALLING); // 使用EnableInterrupt库配置中断
}

void Ul2_update_distance_cm()
{
  if(Ul2_startflag == 1) {
    ++start_flag_count;
    if(start_flag_count > 4) { // 超时未测到距离，复位测距标志
      Ul2_startflag = 0;
      start_flag_count = 0;

    }
  }
  if (Ul2_startflag == 0){   //启动测距
    Ul2_startflag = 1;
    digitalWrite(Ul2_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(Ul2_trigPin, HIGH);
    delayMicroseconds(10); // 10us 高电平
    digitalWrite(Ul2_trigPin, LOW);
    Ul2_starttime = micros();
  }
}

void Ul2_ditonglvbo(){
  if (abs(Ul2_duration - UL2_duration_lvbo) < 20.0)
  {
    UL2_duration_lvbo = 0.15*Ul2_duration + 0.85*UL2_duration_lvbo;
  }
  else
  {
    ++lvbo_count2;
    if(lvbo_count2 > 2) {
      lvbo_count2 = 0;
      UL2_duration_lvbo = Ul2_duration;
    }
  }
}


// #include "NewPing.h"

// // Hook up HC-SR04 with Trig to Arduino Pin 9, Echo to Arduino pin 10
// #define TRIGGER_PIN 22
// #define ECHO_PIN 23

// // Maximum distance we want to ping for (in centimeters).
// #define MAX_DISTANCE 400    

// // NewPing setup of pins and maximum distance.
// NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
// float duration, distance;
int index=-1;  // 通用索引变量

// ============================================================
// setup()函数：系统初始化
// 执行一次，完成所有硬件和软件的初始化工作
// ============================================================
void setup() {
	Serial.begin(115200);  // 初始化串口通信，波特率115200
	//Serial3.begin(115200); // 初始化串口3通信，波特率115200,用于mtf02传感器数据接收
	Serial3.begin(921600); // 初始化串口1通信，波特率921600,用于UWB数据接收
	Serial2.begin(115200); // 初始化串口2通信，波特率115200,用于mtf02
	Init_UWB();			   // 初始化UWB相关变量
	Serial.println("\n===================================\n开始初始化\n");

  // 配置遥控器接收引脚并启用中断
  // 4个引脚分别接收Roll、Pitch、Throttle、Yaw的PWM信号
  for (int i = 0; i < 4; i++) {
	  pinMode(receiver_pins[i], INPUT_PULLUP);           // 设置为上拉输入模式
	  enableInterrupt(receiver_pins[i], pwmReceive, CHANGE);  // 启用电平变化中断
  }
  
  sei();  // 全局中断使能
  
	Wire.begin();      // 启动I2C通信（用于MPU6050）
  // Wire.onReceive(receiveEvent);  // I2C接收事件处理（预留）
	TWBR = 24;         // 设置I2C时钟频率为400kHz，提高通信速度

  // 配置电机控制引脚（8个引脚控制4个电机，每个电机2个引脚）
  // 引脚2,3控制左上电机；引脚4,5控制右上电机
  // 引脚6,7控制右下电机；引脚8,9控制左下电机
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);

  pinMode(36, OUTPUT);
  pinMode(38, OUTPUT);
  pinMode(40, OUTPUT);
  pinMode(42, OUTPUT);
  pinMode(44, OUTPUT);
  pinMode(46, OUTPUT);
  pinMode(48, OUTPUT);
  pinMode(50, OUTPUT);

	// 初始化电调（校准电调PWM范围）
	init_ESC();

	// 初始化超声波测距模块
  	Ul_setup();
  	Ul2_setup();
	// 初始化MPU6050传感器（包括陀螺仪零偏校准）
	SetupMPU6050();

	Serial.println("初始化完毕\n===================================\n");

  delay(100);  // 等待系统稳定

	// 电池电压检测初始化
	// 65是二极管电压补偿值
	// 12.6V对应模拟引脚A0的5V输入
	// 12.6V对应analogRead(0)的1023值
	// 1260 / 1023 = 1.2317转换系数
	// 如果电池电压是10.5V，battery_voltage变量将保存1050
	batteryVoltage = (analogRead(0) + 65) * 1.2317;
	
  loop_timer = micros();  // 初始化主循环计时器

  Angle_rate_ekf.LastP = 0.02;
  Angle_rate_ekf.Now_P = 0.0;
  Angle_rate_ekf.out = 0.0; 
  Angle_rate_ekf.Kg = 0.0; 
  Angle_rate_ekf.Q = 0.005;
  Angle_rate_ekf.R = 5.0;
  
}

// ============================================================
// loop()函数：主控制循环
// 以250Hz频率运行，实现飞控核心算法
// 包括状态机管理、姿态解算、PID控制、电机输出
// ============================================================

void loop() {

  	static uint8_t buffer[12]; // 3字节帧头 + 9字节位置
  	static int buffer_index = 0;

	++loop_count;// 主循环计数,用于超声波
	// 计算三轴PID控制量
	++Mtf02_count;
	if(Mtf02_count>1) {
		Mtf02_count=0;
	}
	//Serial.print(Mtf02_count);Serial.print(",");
  //超声波测距（当前未实现）

  
	// ============================================================
	// 状态机：STATE_INIT - 初始化状态
	// 功能：等待解锁条件，进入飞行模式
	// ============================================================
	if (state == STATE_INIT) {
	

		// 解锁条件：当前设置为自动解锁（条件为true）
		// 原设计：遥控器左摇杆位于中下，右摇杆位于中左时解锁
		//if (receiver_input[2] < 1050 && receiver_input[0] < 1050 || 1) {
		if (1) {
			state = STATE_FLY;           // 切换到飞行状态
			setGyroAngle = false;        // 重置陀螺仪角度标志
			pidController.cleanRollPIDData();   // 清空Roll轴PID历史数据
			pidController.cleanPitchPIDData();  // 清空Pitch轴PID历史数据
			pidController.cleanYawPIDData();    // 清空Yaw轴PID历史数据

		} 
		// 校准模式切换条件（已注释）
		else if (receiver_input[2]<1050 && receiver_input[3]<1050 ) {
			// 遥控器左边位于中下，右边位于中下，进入传感器校准模式
			state = STATE_MEAS;
			Serial.println("\n===================================\n开始测量 gyro 误差\n");
			Serial.println("\n=================\n开始测量 Roll, pitch 误差\n");
		}
	//ESC_Ctrl(1000, 0, 0, 0, 0);  // 电机保持最低转速
	} 

	// ============================================================
	// 状态机：STATE_FLY - 飞行状态
	// 功能：执行完整的飞控算法，包括姿态解算、PID控制、电机输出
	// ============================================================
	else if (state == STATE_FLY) {
		// 停机条件检测（已注释）
		if (false) {
			state = STATE_INIT;           // 回到初始化状态
	  setGyroAngle = false;           // 重置标志
			ESC_Ctrl(1000, 0, 0, 0, 0);       // 电机停止
		} 
		else {
			// ============================================================
			// 遥控器目标设定（已注释，当前使用固定目标）
			// 原设计：根据遥控器摇杆位置计算目标角度
			// ============================================================
			// Serial.print(receiver_input[0]);Serial.print(",");
			// Serial.print(receiver_input[1]);Serial.print(",");
			// Serial.print(receiver_input[2]);Serial.print(",");
			// Serial.print(receiver_input[3]);Serial.print(",");
			// if (receiver_input[0] > 1508) 
			// {
			// 	targetRoll = receiver_input[0] - 1508;
			// } else if (receiver_input[0] < 1492) 
			// {
			// 	targetRoll = receiver_input[0] - 1492;
			// }
			
			// if (receiver_input[1] > 1508)
			// {
			// 	targetPitch = receiver_input[1] - 1508;
			// } else if (receiver_input[1] < 1492) 
			// {
			// 	targetPitch = receiver_input[1] - 1492;
			// }

			// Yaw控制设定（已注释）
			// targetYaw=target[2]*10+1000;
	  // if (targetYaw > 1515)
			// {
			// 	targetYaw = targetYaw - 1515;
			// } 
			// else if (targetYaw < 1485) 
			// {
			// 	targetYaw = targetYaw - 1485;
			// }
			// else{
	  //   targetYaw=0;
	  // }

	  // if (receiver_input[3] > 1515)
			// {
			// 	targetYaw = receiver_input[3] - 1515;
			// } else if (receiver_input[3] < 1485) 
			// {
			// 	targetYaw = receiver_input[3] - 1485;
			// }else{
	  //   targetYaw=0;
	  // }
	  
			// }
			// 电机PWM输出调试（已注释）
			// count++;
			// if (count >= 100){
			// 	Serial.println("-----------------");
			// 	Serial.println("左上，右上");
			// 	Serial.print(ESC_PWM[3]);Serial.print(",");
			// 	Serial.println(ESC_PWM[2]);
			// 	Serial.println("左下，右下");
			// 	Serial.print(ESC_PWM[0]);Serial.print(",");
			// 	Serial.println(ESC_PWM[1]);
			// 	Serial.println("-----------------");
			// 	count = 0;
			// }
			
	  // 位置控制模式（已注释，预留接口）
	  // //进行位置控制
	  // targetPitch=(target[0]-50)*9;
	  // targetRoll=(target[1]-50)*9;

	  // //偏差值处理
			// targetRoll = (targetRoll - roll_level_adjust) / 6;
			// targetPitch = (targetPitch - pitch_level_adjust) / 3;
			// targetYaw = -targetYaw/5;

			// 角速度环PID控制（已注释，当前使用角度环）
			// cal_RATE_PID();
			// float rollCORR = pidController.getRollCorrect(PIDController::RATE);
			// float pitchCORR = pidController.getPitchCorrect(PIDController::RATE);
			// float yawCORR = pidController.getYawCorrect(PIDController::RATE);
			
			// ============================================================
			// 核心算法步骤1：姿态解算
			// 使用Mahony滤波算法融合加速度计和陀螺仪数据
			// 参数：x,y 是陀螺仪的安装误差补偿（roll, pitch），单位：度
			// ============================================================
			++state_dinggao_chaoshengbo_count;
			if(state_dinggao_chaoshengbo_count >= 9)
			{
				state_dinggao_chaoshengbo_count = 0;
				if(state_dinggao_chaoshengbo == 2 && receiver_input[3] < 1200){
					state_dinggao_chaoshengbo = 1; // 退出靠近墙体加定高状态
				}
				else if(state_dinggao_chaoshengbo == 1 && receiver_input[3] > 1800){
					state_dinggao_chaoshengbo = 2; // 进入靠近墙体加定高状态
				}
			}

			GetDataMPU6050(roll, pitch, yaw, gx_input, gy_input, gz_input, MTF_Raw_rollRate, MTF_Raw_pitchRate, -1.3, 1.6);//后两位是安装的roll，pitch误差(单位：度)
			Ul_update_distance_cm(); // 超声波测距更新
			Ul_ditonglvbo(); // 超声波测距低通滤波
			Ul2_update_distance_cm(); // 第二个超声波测距更新
			Ul2_ditonglvbo(); // 第二个超声波测距低通滤波

			// 下面两个用于pid内环输入，单位为°
			LPF_1_MTF_Raw_rollRate = 0.15 * MTF_Raw_rollRate + 0.85 * LPF_1_MTF_Raw_rollRate; // Roll轴角速度低通滤波
			LPF_1_MTF_Raw_pitchRate = 0.15 * MTF_Raw_pitchRate + 0.85 * LPF_1_MTF_Raw_pitchRate; // Pitch轴角速度低通滤波
			EKF_1_MTF_Raw_pitchRate = MTF_Raw_pitchRate;
			EKF_1_MTF_Raw_rollRate = MTF_Raw_rollRate;
			kalman_1(&Angle_rate_ekf, EKF_1_MTF_Raw_rollRate);
			kalman_1(&Angle_rate_ekf, EKF_1_MTF_Raw_pitchRate);


			
			// 输出当前姿态角度
			//Serial.print("(");
			// Serial.print(roll, 2);Serial.print(",");
			// Serial.print(pitch, 2);Serial.print(",");
			// Serial.print(yaw, 2);
			// Serial.print(",");
			//Serial.print(") ");

			//读取UWB数据并解析
			if(Mtf02_count == 1){
				while (Serial3.available() > 0) {
					uint8_t byte = Serial3.read();

				// ---------------- 三字节帧头匹配 ----------------
				if (buffer_index == 0) {
				if (byte == 0x01) buffer[buffer_index++] = byte;
				}
				else if (buffer_index == 1) {
				if (byte == 0x00) buffer[buffer_index++] = byte;
				else buffer_index = 0;
				}
				else if (buffer_index == 2) {
				if (byte == 0x02) buffer[buffer_index++] = byte;
				else buffer_index = 0;
				}
				// ---------------- 读取后续9字节 ----------------
				else if (buffer_index > 2 && buffer_index < 12) {
				buffer[buffer_index++] = byte;

				// ---------------- 完整一帧 ----------------
				if (buffer_index >= 12) {
					// 保存帧头
					for (int i = 0; i < 3; i++) current_frame.frame_header[i] = buffer[i];
					// 解析三个方向
					for (int i = 0; i < 3; i++) {
					current_frame.pos[i] = readInt24LE(&buffer[3 + i*3]);
					}
					// 处理数据
					processFrameData();
					// 重置缓冲区
					buffer_index = 0;
					Is_UWB_work = 1; // 设置UWB工作标志，表示已成功接收并解析一帧数据
				}
				}
			}
		}

			// 读取串口数据并解析
			if(Mtf02_count == 1) {
			// while (Serial3.available() > 0 && 0) {
        		// static int ret;
        		// static unsigned char ch;
				// // uint8_t data = Serial3.read();
				// // Serial.print(data, HEX);
        		// // Serial.println();
        		// ch = Serial3.read();
        		// // Serial.println(ch, HEX); 
        		// ret = up_parse_char(ch);
				// if(!ret){
				// 	static int16_t flow_x_integral = 0;
				// 	static int16_t flow_y_integral = 0;
				// 	static uint16_t ground_distance = 0;
				// 	static uint8_t valid = 0;
				// 	static uint8_t tof_confidence = 0;
				// 	flow_x_integral = up_data.flow_x_integral;
				// 	flow_y_integral = up_data.flow_y_integral;
				// 	ground_distance = up_data.ground_distance;
				// 	valid = up_data.valid;
				// 	tof_confidence = up_data.tof_confidence;
				// 	//printf("flow_x_integral=%d,flow_y_integral=%d,ground_distance=%d,valid=%d,tof_confidence=%d\n",flow_x_integral,flow_y_integral,ground_distance,valid,tof_confidence);
          		// 	// // Serial.print("flow_x_integral=");
          		// 	// Serial.print(flow_x_integral);
          		// 	// Serial.print(",");
          		// 	// // Serial.print(",flow_y_integral=");
          		// 	// Serial.print(flow_y_integral);
				// 	// Serial.print(",");
				// 	// // Serial.print(",ground_distance=");
				// 	// Serial.print(ground_distance);
				// 	// Serial.print(",");
				// 	// // Serial.print(",valid=");
				// 	// // Serial.print(valid);
				// 	// // Serial.print(",tof_confidence=");
				// 	// Serial.println(tof_confidence);
				// 	g_flow_vel_x = flow_x_integral;
				// 	g_flow_vel_y = flow_y_integral;
				// 	g_distance = ground_distance + 20.0;
				// 	g_flow_quality = tof_confidence;
				// 	MTF_height =  g_distance; // 单位毫米
				// 	MTF_confidence = tof_confidence;
					
			//Serial.println("MTF02:");
			while (Serial2.available() > 0) {
				uint8_t data = Serial2.read();
				//Serial.println(data, HEX);
				micolink_decode(data); // 调用C文件中的解析函数
				//micolink_read_data(g_flow_vel_x, g_flow_vel_y, g_distance);
			}
			Serial.print(50);
			Serial.print(",");
			Serial.print(-50);
			Serial.print(",");
			Serial.print(g_flow_vel_x);
			Serial.print(",");
			Serial.print(g_flow_vel_y);
			Serial.print(",");
			Serial.println(g_distance);
			MTF_height =  g_distance; // 单位毫米
			
			}
			
			// Serial.print("A:");Serial.print(g_flow_quality);Serial.print(",");
			//  Serial.print("distance: ");Serial.print(MTF_height);Serial.print(" ");
			//Serial.print("100,-100,");
			// Serial.print("");Serial.print(g_flow_vel_x);
			// Serial.print(",");Serial.print(g_flow_vel_y);Serial.println("");
			if(Mtf02_count == 1)
			{
			calPosition(g_flow_vel_x, g_flow_vel_y, MTF_height); // 单位cm/s@1m,高度单位mm//包含对位移速度低通滤波
			
			
			target_X_position = 0.0;
			target_Y_position = 0.0;

			cal_Position_PID();
			target_X_acceleration = -pidController.getSpeedXCorrect(); // X轴双环pid控制量(单位cm/s^2)
			// Serial.print("CORX:");Serial.print(target_X_acceleration);Serial.print(" ");Serial.println("");
			target_Y_acceleration = -pidController.getSpeedYCorrect(); // Y轴双环pid控制量

			//target_X_acceleration_lvbo = 0.25*target_X_acceleration + 0.75*target_X_acceleration_lvbo;


			target_X_acceleration = target_X_acceleration / 10;//防止过大，同时方便调节pid
			target_Y_acceleration = target_Y_acceleration / 10; 
			
			//target_X_acceleration_lvbo = 0.5*target_X_acceleration + 0.5*target_X_acceleration_lvbo;
			//target_Y_acceleration_lvbo = 0.5*target_Y_acceleration + 0.5*target_Y_acceleration_lvbo;

			}
			targetRoll = -target_Y_acceleration; 
			targetRoll = 0;
			constrain_float(targetRoll, -3, 3);
			targetPitch = target_X_acceleration; // 计算目标Roll和Pitch角度(arctan()函数小角度近似线性)
			targetPitch = 0;
			constrain_float(targetPitch, -3, 3);

			// targetRoll = -(atan2(target_Y_acceleration, 9.8f) * rad2angle); // 计算目标Roll角度（单位：度）
			// targetPitch = (atan2(target_X_acceleration, 9.8f) * rad2angle); // 计算目标Pitch角度（单位：度）
			// Serial.print("CORY:");Serial.print(target_Y_acceleration);Serial.print(" ");Serial.println("");

			if( state_dinggao_chaoshengbo == 1){
				targetHigh = MTF_height;
			}
			
			cal_HEIGHT_PID();
			// float heightCORR = 0;
			float heightCORR = pidController.getHeightRateCorrect();

			// Serial.print("HC:");Serial.print(heightCORR);

			
			// Serial.print("mm ");
			// Serial.print("Xrate: ");Serial.print(g_flow_vel_x);
			// Serial.print("  Yrate: ");Serial.print(g_flow_vel_y);



			// // ============================================================
			// // 核心算法步骤2：设定目标姿态
			// // 当前设置为水平悬停模式（所有角度目标为0）
			// // ============================================================
			// targetPitch = 0;  // 目标俯仰角：0度（水平）
			// targetRoll = 0;   // 目标横滚角：0度（水平）
			// targetYaw = 0;    // 目标偏航角：0度（保持当前朝向）

			// ============================================================
			// 核心算法步骤2：设定目标姿态
			// 当前设置为光流姿态矫正模式
			// ============================================================
			

			rcinput_0 = receiver_input[0];  // 获取遥控器输入通道值
			rcinput_1 = receiver_input[1];  // 获取遥控器输入通
			// Serial.print("rcinput0:");Serial.print(rcinput_0);Serial.print(",");
			targetPitch += ((-(rcinput_1-1500))/500) * 10;
			if(targetPitch > 10) {targetPitch = 10;} 
			if(targetPitch < -10) {targetPitch = -10;} // 限制最大俯仰角
			//Serial.print("tgP:");
			//Serial.print("targetPitch:");Serial.print(targetPitch);Serial.print("  ");

			targetRoll += ((rcinput_0-1500)/500) * 10; // 由遥控器输入决定
			if(targetRoll > 10) {targetRoll = 10;} 
			if(targetRoll < -10) {targetRoll = -10;} // 限制最大俯仰角
			//Serial.print("tgR:");
			targetYaw = 0;    // 目标偏航角：0度（保持当前朝向）

			
			// ============================================================
			// 核心算法步骤3：PID控制计算
			// 使用角度环PID，根据当前姿态和目标姿态计算控制量
			// ============================================================
			if(state_check == 1){cal_ANGLE_PID();}// 计算三轴PID控制量
			

			targetRollRate = pidController.getRollCorrect(PIDController::ANGLE);   // Roll轴控制量
			targetPitchRate = pidController.getPitchCorrect(PIDController::ANGLE); // Pitch轴控制量
			targetYawRate = pidController.getYawCorrect(PIDController::ANGLE); // Yaw轴控制量

			if(state_check == 1){cal_RATE_PID();   }// 计算角速度环PID控制量	

			float rollCORR = pidController.getRollCorrect(PIDController::RATE);      // Roll轴控制量（角速度环）
			float pitchCORR = pidController.getPitchCorrect(PIDController::RATE);    // Pitch轴控制量（角速度环）
			float yawCORR = pidController.getYawCorrect(PIDController::RATE);      // Yaw轴控制量（角速度环）
			if(yawCORR > 300) {yawCORR = 300;} // 限制最大偏航控制量
			if(yawCORR < -300) {yawCORR = -300;} // 限制最小偏航控制量	
			
			// 输出PID控制量
			// Serial.print("PID:");Serial.print(rollCORR);Serial.print(",");
			// Serial.print(pitchCORR);Serial.print(",");
			// Serial.print(yawCORR);Serial.print("  ");
			// //Serial.println("");
			// Serial.print("LD:"); Serial.print(ESC_PWM[0]);Serial.print(",");
			// Serial.print("RD:"); Serial.print(ESC_PWM[1]);Serial.print(",");
			// Serial.print("RU:"); Serial.print(ESC_PWM[2]);Serial.print(",");
			// Serial.print("LU:"); Serial.print(ESC_PWM[3]);Serial.print(",");
			// Serial.print("Con:");Serial.print(receiver_input[0]);Serial.print(",");
			// Serial.print(receiver_input[1]);Serial.print(",");
			// Serial.print(receiver_input[2]);Serial.print(",");
			// Serial.print(receiver_input[3]);Serial.print(",");
			//Serial.println("");

			
			

			// Serial.print(rollCORR);
			// Serial.print(pitchCORR);
			
			// ============================================================
			// 电池电压监测和补偿
			// 使用互补滤波器减少噪声，系数0.92用于平滑
			// 0.09853 = 0.08 * 1.2317（转换系数）
			// ============================================================
			batteryVoltage = batteryVoltage * 0.92 + (analogRead(0) + 65) * 0.09853;


			// 备用控制方案（已注释）
			// ESC_Ctrl(1200, rollCORR, pitchCORR, yawCORR, 0);
	  // receiver_input[2]=target[3]*10+1000;
	  // count1++;
	  // if(count1>=250){
	  //   Serial.print("target值:");Serial.print(target[0]);Serial.print(target[1]);Serial.print(target[2]);Serial.println(target[3]);
	  //   count1=0;
	  // }
	  
	  // ============================================================
	  // 核心算法步骤4：电机控制输出
	  // 只有当油门通道值大于1020时才启动电机（安全保护）
	  // ============================================================
	  if (receiver_input[2] > 1020) {
				ESC_Ctrl(receiver_input[2], rollCORR, pitchCORR, yawCORR, heightCORR);  // 执行电机混控
				state_check = 1;  // 设置状态检查标志为已起飞
				// ESC_Ctrl(receiver_input[2], 0, 0, 0, 0);  // 调试用：仅油门，无姿态控制
				
			}
	  else  {
		ESC_Ctrl(1000, 0, 0, 0, 0);       // 电机停止
		state_check = 0;  // 设置状态检查标志为未起飞
	  }
			
			// 外部控制接口（已注释，预留树莓派控制）
			// ESC_Ctrl(receiver_input[2], rollCORR, pitchCORR, yawCORR);
			// Serial.println(receiver_input[2]);
			// if (target[3]*10+1000 > 1020) {
			// 	ESC_Ctrl(target[3]*10+1000, rollCORR, pitchCORR, yawCORR);
			// } 
			// else {
			// 	ESC_Ctrl(1500, 0, 0, 0);
			// 	Serial.print("1");
			// 	pidController.cleanRollPIDData();
			// 	pidController.cleanPitchPIDData();
			// 	pidController.cleanYawPIDData();
			// }
			
	  // 飞行数据输出（已注释）
			// count++;
			// if (count >= 25) {
			// 	outputMidVal(roll, pitch, gz_input, gy_input, gx_input, rollCORR, pitchCORR, yawCORR, receiver_input, targetRoll, targetPitch, targetYaw);
			// 	count = 0;
			// }
		}
	} 
	// ============================================================
	// 状态机：STATE_MEAS - 传感器校准状态
	// 功能：校准陀螺仪零偏和加速度计偏差
	// ============================================================
	else if (state == STATE_MEAS) {
		// measureRPError();  // 执行校准程序（函数已注释）
		// setGyroAngle = false;
		// ESC_Ctrl(1000, 0, 0, 0);  // 电机停止
		
	// 校准完成，返回飞行状态的条件
	if (receiver_input[2]<1050 && receiver_input[3]>1950) {
	  state = STATE_FLY;             // 切换到飞行状态
		  setGyroAngle = false;      // 重置标志
	  pidController.cleanRollPIDData();    // 清空PID数据
	  pidController.cleanPitchPIDData();
	  pidController.cleanYawPIDData();
		}
	}
	last_Height = MTF_height; // 记录上次高度值
	Newcount++;  // 增加新计数器
	if(Newcount>=20 && 0) {
			Serial.print(10);Serial.print(",");
			Serial.print(-10);Serial.print(",");
			// Serial.print(LPF_MTF_dy);Serial.print(",");
			// Serial.print(-LPF_MTF_Raw_rollRate*2.1);Serial.print(",");
			// Serial.print(Ul_duration);Serial.print(","); // 超声波测距距离
			// Serial.print(UL_duration_lvbo);Serial.print(","); // 超声波测距距离低通滤波
			Serial.print(roll, 2);Serial.print(",");
			Serial.print(pitch, 2);Serial.print(",");
			Serial.print(yaw, 2);Serial.print(",");
			// Serial.print(g_flow_vel_x);Serial.print(",");
			// Serial.print(g_flow_vel_y);Serial.print(",");
			// Serial.print(targetRoll);Serial.print(","); // 目标Roll角度
			// Serial.print(targetPitch);Serial.print(","); // 目标Pitch角度
			// Serial.print(Ul_duration);Serial.print(","); // 超声波测距距离
			// Serial.print(Ul_Pos_X_lvbo);Serial.print(","); // 超声波测距距离低通滤波
			// Serial.print(Ul2_Pos_Y_lvbo);Serial.print(","); // 第二个超声波测距距离低通滤波
			// Serial.print(Ul_duration);Serial.print(","); // 超声波测距距离
			// Serial.print(Ul_Pos_X_lvbo);Serial.print(","); // 超声波X轴位置
			// Serial.print(target_Y_speed);Serial.print(","); // 光流X轴位置
			// Serial.print(New_MTF_measured_X_speed);Serial.print(","); 
			// Serial.print(New_MTF_measured_Y_speed);Serial.print(","); 
			// Serial.print(MTF_gyro_fix_dx);Serial.print(",");
			// Serial.print(MTF_gyro_fix_dy);Serial.print(",");
			// Serial.print(MTF_PosX);Serial.print(","); // 光流X轴位置
			// Serial.print(MTF_PosY);Serial.print(","); // 光流Y轴位置
			// Serial.print(MTF_confidence);Serial.print(","); // 光流置信度
			// Serial.print(yaw);Serial.print(",");
			//Serial.print(targetRollRate);Serial.print(",");
			//Serial.print(gx_input);Serial.print(",");
			// Serial.print(gy_input);Serial.print(",");
			// Serial.print(gz_input);Serial.print(",");
			// Serial.print(targetYawRate);Serial.print(",");
			// Serial.print(",");
			//Serial.print(targetRoll);
			//Serial.print(",");
			// Serial.print(targetPitch);
			//Serial.print(",");
			// Serial.print(LPF_1_MTF_Raw_pitchRate);Serial.print(",");
			// Serial.print(LPF_1_MTF_Raw_rollRate);Serial.print(",");
			// Serial.print(EKF_1_MTF_Raw_rollRate);Serial.print(",");
			//Serial.print(MTF_height);Serial.print(",");
			// Serial.print(Pos_dt, 6);Serial.print(",");
			//Serial.print(MTF_gyro_fix_dx);Serial.print(",");
			//Serial.print(MTF_gyro_fix_dy);Serial.print(",");
			// Serial.print(MTF_measured_X_speed);Serial.print(",");
			// Serial.print(MTF_measured_Y_speed);
			//Serial.print(MTF_init_dx);Serial.print(",");
			//Serial.print(MTF_init_dy*2);Serial.print(",");
			//Serial.print(MTF_Test_PosX);Serial.print(",");
			//Serial.print(LPF_MTF_Raw_pitchRate);Serial.print(",");
			// Serial.print(LPF_MTF_Raw_rollRate);Serial.print(",");
			//Serial.print(gy_input);Serial.print(",");
			//Serial.print(gx_input);Serial.print(",");
			//Serial.print(g_flow_quality);
			
			//Serial.print(MTF_Test_PosY);
			// Serial.print(ESC_PWM[0]);Serial.print(",");
			// Serial.print(ESC_PWM[1]);Serial.print(",");
			// Serial.print(ESC_PWM[2]);Serial.print(",");
			// Serial.print(ESC_PWM[3]);//Serial.println("");
			Serial.println("");
			Newcount = 0;  // 重置计数器
		}
	// ============================================================
	// 主循环频率控制：100Hz
	// 确保飞控算法以固定频率运行，保证控制性能
	// 10000微秒 = 10毫秒 = 100Hz频
	// ============================================================
	while (micros() - loop_timer < 10000);  // 等待到达下一个循环时间
	// Serial.print("lt:");Serial.print(micros() - loop_timer);  Serial.println("");
	loop_timer = micros();                  // 更新循环计时器
	// 
}

float mtf_pos_x_i;
float mtf_pos_y_i;
float flow_x_vel_lpf_i;
float flow_y_vel_lpf_i;
float flow_x_pos_lpf_i;
float flow_y_pos_lpf_i;
// float flow_x_lpf_att_i;
// float flow_y_lpf_att_i;
float flow_x_att;
float flow_y_att;
float flow_vel_x_i;
float flow_vel_y_i;
float last_flow_pos_x_i;
float last_flow_pos_y_i;

// float MTF_Pos_dx;
// float MTF_Pos_dy;
// float MTF_Pos_dx_lpf;
// float MTF_Pos_dy_lpf;
// float MTF_dx_att;
// float MTF_dy_att;
// float Last_MTF_Pos_dx;
// float Last_MTF_Pos_dy;
// float MTF_vel_lpf_dx;
// float MTF_vel_lpf_dy;
// float MTF_dx_real;
// float MTF_dy_real;
// float MTF_vel_dx;
// float MTF_vel_dy;
// float MTF_Pos_corbyangle_x;
// float MTF_Pos_corbyangle_y;


void calPosition(int MTF_dx, int MTF_dy, float MTF_height) {

	// ============================================================
	// 1. 记录当前时间，计算与上次的时间间隔（秒）
	Pos_nowtime = micros();
	if(Pos_lasttime == 0.0) {
		Pos_lasttime = Pos_nowtime;  // 第一次调用时初始化
		return;
	}
	Pos_dt = (Pos_nowtime - Pos_lasttime) / 1000000.0;  // 时间间隔，单位：秒
	//Pos_dt = 0.02;
	Pos_lasttime = Pos_nowtime;

	// Serial.print(MTF_dx);Serial.print(",");Serial.print(MTF_dy);Serial.print(",");
	// Serial.print(MTF_height);Serial.print(",");
	//以下参考 《飞 控 端 调 试 光 流 方 法 说 明.pdf》文档
	/*
	第二步：光流角速度的计算
文档描述：光流角速度的计算。若 speed_x 不乘高度即为光流角速度，
表示为 fx = flow_dat.x / (integration_timespan * 0.000001)，同理可求 fy，单位为 rad/s。
	*/

	// 光流速度低通滤波
    //LPF_1_(2.0f, 0.01f, MTF_dx, LPF_MTF_dx); // 光流X轴速度低通滤波
	//LPF_1_(2.0f, 0.01f, MTF_Raw_pitchRate, LPF_MTF_Raw_pitchRate); // 陀螺仪Y轴速度低通滤波

	/*
	*注：由于光流传感器读出数据有延时，用不同的低通滤波参数过滤光流传感器的读出延时
	*/
	LPF_MTF_dx = 0.25 * MTF_dx + 0.75 * LPF_MTF_dx; 
	LPF_MTF_dy = 0.25 * MTF_dy + 0.75 * LPF_MTF_dy; // 光流Y轴速度低通滤波

	//pitch_Delay_three_times(MTF_Raw_pitchRate, Delay_MTF_Raw_pitchRate); // 陀螺仪Y轴速度延时//单位°每秒
	pitch_Delay_two_point_five_times(MTF_Raw_pitchRate, Delay_MTF_Raw_pitchRate); // 陀螺仪Y轴速度延时//单位°每秒
	//roll_Delay_three_times(MTF_Raw_rollRate, Delay_MTF_Raw_rollRate); // 陀螺仪X轴速度延时
	roll_Delay_three_times(MTF_Raw_rollRate, Delay_MTF_Raw_rollRate); // 陀螺仪X轴速度延时
	LPF_MTF_Raw_pitchRate = 0.25* Delay_MTF_Raw_pitchRate + 0.75 * LPF_MTF_Raw_pitchRate; // 陀螺仪Y轴速度低通滤波
	LPF_MTF_Raw_rollRate = 0.25 * Delay_MTF_Raw_rollRate + 0.75 * LPF_MTF_Raw_rollRate; // 陀螺仪X轴速度低通滤波
	// 下面是旋转补偿

	//注意：对于优象光流，输入的MTF_dx单位为rad/10000,时间间隔可取20ms
	MTF_init_dx = LPF_MTF_dx;
	MTF_init_dy = LPF_MTF_dy;
	//MTF_init_dx = MTF_init_dx /10000.0f /0.02f * 57.2958f; // 转换为度每秒
	//MTF_init_dx *= 0.2865; // 转换为度每秒,系数为57.2958/10000/0.02
	MTF_init_dx = MTF_init_dx *57.2958 /Pos_dt /10000.0; // 转换为度每秒	
	MTF_init_dy *= 0.2865; // 转换为度每秒
	

	LPF_2_MTF_Raw_pitchRate = LPF_MTF_Raw_pitchRate;
	LPF_2_MTF_Raw_rollRate = LPF_MTF_Raw_rollRate;
	// constrain_float(LPF_2_MTF_Raw_pitchRate, -15, 15);
	// constrain_float(LPF_2_MTF_Raw_rollRate, -15, 15);
	// Serial.print(50);Serial.print(",");
	// Serial.print(-50);Serial.print(",");
	// Serial.print(MTF_init_dy);Serial.print(",");
	// Serial.print(-0.8*LPF_2_MTF_Raw_rollRate);Serial.print(",");	
	// Serial.print(MTF_init_dx);Serial.print(",");
	// Serial.print(0.8*LPF_2_MTF_Raw_pitchRate);
	// Serial.println("");

	MTF_init_dx =MTF_init_dx - 0.7*LPF_2_MTF_Raw_pitchRate;
	MTF_init_dy =MTF_init_dy + 0.8*LPF_2_MTF_Raw_rollRate;


	// LPF_MTF_dx = LPF_MTF_dx - 1.5*LPF_MTF_Raw_pitchRate;
	// LPF_MTF_dy -= -LPF_MTF_Raw_rollRate * 2.1; //系数2.1是实验出来的
	// 光流速度计算：单位为m/s@1m（即rad/s）
	//MTF_init_dx = LPF_MTF_dx;
	//MTF_init_dy = LPF_MTF_dy;
	MTF_init_dx /= 57.2958f; // 转换为m/s@1m(即rad/s)
	MTF_init_dy /= 57.2958f; // 转换为m/s@1m(即rad/s)
	/*
	第三步：光流旋转补偿
	文档描述：使用陀螺仪角速度对光流进行旋转补偿，目的是使得飞机在只有旋转没有平移时光流最终输出为零。
	*/

	
	//MTF_height = 300.0f; // 测试用，固定高度30cm
	MTF_gyro_fix_dx = MTF_init_dx;
	MTF_gyro_fix_dy = MTF_init_dy;
	MTF_gyro_fix_dx *= (MTF_height / 10.0f); // 计算速度，单位为cm/s(原本是m/s@1m(rad/s))
	MTF_gyro_fix_dy *= (MTF_height / 10.0f); // 计算速度，单位为cm/s
	//LPF_1_(2.0f, 0.01f, MTF_gyro_fix_dx, LPF_MTF_gyro_fix_dx); // 光流X轴速度低通滤波
	MTF_init_dx *= (MTF_height / 10.0f); // 光流速度转换为cm/s@1m
	MTF_init_dy *= (MTF_height / 10.0f); // 光流速度转换为cm/s@1m(测试用)

	//LPF_1_(2.0f, 0.01f, MTF_init_dx, LPF_MTF_init_dx); // 光流X轴速度低通滤波

	//第五步，积分得到位移
	MTF_PosX += MTF_gyro_fix_dx * Pos_dt; // X轴位移，单位为cm
	MTF_PosY += MTF_gyro_fix_dy * Pos_dt; // Y轴位移，单位为cm

	MTF_Test_PosX += MTF_init_dx * Pos_dt; // 测试用，X轴位移，单位为cm
	MTF_Test_PosY += MTF_init_dy * Pos_dt; // 测试用，Y轴位移，单位为cm

	//速度滤波
	MTF_measured_X_speed = 0.15 * MTF_gyro_fix_dx + 0.85 * MTF_measured_X_speed; 
	MTF_measured_Y_speed = 0.15 * MTF_gyro_fix_dy + 0.85 * MTF_measured_Y_speed;

	// 获得水平航向坐标系下的加速度
	float AccX;
	float AccY;
	getWorldAcc(AccX, AccY);//m/s^2
	// 坐标解耦：从水平航向坐标系转移到世界坐标系(东北地坐标系)
	float AccX_world =  AccX * cosf(yaw * angle2rad) - AccY * sinf(yaw * angle2rad);
	float AccY_world =  AccX * sinf(yaw * angle2rad) + AccY * cosf(yaw * angle2rad);
	//加速度积分得到速度
	// float AccX_speed += AccX * Pos_dt * 100.0f; // cm/s
	// float AccY_speed += AccY * Pos_dt * 100.0f; // cm/s

	New_MTF_measured_X_speed = 0.95* (New_MTF_measured_X_speed + AccX * Pos_dt * 100.0) + 0.05* MTF_gyro_fix_dx;
	New_MTF_measured_Y_speed = 0.95* (New_MTF_measured_Y_speed + AccY * Pos_dt * 100.0) + 0.05* MTF_gyro_fix_dy;

	New_MTF_measured_X_speed_lvbo = 0.2* New_MTF_measured_X_speed + 0.8* New_MTF_measured_X_speed_lvbo;
	New_MTF_measured_Y_speed_lvbo = 0.2* New_MTF_measured_Y_speed + 0.8* New_MTF_measured_Y_speed_lvbo;

	Acc_PosX += New_MTF_measured_X_speed * Pos_dt; // X轴位移，单位为cm
	Acc_PosY += New_MTF_measured_Y_speed * Pos_dt; // Y轴位移，单位为cm

	real_MTF_PosX = 0.9* (real_MTF_PosX + New_MTF_measured_X_speed_lvbo * Pos_dt) + 0.1 * MTF_PosX;// 实际位置融合
	real_MTF_PosY = 0.9* (real_MTF_PosY + New_MTF_measured_Y_speed_lvbo * Pos_dt) + 0.1 * MTF_PosY;


	//这一部分用于uwb和imu融合方案

	float World_AccX;
	float World_AccY;
	get_WorldAcc(World_AccX, World_AccY);//m/s^2
	// EKF融合位置和速度
	ekfX.update(World_AccX, real_pos[0], Is_UWB_work);
	ekfY.update(World_AccY, real_pos[1], Is_UWB_work);
  	ekf_X.pos = ekfX.getPos();
  	ekf_X.speed = ekfX.getSpeed();
  	ekf_Y.pos = ekfY.getPos();
  	ekf_Y.speed = ekfY.getSpeed();

 	// 可选：串口打印结果（方便调试，无其他功能）
  	// Serial.print("X: "); Serial.print(ekf_X.pos, 2);
  	// Serial.print(","); Serial.print(ekf_X.speed, 2);
  	// Serial.print(" Y: "); Serial.print(ekf_Y.pos, 2);
  	// Serial.print(","); Serial.print(ekf_Y.speed, 2);
  	// Serial.println();

	// Serial.print(5);Serial.print(",");
	// Serial.print(-5);Serial.print(",");
	// Serial.print(World_AccX);Serial.print(",");
	// Serial.println(World_AccY);
	// Serial.print(ekf_X.pos);
	// Serial.print(",");
	// Serial.print(ekf_Y.pos);
	// Serial.print(",");
	// Serial.print(real_pos[0]);
	// Serial.print(",");
	// Serial.print(real_pos[1]);
	// Serial.println("");


	// // 输出数据，用于存储到电脑
	// // Serial.print("UWB:");
	// Serial.print(current_frame.pos[0]);
	// Serial.print(" ");
	// Serial.print(current_frame.pos[1]);
	// Serial.println();

	//测试可视化功能
	// static float test_temp = 0;
	// if (test_temp <4.5){
	// 	test_temp += 0.005;
	// }
	// else {
	// 	test_temp = 0;
	// }
	// Serial.print(test_temp);
	// Serial.print(",");
	// Serial.print(test_temp);
	// Serial.print(",");
	// Serial.print(test_temp);
	// Serial.print(",");
	// Serial.print(4.5 - test_temp);
	// Serial.println("");

	// Last_MTF_gyro_fix_dx = MTF_gyro_fix_dx; 
	// Last_MTF_gyro_fix_dy = MTF_gyro_fix_dy; 
	
	
	// // 2. 光流速度积分，得到累计位移（简单积分，未乘以dt，假定采样周期较稳定）cm/s@1m转化为m/s@1m(既rad/s)
	// mtf_pos_x_i += MTF_dx/10.0f; // X轴速度积分
	// mtf_pos_y_i += MTF_dy/10.0f; // Y轴速度积分



	// // 3. 对积分得到的位移做一阶低通滤波，减少抖动
	// flow_x_pos_lpf_i += (mtf_pos_x_i - flow_x_pos_lpf_i) * 0.10;
	// flow_y_pos_lpf_i += (mtf_pos_y_i - flow_y_pos_lpf_i) * 0.10;

	// // 4. 姿态角补偿：补偿因pitch/roll倾斜导致的地面投影误差
	// // flow_x_att += (700.0f*tanf(pitch *angle2rad) - flow_x_att) *0.10; // 前后补偿
	// // flow_y_att += (700.0f*tanf(roll *angle2rad) - flow_y_att) *0.10;  // 左右补偿
	// flow_x_att += (700.0f*tanf(0 *angle2rad) - flow_x_att) *0.10; // 前后补偿
	// flow_y_att += (700.0f*tanf(0 *angle2rad) - flow_y_att) *0.10;  // 左右补偿

	// // 5. 融合补偿后的位移，再做一次低通滤波，得到最终用于控制的位移估计
	// flow_x_lpf_att_i = (flow_x_pos_lpf_i + flow_x_att)*0.08f;
	// flow_y_lpf_att_i = (flow_y_pos_lpf_i + flow_y_att)*0.08f;

	// // 6. 用当前和上次的位移差估算速度（乘以10是因为主循环大约100Hz）
	// flow_vel_x_i = (flow_x_lpf_att_i - last_flow_pos_x_i)*10;
	// flow_vel_y_i = (flow_y_lpf_att_i - last_flow_pos_y_i)*10;
	// last_flow_pos_x_i = flow_x_lpf_att_i;
	// last_flow_pos_y_i = flow_y_lpf_att_i;

	// // 7. 对速度做低通滤波，并根据高度做动态增益补偿
	// flow_x_vel_lpf_i += (flow_vel_x_i - flow_x_vel_lpf_i) * 0.15f;
	// flow_y_vel_lpf_i += (flow_vel_y_i - flow_y_vel_lpf_i) * 0.15f;
	// flow_x_vel_lpf_i += +flow_vel_x_i*(MTF_height/3400.f);
	// flow_y_vel_lpf_i += +flow_vel_y_i*(MTF_height/3400.f);

	// 8. 串口输出调试信息
	// Serial.print(5);Serial.print(",");
	// Serial.print(-5);Serial.print(",");
	//Serial.print(flow_x_lpf_att_i);Serial.print(",");
	//Serial.print(flow_y_lpf_att_i);Serial.print("");

	// 下面是注释掉的其他位置估算方案和调试代码
	// ...existing code...
		// 计算位置估计
	// MTF_dx, MTF_dy单位为cm/s@1m,高度单位mm

	// MTF_Pos_dx += MTF_dx;  // X轴位移（未考虑高度和积分时间）
	// MTF_Pos_dy += MTF_dy;  // Y轴位移（未考虑高度和积分时间）

	// MTF_Pos_dx_lpf += (MTF_Pos_dx - MTF_Pos_dx_lpf) * 0.1;  // X轴位置低通滤波
	// MTF_Pos_dy_lpf += (MTF_Pos_dy - MTF_Pos_dy_lpf) * 0.1;  // Y轴位置低通滤波

    // MTF_dx_att += (600.0f*tanf(pitch * angle2rad) -MTF_dx_att) * 0.1;  // X轴位移角度补偿（低通滤波）
	// MTF_dy_att += (600.0f*tanf(roll * angle2rad) -MTF_dy_att) * 0.1;  // Y轴位移角度补偿（低通滤波）

	// // MTF_dx_att += (MTF_height*tanf(pitch * angle2rad) -MTF_dx_att) * 0.9;  // X轴位移角度补偿（低通滤波）
	// // MTF_dy_att += (MTF_height*tanf(roll * angle2rad) -MTF_dy_att) * 0.9;  // Y轴位移角度补偿（低通滤波）

	
	// MTF_Pos_corbyangle_x = (MTF_Pos_dx_lpf + MTF_dx_att)*0.08;  // X轴位置角度补偿
	// MTF_Pos_corbyangle_y = (MTF_Pos_dy_lpf + MTF_dy_att)*0.08;  // Y轴位置角度补偿
	
	// Serial.print("MTF_Pos_corbyangle_x:");Serial.print(MTF_Pos_corbyangle_x);Serial.print(" ");
	// Serial.print("MTF_Pos_corbyangle_y:");Serial.print(MTF_Pos_corbyangle_y);Serial.print(" ");

	// MTF_vel_dx = MTF_Pos_corbyangle_x - Last_MTF_Pos_dx;  // X轴位移速度
	// MTF_vel_dy = MTF_Pos_corbyangle_y - Last_MTF_Pos_dy;  // Y轴位移速度

	// Last_MTF_Pos_dx = MTF_Pos_corbyangle_x;  // 保存上次X轴位置
	// Last_MTF_Pos_dy = MTF_Pos_corbyangle_y;  // 保存上次Y轴位置

	// MTF_vel_lpf_dx += (MTF_vel_dx - MTF_vel_lpf_dx) * 0.1;  // X轴位移速度低通滤波
	// MTF_vel_lpf_dy += (MTF_vel_dy - MTF_vel_lpf_dy) * 0.1;  // Y轴位移速度低通滤波

	// MTF_dx_real = MTF_vel_lpf_dx;  // 更新X轴位移速度
	// MTF_dy_real = MTF_vel_lpf_dy;  // 更新Y轴位移速度

	// MTF_dx_real = MTF_dx
	//计算实际位移速度
	// float Pos_real_dx = MTF_dx_real * (MTF_height / 1000.0);  // 光流单位：cm/s@1m 
	// float Pos_real_dy = MTF_dy_real * (MTF_height / 1000.0);  // 光流单位：cm/s@1m 
	// Serial.print("Pos_real_dx:");Serial.print(Pos_real_dx);Serial.print(" ");
	// Serial.print("Pos_real_dy:");Serial.print(Pos_real_dy);Serial.print(" ");

	// X_position += Pos_real_dx * Pos_dt;  // 更新X轴位置
	// Y_position += Pos_real_dy * Pos_dt;  // 更新Y轴位置 

	// X_position = 0.5 * Last_X_position + 0.5 * X_position;  // 低通滤波（错误写法，会导致量纲变小）
	// Y_position = 0.5 * Last_Y_position + 0.5 * Y_position;  // 低通滤波


	// Last_X_position = X_position;  // 保存上次位置
	// Last_Y_position = Y_position;  // 保存上次位置


	// Serial.print(X_position); Serial.print(",");
	// Serial.print(Y_position); 
//	Serial.println("");
}

// void EKF_UWB_IMU(float imu_accel, float uwb_pos, int Is_UWB_work) {
// 	// EKF融合UWB和IMU数据
// 	// imu_accel：IMU加速度数据，单位m/s^2
// 	// uwb_pos：UWB位置数据，单位m
// 	// Is_UWB_work：UWB工作状态标志，1表示工作正常，0表示异常
// 	//使用二维卡尔曼滤波器进行融合
// 	//默认时间间隔德塔t=0.01s

// 	static float fusion_pos = 0.0; // 融合位置
// 	static float fusion_vel = 0.0; // 融合速度
// 	static float predicted_pos = 0.0; // 预测位置
// 	static float predicted_vel = 0.0; // 预测速度
// 	static float P[2][2] = { {1, 1}, {1, 1} }; // 误差协方差矩阵
// 	static float Q_pos = 0.1; // 过程噪声方差（位置）
// 	static float Q_vel = 0.1; // 过程噪声方差（速度）
// 	static float uwb_dt = 0.01; // 时间间隔，单位秒
// 	static float K[2]; // 卡尔曼增益
// 	static float R_uwb = 0.1; // UWB测量噪声方差

// 	// 第一步：预测步骤
// 	predicted_pos = fusion_pos + fusion_vel * uwb_dt + 0.5 * imu_accel * uwb_dt * uwb_dt;
// 	predicted_vel = fusion_vel + imu_accel * uwb_dt;

// 	// 第二步：更新P矩阵
// 	float p00 = P[0][0]; // 保存原始P[0][0]
//     float p01 = P[0][1]; // 保存原始P[0][1]
//     float p10 = P[1][0]; // 保存原始P[1][0]
//     float p11 = P[1][1]; // 保存原始P[1][1]

//     // 用原始值同步计算新的P矩阵元素（无交叉依赖，同步更新）
//     // 修正原代码符号错误：-p00 → +p00，符合EKF协方差预测公式
//     P[0][0] = p00 + 2 * uwb_dt * p01 + uwb_dt * uwb_dt * p11 + Q_pos;
//     P[0][1] = p01 + uwb_dt * p11;
//     P[1][0] = P[0][1]; // 协方差矩阵是对称的，直接赋值，无需重复计算
//     P[1][1] = p11 + Q_vel;

// 	// 第三步：计算卡尔曼增益(当UWB工作正常时)
// 	if (Is_UWB_work == 1) {
// 		K[0] = P[0][0] / (P[0][0] + R_uwb);
// 		K[1] = P[1][0] / (P[0][0] + R_uwb);

// 		// 第四步：更新步骤
// 		fusion_pos = predicted_pos + K[0] * (uwb_pos - predicted_pos);
// 		fusion_vel = predicted_vel + K[1] * (uwb_pos - predicted_pos);

// 		// 第五步：更新P矩阵
// 		P[0][0] = (1 - K[0]) * P[0][0];
// 		P[0][1] = (1 - K[0]) * P[0][1];
// 		P[1][0] = P[0][1]; // 协方差矩阵是对称的，直接赋值，无需重复计算
// 		P[1][1] = P[1][1] - K[1] * P[0][1];
// 	}
// }

// ============================================================
// PID控制函数：角度环PID计算
// 功能：根据当前姿态和目标姿态计算PID控制量
// ============================================================
void cal_ANGLE_PID(){
	pidController.calCurrentRollAnglePID(roll, targetRoll);      // Roll轴角度环PID
	pidController.calCurrentPitchAnglePID(pitch, targetPitch);   // Pitch轴角度环PID
	pidController.calCurrentYawAnglePID(yaw, targetYaw);         // Yaw轴角速度环PID（注意：Yaw使用角速度环）
}

void cal_Position_PID(){
	// 计算位置PID控制量
	++Mtf02_count_1;
	if(Mtf02_count_1 > 1){ Mtf02_count_1 =0; }
	if(Mtf02_count_1 == 0)
	{
		if(loop_count <= 110 || 1)
		{
			pidController.calCurrentPosXPID(real_MTF_PosX, target_X_position);  // X轴位置环PID(单位cm)
			pidController.calCurrentPosYPID(real_MTF_PosY, target_Y_position);  // Y轴位置环PID
		}
		else if(loop_count > 110 && 0 )
		{
			pidController.calCurrentPosXPID(Ul_Pos_X_lvbo, target_X_position);  // X轴位置环PID(单位cm)
			pidController.calCurrentPosYPID(real_MTF_PosY, target_Y_position);  // Y轴位置环PID
		}
	}
	target_X_speed = pidController.getPosXCorrect(); // 获取X轴期望速度(单位cm/s)
	if(state_dinggao_chaoshengbo == 2 && UL_duration_lvbo > 60) {
		target_X_speed = 1.0;
		real_MTF_PosX = 0.0;
	}
	target_Y_speed = pidController.getPosYCorrect(); // 获取Y轴期望速度
	pidController.calCurrentSpeedXPID(New_MTF_measured_X_speed_lvbo, target_X_speed); // X轴速度环PID
	pidController.calCurrentSpeedYPID(New_MTF_measured_Y_speed_lvbo, target_Y_speed); // Y轴速度环PID	
	// Serial.print(10);Serial.print(",");Serial.print(-10);Serial.print(",");
	// Serial.print(MTF_measured_X_speed);Serial.print(",");Serial.print(target_X_speed);Serial.print(",");
	// Serial.print(MTF_measured_Y_speed);Serial.print(",");Serial.println(target_Y_speed);
	//结果单位(cm/(s*s))
	// Serial.print(" X_position:");Serial.print(X_position);Serial.print(" ");
	// Serial.print(" Y_position:");Serial.print(Y_position);Serial.print(" ");
}

void cal_HEIGHT_PID(){
	// 计算高度PID控制量
	pidController.calCurrentHeightPID(g_distance/10, targetHigh/10);  // 高度环PID控制
	targetHeightRate = pidController.getHeightCorrect(); // 获取期望高度变化率(单位mm/s)
	Height_Rate = (MTF_height - last_Height) / 0.1; // 实际高度变化率(单位mm/s2)
	LPF_Height_Rate = 0.3 * Height_Rate + 0.7 * LPF_Height_Rate; // 高度变化率低通滤波
	pidController.calCurrentHeightRatePID(Height_Rate, targetHeightRate); // 高度变化率环PID
}



// ============================================================
// PID控制函数：角速度环PID计算（备用）
// 功能：直接对角速度进行PID控制，响应更快但稳定性较差
// ============================================================
void cal_RATE_PID() {
	LPF_1_MTF_Raw_rollRate = 0.15 * MTF_Raw_rollRate + 0.85 * LPF_1_MTF_Raw_rollRate; // Roll轴角速度低通滤波
	LPF_1_MTF_Raw_pitchRate = 0.15 * MTF_Raw_pitchRate + 0.85 * LPF_1_MTF_Raw_pitchRate; // Pitch轴角速度低通滤波
	pidController.calCurrentRollRatePID(gx_input, targetRollRate);   // Roll轴角速度环PID
	pidController.calCurrentPitchRatePID(gy_input, targetPitchRate); // Pitch轴角速度环PID
	pidController.calCurrentYawRatePID(gz_input, targetYawRate);  // Yaw轴角速度环PID
}



// ============================================================
// 电机控制函数：四轴电机混控算法
// 功能：根据油门和三轴控制量计算每个电机的PWM值
// 参数：throttle-油门值, rollCORR-Roll修正, pitchCORR-Pitch修正, yawCORR-Yaw修正
// ============================================================
void ESC_Ctrl(int throttle, float rollCORR, float pitchCORR, float yawCORR, float heightCORR) {
  //count1++;
  
	// if (throttle > 1700) throttle = 1700;  // 油门限幅保护
	
  // ============================================================
  // 四轴电机混控公式
  // 基本原理：每个电机的转速 = 基础油门 ± 各轴控制量
  // ============================================================
  int tem;
  if(rollCORR == 0) {
	tem = 0;}
  else {
	tem = 0;}
  	int temp= - rollCORR + pitchCORR - yawCORR - heightCORR;
	constrain_int(temp, -200, 200);
  	// temp=0;
	ESC_PWM[0] = throttle- temp + tem;  // 左下电机(LD)：7,8号引脚控制
	constrain_int(ESC_PWM[0], 1000, 2000);
	// Serial.print("LD:"); Serial.print(ESC_PWM[0]);Serial.print(",");
	 
  	temp=  rollCORR + pitchCORR + yawCORR - heightCORR;
	constrain_int(temp, -200, 200);
  	// temp=0;
	ESC_PWM[1] = throttle- temp + tem;  // 右下电机(RD)：5,6号引脚控制
	constrain_int(ESC_PWM[1], 1000, 2000);
	// Serial.print("RD:"); Serial.print(ESC_PWM[1]);Serial.print(",");
	
  	temp=  rollCORR - pitchCORR - yawCORR - heightCORR;
  	constrain_int(temp, -200, 200);
	// temp=0;
	ESC_PWM[2] = throttle- temp - tem;  // 右上电机(RU)：3,4号引脚控制
	constrain_int(ESC_PWM[2], 1000, 2000);
	// Serial.print("RU:"); Serial.print(ESC_PWM[2]);Serial.print(",");
	
  	temp= - rollCORR - pitchCORR + yawCORR - heightCORR;
  	constrain_int(temp, -200, 200);
  	// temp=0;
	ESC_PWM[3] = throttle- temp - tem;  // 左上电机(LU)：1,2号引脚控制
	constrain_int(ESC_PWM[3], 1000, 2000);
	// Serial.print("LU:"); Serial.print(ESC_PWM[3]);Serial.print("");
	
  // 调试输出（已注释）
  // if(count1>=25){
  //   count1=0;
  //   Serial.print("throttle:");Serial.print(ESC_PWM[0]);Serial.print(ESC_PWM[1]);Serial.print(ESC_PWM[2]);Serial.println(ESC_PWM[3]);
  // }
  
	// 电池电压补偿（已注释）
	// 当电池电压下降时，自动增加PWM值以维持相同的动力输出
	// if (batteryVoltage < 1240 && batteryVoltage > 800) {
	// 	ESC_PWM[0] += ESC_PWM[0] * ((1240 - batteryVoltage) / (float)3500);
	// 	ESC_PWM[1] += ESC_PWM[1] * ((1240 - batteryVoltage) / (float)3500);
	// 	ESC_PWM[2] += ESC_PWM[2] * ((1240 - batteryVoltage) / (float)3500);
	// 	ESC_PWM[3] += ESC_PWM[3] * ((1240 - batteryVoltage) / (float)3500);
	// }

  // Serial.print("yaw修正值:");Serial.print(gz_input);Serial.println(yawCORR);

	// ============================================================
	// 硬件PWM输出实现
	// 使用软件模拟PWM信号，周期20ms，脉宽1-2ms
	// ============================================================
	zero_timer = micros();  // 记录PWM周期开始时间
	esc_loop_timer = zero_timer;  // 初始化循环计时器
	setHigh();              // 所有引脚输出高电平
  
	timer_channel_1 = ESC_PWM[0] + zero_timer;  // 计算左下电机PWM结束时间
	timer_channel_2 = ESC_PWM[1] + zero_timer;  // 计算右下电机PWM结束时间
	timer_channel_3 = ESC_PWM[2] + zero_timer;  // 计算右上电机PWM结束时间
	timer_channel_4 = ESC_PWM[3] + zero_timer;  // 计算左上电机PWM结束时间

	// PWM输出循环：根据时间精确控制每个引脚的高电平持续时间
	// 确保此代码与物理电机模型匹配
	ESC_Ctrl_Flag_1 = false;  // 重置电调控制标志1
	ESC_Ctrl_Flag_2 = false;  // 重置电调控制标志2
	ESC_Ctrl_Flag_3 = false;  // 重置电调控制标志3
	ESC_Ctrl_Flag_4 = false;  // 重置电调控制标志4

	while (ESC_Ctrl_Flag_1 == false || ESC_Ctrl_Flag_2 == false || ESC_Ctrl_Flag_3 == false || ESC_Ctrl_Flag_4 == false) {                                         
		esc_loop_timer = micros();                // 检查当前时间
		if(timer_channel_1 <= esc_loop_timer) {digitalWrite(8,LOW);digitalWrite(9,LOW);digitalWrite(48,LOW);digitalWrite(50,LOW);ESC_Ctrl_Flag_1 = true;}  // 左下电机停止高电平
		if(timer_channel_2 <= esc_loop_timer) {digitalWrite(6,LOW);digitalWrite(7,LOW);digitalWrite(44,LOW);digitalWrite(46,LOW);ESC_Ctrl_Flag_2 = true;}  // 右下电机停止高电平
		if(timer_channel_3 <= esc_loop_timer) {digitalWrite(4,LOW);digitalWrite(5,LOW);digitalWrite(40,LOW);digitalWrite(42,LOW);ESC_Ctrl_Flag_3 = true;}  // 右上电机停止高电平
		if(timer_channel_4 <= esc_loop_timer) {digitalWrite(2,LOW);digitalWrite(3,LOW);digitalWrite(36,LOW);digitalWrite(38,LOW);ESC_Ctrl_Flag_4 = true;}  // 左上电机停止高电平
	}
}


// void measureRPError() {
// 	for (measureCount; measureCount < measureTime; measureCount++)
// 	{
// 		mpu.readSensor();
//     	ax = mpu.getAccX();
//     	ay = mpu.getAccY();
//     	az = mpu.getAccZ();

// 		acc_total = sqrt((ax*ax)+(ay*ay)+(az*az));

// 		if (abs(ay) < acc_total) {
// 		RAve += asin((float)ay / acc_total) * 57.296;
// 		}
	
// 		if (abs(ax) < acc_total) {
// 		PAve += asin((float)ax / acc_total) * -57.296;
// 		}
// 	}
// 	RAve /= measureTime;
// 	PAve /= measureTime;
// 	// Serial.print("    RollAccError: "); Serial.println(RAve);
// 	// Serial.print("    PitchAccError: "); Serial.println(PAve);
// 	// Serial.println("\n测量完毕\n===================================\n");
// 	delay(2000);
// 	state = STATE_INIT;
// 	measureCount = 0;
// }




void printSpace(float val){
	if (val < 0){
		if (abs(val) >= 100){ Serial.print(" "); }
		else if (abs(val) >= 10) { Serial.print("  "); }
		else {Serial.print("   ");}
	}
	else {
		if (abs(val) >= 100){ Serial.print("  "); }
		else if (abs(val) >= 10) { Serial.print("   "); }
		else {Serial.print("    ");}
	}
}


void init_ESC() {
  Serial.println("开始校准");
  pinMode(4,OUTPUT);//选取引脚接电调信号线
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);

  pinMode(36,OUTPUT);
  pinMode(38,OUTPUT);
  pinMode(40,OUTPUT);
  pinMode(42,OUTPUT);
  pinMode(44,OUTPUT);
  pinMode(46,OUTPUT);
  pinMode(48,OUTPUT);
  pinMode(50,OUTPUT);

  
//这段代码禁止启用！！！，输出2000us高电平时若电机不处于校准状态，会直接满转速//或不上浆
//for(int i=0;i<=1;i++){
//  setHigh();
//  delayMicroseconds(2);//高电平持续2000微秒（油门最高点，脉宽为2毫秒）
//  setLow();
//  delayMicroseconds(1);
// //}
//   Serial.println("高位完成");

//   //该循环运行时会伴有N声短鸣声（表示锂电池节数）和“哔-”油门最低点确认音
// //   for(int i=0;i<=1;i++){
//     setHigh();
//     delayMicroseconds(1);//高电平持续1000微秒（油门最低点，脉宽为1毫秒）
//     setLow();
//     delayMicroseconds(1);
// //  }
//   Serial.println("低位完成");

  // for(int i=0;i<=1500;i++){
  //   setHigh();
  //   delayMicroseconds(1100);//油门1100
  //   setLow();
  //   delayMicroseconds(1100);
  // }

	//不要启用，电机会转
	// setHigh();
	// delayMicroseconds(2000);
	// setLow();
	// delayMicroseconds(200);  
	// for(int i=0;i<200;i++){
	// 	setHigh();  // 设置所有引脚为高电平
	// 	delayMicroseconds(2000);  // 高电平持续2000微秒（油门最高点，脉宽为2毫秒）
	// 	setLow();   // 设置所有引脚为低电平
	// 	delayMicroseconds(8000);   // 低电平持续8000微秒
	// }
	Serial.println("高位完成");
	// for(int i=0;i<400;i++){
	// 	setHigh();  // 设置所有引脚为高电平
	// 	delayMicroseconds(1000);  // 高电平持续1000微秒（油门最低点，脉宽为1毫秒）
	// 	setLow();   // 设置所有引脚为低电平
	// 	delayMicroseconds(9000);   // 低电平持续9000微秒
	// }
	Serial.println("低位完成");  
	Serial.println("校准完成(111232为正确初始化代码,111为错误初始化代码)");


}

void setHigh(){
  digitalWrite(2,HIGH);
  digitalWrite(3,HIGH);
  digitalWrite(8,HIGH);
  digitalWrite(9,HIGH);
  digitalWrite(4,HIGH);
  digitalWrite(5,HIGH);
  digitalWrite(6,HIGH);
  digitalWrite(7,HIGH);

  digitalWrite(36,HIGH);
  digitalWrite(38,HIGH);
  digitalWrite(40,HIGH);
  digitalWrite(42,HIGH);
  digitalWrite(44,HIGH);
  digitalWrite(46,HIGH);
  digitalWrite(48,HIGH);
  digitalWrite(50,HIGH);
}

void setLow(){
  digitalWrite(2,LOW);
  digitalWrite(3,LOW);
  digitalWrite(8,LOW);
  digitalWrite(9,LOW);
  digitalWrite(4,LOW);
  digitalWrite(5,LOW);
  digitalWrite(6,LOW);
  digitalWrite(7,LOW);
}

 
 
// ============================================================
// PWM接收中断服务函数
// 功能：检测遥控器PWM信号的上升沿和下降沿，计算脉宽
// 工作原理：在信号变化时触发中断，记录时间差得到PWM脉宽
// ============================================================
void pwmReceive() {
	// 获取触发中断的引脚编号
	int currPin = arduinoInterruptedPin;
	// 获取当前微秒时间
	unsigned long currTime = micros();
 
	// 获取当前引脚的电平状态
	// 0表示从高电平到低电平，>0表示从低电平到高电平
	int pinLevel = arduinoPinState;
 
	// ============================================================
	// Roll通道处理（引脚10）
	// ============================================================
	if (currPin == 10 && pinLevel > 0) {
		// 检测到上升沿，记录开始时间
		timer_1 = currTime;
	} else if (currPin == 10 && pinLevel == 0) {
		// 检测到下降沿，计算脉宽
		receiver_input[0] = currTime - timer_1;
	}
 
	// ============================================================
	// Pitch通道处理（引脚11）
	// ============================================================
	if (currPin == 11 && pinLevel > 0) {
		// 检测到上升沿，记录开始时间
		timer_2 = currTime;
	} else if (currPin == 11 && pinLevel == 0) {
		// 检测到下降沿，计算脉宽
		receiver_input[1] = currTime - timer_2;
	}
 
	// ============================================================
	// Throttle通道处理（引脚12）
	// ============================================================
	if (currPin == 12 && pinLevel > 0) {
		// 检测到上升沿，记录开始时间
		timer_3 = currTime;
	} else if (currPin == 12 && pinLevel == 0) {
		// 检测到下降沿，计算脉宽
		receiver_input[2] = currTime - timer_3;
	}
 
	// ============================================================
	// Yaw通道处理（引脚13）
	// ============================================================
	if (currPin == 13 && pinLevel > 0) {
		// 检测到上升沿，记录开始时间
		timer_4 = currTime;
	} else if (currPin == 13 && pinLevel == 0) {
		// 检测到下降沿，计算脉宽
		receiver_input[3] = currTime - timer_4;
	}
}
// receiveEvent函数（已注释，预留I2C通信接口）
// 功能：接收来自其他设备（如树莓派）的控制指令
// void receiveEvent(int howMany) {
//   target[0] = Serial.read(); // 接收字符
//   target[1] = Serial.read();
//   Serial.print("接受到树莓派信号:");Serial.print(target[0]);Serial.println(target[1]);
// }

// ============================================================
// 调试和工具函数（已注释）
// ============================================================

// 备用电调初始化方案（使用Servo库）
// void init_ESC(){
// 	Serial.println("电机开始校准");
// 	for(int i=2; i <= 9; i++){
// 		edf.attach(i);
//   	edf.writeMicroseconds(1000);
// 		delay(1000);
// 	}
// 	Serial.println("电机校准完成");
// }

// 低通滤波器函数（已注释）
// void low_permit_fliter(float* list){
// 	float alpha = 0.1;
// 	list[0] = alpha * list[0] + (1 - alpha) * list[1];
// 	list[1] = list[0];
// }

// 限幅滤波器函数（已注释）
// void limit_fliter(float* list){
// 	float limit = 40.0;
// 	if (abs(list[0]) > limit) list[0] = list[1];
// 	else list[1] = list[0];
// }

// 快速平方根倒数函数（已注释，已在AttitudeEstimator中实现）
// float Q_rsqrt(float number)
// {
//     long i;
//     float x2, y;
//     const float threehalfs = 1.5F;
//     x2 = number * 0.5F;
//     y = number;
//     i = *(long *)&y;
//     i = 0x5f3759df - (i >> 1);
//     y = *(float *)&i;
//     y = y * (threehalfs - (x2 * y * y)); // 1st iteration （第一次牛顿迭代）
//     return y;
// }

// ============================================================
// 加速度计校准函数（已注释，功能已集成到AttitudeEstimator中）
// 功能：测量并校正加速度计的初始偏差
// ============================================================

// // 	for (measureCount; measureCount < measureTime; measureCount++)
// // 	{
// // 		mpu.readSensor();
// //     	ax = -mpu.getAccX();
// //     	ay = -mpu.getAccY();
// //     	az = -mpu.getAccZ();

// // 		acc0_x += ax;
// // 		acc0_y += ay;
// // 		acc0_z += az;
// // 		acc0_total += sqrt(ax*ax+ay*ay+az*az);

// // 	}

// // 	acc0_x /= measureTime;
// // 	acc0_y /= measureTime;
// // 	acc0_z /= measureTime;
// // 	acc0_total /= measureTime;

// // 	roll_0 = atan2(acc0_y,acc0_z) * rad2angle;
// // 	float pitch_uncheck = 0.0;

// // 	pitch_uncheck = atan2(acc0_x,sqrt(acc0_z*acc0_z + acc0_y*acc0_y));
// // 		if (acc0_x > 0 && pitch_uncheck > 0){
// // 			pitch_0 = -pitch_uncheck * rad2angle;
// // 		}
// // 		else if (acc0_x < 0 && pitch_uncheck < 0){
// // 			pitch_0 = -pitch_uncheck * rad2angle;
// // 		}
// // 		else{
// // 			pitch_0 = pitch_uncheck * rad2angle;
// // 		}
// // 	pitch_0 *= angle2rad;roll_0 *= angle2rad;

// // 	ax_cali = acc0_x + pitch_0 * roll_0 * acc0_y + pitch_0 * acc0_z;
// // 	ay_cali = acc0_y + roll_0 * acc0_z;
// // 	az_cali = -pitch_0 * acc0_x + roll_0 * acc0_y + acc0_z;
// // 	float scale = 0.00239;
// // 	Serial.print("    ax: "); Serial.println(acc0_x*scale);
// // 	Serial.print("    ay: "); Serial.println(acc0_y*scale);
// // 	Serial.print("    az: "); Serial.println(acc0_z*scale);
// // 	Serial.print("    pitch0(rad): "); Serial.println(pitch_0);
// // 	Serial.print("    roll_0(rad): "); Serial.println(roll_0);
// // 	Serial.println(" 矫正后----------------------: ");
// // 	Serial.print("    ax: "); Serial.println(ax_cali*scale);
// // 	Serial.print("    ay: "); Serial.println(ay_cali*scale);
// // 	Serial.print("    az: "); Serial.println(az_cali*scale);
// // 	Serial.println("\n测量完毕\n===================================\n");
// // 	delay(2000);
// // 	state = STATE_INIT;
// // 	measureCount = 0;

// // }

