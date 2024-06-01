#include <HardwareSerial.h>
#include <math.h>
#include <Arduino.h>
#include "ESP32CAN.h"
#include "CAN_config.h"
#include "SBUS.h"
#include "BluetoothSerial.h"
#include "driver/twai.h"
//测试数据
int       Lx  = 85;  
int       Ly  = 110;
int       d   = 2;
float     u   = 0;//目标车速
int       r   = 2;
float     R   = 200;//目标转弯半径
float  theta  = 0;//目标转向角度
float   R_l   = 0.12;
float output_u= u;//目标车速

#define PI      3.14159265
#define SBUS_RX 14
#define None    25//D1(空）
#define MAX_BUFFER_SIZE 1024
#define byte0(dw_temp)     (*(char*)(&dw_temp))
#define byte1(dw_temp)     (*((char*)(&dw_temp) + 1))
#define byte2(dw_temp)     (*((char*)(&dw_temp) + 2))
#define byte3(dw_temp)     (*((char*)(&dw_temp) + 3))
#define LIMIT_V_MAX     0.8
#define LIMIT_V_MIN     -0.8

#define PI 3.14159265
#define INTEGRAL_MAX 1   // 设定积分项的最大值  
#define INTEGRAL_MIN -1  // 设定积分项的最小值  
#define ERROR_THRESHOLD 0.25  // 设定误差阈值，用于积分分离 
#define LOC_DEAD_ZONE 0.01 /*位置环死区*/ 
// PID结构体  
struct PID_DATA{  
    float setpoint;       // 设定点  
    float kp;             // 比例系数  
    float ki;             // 积分系数  
    float kd;             // 微分系数  
    float last_error;     // 上一次的误差  
    float integral;       // 误差的积分  
};  
struct PID_DATA pid_data    ={0,0,0,0,0,0};
struct PID_DATA pid_data_V1 ={0,0,0,0,0,0};
struct PID_DATA pid_data_V2 ={0,0,0,0,0,0};
struct PID_DATA pid_data_V3 ={0,0,0,0,0,0};
struct PID_DATA pid_data_V4 ={0,0,0,0,0,0};
// 定义一个名为Servo的结构体(保存舵机数据)
struct sServo {  
    int32_t  Actual_angle;
    uint16_t State;
}; 
struct sServo servo_1 = {0,0};
struct sServo servo_2 = {0,0};
struct sServo servo_3 = {0,0};
struct sServo servo_4 = {0,0};

// 定义一个名为motor的结构体(保存轮子数据)
struct sMotor {  
    int32_t  Actual_V;
    uint16_t State;
    uint16_t Fault_codes;
}; 
struct sMotor motor_1 = {0,0,0};
struct sMotor motor_2 = {0,0,0};
struct sMotor motor_3 = {0,0,0};
struct sMotor motor_4 = {0,0,0};

// 定义一个名为top的结构体(上位机(地址0x01)发给该设备的数据)  
struct sTop {
    int16_t   R;
    int16_t   theta;
    int16_t   u;
    int16_t   mode;
};
struct sTop top = {0,0,0,0};

SemaphoreHandle_t xMutexInventory;
SemaphoreHandle_t xSerialMutex;
SemaphoreHandle_t xCanMutex;
//can相关数据定义

CAN_device_t CAN_cfg;               // CAN Config
uint8_t canin[8] = {0};  //上位机CAN输入数据 
CAN_frame_t   tx_frame;
static bool driver_installed = false;

//蓝牙相关数据定义
BluetoothSerial SerialBT;
volatile int BTread = 0;
uint8_t BTdebug[10];

//运动解析后数据
float theta1,theta2,theta3,theta4 = 0;
float V1,V2,V3,V4 = 0;

//SBUS相关数据
SBUS x8r(Serial2);
uint16_t CH[16]; //遥控数据
bool failSafe;
bool lostFrame;
uint8_t is_sbus = 0;
uint8_t is_can  = 0;
uint8_t control_mode = 0;
// 定义一个名为sbus_data的结构体(sbus数据解析)  
struct S_sbus_data {
    int16_t   zuo_up_down;
    int16_t   zuo_zuo_you;
    int16_t   you_up_down;
    int16_t   you_zuo_you;
    int8_t    key_a;
    int8_t    key_b;
    int8_t    key_c;
    int8_t    key_d;
    int16_t   vr_a;
    int16_t   vr_b;
};
struct S_sbus_data sbus_data = {0,0,0,0,0,0,0,0,0,0};
//vofa相关数据
uint16_t cnt = 0;
uint8_t send_buf[MAX_BUFFER_SIZE];

float                 out_v1_i;float                 out_v2_i;
float                 out_v3_i;float                 out_v4_i;

// 定义一个平方函数  
float square(float x) {  
    return x * x;  
}  
// 将弧度转换为角度的函数  
float rad_to_deg(float rad) {  
    return rad * (180.0 / PI);  
}  
int32_t uint8_to_int32(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {  
    // 假设 b3 是最高有效字节，b0 是最低有效字节（大端字节序）  
    return (int32_t)(b3 << 24 | b2 << 16 | b1 << 8 | b0);  
}
int16_t uint8_to_int16(uint8_t b0, uint8_t b1) {  
    // 假设 b1 是最高有效字节，b0 是最低有效字节（大端字节序）  
    return (int16_t)(b1 << 8 | b0);
}
float uint8_to_float(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {  
    // 组合成32位整数，大端字节序  
    uint32_t combined = ((uint32_t)b3 << 24) |  
                        ((uint32_t)b2 << 16) |  
                        ((uint32_t)b1 << 8)  |  
                        (uint32_t)b0;  
  
    // 强制转换为float  
    float result;  
    memcpy(&result, &combined, sizeof(result));  
    return result;  
}

void float_to_uint8(float f, uint8_t *bytes) {  
    union {  
        float f;  
        uint8_t b[sizeof(float)];  
    } converter;  
    converter.f = f;
    // 假设我们是在大端字节序系统上  
    bytes[0] = converter.b[0]; // 最高有效字节  
    bytes[1] = converter.b[1];  
    bytes[2] = converter.b[2];  
    bytes[3] = converter.b[3]; // 最低有效字节  
}

void int32_to_uint8(int32_t i, uint8_t *bytes) {  
    // 假设我们是在大端字节序系统上  
    bytes[3] = (uint8_t)(i >> 24); // 最高有效字节  
    bytes[2] = (uint8_t)(i >> 16);  
    bytes[1] = (uint8_t)(i >> 8);  
    bytes[0] = (uint8_t)i; // 最低有效字节  
}
void int16_to_uint8(int16_t i, uint8_t *bytes) {  
    // 假设我们是在大端字节序系统上  
    bytes[1] = (uint8_t)(i >> 8);  // 最高有效字节  
    bytes[0] = (uint8_t)i;         // 最低有效字节  
}

float normalize_int16(int16_t value, int16_t min_input, int16_t zero_ref, int16_t max_input, float min_output, float max_output) {  
    if (value < min_input || value > max_input) {  
        // 值不在指定输入范围内，可以根据需要返回特定值或进行错误处理  
        return 0.0f; // 表示错误或超出范围的值  
    }  
  
    if (value < zero_ref) {  
        // 映射 min_input 到 zero_ref-1 的值到 min_output 到 0 之间  
        return min_output + (value - min_input) * (0 - min_output) / (float)(zero_ref - min_input);  
    } else if (value > zero_ref) {  
        // 映射 zero_ref+1 到 max_input 的值到 0 到 max_output 之间  
        return (value - zero_ref) * (max_output - 0) / (float)(max_input - zero_ref);  
    } else {  
        // value 等于 zero_ref，直接返回 0  
        return 0.0f;  
    }  
}  

float normalize_int16_reversed(int16_t value, int16_t max_input, int16_t zero_ref, int16_t min_input, float max_output, float min_output) {  
    // 检查输入值是否在有效范围内  
    if (value < min_input || value > max_input) {  
        return 0.0f; // 表示错误或超出范围的值  
    }  
  
    // 处理zero_ref和max_input相等的情况  
    if (zero_ref == max_input) {  
        if (value == zero_ref) {  
            return max_output;  
        }  
        // 由于范围是反向的，并且zero_ref是范围的“最大值”，  
        // 所以当value减小时，输出应该从max_output线性减小到min_output  
        return max_output - (float)(zero_ref - value) * (max_output - min_output) / (float)(zero_ref - min_input);  
    }  
  
    // 如果zero_ref不等于max_input，则按正常反向范围处理  
    if (value > zero_ref) {  
        return max_output - (float)(value - zero_ref) * (max_output - min_output) / (float)(zero_ref - min_input);  
    } else if (value < zero_ref) {  
        return min_output + (float)(zero_ref - value) * (min_output - max_output) / (float)(max_input - zero_ref);  
    } else {  
        // value等于zero_ref的情况  
        return (max_output + min_output) / 2.0f;  
    }  
}

void vofa_send_data(uint8_t num, float data) 
{
	send_buf[cnt++] = byte0(data);
	send_buf[cnt++] = byte1(data);
	send_buf[cnt++] = byte2(data);
	send_buf[cnt++] = byte3(data);
}

void vofa_sendframetail(void) 
{
	send_buf[cnt++] = 0x00;
	send_buf[cnt++] = 0x00;
	send_buf[cnt++] = 0x80;
	send_buf[cnt++] = 0x7f;
	/* 将数据和帧尾打包发送 */
  // 使用Serial.write()来发送数据
  if(xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE)
  {   
    for (uint16_t i = 0; i < cnt; i++) {  
        Serial.write(send_buf[i]);  
    }
    xSemaphoreGive(xSerialMutex);
    cnt = 0;// 每次发送完帧尾都需要清零
  }
}

// PID初始化函数  
void MY_PID_Init(PID_DATA *pid,float setpoint,float kp,float ki,float kd) 
{  
    pid->setpoint = setpoint;  
    pid->kp = kp;  
    pid->ki = ki;  
    pid->kd = kd;  
    pid->last_error = 0.0;  
    pid->integral = 0.0;  
}  
// PID更新函数  
float V4_PID_Update(PID_DATA *pid,float feedback) {
    float error = pid->setpoint - feedback;  
    pid->integral += error;  
//         //积分分离逻辑  
//    if (fabs(error) > ERROR_THRESHOLD) {  
//        pid->integral = 0; // 当误差较大时，重置积分项  
//    } else {  
//        pid->integral += error; // 当误差较小时，正常累加积分项  
//  
//        // 抗积分饱和逻辑  
//        if (pid->integral > INTEGRAL_MAX) {  
//            pid->integral = INTEGRAL_MAX;  
//        } else if (pid->integral < INTEGRAL_MIN) {  
//            pid->integral = INTEGRAL_MIN;  
//        }  
//    } 
//    if((error >= -LOC_DEAD_ZONE) && (error <= LOC_DEAD_ZONE))
//    {
//        error = 0;
//        pid->last_error = 0;
//        pid->integral = 0;
//    }
    float p = pid->kp * error;  
    float i = pid->ki * pid->integral;  
    float derivative = error - pid->last_error;  
    float d = pid->kd * derivative;  
    pid->last_error = error;  
  
    // 计算PID输出  
    float output = p + i + d + 10 * error;  
  
    // 这里可以添加一些逻辑来限制输出，比如积分饱和等  
    if(output > 30000)
    {
      output = 30000;
    }
    if(output < -30000)
    {
      output = -30000;
    }
    return output;  
}  

// PID更新函数  
float MY_PID_Update(PID_DATA *pid,float feedback) {
    float error = pid->setpoint - feedback;  
    pid->integral += error;  
//         //积分分离逻辑  
//    if (fabs(error) > ERROR_THRESHOLD) {  
//        pid->integral = 0; // 当误差较大时，重置积分项  
//    } else {  
//        pid->integral += error; // 当误差较小时，正常累加积分项  
//  
//        // 抗积分饱和逻辑  
//        if (pid->integral > INTEGRAL_MAX) {  
//            pid->integral = INTEGRAL_MAX;  
//        } else if (pid->integral < INTEGRAL_MIN) {  
//            pid->integral = INTEGRAL_MIN;  
//        }  
//    } 
//    if((error >= -LOC_DEAD_ZONE) && (error <= LOC_DEAD_ZONE))
//    {
//        error = 0;
//        pid->last_error = 0;
//        pid->integral = 0;
//    }
    float p = pid->kp * error;  
    float i = pid->ki * pid->integral;  
    float derivative = error - pid->last_error;  
    float d = pid->kd * derivative;  
    pid->last_error = error;  
  
    // 计算PID输出  
    float output = p + i + d + 0.05*error;  
  
    // 这里可以添加一些逻辑来限制输出，比如积分饱和等  
  
    return output;  
}  

void Motion_solving_Fun(float *theta1,float *theta2,float *theta3,float *theta4,float *V1,float *V2,float *V3,float *V4)
{
  float R1,R2,R3,R4 = 0;

	if((theta>=0&&theta<=90) || (theta>=270&&theta<=360))
	{
    R1 = sqrt(square(R*sin(theta*PI/180)-Ly/2-d)+square(R*cos(theta*PI/180)+Lx/2))+r;
    R2 = sqrt(square(R*sin(theta*PI/180)-Ly/2)  +square(R*cos(theta*PI/180)-Lx/2))-r;
    R3 = sqrt(square(R*sin(theta*PI/180)+Ly/2-d)+square(R*cos(theta*PI/180)+Lx/2))+r;	
    R4 = sqrt(square(R*sin(theta*PI/180)+Ly/2)  +square(R*cos(theta*PI/180)-Lx/2))-r;
	}
	else
	{
    R1 = sqrt(square(R*sin(theta*PI/180)-Ly/2-d)+square(R*cos(theta*PI/180)+Lx/2))+r;
    R2 = sqrt(square(R*sin(theta*PI/180)-Ly/2)  +square(R*cos(theta*PI/180)-Lx/2))-r;
    R3 = sqrt(square(R*sin(theta*PI/180)+Ly/2-d)+square(R*cos(theta*PI/180)+Lx/2))+r;	
    R4 = sqrt(square(R*sin(theta*PI/180)+Ly/2)  +square(R*cos(theta*PI/180)-Lx/2))-r;
	}
	if(R != 0)
	{
		*V1 = output_u*R1/R;
		*V2 = output_u*R2/R;
		*V3 = output_u*R3/R;
		*V4 = output_u*R4/R;
	}
	else 
	{
		*V1 = u;
		*V2 = u;
		*V3 = u;
		*V4 = u;
	}
	// *theta1 = atan(((R*cos(theta*PI/180))+Lx/2) / (-R*sin(theta*PI/180)+Ly/2+d));
	// *theta2 = atan(((R*cos(theta*PI/180))-Lx/2) / (-R*sin(theta*PI/180)+Ly/2));
	// *theta3 = atan(((R*cos(theta*PI/180))+Lx/2) / (-R*sin(theta*PI/180)-Ly/2+d));	
	// *theta4 = atan(((R*cos(theta*PI/180))-Lx/2) / (-R*sin(theta*PI/180)-Ly/2));	
	*theta1 = atan(((R*sin(theta*PI/180)-Ly/2-d)  / ((R*cos(theta*PI/180))+Lx/2)));	
	*theta2 = atan(((R*sin(theta*PI/180)-Ly/2)    / ((R*cos(theta*PI/180))-Lx/2)));	
	*theta3 = atan(((R*sin(theta*PI/180)+Ly/2-d)  / ((R*cos(theta*PI/180))+Lx/2)));	
	*theta4 = atan(((R*sin(theta*PI/180)+Ly/2)    / ((R*cos(theta*PI/180))-Lx/2)));	

	*theta1 = rad_to_deg(*theta1);
	*theta2 = rad_to_deg(*theta2);
	*theta3 = rad_to_deg(*theta3);
	*theta4 = rad_to_deg(*theta4);

}
void can_init(void)
{
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)27, (gpio_num_t)26, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();  //Look in the api-reference for other speed sets.
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  g_config.rx_queue_len = 5120;
  g_config.tx_queue_len = 5120;
  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    //Serial.println("Driver installed");
  } else {
    //Serial.println("Failed to install driver");
    return;
  }
  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    //Serial.println("Driver started");
  } else {
    //Serial.println("Failed to start driver");
    return;
  }
  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    //Serial.println("CAN Alerts reconfigured");
  } else {
    //Serial.println("Failed to reconfigure alerts");
    return;
  }
  // TWAI driver is now successfully installed and started
  driver_installed = true;
}

void my_can_tx_fun(uint32_t ID, uint8_t data_0, uint8_t data_1, uint8_t data_2, uint8_t data_3, 
                                uint8_t data_4, uint8_t data_5, uint8_t data_6, uint8_t data_7)
{
  twai_message_t tx_message;
  twai_status_info_t twaistatus;
  tx_message.identifier         = ID; 
  tx_message.extd               = 0;
  tx_message.data_length_code   = 8;
  tx_message.rtr                = 0;
  tx_message.self               = 0;
  tx_message.ss                 = 0;
  tx_message.data[0]            = data_0;
  tx_message.data[1]            = data_1;
  tx_message.data[2]            = data_2;
  tx_message.data[3]            = data_3;
  tx_message.data[4]            = data_4;
  tx_message.data[5]            = data_5;
  tx_message.data[6]            = data_6;
  tx_message.data[7]            = data_7;

  twai_get_status_info(&twaistatus);
  // 总线关闭
  if(twaistatus.state == TWAI_STATE_BUS_OFF)
  {
    twai_initiate_recovery();
  }
  // 总线停止
  else if(twaistatus.state == TWAI_STATE_STOPPED)
  {
    //twai_driver_install()
    //can_init();
    twai_start();
  }
  // if(xSemaphoreTake(xCanMutex, portMAX_DELAY) == pdTRUE)
  // {
  //   Serial.println("car_state_safe_app");
    twai_transmit(&tx_message,pdMS_TO_TICKS(0));
  //   xSemaphoreGive(xCanMutex);
  // }
  //twai_transmit(&tx_message,portMAX_DELAY);

  // tx_frame.MsgID      = ID;
  // tx_frame.data.u8[0] = data_0;tx_frame.data.u8[1] = data_1;
  // tx_frame.data.u8[2] = data_2;tx_frame.data.u8[3] = data_3;
  // tx_frame.data.u8[4] = data_4;tx_frame.data.u8[5] = data_5;
  // tx_frame.data.u8[6] = data_6;tx_frame.data.u8[7] = data_7;
  // ESP32Can.CANWriteFrame(&tx_frame);
  //判断CAN是否发送成功
  // if(ESP32Can.CANWriteFrame(&tx_frame) == 0)
  // {
  //   if(xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE)
  //   {
  //     Serial.println("send success!");
  //     xSemaphoreGive(xSerialMutex);
  //   }
  // }
  // else
  // {
  //   if(xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE)
  //   {
  //     Serial.println("send error!");
  //     xSemaphoreGive(xSerialMutex);
  //   }
  // }
}

uint8_t remoter_2stage_switch_parse(uint16_t ch, int16_t mid_val)
{
    if (ch < mid_val)
        ch = 0;
    else
        ch = 1;

    return ch;
}

uint8_t remoter_3stage_switch_parse(uint16_t ch, int16_t mid_val0, int16_t mid_val1)
{
    if (ch >= 0 && ch < mid_val0)
        ch = 0;
    else if (ch >= mid_val0 && ch < mid_val1)
        ch = 1;
    else
        ch = 2;
    return ch;
}
void ch_to_data(uint16_t *ch)
{
  sbus_data.zuo_up_down   = ch[2];
  sbus_data.zuo_zuo_you   = ch[3];
  sbus_data.you_up_down   = ch[1];
  sbus_data.you_zuo_you   = ch[0];
  sbus_data.key_a         = remoter_2stage_switch_parse(ch[4],1000);
  sbus_data.key_b         = remoter_2stage_switch_parse(ch[5],1350);
  sbus_data.key_c         = remoter_3stage_switch_parse(ch[8],500,1500);
  sbus_data.key_d         = remoter_2stage_switch_parse(ch[9],1000);
  sbus_data.vr_a          = ch[6];
  sbus_data.vr_b          = ch[7];
}

void rx_data_app(void *pt)
{
  CAN_frame_t      rx_frame;
  uint8_t       v1_data_float[4];uint8_t       v2_data_float[4];
  uint8_t       v3_data_float[4];uint8_t       v4_data_float[4];
  uint8_t       t1_data_float[4];uint8_t       t2_data_float[4];
  uint8_t       t3_data_float[4];uint8_t       t4_data_float[4];
  uint8_t       i1_data_float[2];uint8_t       i2_data_float[2];
  uint8_t       i3_data_float[2];uint8_t       i4_data_float[2];

  float         rounded_f     = 0;
  float         theta_temp    = 0;
  int32_t       i             = 0; 
  TickType_t       xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency    = 20; // 间隔 5 ticks
  twai_message_t   rx_msg;
  twai_status_info_t twaistatus;
  for(;;)
  {
    twai_get_status_info(&twaistatus);
    // 总线关闭
    if(twaistatus.state == TWAI_STATE_BUS_OFF)
    {
      twai_initiate_recovery();
    }
    // 总线停止
    else if(twaistatus.state == TWAI_STATE_STOPPED)
    {
      //twai_driver_install()
      //can_init();
      twai_start();
    }
    while(twaistatus.msgs_to_rx>0)
    {
      //if(canin[8]<200) canin[8]++;
      if (twai_receive(&rx_msg, pdMS_TO_TICKS(0))==ESP_OK){
      switch(rx_msg.identifier)
      {
        //轮子数据接收
        case 0x184:
          motor_1.Actual_V      = uint8_to_int32(rx_msg.data[0],rx_msg.data[1],rx_msg.data[2],rx_msg.data[3]);
          motor_1.State         = uint8_to_int16(rx_msg.data[4],rx_msg.data[5]);
          motor_1.Fault_codes   = uint8_to_int16(rx_msg.data[6],rx_msg.data[7]);
          break;
        case 0x185:
          motor_2.Actual_V      = uint8_to_int32(rx_msg.data[0],rx_msg.data[1],rx_msg.data[2],rx_msg.data[3]);
          motor_2.State         = uint8_to_int16(rx_msg.data[4],rx_msg.data[5]);
          motor_2.Fault_codes   = uint8_to_int16(rx_msg.data[6],rx_msg.data[7]);
          break;
        case 0x186:
          motor_3.Actual_V      = uint8_to_int32(rx_msg.data[0],rx_msg.data[1],rx_msg.data[2],rx_msg.data[3]);
          motor_3.State         = uint8_to_int16(rx_msg.data[4],rx_msg.data[5]);
          motor_3.Fault_codes   = uint8_to_int16(rx_msg.data[6],rx_msg.data[7]);
          break;      
        case 0x187:
          motor_4.Actual_V      = uint8_to_int32(rx_msg.data[0],rx_msg.data[1],rx_msg.data[2],rx_msg.data[3]);
          motor_4.State         = uint8_to_int16(rx_msg.data[4],rx_msg.data[5]);
          motor_4.Fault_codes   = uint8_to_int16(rx_msg.data[6],rx_msg.data[7]);
          break;

        //舵机数据接收
        case 0x488:
          servo_1.Actual_angle  = uint8_to_int32(rx_msg.data[0],rx_msg.data[1],rx_msg.data[2],rx_msg.data[3]);
          servo_1.State         = uint8_to_int16(rx_msg.data[4],rx_msg.data[5]);
          //Serial.println("0x488 OK");
          break;
        case 0x489:
          servo_2.Actual_angle  = uint8_to_int32(rx_msg.data[0],rx_msg.data[1],rx_msg.data[2],rx_msg.data[3]);
          servo_2.State         = uint8_to_int16(rx_msg.data[4],rx_msg.data[5]);   
          //Serial.println("0x489 OK");     
          break;
        case 0x490:
          servo_3.Actual_angle  = uint8_to_int32(rx_msg.data[0],rx_msg.data[1],rx_msg.data[2],rx_msg.data[3]);
          servo_3.State         = uint8_to_int16(rx_msg.data[4],rx_msg.data[5]);        
          break;      
        case 0x491:
          servo_4.Actual_angle  = uint8_to_int32(rx_msg.data[0],rx_msg.data[1],rx_msg.data[2],rx_msg.data[3]);
          servo_4.State         = uint8_to_int16(rx_msg.data[4],rx_msg.data[5]);        
          break;
        // 解析上位机发来的数据
        case 0x01:
          top.R                 = uint8_to_int16(rx_msg.data[0],rx_msg.data[1]);
          top.theta             = uint8_to_int16(rx_msg.data[2],rx_msg.data[3]);
          top.u                 = uint8_to_int16(rx_msg.data[4],rx_msg.data[5]);
          top.mode              = uint8_to_int16(rx_msg.data[6],rx_msg.data[7]);
          break;
        default:
          //printf("无效的输入。\n");  
          break;  
      }
      }
      twai_get_status_info(&twaistatus);
    }
    //解析数据
    // if(xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE)
    // {
    //   Serial.print("servo_1.Actual_angle=");Serial.print(servo_1.Actual_angle);
    //   Serial.print(" servo_1.State=");      Serial.println(servo_1.State);

    //   xSemaphoreGive(xSerialMutex);  
    // }
    // tartTime = micros();
    if(is_can == 1 && is_sbus == 0)
    {
      // top.R      cm
      // top.theta  °
      // top.u      cm/s
      R             = (float)(top.R/100.0);//cm->m
      theta         = (float)(top.theta);
      u             = (float)(top.u/100.0);
      control_mode  = top.mode;

      MY_PID_Init(&pid_data,u,0.1,0.1,0.01); // 设定点100，比例系数1，积分系数0.1，微分系数0.01
    }
    else if(is_can == 0 && is_sbus == 1)
    {
      //切换模式后车速为0，模式为斜行模式
      top.u = 0;
      top.mode = 0;
      if(sbus_data.key_d == 1)
      {
        control_mode = 4;
      }
      else if(sbus_data.key_a == 1 && sbus_data.key_b == 0 && sbus_data.key_d == 0)
      {
        control_mode = 2;
      }
      else if(sbus_data.key_a == 0 && sbus_data.key_b == 1 && sbus_data.key_d == 0)
      {
        control_mode = 1;
      }
      else if(sbus_data.key_a == 1 && sbus_data.key_b == 1 && sbus_data.key_d == 0)
      {
        control_mode = 0;
      }
      else if(sbus_data.key_a == 0 && sbus_data.key_b == 0 && sbus_data.key_c == 0 && sbus_data.key_d == 0)
      {
        control_mode = 0;
      }
      else if(sbus_data.key_a == 1 && sbus_data.key_b == 1 && sbus_data.key_c == 1 && sbus_data.key_d == 0)
      {
        control_mode = 3;
      }
      R              = (float)(normalize_int16(sbus_data.you_zuo_you,240,1024,1807, 20.0f,200.0f));
      theta          = (float)(normalize_int16(sbus_data.you_up_down,240,1024,1807, -180.0f,180.0f));
      u              = (float)(normalize_int16(sbus_data.zuo_up_down,240,1024,1807,-1.0f,1.0f));
      MY_PID_Init(&pid_data,u,0.1,0.1,0.01); // 设定点100，比例系数1，积分系数0.1，微分系数0.01
    }
    output_u = u;
    //else
    //得到每个轮子的目标速度和目标角度
    //Motion_solving_Fun(&theta1,&theta2,&theta3,&theta4,&V1,&V2,&V3,&V4);
    //斜行模式
    if(control_mode == 0)
    {
      if(theta >= 90)
      {
        theta = 90;
      }
      if(theta <= -90)
      {
        theta = -90;
      }
      theta1 = (float)theta;theta2 = (float)theta;
      theta3 = (float)theta;theta4 = (float)theta;
      V1     = u;           V2     = u;
      V3     = u;           V4     = u;
    }
    //旋转模式
    else if(control_mode == 3)
    {
      R = 0;
      Motion_solving_Fun(&theta1,&theta2,&theta3,&theta4,&V1,&V2,&V3,&V4);
    }
    //急停模式
    else if(control_mode == 2)
    {
      //解除急停指令 后需进入某种模式
      my_can_tx_fun(0x604,0x2B,0x71,0x60,0x00,0x00,0x00,0x00,0x00);//设置目标转矩 0
      my_can_tx_fun(0x604,0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00);//初始化驱动器状态机

      my_can_tx_fun(0x605,0x2B,0x71,0x60,0x00,0x00,0x00,0x00,0x00);//设置目标转矩 0
      my_can_tx_fun(0x605,0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00);//初始化驱动器状态机

      my_can_tx_fun(0x606,0x2B,0x71,0x60,0x00,0x00,0x00,0x00,0x00);//设置目标转矩 0
      my_can_tx_fun(0x606,0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00);//初始化驱动器状态机

      my_can_tx_fun(0x607,0x2B,0x71,0x60,0x00,0x00,0x00,0x00,0x00);//设置目标转矩 0
      my_can_tx_fun(0x607,0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00);//初始化驱动器状态机

      // my_can_tx_fun(0x607,0x2B,0x40,0x60,0x00,0x00,0x00,0x00,0x00);//初始化驱动器状态机
      // my_can_tx_fun(0x607,0x23,0x83,0x60,0x00,0xe3,0x03,0x00,0x00);//设置加速度时间为1000ms
      // my_can_tx_fun(0x607,0x23,0x84,0x60,0x00,0xe3,0x03,0x00,0x00);//设置减速度时间为1000ms
      // my_can_tx_fun(0x607,0x23,0xff,0x60,0x00,0x3c,0x00,0x00,0x00);//设置目标速度为60rpm
      // my_can_tx_fun(0x607,0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00);//设置为速度模式
      // my_can_tx_fun(0x607,0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00);//切换驱动器状态机为shutdown
      // my_can_tx_fun(0x607,0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00);//切换状态机为switch on
      // my_can_tx_fun(0x607,0x2B,0x40,0x60,0x00,0x0f,0x00,0x00,0x00);//切换状态机为OPERATION ENABLE+SWITCH ON，没有检测到故障，驱动功能启用，并对电机上电

      // my_can_tx_fun(0x608,0x23,0x81,0x60,0x00,0xe8,0x03,0x00,0x00);//梯形速度写入1000RPM
      // my_can_tx_fun(0x608,0x23,0x83,0x60,0x00,0x20,0x4e,0x00,0x00);//梯形加减速写入20000RPM/S
      // my_can_tx_fun(0x608,0x2b,0x40,0x60,0x00,0x2f,0x00,0x00,0x00);//绝对位置控制模式+新位置立即执行

      // my_can_tx_fun(0x609,0x23,0x81,0x60,0x00,0xe8,0x03,0x00,0x00);//梯形速度写入1000RPM
      // my_can_tx_fun(0x609,0x23,0x83,0x60,0x00,0x20,0x4e,0x00,0x00);//梯形加减速写入20000RPM/S
      // my_can_tx_fun(0x609,0x2b,0x40,0x60,0x00,0x2f,0x00,0x00,0x00);//绝对位置控制模式+新位置立即执行

      // my_can_tx_fun(0x610,0x23,0x81,0x60,0x00,0xe8,0x03,0x00,0x00);//梯形速度写入1000RPM
      // my_can_tx_fun(0x610,0x23,0x83,0x60,0x00,0x20,0x4e,0x00,0x00);//梯形加减速写入20000RPM/S
      // my_can_tx_fun(0x610,0x2b,0x40,0x60,0x00,0x2f,0x00,0x00,0x00);//绝对位置控制模式+新位置立即执行

      // my_can_tx_fun(0x611,0x23,0x81,0x60,0x00,0xe8,0x03,0x00,0x00);//梯形速度写入1000RPM
      // my_can_tx_fun(0x611,0x23,0x83,0x60,0x00,0x20,0x4e,0x00,0x00);//梯形加减速写入20000RPM/S
      // my_can_tx_fun(0x611,0x2b,0x40,0x60,0x00,0x2f,0x00,0x00,0x00);//绝对位置控制模式+新位置立即执行
      u      = 0;
    }
    else if(control_mode == 1)
    {
      if(theta >= 0)
      {
        theta = 180;
      }
      else
      {
        theta = 0;
      }
      if(fabs(u - ((0.1*motor_1.Actual_V*R_l/60)+(0.1*motor_2.Actual_V*R_l/60)+
                   (0.1*motor_3.Actual_V*R_l/60)+(0.1*motor_4.Actual_V*R_l/60))/4) > 0.02)
	    {
        output_u = MY_PID_Update(&pid_data, ((0.1*motor_1.Actual_V*R_l/60)+(0.1*motor_2.Actual_V*R_l/60)+
                                        (0.1*motor_3.Actual_V*R_l/60)+(0.1*motor_4.Actual_V*R_l/60))/4);
      }
      Motion_solving_Fun(&theta1,&theta2,&theta3,&theta4,&V1,&V2,&V3,&V4);
    }
    else
    {
      //急停指令
      my_can_tx_fun(0x604,0x2B,0x40,0x60,0x00,0x02,0x00,0x00,0x00);//初始化驱动器状态机
      my_can_tx_fun(0x605,0x2B,0x40,0x60,0x00,0x02,0x00,0x00,0x00);//初始化驱动器状态机
      my_can_tx_fun(0x606,0x2B,0x40,0x60,0x00,0x02,0x00,0x00,0x00);//初始化驱动器状态机
      my_can_tx_fun(0x607,0x2B,0x40,0x60,0x00,0x02,0x00,0x00,0x00);//初始化驱动器状态机
      //舵轮急停
      // my_can_tx_fun(0x608,0x23,0x01,0x26,0x00,0x00,0x00,0x00,0x00);//初始化驱动器状态机
      // my_can_tx_fun(0x609,0x23,0x01,0x26,0x00,0x00,0x00,0x00,0x00);//初始化驱动器状态机
      // my_can_tx_fun(0x610,0x23,0x01,0x26,0x00,0x00,0x00,0x00,0x00);//初始化驱动器状态机
      // my_can_tx_fun(0x611,0x23,0x01,0x26,0x00,0x00,0x00,0x00,0x00);//初始化驱动器状态机
    }

    // 限制每个轮子速度
    if(V1 >= LIMIT_V_MAX)
      V1 = LIMIT_V_MAX;
    else if(V1 <= LIMIT_V_MIN)
      V1 = LIMIT_V_MIN;
    if(V2 >= LIMIT_V_MAX)
      V2 = LIMIT_V_MAX;
    else if(V2 <= LIMIT_V_MIN)
      V2 = LIMIT_V_MIN;
    if(V3 >= LIMIT_V_MAX)
      V3 = LIMIT_V_MAX;
    else if(V3 <= LIMIT_V_MIN)
      V3 = LIMIT_V_MIN;
    if(V4 >= LIMIT_V_MAX)
      V4 = LIMIT_V_MAX;
    else if(V4 <= LIMIT_V_MIN)
      V4 = LIMIT_V_MIN;

    // endTime = micros();
    // elapsedTime = endTime - startTime;
    // Serial.print("Function execution took ");  
    // Serial.print(elapsedTime);
    // Serial.println(" microseconds.");

    // Serial.print("theta1 = "); Serial.print(theta1);
    // Serial.print(" theta2 = ");Serial.print(theta2);
    // Serial.print(" theta3 = ");Serial.print(theta3);    
    // Serial.print(" theta4 = ");Serial.println(theta4);    
    // Serial.print("V1 = ");     Serial.print(V1);
    // Serial.print(" V2 = ");    Serial.print(V2);
    // Serial.print(" V3 = ");    Serial.print(V3);
    // Serial.print(" V4 = ");    Serial.print(V4);
    // Serial.print(" Vu = ");    Serial.println((V1+V2+V3+V4)/4);

    //目标速度以及目标转弯角度
    // vofa_send_data(0,theta1);                                      vofa_send_data(1,theta2);
    // vofa_send_data(2,theta3);                                      vofa_send_data(3,theta4);
    // vofa_send_data(4,V1);                                          vofa_send_data(5,V2);
    // vofa_send_data(6,V3);                                          vofa_send_data(7,V4);
    // //每个轮子的实际速度以及每个车轮的实际转弯角度
    // vofa_send_data(8, (float)(servo_1.Actual_angle*180/90/32768)); vofa_send_data(9, (float)(servo_2.Actual_angle*180/90/32768));
    // vofa_send_data(10,(float)(servo_3.Actual_angle*180/90/32768)); vofa_send_data(11,(float)(servo_4.Actual_angle*180/90/32768));
    // vofa_send_data(12,(float)(10*motor_1.Actual_V*R_l/60));        vofa_send_data(13,(float)(10*motor_2.Actual_V*R_l/60));    
    // vofa_send_data(14,(float)(10*motor_3.Actual_V*R_l/60));        vofa_send_data(15,(float)(10*motor_4.Actual_V*R_l/60));
    // //目标车速                                                      //理论计算车速
    // vofa_send_data(16,(float)(u));                                 vofa_send_data(17,(float)((V1+V2+V3+V4)/4));
    // //转弯半径                                                      //转弯角度
    // vofa_send_data(18,R);                                          vofa_send_data(19,theta);
    // vofa_send_data(20,control_mode);                               vofa_send_data(21,is_can);
    // vofa_send_data(22,is_sbus);
    // //实际车速
    // vofa_send_data(23,((float)(10*motor_1.Actual_V*R_l/60) + (float)(10*motor_1.Actual_V*R_l/60)+
    //               (float)(10*motor_1.Actual_V*R_l/60) + (float)(10*motor_1.Actual_V*R_l/60))/4); 
    // //发送vofa数据
    // vofa_sendframetail();
    MY_PID_Init(&pid_data_V1,V1,2000,200,0);
    MY_PID_Init(&pid_data_V2,V2,2000,200,0);
    MY_PID_Init(&pid_data_V3,V3,2000,200,0);
    MY_PID_Init(&pid_data_V4,V4,2000,200,0);

    out_v1_i = V4_PID_Update(&pid_data_V1,(0.1*motor_1.Actual_V*R_l/60));
    out_v2_i = V4_PID_Update(&pid_data_V2,(0.1*motor_2.Actual_V*R_l/60));
    out_v3_i = V4_PID_Update(&pid_data_V3,(0.1*motor_3.Actual_V*R_l/60));
    out_v4_i = V4_PID_Update(&pid_data_V4,(0.1*motor_4.Actual_V*R_l/60));

    //轮子目标转矩
    rounded_f   = roundf(out_v1_i);
    i           = (int16_t)rounded_f;
    int16_to_uint8(i,i1_data_float);

    rounded_f   = roundf(out_v2_i);
    i           = (int16_t)rounded_f;
    int16_to_uint8(i,i2_data_float);

    rounded_f   = roundf(out_v3_i);
    i           = (int16_t)rounded_f;
    int16_to_uint8(i,i3_data_float);

    rounded_f   = roundf(out_v4_i);
    i           = (int16_t)rounded_f;
    int16_to_uint8(i,i4_data_float);

    // Serial.println(i4_data_float[0],HEX);
    // Serial.println(i4_data_float[1],HEX);
    //轮子1速度
    rounded_f   = roundf(-60*V1/R_l);  // 四舍五入到最接近的整数
    i           = (int32_t)rounded_f; // 将四舍五入后的float值转换为int32_t
    int32_to_uint8(i,v1_data_float);
    // 轮子2速度
    rounded_f   = roundf(60*V2/R_l);  // 四舍五入到最接近的整数
    i           = (int32_t)rounded_f; // 将四舍五入后的float值转换为int32_t
    int32_to_uint8(i,v2_data_float);
    // 轮子3速度
    rounded_f   = roundf(-60*V3/R_l);  // 四舍五入到最接近的整数
    i           = (int32_t)rounded_f; // 将四舍五入后的float值转换为int32_t
    int32_to_uint8(i,v3_data_float);
    // 轮子4速度
    rounded_f   = roundf(60*V4/R_l);  // 四舍五入到最接近的整数
    i           = (int32_t)rounded_f; // 将四舍五入后的float值转换为int32_t
    int32_to_uint8(i,v4_data_float);  

    // 舵轮1角度
    theta_temp  = roundf(-theta1*90*32768/180);
    i           = (int32_t)theta_temp;
    int32_to_uint8(i,t1_data_float);
    // 舵轮2角度
    theta_temp  = roundf(-theta2*90*32768/180);
    i           = (int32_t)theta_temp;
    int32_to_uint8(i,t2_data_float);    
    // 舵轮3角度
    theta_temp  = roundf(-theta3*90*32768/180);
    i           = (int32_t)theta_temp;
    int32_to_uint8(i,t3_data_float);     
    // 舵轮4角度
    theta_temp  = roundf(-theta4*90*32768/180);
    i           = (int32_t)theta_temp;
    int32_to_uint8(i,t4_data_float); 

    // my_can_tx_fun(0x604,0x23,0xff,0x60,0x00,v1_data_float[0],v1_data_float[1],v1_data_float[2],v1_data_float[3]);
    // my_can_tx_fun(0x605,0x23,0xff,0x60,0x00,v2_data_float[0],v2_data_float[1],v2_data_float[2],v2_data_float[3]);
    // my_can_tx_fun(0x606,0x23,0xff,0x60,0x00,v3_data_float[0],v3_data_float[1],v3_data_float[2],v3_data_float[3]);
    //my_can_tx_fun(0x607,0x23,0xff,0x60,0x00,v4_data_float[0],v4_data_float[1],v4_data_float[2],v4_data_float[3]);
    my_can_tx_fun(0x604,0x2B,0x71,0x60,0x00,i1_data_float[0],i1_data_float[1],0x00,0x00);
    my_can_tx_fun(0x605,0x2B,0x71,0x60,0x00,i2_data_float[0],i2_data_float[1],0x00,0x00);
    my_can_tx_fun(0x606,0x2B,0x71,0x60,0x00,i3_data_float[0],i3_data_float[1],0x00,0x00);
    my_can_tx_fun(0x607,0x2B,0x71,0x60,0x00,i4_data_float[0],i4_data_float[1],0x00,0x00);
    //delayMicroseconds(500);
    my_can_tx_fun(0x508,t1_data_float[0],t1_data_float[1],t1_data_float[2],t1_data_float[3],0x00,0x00,0x00,0x00);
    my_can_tx_fun(0x509,t2_data_float[0],t2_data_float[1],t2_data_float[2],t2_data_float[3],0x00,0x00,0x00,0x00);
    my_can_tx_fun(0x510,t3_data_float[0],t3_data_float[1],t3_data_float[2],t3_data_float[3],0x00,0x00,0x00,0x00);
    my_can_tx_fun(0x511,t4_data_float[0],t4_data_float[1],t4_data_float[2],t4_data_float[3],0x00,0x00,0x00,0x00);
    // 舵轮同步信号
    my_can_tx_fun(0x80, 0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// void motor_control_app(void *pt)
// {
//   TickType_t    xLastWakeTime = xTaskGetTickCount();
//   const TickType_t xFrequency = 20; // 控制频率为50Hz
//   twai_status_info_t twaistatus;
//   for(;;)
//   {

//     vTaskDelayUntil(&xLastWakeTime, xFrequency);
//   }
// }

void car_state_safe_app(void *pt)
{
  TickType_t xLastWakeTime    = xTaskGetTickCount();
  const TickType_t xFrequency = 20; // 间隔 20 ticks
  uint16_t        err_code[4];
  for(;;)
  {
    // 轮子错误码
    // 0000h：无错误
    // 0001h：过压
    // 0002h：欠压
    // 0004h：过流
    // 0008h：过载
    // 0010h：电流超差
    // 0020h：编码器超差
    // 0040h：速度超差
    // 0080h：参考电压出错
    // 0100h：EEPROM 读写错误
    // 0200h：霍尔出错
    // 解析车身状态以及保护措施
    err_code[0] = motor_1.Fault_codes;
    //my_can_tx_fun(0x10,);
    // 发送急停指令
    if(motor_1.Fault_codes > 0 || motor_2.Fault_codes > 0 || motor_3.Fault_codes > 0 || motor_4.Fault_codes > 0)
    {
      //急停指令
      my_can_tx_fun(0x604,0x2B,0x40,0x60,0x00,0x02,0x00,0x00,0x00);//初始化驱动器状态机
      my_can_tx_fun(0x605,0x2B,0x40,0x60,0x00,0x02,0x00,0x00,0x00);//初始化驱动器状态机
      my_can_tx_fun(0x606,0x2B,0x40,0x60,0x00,0x02,0x00,0x00,0x00);//初始化驱动器状态机
      my_can_tx_fun(0x607,0x2B,0x40,0x60,0x00,0x02,0x00,0x00,0x00);//初始化驱动器状态机

      // //舵轮急停
      // my_can_tx_fun(0x608,0x23,0x01,0x26,0x00,0x00,0x00,0x00,0x00);//初始化驱动器状态机
      // my_can_tx_fun(0x609,0x23,0x01,0x26,0x00,0x00,0x00,0x00,0x00);//初始化驱动器状态机
      // my_can_tx_fun(0x610,0x23,0x01,0x26,0x00,0x00,0x00,0x00,0x00);//初始化驱动器状态机
      // my_can_tx_fun(0x611,0x23,0x01,0x26,0x00,0x00,0x00,0x00,0x00);//初始化驱动器状态机
    }
    // 发送车身状态

    // 调试信息打印
    // if(xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE)
    // {
    //   Serial.println("car_state_safe_app");
    //   xSemaphoreGive(xSerialMutex);
    // }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// void BT_debug_APP(void *pt) {
//   TickType_t xLastWakeTime = xTaskGetTickCount();
//   const TickType_t xFrequency = 100; // 间隔 20 ticks
//   String device_name = "esp32-bt-test";
//   SerialBT.begin(device_name);
//   // if(xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE)
//   // {
//   //   Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
//   //   xSemaphoreGive(xSerialMutex);
//   // }
//   for(;;){
//     vTaskDelay(10);
//     //vTaskDelayUntil(&xLastWakeTime, xFrequency);
//   }
// }

void sbus_rx_app(void *pt)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = 14; // 间隔 20 ticks
  for(;;)
  { 
    if (!x8r.read(&CH[0], &failSafe, &lostFrame))
    {
      //   xSemaphoreGive(xSerialMutex);
      // }
    }
    ch_to_data(&CH[0]);
    // if(1)
    // {
    //   ch_to_data(&CH[0]);
    //   Serial.print(sbus_data.zuo_up_down);
    //   Serial.print(" ");
    //   Serial.print(sbus_data.zuo_zuo_you);
    //   Serial.print(" ");
    //   Serial.print(sbus_data.you_up_down);
    //   Serial.print(" ");
    //   Serial.print(sbus_data.you_zuo_you);
    //   Serial.print(" ");
    //   Serial.print(sbus_data.key_a);
    //   Serial.print(" ");
    //   Serial.print(sbus_data.key_b);
    //   Serial.print(" ");
    //   Serial.print(sbus_data.key_c);
    //   Serial.print(" ");
    //   Serial.print(sbus_data.key_d);
    //   Serial.print(" ");
    //   Serial.print(sbus_data.vr_a);
    //   Serial.print(" ");
    //   Serial.print(sbus_data.vr_b);
    //   Serial.print(" ");

    //   Serial.print(failSafe);
    //   Serial.print(" ");
    //   Serial.print(lostFrame);
    //   Serial.print(" ");
    //   Serial.print(is_sbus);
    //   Serial.print(" ");        
    //   Serial.print(is_can);
    //   Serial.print(" ");  
    //   Serial.print(R);
    //   Serial.print(" ");  
    //   Serial.print(theta);
    //   Serial.print(" ");  
    //   Serial.print(u);
    //   Serial.print(" ");  
    //   Serial.println();
    // }
    if(lostFrame == 0)
    {
      is_sbus = 1;
      is_can  = 0;
    }
    else
    {
      is_sbus = 0;
      is_can  = 1;
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void bt_rx_app(void *pt)
{
  uint8_t           bt_rx_data[20];
  int               RX_DATA_LEN     = 0;
  int               flag            = 0;
  TickType_t        xLastWakeTime   = xTaskGetTickCount();
  const TickType_t  xFrequency      = 20; // 间隔 20 ticks
  for(;;)
  { 
    if (SerialBT.available()) {
        bt_rx_data[RX_DATA_LEN ++]  = SerialBT.read();  
        flag = 1;
    }

    vofa_send_data(0,theta1);                                      vofa_send_data(1,theta2);
    vofa_send_data(2,theta3);                                      vofa_send_data(3,theta4);
    vofa_send_data(4,V1);                                          vofa_send_data(5,V2);
    vofa_send_data(6,V3);                                          vofa_send_data(7,V4);
    //每个轮子的实际速度以及每个车轮的实际转弯角度
    vofa_send_data(8, (float)(servo_1.Actual_angle*180/90/32768)); vofa_send_data(9, (float)(servo_2.Actual_angle*180/90/32768));
    vofa_send_data(10,(float)(servo_3.Actual_angle*180/90/32768)); vofa_send_data(11,(float)(servo_4.Actual_angle*180/90/32768));
    vofa_send_data(12,(float)(0.1*motor_1.Actual_V*R_l/60));        vofa_send_data(13,(float)(0.1*motor_2.Actual_V*R_l/60));    
    vofa_send_data(14,(float)(0.1*motor_3.Actual_V*R_l/60));        vofa_send_data(15,(float)(0.1*motor_4.Actual_V*R_l/60));
    //目标车速                                                      //理论计算车速
    vofa_send_data(16,(float)(u));                                 vofa_send_data(17,(float)((V1+V2+V3+V4)/4));
    //转弯半径                                                      //转弯角度
    vofa_send_data(18,R);                                          vofa_send_data(19,theta);
    vofa_send_data(20,control_mode);                               vofa_send_data(21,is_can);
    vofa_send_data(22,is_sbus);
    //实际车速
    vofa_send_data(23,((float)(0.1*motor_1.Actual_V*R_l/60) + (float)(0.1*motor_1.Actual_V*R_l/60)+
                  (float)(0.1*motor_1.Actual_V*R_l/60) + (float)(0.1*motor_1.Actual_V*R_l/60))/4); 
    vofa_send_data(24,float(out_v4_i));
    //发送vofa数据
    vofa_sendframetail();

    // if(flag == 1)
    // {
    //   // 急停指令
    //   my_can_tx_fun(0x604,0x2B,0x40,0x60,0x00,0x02,0x00,0x00,0x00);//初始化驱动器状态机
    //   my_can_tx_fun(0x605,0x2B,0x40,0x60,0x00,0x02,0x00,0x00,0x00);//初始化驱动器状态机
    //   my_can_tx_fun(0x606,0x2B,0x40,0x60,0x00,0x02,0x00,0x00,0x00);//初始化驱动器状态机
    //   my_can_tx_fun(0x607,0x2B,0x40,0x60,0x00,0x02,0x00,0x00,0x00);//初始化驱动器状态机
    //   //舵轮急停
    //   my_can_tx_fun(0x608,0x23,0x01,0x26,0x00,0x00,0x00,0x00,0x00);//初始化驱动器状态机
    //   my_can_tx_fun(0x609,0x23,0x01,0x26,0x00,0x00,0x00,0x00,0x00);//初始化驱动器状态机
    //   my_can_tx_fun(0x610,0x23,0x01,0x26,0x00,0x00,0x00,0x00,0x00);//初始化驱动器状态机
    //   my_can_tx_fun(0x611,0x23,0x01,0x26,0x00,0x00,0x00,0x00,0x00);//初始化驱动器状态机
    //   flag        = 0;
    //   RX_DATA_LEN = 0;
    // }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}


void setup() {
  //创建互斥锁
  xMutexInventory = xSemaphoreCreateMutex();
  xSerialMutex    = xSemaphoreCreateMutex();
  xCanMutex       = xSemaphoreCreateMutex();
  //CAN初始化
  // CAN_cfg.speed     = CAN_SPEED_1000KBPS;
  // CAN_cfg.tx_pin_id = GPIO_NUM_27;
  // CAN_cfg.rx_pin_id = GPIO_NUM_26;
  // // CAN_cfg.tx_queue  = xQueueCreate(tx_queue_size, sizeof(CAN_frame_t)); // Init CAN Module
  // CAN_cfg.rx_queue  = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t)); // Init CAN Module
  // ESP32Can.CANInit();
  //CAN初始化
  can_init();
  //调试串口初始化
  Serial.begin(921600);

  //SBUS初始化
  x8r.begin();
  Serial2.begin(100000, SERIAL_8E2, SBUS_RX,None,true);

  is_can        = 1;
  is_sbus       = 0;
  control_mode  = 0;
  //PID初始化
  MY_PID_Init(&pid_data,u,0.1,0.1,0.01); // 设定点100，比例系数1，积分系数0.1，微分系数0.01
  //MY_PID_Init(&pid_data_V4,V4,5000,0,0.01);
  //调试信息打印
  // Serial.println("Hello, control_v1.0!");
  // Serial.println(CAN_cfg.speed);

  // tx_frame.FIR.B.FF           = CAN_frame_std;
  // tx_frame.FIR.B.DLC          = 8;
  // tx_frame.FIR.B.RTR          = CAN_no_RTR;
  //轮子初始化
  // my_can_tx_fun(0x604,0x2B,0x40,0x60,0x00,0x00,0x00,0x00,0x00);//初始化驱动器状态机
  // my_can_tx_fun(0x604,0x23,0x83,0x60,0x00,0xe3,0x03,0x00,0x00);//设置加速度时间为1000ms
  // my_can_tx_fun(0x604,0x23,0x84,0x60,0x00,0xe3,0x03,0x00,0x00);//设置减速度时间为1000ms
  // my_can_tx_fun(0x604,0x23,0xff,0x60,0x00,0x3c,0x00,0x00,0x00);//设置目标速度为60rpm
  // my_can_tx_fun(0x604,0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00);//设置为速度模式
  // my_can_tx_fun(0x604,0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00);//切换驱动器状态机为shutdown
  // my_can_tx_fun(0x604,0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00);//切换状态机为switch on
  // my_can_tx_fun(0x604,0x2B,0x40,0x60,0x00,0x0f,0x00,0x00,0x00);//切换状态机为OPERATION ENABLE+SWITCH ON，没有检测到故障，驱动功能启用，并对电机上电
  
  // my_can_tx_fun(0x605,0x2B,0x40,0x60,0x00,0x00,0x00,0x00,0x00);//初始化驱动器状态机
  // my_can_tx_fun(0x605,0x23,0x83,0x60,0x00,0xe3,0x03,0x00,0x00);//设置加速度时间为1000ms
  // my_can_tx_fun(0x605,0x23,0x84,0x60,0x00,0xe3,0x03,0x00,0x00);//设置减速度时间为1000ms
  // my_can_tx_fun(0x605,0x23,0xff,0x60,0x00,0x3c,0x00,0x00,0x00);//设置目标速度为60rpm
  // my_can_tx_fun(0x605,0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00);//设置为速度模式
  // my_can_tx_fun(0x605,0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00);//切换驱动器状态机为shutdown
  // my_can_tx_fun(0x605,0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00);//切换状态机为switch on
  // my_can_tx_fun(0x605,0x2B,0x40,0x60,0x00,0x0f,0x00,0x00,0x00);//切换状态机为OPERATION ENABLE+SWITCH ON，没有检测到故障，驱动功能启用，并对电机上电
  
  // my_can_tx_fun(0x606,0x2B,0x40,0x60,0x00,0x00,0x00,0x00,0x00);//初始化驱动器状态机
  // my_can_tx_fun(0x606,0x23,0x83,0x60,0x00,0xe3,0x03,0x00,0x00);//设置加速度时间为1000ms
  // my_can_tx_fun(0x606,0x23,0x84,0x60,0x00,0xe3,0x03,0x00,0x00);//设置减速度时间为1000ms
  // my_can_tx_fun(0x606,0x23,0xff,0x60,0x00,0x3c,0x00,0x00,0x00);//设置目标速度为60rpm
  // my_can_tx_fun(0x606,0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00);//设置为速度模式
  // my_can_tx_fun(0x606,0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00);//切换驱动器状态机为shutdown
  // my_can_tx_fun(0x606,0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00);//切换状态机为switch on
  // my_can_tx_fun(0x606,0x2B,0x40,0x60,0x00,0x0f,0x00,0x00,0x00);//切换状态机为OPERATION ENABLE+SWITCH ON，没有检测到故障，驱动功能启用，并对电机上电

  // my_can_tx_fun(0x607,0x2B,0x40,0x60,0x00,0x00,0x00,0x00,0x00);//初始化驱动器状态机
  // my_can_tx_fun(0x607,0x23,0x83,0x60,0x00,0xe3,0x03,0x00,0x00);//设置加速度时间为1000ms
  // my_can_tx_fun(0x607,0x23,0x84,0x60,0x00,0xe3,0x03,0x00,0x00);//设置减速度时间为1000ms
  // my_can_tx_fun(0x607,0x23,0xff,0x60,0x00,0x3c,0x00,0x00,0x00);//设置目标速度为60rpm
  // my_can_tx_fun(0x607,0x2f,0x60,0x60,0x00,0x03,0x00,0x00,0x00);//设置为速度模式
  // my_can_tx_fun(0x607,0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00);//切换驱动器状态机为shutdown
  // my_can_tx_fun(0x607,0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00);//切换状态机为switch on
  // my_can_tx_fun(0x607,0x2B,0x40,0x60,0x00,0x0f,0x00,0x00,0x00);//切换状态机为OPERATION ENABLE+SWITCH ON，没有检测到故障，驱动功能启用，并对电机上电
  my_can_tx_fun(0x604,0x2B,0x40,0x60,0x00,0x00,0x00,0x00,0x00);//初始化驱动器状态机
  my_can_tx_fun(0x604,0x23,0x87,0x60,0x00,0x64,0x00,0x00,0x00);//设置转矩减速时间100ms
  my_can_tx_fun(0x604,0x2F,0x60,0x00,0x00,0x04,0x00,0x00,0x00);//切换转矩模式
  my_can_tx_fun(0x604,0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00);//切换驱动器状态机为shutdown
  my_can_tx_fun(0x604,0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00);//切换状态机为switch on
  my_can_tx_fun(0x604,0x2B,0x40,0x60,0x00,0x0f,0x00,0x00,0x00);//切换状态机为OPERATION ENABLE+SWITCH ON，没有检测到故障，驱动功能启用，并对电机上电

  my_can_tx_fun(0x605,0x2B,0x40,0x60,0x00,0x00,0x00,0x00,0x00);//初始化驱动器状态机
  my_can_tx_fun(0x605,0x23,0x87,0x60,0x00,0x64,0x00,0x00,0x00);//设置转矩减速时间100ms
  my_can_tx_fun(0x605,0x2F,0x60,0x00,0x00,0x04,0x00,0x00,0x00);//切换转矩模式
  my_can_tx_fun(0x605,0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00);//切换驱动器状态机为shutdown
  my_can_tx_fun(0x605,0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00);//切换状态机为switch on
  my_can_tx_fun(0x605,0x2B,0x40,0x60,0x00,0x0f,0x00,0x00,0x00);//切换状态机为OPERATION ENABLE+SWITCH ON，没有检测到故障，驱动功能启用，并对电机上电

  my_can_tx_fun(0x606,0x2B,0x40,0x60,0x00,0x00,0x00,0x00,0x00);//初始化驱动器状态机
  my_can_tx_fun(0x606,0x23,0x87,0x60,0x00,0x64,0x00,0x00,0x00);//设置转矩减速时间100ms
  my_can_tx_fun(0x606,0x2F,0x60,0x00,0x00,0x04,0x00,0x00,0x00);//切换转矩模式
  my_can_tx_fun(0x606,0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00);//切换驱动器状态机为shutdown
  my_can_tx_fun(0x606,0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00);//切换状态机为switch on
  my_can_tx_fun(0x606,0x2B,0x40,0x60,0x00,0x0f,0x00,0x00,0x00);//切换状态机为OPERATION ENABLE+SWITCH ON，没有检测到故障，驱动功能启用，并对电机上电

  my_can_tx_fun(0x607,0x2B,0x40,0x60,0x00,0x00,0x00,0x00,0x00);//初始化驱动器状态机
  my_can_tx_fun(0x607,0x23,0x87,0x60,0x00,0x64,0x00,0x00,0x00);//设置转矩减速时间100ms
  my_can_tx_fun(0x607,0x2F,0x60,0x00,0x00,0x04,0x00,0x00,0x00);//切换转矩模式
  my_can_tx_fun(0x607,0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00);//切换驱动器状态机为shutdown
  my_can_tx_fun(0x607,0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00);//切换状态机为switch on
  my_can_tx_fun(0x607,0x2B,0x40,0x60,0x00,0x0f,0x00,0x00,0x00);//切换状态机为OPERATION ENABLE+SWITCH ON，没有检测到故障，驱动功能启用，并对电机上电

  my_can_tx_fun(0x608,0x23,0x81,0x60,0x00,0xe8,0x03,0x00,0x00);//梯形速度写入1000RPM
  my_can_tx_fun(0x608,0x23,0x83,0x60,0x00,0x20,0x4e,0x00,0x00);//梯形加减速写入20000RPM/S
  my_can_tx_fun(0x608,0x2b,0x40,0x60,0x00,0x2f,0x00,0x00,0x00);//绝对位置控制模式+新位置立即执行

  my_can_tx_fun(0x609,0x23,0x81,0x60,0x00,0xe8,0x03,0x00,0x00);//梯形速度写入1000RPM
  my_can_tx_fun(0x609,0x23,0x83,0x60,0x00,0x20,0x4e,0x00,0x00);//梯形加减速写入20000RPM/S
  my_can_tx_fun(0x609,0x2b,0x40,0x60,0x00,0x2f,0x00,0x00,0x00);//绝对位置控制模式+新位置立即执行

  my_can_tx_fun(0x610,0x23,0x81,0x60,0x00,0xe8,0x03,0x00,0x00);//梯形速度写入1000RPM
  my_can_tx_fun(0x610,0x23,0x83,0x60,0x00,0x20,0x4e,0x00,0x00);//梯形加减速写入20000RPM/S
  my_can_tx_fun(0x610,0x2b,0x40,0x60,0x00,0x2f,0x00,0x00,0x00);//绝对位置控制模式+新位置立即执行

  my_can_tx_fun(0x611,0x23,0x81,0x60,0x00,0xe8,0x03,0x00,0x00);//梯形速度写入1000RPM
  my_can_tx_fun(0x611,0x23,0x83,0x60,0x00,0x20,0x4e,0x00,0x00);//梯形加减速写入20000RPM/S
  my_can_tx_fun(0x611,0x2b,0x40,0x60,0x00,0x2f,0x00,0x00,0x00);//绝对位置控制模式+新位置立即执行


  my_can_tx_fun(0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00); //启动NMT管理器

  //can接收数据并解析任务
  xTaskCreate(            rx_data_app,        "rx_data_app",        1024 * 20, NULL,1,NULL);
  //运动解算控制任务
  //xTaskCreate(            motor_control_app,  "motor_control_app",  1024 * 10, NULL,1,NULL);
  //车身解析以及保护任务
  xTaskCreate(            car_state_safe_app, "car_state_safe_app", 1024 * 10, NULL,1,NULL);  
  //ESP32 Board MAC Address:  80:64:6F:2D:30:D8
  // xTaskCreatePinnedToCore(BT_debug_APP,       "BTdebugTask",        1024 * 10,NULL,1,NULL,0);
  //SBUS数据解析任务
  xTaskCreate(            sbus_rx_app,        "sbus_rx_app",        1024 * 10, NULL,1,NULL);  
  //蓝牙接收数据任务
  xTaskCreate(            bt_rx_app,          "bt_rx_app",          1024 * 10, NULL,1,NULL);  
}

void loop() {
  //删除任务
  vTaskDelete(NULL);
}
