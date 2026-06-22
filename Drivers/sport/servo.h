#ifndef __SERVO_H
#define __SERVO_H

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "tim.h"
#include "usart.h"
#include "task.h"
#include "cmsis_os.h"
// 动作执行模式
typedef enum {
    ACTION_MODE_ONCE,    // 单次执行
    ACTION_MODE_HOLD     // 持续保持
} ActionMode;
typedef enum {
    ACTION_INVALID = 0xFF // 无效动作标志
} ActionStatus;
// 预设动作类型
typedef enum {
    ACTION_STOP = 0,       // 停止 (1.5ms)
    ACTION_FORWARD = 1,   // 前进动作
    ACTION_BACKWARD = 2,   // 后退动作
    ACTION_TURN_LEFT = 3,  // 左转动作
    ACTION_TURN_RIGHT = 4, // 右转动作
    ACTION_DIVE = 5,       // 下潜动作 (0.5ms)
    ACTION_SURFACE = 6     // 上浮动作 (2.5ms)
} ActionType;
// 舵机位置结构体
typedef struct {
    uint16_t servo1_angle; // 舵机1角度 (0-270°)
    uint16_t servo2_angle; // 舵机2角度 (0-270°)
    uint16_t servo3_angle; // 舵机3角度 (0-270°)
} GroupAction;
/*焦伟写*/
// 全局变量定义,状态对应动作
typedef enum {
    PWM_STATE_DUTY1,
    PWM_STATE_DUTY2,
    PWM_STATE_DUTY3,
    PWM_STATE_COUNT
} PwmState;
// 电机状态
//typedef enum {
//    MOTOR_STOP,
//    MOTOR_FORWARD,
//    MOTOR_REVERSE
//} MotorState;
// 螺旋桨引脚定义
#define MOTOR_LUO1_PIN1 GPIO_PIN_2      // 螺旋桨1引脚1 (PA2)
#define MOTOR_LUO1_PIN2 GPIO_PIN_3      // 螺旋桨1引脚2 (PA3)
#define MOTOR_LUO2_PIN1 GPIO_PIN_9      // 螺旋桨2引脚1 (PA9)
#define MOTOR_LUO2_PIN2 GPIO_PIN_10     // 螺旋桨2引脚2 (PA10)
#define MOTOR_PORT GPIOA

// 螺旋桨状态枚举
typedef enum {
    PROPELLER_STOP,      // 停止
    PROPELLER_FORWARD,   // 正转
    PROPELLER_REVERSE    // 反转
} PropellerState;

// 螺旋桨控制结构体
typedef struct {
    PropellerState state;   // 当前状态
    uint32_t start_time;    // 开始时间
    uint8_t is_running;     // 是否正在运行
} 
PropellerControl;typedef enum {
    PWM_STATE_CH4_MIN,      // 最小角度
    PWM_STATE_CH4_LOW1,      // 低角度1
    PWM_STATE_CH4_LOW2,      // 低角度2
    PWM_STATE_CH4_MID,       // 中间角度
    PWM_STATE_CH4_HIGH1,     // 高角度1
    PWM_STATE_CH4_HIGH2,     // 高角度2
    PWM_STATE_CH4_MAX,       // 最大角度
	PWM_STATE_CH4_MAX2,       // 最大角度
	PWM_STATE_CH4_MAX3,       // 最大角度
	PWM_STATE_CH4_MAX4,       // 最大角度
    PWM_STATE_CH4_COUNT
} PwmStateCh4;
typedef enum {
    DIRECTION_STOP,
    DIRECTION_FORWARD,
	DIRECTION_LEFT,
    DIRECTION_RIGHT
} SwimDirection;
// 巡游方向
typedef enum {
    CRUISE_IDLE,        // 空闲状态
	CRUISE_TURNING,     // 正在转弯
    CRUISE_STRAIGHT     // 正在直行
} CruiseState;
typedef enum {
    CRUISE_LEFT,        // 左转巡游
    CRUISE_RIGHT        // 右转巡游
} CruiseDirection;
// PID控制器结构体
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float integral;      // 积分项
    float prev_error;    // 上一次误差
    float output;        // 当前输出
} PID_Controller;
// 巡游控制参数
typedef struct {
    CruiseState state;             // 当前状态
    CruiseDirection direction;     // 巡游方向
    float initial_yaw;             // 初始偏航角
    float target_turn_rate;        // 目标转弯速率(度/秒)
    float total_angle;             // 累计转弯角度
    uint32_t start_time;           // 开始时间(ms)
    PID_Controller pid;            // PID控制器
	uint8_t step;                  // 当前步骤 (0-7)
    uint32_t phase_start_time;     // 当前阶段开始时间
	uint8_t active;				   //是否巡游
	float target_angle;            // 目标转弯角度 (90度)
} CruiseControl;
// 电机控制结构体
//typedef struct {
//    MotorState state;              // 当前状态
//    uint32_t start_time;           // 开始时间
//    uint8_t is_running;            // 是否正在运行
//} MotorControl;
extern volatile PwmState current_state_x;  //初始状态
extern uint32_t last_jiao_switch_time;  //初始时间
extern volatile PropellerState propeller_state;
// 占空比和时间配置（单位：毫秒）
extern const float duty_cycles1[PWM_STATE_COUNT];
extern const uint32_t state_durations1[PWM_STATE_COUNT];  // 各状态持续时间
extern const float duty_cycles2[PWM_STATE_COUNT];
extern volatile uint8_t Flag_Servo;
extern volatile SwimDirection turn_direction;
extern osMutexId_t cruiseMutex;
extern osMutexId_t servoMutex;
extern CruiseControl cruise_control;//巡游控制
void Servo_Init(void);
void Servo_SetAngle(uint8_t servo_id, uint16_t angle);
void Servo_ExecuteGroupAction(ActionType action, ActionMode mode);
void set_pwm(TIM_HandleTypeDef *htim, uint32_t Channel, float duty_cycle);
void Servo_UpdateTask(void);
void Turn_Left(void); 
void Turn_Right(void);
void Cruise_Start(CruiseDirection direction);
void Cruise_Update(void);
void Cruise_Stop(void);
float Cruise_Calculate_PID(PID_Controller* pid, float error, float dt);
void Servo_Stop(void); 
void Servo_ForwardTask(void); 
void Motor_Init(void);
void ControlPropellers(PropellerState state);
void UpdatePropellers(void);

#endif
