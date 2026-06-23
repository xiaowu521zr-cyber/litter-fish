#include "stm32f4xx_hal.h"                  // Device header
#include "tim.h"
#include "math.h"
#include "servo.h"
#include "cmsis_os.h"
#include <stdio.h>
#include "sensors.h"
// 舵机运动状态标志
static volatile uint8_t isServoRunning = 0;
#define TIM_PERIOD 200  // 定时器周期值
#define MIN_PULSE 5     // 0.5ms对应计数值 (0.5/20 * 200=5)
#define MAX_PULSE 25    // 2.5ms对应计数值 (2.5/20 * 200=25)

#define Servo_1_GPIO_GROUP GPIOA
#define Servo_1_GPIO_PIN   GPIO_PIN_6

#define Servo_2_GPIO_GROUP GPIOA
#define Servo_2_GPIO_PIN   GPIO_PIN_7

#define Servo_3_GPIO_GROUP GPIOB
#define Servo_3_GPIO_PIN   GPIO_PIN_0

#define Servo_4_GPIO_GROUP GPIOB
#define Servo_4_GPIO_PIN   GPIO_PIN_1

//#define MOTOR_PIN_PC12 GPIO_PIN_12
//#define MOTOR_PIN_PD2  GPIO_PIN_2

#define Servo_1_CHANNEL    TIM_CHANNEL_1
#define Servo_2_CHANNEL    TIM_CHANNEL_2
#define Servo_3_CHANNEL    TIM_CHANNEL_3
#define Servo_4_CHANNEL    TIM_CHANNEL_4

#define MOTOR_LUO1_PIN1 GPIO_PIN_2      // 螺旋桨1引脚1 (PA2)
#define MOTOR_LUO1_PIN2 GPIO_PIN_3      // 螺旋桨1引脚2 (PA3)
#define MOTOR_LUO2_PIN1 GPIO_PIN_9      // 螺旋桨2引脚1 (PA9)
#define MOTOR_LUO2_PIN2 GPIO_PIN_10     // 螺旋桨2引脚2 (PA10)
#define MOTOR_PORT GPIOA

volatile PwmState current_state_x = PWM_STATE_DUTY1;  //初始状态，记得初始化p波
uint32_t last_jiao_switch_time = 0;  //初始时间，一起初始化
volatile PwmStateCh4 current_state_ch4 = PWM_STATE_CH4_MID;
volatile SwimDirection turn_direction  = DIRECTION_STOP;   //转弯标志位
uint32_t last_ch4_switch_time = 0;
// 占空比和时间配置（单位：毫秒）胸部
const float duty_cycles2[PWM_STATE_COUNT] = {7.0f, 3.0f, 11.0f}; 
const uint32_t state_durations1[PWM_STATE_COUNT] = {200, 200, 200};  // 各状态持续时间
volatile PropellerState propeller_state = PROPELLER_STOP;
// 尾部舵机参数：缩小运动范围
const float tail_duty_cycles[PWM_STATE_COUNT] = {
    6.5f,  // 小幅度1 
    7.5f,  // 小幅度2
    8.5f   // 小幅度3
};// 预设动作数组
const float turn_duty_cycles[2][PWM_STATE_COUNT][2] = {
    // 左转配置 [状态][通道]
    {
        {6.5f, 8.5f},   // 状态0: CH1小幅度, CH2大幅度
        {2.8f, 12.2f},  // 状态1: CH1幅度最小, CH2幅度最大
        {9.5f, 5.5f}    // 状态2: CH1中幅度, CH2小幅度
    },
    // 右转配置 [状态][通道]
    {
        {8.5f, 6.5f},   // 状态0: CH1大幅度, CH2小幅度
        {12.2f, 2.8f},  // 状态1: CH1幅度最大, CH2幅度最小
        {5.5f, 9.5f}    // 状态2: CH1小幅度, CH2中幅度
    }
};
static const GroupAction preset_actions[] = {
	[ACTION_STOP]      = { 135,  135, 135},  // 中立位 (1.8ms)
    [ACTION_FORWARD]   = {  30,  240, 180},  // 最大前倾
    [ACTION_BACKWARD]  = { 240,   30,  90},  // 最大后仰
    [ACTION_TURN_LEFT] = {  30,  240,  30},  // 左转极限
    [ACTION_TURN_RIGHT]= { 240,   30, 240},  // 右转极限
    [ACTION_DIVE]      = {   0,    0,   0},  // 下潜极限
    [ACTION_SURFACE]   = { 270,  270, 270}   // 上浮极限
};
const float duty_cycles4[PWM_STATE_CH4_COUNT] = {
    4.5f,  // 0° - 最小角度
    5.5f,  // 45°
    6.5f,  // 90° - 低角度2
    7.5f,  // 135° - 中间角度
    8.5f,  // 160° - 高角度1
    9.5f,  // 175° - 高角度2
    10.5f, // 180° - 最大角度
	11.5,
	13.5,
	35.5
};
const uint32_t state_durations4[PWM_STATE_CH4_COUNT] = {
    1500,  // 最小角度
    1500,  // 45°
    1500,  // 90°
    1500,  // 135°
    1500,  // 160°
    1500,  // 175°
    1500,  // 最大角度
	1500,
	1500,
	1500
};
extern TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef *g_servoTimer = &htim3;
extern TIM_HandleTypeDef htim2;
float Cruise_Calculate_PID(PID_Controller* pid, float error, float dt)
{
    // 积分项累加
    pid->integral += error * dt;
    // 防止积分饱和
    const float max_integral = 50.0f;
    if (pid->integral > max_integral) pid->integral = max_integral;
    if (pid->integral < -max_integral) pid->integral = -max_integral;
    // 微分项计算
    float derivative = (error - pid->prev_error) / dt;
    // 保存当前误差用于下一次计算
    pid->prev_error = error;
    // 计算PID输出
    return pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
}
PropellerControl propeller_control = {
    .state = PROPELLER_STOP,
    .start_time = 0,
    .is_running = 0
};
// 在此添加 cruise_control 的定义
CruiseControl cruise_control = {
    .state = CRUISE_IDLE,
    .direction = CRUISE_LEFT,
    .initial_yaw = 0.0f,
    .target_turn_rate = 45.0f,  // 默认45°/s转弯速率
    .total_angle = 0.0f,
    .start_time = 0,
	.step = 0,
    .active = 0,
    .pid = {
        .Kp = 0.8f,   // 比例系数 - 调节响应速度
        .Ki = 0.02f,  // 积分系数 - 消除静态误差
        .Kd = 0.1f,   // 微分系数 - 阻尼震荡
        .integral = 0.0f,
        .prev_error = 0.0f,
        .output = 0.0f
    }
};
void Servo_Init(void)
{
	// 启动所有4个PWM通道
	last_jiao_switch_time = HAL_GetTick();
	set_pwm(&htim3, TIM_CHANNEL_1, 7.5f);
	set_pwm(&htim3, TIM_CHANNEL_2, 7.5f);
	set_pwm(&htim3, TIM_CHANNEL_3, 7.5f);
	set_pwm(&htim3, TIM_CHANNEL_4, 8.5f);
	current_state_x = PWM_STATE_DUTY1;
	turn_direction = DIRECTION_STOP;
	printf("舵机驱动已初始化\n");
}
/* 转弯控制接口 */
void Turn_Left(void) 
{
     // 重置状态机
    current_state_x = PWM_STATE_DUTY1;
    last_jiao_switch_time = HAL_GetTick();
	turn_direction  = DIRECTION_LEFT;
    printf("开始左转\n");
}
void Turn_Right(void) 
{
    // 重置状态机
    current_state_x = PWM_STATE_DUTY1;
    last_jiao_switch_time = HAL_GetTick();
	turn_direction  = DIRECTION_RIGHT;
    printf("开始右转\n");
}
void Turn_Stop(void) 
{
    turn_direction  = DIRECTION_STOP;
    // 恢复到中立位置
    set_pwm(&htim3, TIM_CHANNEL_1, 7.5f);
    set_pwm(&htim3, TIM_CHANNEL_2, 7.5f);
    printf("停止转弯\n");
}
// 设置舵机角度
void Servo_SetAngle(uint8_t servo_id, uint16_t angle)
{
    // 1. 角度范围限定
    if(angle > 180) angle = 180;
    
    // 2. 线性映射角度到脉宽
    uint16_t pulse = MIN_PULSE + (angle * (MAX_PULSE - MIN_PULSE)) / 180;
    
    // 3. 设置PWM比较值
    switch(servo_id) {
        case 0: __HAL_TIM_SET_COMPARE(g_servoTimer, TIM_CHANNEL_1, pulse); break;
        case 1: __HAL_TIM_SET_COMPARE(g_servoTimer, TIM_CHANNEL_2, pulse); break;
        case 2: __HAL_TIM_SET_COMPARE(g_servoTimer, TIM_CHANNEL_3, pulse); break;
        case 3: __HAL_TIM_SET_COMPARE(g_servoTimer, TIM_CHANNEL_4, pulse); break;
    }
    
    // 4. 调试输出（可显示实际占空比）
    printf("舵机%d: 角度=%u° → 占空比=%.3f%% (脉冲=%u)\n",
           servo_id, angle, 
           (pulse * 100.0f) / TIM_PERIOD,  // 实际占空比计算
           pulse);
}
void Servo_ExecuteGroupAction(ActionType action, ActionMode mode)
{
static ActionType last_action = ACTION_INVALID;
    
    // 过滤重复指令 (模式为HOLD且动作未变化)
    if(mode == ACTION_MODE_HOLD && action == last_action) {
        return;
    }
    
    last_action = action;
    
    if (action > ACTION_SURFACE) {
        printf("无效动作ID: %d\n", action);
        return;
    }
    
    const GroupAction *act = &preset_actions[action];
    Servo_SetAngle(0, act->servo1_angle);
    Servo_SetAngle(1, act->servo2_angle);
    Servo_SetAngle(2, act->servo3_angle);
    
    printf("执行舵机组动作: %d (%s)\n", 
           action, 
           mode == ACTION_MODE_ONCE ? "单次" : "保持");
}
// 设置 PWM 占空比的函数（单位：百分比）
void set_pwm(TIM_HandleTypeDef *htim, uint32_t Channel, float duty_cycle) {
    // 计算 CCR 值：CCRx = (占空比 / 100) * (ARR + 1)
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
	uint32_t pulse = (uint32_t)((duty_cycle / 100.0f) * (arr + 1));
    // 使用 HAL 宏设置比较值
    __HAL_TIM_SET_COMPARE(htim, Channel, pulse);
}
///* 新增通道四状态切换函数 */
void UpdateCh4State(void) {
    uint32_t current_time = HAL_GetTick();
    switch(current_state_ch4) {
		case PWM_STATE_CH4_MIN:
			if(current_time - last_ch4_switch_time >= state_durations4[current_state_ch4]) {
                current_state_ch4 = PWM_STATE_CH4_LOW1;
                set_pwm(&htim3, TIM_CHANNEL_4, duty_cycles4[current_state_ch4]);
				last_ch4_switch_time = current_time;
			}
			break;
		case PWM_STATE_CH4_LOW1:
		if(current_time - last_ch4_switch_time >= state_durations4[current_state_ch4]) {
                current_state_ch4 = PWM_STATE_CH4_LOW2;
                set_pwm(&htim3, TIM_CHANNEL_4, duty_cycles4[current_state_ch4]);
			last_ch4_switch_time = current_time;
			}
			break;
		case PWM_STATE_CH4_MID:
		if(current_time - last_ch4_switch_time >= state_durations4[current_state_ch4]) {
                current_state_ch4 = PWM_STATE_CH4_MID;
                set_pwm(&htim3, TIM_CHANNEL_4, duty_cycles4[current_state_ch4]);
			last_ch4_switch_time = current_time;
			}
			break;
		case PWM_STATE_CH4_HIGH1:
		if(current_time - last_ch4_switch_time >= state_durations4[current_state_ch4]) {
                current_state_ch4 = PWM_STATE_CH4_HIGH1;
                set_pwm(&htim3, TIM_CHANNEL_4, duty_cycles4[current_state_ch4]);
			last_ch4_switch_time = current_time;
			}
			break;
		case PWM_STATE_CH4_HIGH2:
		if(current_time - last_ch4_switch_time >= state_durations4[current_state_ch4]) {
                current_state_ch4 = PWM_STATE_CH4_HIGH2;
                set_pwm(&htim3, TIM_CHANNEL_4, duty_cycles4[current_state_ch4]);
			last_ch4_switch_time = current_time;
			}
			break;
		case PWM_STATE_CH4_MAX:
		if(current_time - last_ch4_switch_time >= state_durations4[current_state_ch4]) {
                current_state_ch4 = PWM_STATE_CH4_MAX;
                set_pwm(&htim3, TIM_CHANNEL_4, duty_cycles4[current_state_ch4]);
			last_ch4_switch_time = current_time;
			}
			break;
	case PWM_STATE_CH4_MAX2:
		if(current_time - last_ch4_switch_time >= state_durations4[current_state_ch4]) {
                current_state_ch4 = PWM_STATE_CH4_MAX2;
                set_pwm(&htim3, TIM_CHANNEL_4, duty_cycles4[current_state_ch4]);
			last_ch4_switch_time = current_time;
			}
			break;
	case PWM_STATE_CH4_MAX3:
		if(current_time - last_ch4_switch_time >= state_durations4[current_state_ch4]) {
                current_state_ch4 = PWM_STATE_CH4_MAX3;
                set_pwm(&htim3, TIM_CHANNEL_4, duty_cycles4[current_state_ch4]);
			last_ch4_switch_time = current_time;
			}
			break;
	case PWM_STATE_CH4_MAX4:
		if(current_time - last_ch4_switch_time >= state_durations4[current_state_ch4]) {
                current_state_ch4 = PWM_STATE_CH4_MAX4;
                set_pwm(&htim3, TIM_CHANNEL_4, duty_cycles4[current_state_ch4]);
			last_ch4_switch_time = current_time;
			}
			break;

	}
}
void Servo_UpdateTask(void) {
    uint32_t current_time = HAL_GetTick();
    //有任务时执行巡游任务
//	if (cruise_control.state != CRUISE_IDLE) {
//        Cruise_Update();
//    }
//	/* 状态机执行 */
//    if(turn_direction == DIRECTION_STOP || turn_direction == DIRECTION_FORWARD){
//	switch(current_state_x) {
//        case PWM_STATE_DUTY1:
//			// 检查状态持续时间是否结束
//            if(current_time - last_jiao_switch_time >= state_durations1[current_state_x]) {
//                // 切换至下一状态
//                current_state_x = PWM_STATE_DUTY2;
//				 const float chest_duty = duty_cycles2[PWM_STATE_DUTY2]; 
//                // 应用新占空比                           
//				set_pwm(&htim3, TIM_CHANNEL_1, chest_duty);
//                set_pwm(&htim3, TIM_CHANNEL_2, 15.0f - chest_duty);				
//				set_pwm(&htim3, TIM_CHANNEL_3, tail_duty_cycles[PWM_STATE_DUTY2]);
//                
//                // 更新切换时间戳
//                last_jiao_switch_time = current_time;
//            }
//			break;
//        case PWM_STATE_DUTY2:
//			// 检查状态持续时间是否结束
//            if(current_time - last_jiao_switch_time >= state_durations1[current_state_x]) {
//                // 切换至下一状态
//                current_state_x = PWM_STATE_DUTY3;
//                // 应用新占空比
//				const float chest_duty = duty_cycles2[PWM_STATE_DUTY3];
//				// 胸部舵机同步设置
//                set_pwm(&htim3, TIM_CHANNEL_1, chest_duty);
//                set_pwm(&htim3, TIM_CHANNEL_2, 15.0f - chest_duty);			
//				// 尾部舵机小幅度运动
//                set_pwm(&htim3, TIM_CHANNEL_3, tail_duty_cycles[PWM_STATE_DUTY3]);                
//                // 更新切换时间戳
//                last_jiao_switch_time = current_time;
//            }
//			break;
//        case PWM_STATE_DUTY3:
//            // 检查状态持续时间是否结束
//            if(current_time - last_jiao_switch_time >= state_durations1[current_state_x]) {
//                // 切换至下一状态
//				current_state_x = PWM_STATE_DUTY1;// 应用新占空比
//				const float chest_duty = duty_cycles2[PWM_STATE_DUTY1];
//                set_pwm(&htim3, TIM_CHANNEL_1, chest_duty);
//                set_pwm(&htim3, TIM_CHANNEL_2, 15.0f - chest_duty);
//				set_pwm(&htim3, TIM_CHANNEL_3, tail_duty_cycles[PWM_STATE_DUTY1]);
////				set_pwm(&htim3, TIM_CHANNEL_4, duty_cycles1[current_state_x]);
//                // 更新切换时间戳
//                last_jiao_switch_time = current_time;
//            }
//            break;
//			        default:
//            current_state_x = PWM_STATE_DUTY1;  // 错误处理
//            break;
//    }
//}
			 /* 转弯控制逻辑 */
	if (turn_direction == DIRECTION_LEFT || turn_direction == DIRECTION_RIGHT) {
	switch(current_state_x) {
        case PWM_STATE_DUTY1:
        case PWM_STATE_DUTY2:
        case PWM_STATE_DUTY3:
            if(current_time - last_jiao_switch_time >= state_durations1[current_state_x]) {
                // 计算下一个状态
                uint8_t next_state = (current_state_x + 1) % PWM_STATE_COUNT;
                
                // 获取转弯方向索引 (0=左转,1=右转)
                uint8_t turn_idx = (turn_direction == DIRECTION_LEFT) ? 0 : 1;
          // 应用转弯占空比并添加PID调整
              float base_ch1 = turn_duty_cycles[turn_idx][next_state][0];
              float base_ch2 = turn_duty_cycles[turn_idx][next_state][1];
			// PID调整 - 左转增加CH1、减少CH2，右转相反
                float pid_adjust = cruise_control.pid.output;
                if (turn_direction == DIRECTION_LEFT) {
                    base_ch1 += pid_adjust;
                    base_ch2 -= pid_adjust;
                } else {
                    base_ch1 -= pid_adjust;
                    base_ch2 += pid_adjust;
                }
                // 应用转弯占空比                           
                set_pwm(&htim3, TIM_CHANNEL_1, base_ch1);
                set_pwm(&htim3, TIM_CHANNEL_2, base_ch2);
                set_pwm(&htim3, TIM_CHANNEL_3, tail_duty_cycles[next_state]);
                current_state_x = (PwmState)next_state;
                last_jiao_switch_time = current_time;
				static uint32_t last_print = 0;
				if (current_time - last_print > 500) {
                        printf("%s转: 状态%d | CH1=%.1f%%, CH2=%.1f%%, PID=%.2f\n",
                               turn_direction == DIRECTION_LEFT ? "左" : "右",
                               next_state,
                               base_ch1, base_ch2, pid_adjust);
                        last_print = current_time;
                    } 
				}
				break;
			default:
                current_state_x = PWM_STATE_DUTY1;  // 错误处理
                break;
						}
				}
	}
void Servo_ForwardTask(void) 
{
	if (turn_direction != DIRECTION_FORWARD) {
        return;
    }
    uint32_t current_time = HAL_GetTick();
    
    if(turn_direction == DIRECTION_STOP || turn_direction == DIRECTION_FORWARD){
	switch(current_state_x) {
        case PWM_STATE_DUTY1:
			// 检查状态持续时间是否结束
            if(current_time - last_jiao_switch_time >= state_durations1[current_state_x]) {
                // 切换至下一状态
                current_state_x = PWM_STATE_DUTY2;
				 const float chest_duty = duty_cycles2[PWM_STATE_DUTY2]; 
                // 应用新占空比                           
//				set_pwm(&htim3, TIM_CHANNEL_1, chest_duty);
//                set_pwm(&htim3, TIM_CHANNEL_2, 15.0f - chest_duty);				
				set_pwm(&htim3, TIM_CHANNEL_3, tail_duty_cycles[PWM_STATE_DUTY2]);
                
                // 更新切换时间戳
                last_jiao_switch_time = current_time;
            }
			break;
        case PWM_STATE_DUTY2:
			// 检查状态持续时间是否结束
            if(current_time - last_jiao_switch_time >= state_durations1[current_state_x]) {
                // 切换至下一状态
                current_state_x = PWM_STATE_DUTY3;
                // 应用新占空比
				const float chest_duty = duty_cycles2[PWM_STATE_DUTY3];
				// 胸部舵机同步设置
//                set_pwm(&htim3, TIM_CHANNEL_1, chest_duty);
//                set_pwm(&htim3, TIM_CHANNEL_2, 15.0f - chest_duty);			
				// 尾部舵机小幅度运动
                set_pwm(&htim3, TIM_CHANNEL_3, tail_duty_cycles[PWM_STATE_DUTY3]);                
                // 更新切换时间戳
                last_jiao_switch_time = current_time;
            }
			break;
        case PWM_STATE_DUTY3:
            // 检查状态持续时间是否结束
            if(current_time - last_jiao_switch_time >= state_durations1[current_state_x]) {
                // 切换至下一状态
				current_state_x = PWM_STATE_DUTY1;// 应用新占空比
				const float chest_duty = duty_cycles2[PWM_STATE_DUTY1];
//                set_pwm(&htim3, TIM_CHANNEL_1, chest_duty);
//                set_pwm(&htim3, TIM_CHANNEL_2, 15.0f - chest_duty);
				set_pwm(&htim3, TIM_CHANNEL_3, tail_duty_cycles[PWM_STATE_DUTY1]);
//				set_pwm(&htim3, TIM_CHANNEL_4, duty_cycles1[current_state_x]);
                // 更新切换时间戳
                last_jiao_switch_time = current_time;
            }
            break;
			        default:
            current_state_x = PWM_STATE_DUTY1;  // 错误处理
            break;
    }
}
}

// 停止函数（专为D0指令）
void Servo_Stop(void) 
{
//	static uint8_t first_call = 1;
//     if(first_call) {
	set_pwm(&htim3, TIM_CHANNEL_1, 7.5f);
    set_pwm(&htim3, TIM_CHANNEL_2, 7.5f);
    set_pwm(&htim3, TIM_CHANNEL_3, 7.5f);
    current_state_x = PWM_STATE_DUTY1; // 重置状态机
	turn_direction = DIRECTION_STOP;
	last_jiao_switch_time = HAL_GetTick();
//	printf("asdfasdfasdfasdf\r\n");
//    first_call = 0;
//	 }
}
float angle_diff(float current, float target) {
    float diff = target - current;
    if (diff > 180.0f) {
        diff -= 360.0f;
    } else if (diff < -180.0f) {
        diff += 360.0f;
    }
    return diff;
}
void Cruise_Update(void)
{
	osMutexAcquire(cruiseMutex, osWaitForever);
    
    if (!cruise_control.active) {
        osMutexRelease(cruiseMutex);
        return;
    }   
	uint32_t current_time = HAL_GetTick();
    uint32_t phase_duration = current_time - cruise_control.phase_start_time; // 状态处理
	float dt = (current_time - cruise_control.start_time) / 1000.0f; // 秒
    switch (cruise_control.state) {
        case CRUISE_TURNING:
	if (phase_duration < 500) { // 转弯持续2秒
                // 根据方向设置固定转弯占空比
                if (cruise_control.direction == CRUISE_LEFT) {
                    // 左转：设置CH1和CH2的占空比
                    set_pwm(&htim3, TIM_CHANNEL_1, 3.5f);
                    set_pwm(&htim3, TIM_CHANNEL_2, 11.5f);
                } else {
                    // 右转：设置CH1和CH2的占空比
                    set_pwm(&htim3, TIM_CHANNEL_1, 11.5f);
                    set_pwm(&htim3, TIM_CHANNEL_2, 3.5f);
                }
			}				
				else {
// 转弯完成，进入直行阶段
                cruise_control.state = CRUISE_STRAIGHT;
                cruise_control.phase_start_time = current_time;
                
                // 设置直行模式
                osMutexAcquire(servoMutex, osWaitForever);
                turn_direction = DIRECTION_FORWARD;
                current_state_x = PWM_STATE_DUTY1;
                last_jiao_switch_time = current_time;
					// 设置胸鳍到中立位置
                set_pwm(&htim3, TIM_CHANNEL_1, 7.5f);
                set_pwm(&htim3, TIM_CHANNEL_2, 7.5f);
                osMutexRelease(servoMutex);                
		}
            break;
        case CRUISE_STRAIGHT:
				if (phase_duration < 5000) {
                // 调用您原来的直行任务
                Servo_ForwardTask();
            } else {
                // 直行完成，准备下一次转弯
                cruise_control.step++;
                
                if (cruise_control.step >= 8) {
                    // 完成4次转弯+4次直行
                    Cruise_Stop();
                    printf("巡游完成！\n");
                } else {
                    // 进入下一次转弯
					cruise_control.state = CRUISE_TURNING;
                    cruise_control.phase_start_time = current_time;
                    
                    // 设置转弯方向
                    osMutexAcquire(servoMutex, osWaitForever);
                    turn_direction = (cruise_control.direction == CRUISE_LEFT) ? 
                                    DIRECTION_LEFT : DIRECTION_RIGHT;
                    current_state_x = PWM_STATE_DUTY1;
                    last_jiao_switch_time = current_time;
                    osMutexRelease(servoMutex);
//                    CruiseDirection next_dir = cruise_control.direction;
                    
//                    // 应用转弯设置
//                    osMutexAcquire(servoMutex, osWaitForever);
//                    turn_direction = (cruise_control.direction == CRUISE_LEFT) ? 
//                                    DIRECTION_LEFT : DIRECTION_RIGHT;
//                    current_state_x = PWM_STATE_DUTY1;
//                    last_jiao_switch_time = current_time;
//                    osMutexRelease(servoMutex);
		}
    }break;
			}
	osMutexRelease(cruiseMutex);
}
void Cruise_Start(CruiseDirection direction)
{
	osMutexAcquire(cruiseMutex, osWaitForever);
	if(cruise_control.active){
		osMutexRelease(cruiseMutex);
		return;
	}
	// 设置转弯方向
    osMutexAcquire(servoMutex, osWaitForever);
    turn_direction = (direction == CRUISE_LEFT) ? 
                    DIRECTION_LEFT : DIRECTION_RIGHT;
    current_state_x = PWM_STATE_DUTY1;
    last_jiao_switch_time = HAL_GetTick();
	set_pwm(&htim3, TIM_CHANNEL_1, 7.5f);
    set_pwm(&htim3, TIM_CHANNEL_2, 7.5f);
    osMutexRelease(servoMutex);
	// 使用全局传感器数据获取当前偏航角
// 设置巡航参数（X/Z保持不变）
    cruise_control.state = CRUISE_TURNING;
    cruise_control.direction = direction;
	cruise_control.step = 0;
    cruise_control.phase_start_time = HAL_GetTick();
    cruise_control.active = 1;
	printf("Cruise start! Direction: %s\n", 
          direction == CRUISE_LEFT ? "Left" : "Right");
    
    osMutexRelease(cruiseMutex);
}
void Cruise_Stop(void)
{
	osMutexAcquire(cruiseMutex, osWaitForever);
	if (cruise_control.active) {
        cruise_control.active = 0;
        cruise_control.state = CRUISE_IDLE;		// 重置舵机状态
		
        osMutexAcquire(servoMutex, osWaitForever);
        turn_direction = DIRECTION_STOP;
        current_state_x = PWM_STATE_DUTY1;
        last_jiao_switch_time = HAL_GetTick();
        osMutexRelease(servoMutex);
		Servo_Stop();
        // 设置舵机中立位
        set_pwm(&htim3, TIM_CHANNEL_1, 7.5f);
        set_pwm(&htim3, TIM_CHANNEL_2, 7.5f);
        set_pwm(&htim3, TIM_CHANNEL_3, 7.5f);
    }
	osMutexRelease(cruiseMutex);
    // 停止转弯，但保持前进
//    turn_direction = DIRECTION_FORWARD;
    printf("Cruise Stop\n");
}
void Motor_Init(void)
{
    // 初始化GPIO
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // 使能GPIOA时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    // 配置PA12 (使能端) 和 PA11 (方向控制) 为输出
    GPIO_InitStruct.Pin = MOTOR_LUO1_PIN1 | MOTOR_LUO1_PIN2 | 
                          MOTOR_LUO2_PIN1 | MOTOR_LUO2_PIN2;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // 初始状态：禁用电机
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO1_PIN1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO1_PIN2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO2_PIN1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO2_PIN2, GPIO_PIN_RESET);

//    // 设置默认方向 (向内，方向端低电平)
//    HAL_GPIO_WritePin(MOTOR_DIRECTION_PORT, MOTOR_DIRECTION_PIN, GPIO_PIN_RESET);
    // 停止TIM2 PWM输出
//    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
//	
//    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    printf("Motor drive initialization\n");
}
// 电机控制函数
void ControlPropellers(PropellerState state)
{
	propeller_state = state;
    uint32_t current_time = HAL_GetTick();
    
    switch(state) {
        case PROPELLER_FORWARD:  // 正转
            // 设置方向为正转 (PA12高电平)
			// 螺旋桨1正转：PIN1高，PIN2低
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO1_PIN1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO1_PIN2, GPIO_PIN_RESET);
            
            // 螺旋桨2反转：PIN1低，PIN2高
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO2_PIN1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO2_PIN2, GPIO_PIN_SET);
//		    set_pwm(&htim3, TIM_CHANNEL_1, 9.5f);
//		    set_pwm(&htim3, TIM_CHANNEL_2, 5.5f);
            break;
		
        case PROPELLER_REVERSE:  // 反转
 // 螺旋桨1反转：PIN1低，PIN2高
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO1_PIN1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO1_PIN2, GPIO_PIN_SET);
            
            // 螺旋桨2正转：PIN1高，PIN2低
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO2_PIN1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO2_PIN2, GPIO_PIN_RESET);
            break;
//			set_pwm(&htim3, TIM_CHANNEL_1, 5.5f);
//		    set_pwm(&htim3, TIM_CHANNEL_2, 9.5f);		
        case PROPELLER_STOP:  // 停止
            // 禁用电机 (PA12低电平)
		default:
            // 所有引脚低电平
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO1_PIN1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO1_PIN2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO2_PIN1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO2_PIN2, GPIO_PIN_RESET);
//			set_pwm(&htim3, TIM_CHANNEL_1, 7.5f);
//		    set_pwm(&htim3, TIM_CHANNEL_2, 7.5f);		
            break;
    }
}
// 新增螺旋桨控制函数（在任务中调用）
void UpdatePropellers(void) 
	{
    switch(propeller_state) {
        case PROPELLER_FORWARD:
//            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO1_PIN1, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO1_PIN2, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO2_PIN1, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO2_PIN2, GPIO_PIN_SET);
		    HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO1_PIN1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO1_PIN2, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO2_PIN1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO2_PIN2, GPIO_PIN_RESET);

            set_pwm(&htim3, TIM_CHANNEL_1, 9.5f);
            set_pwm(&htim3, TIM_CHANNEL_2, 5.5f);
            break;
            
        case PROPELLER_REVERSE:
//            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO1_PIN1, GPIO_PIN_RESET);
//            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO1_PIN2, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO2_PIN1, GPIO_PIN_SET);
//            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO2_PIN2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO1_PIN1, GPIO_PIN_SET);
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO1_PIN2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO2_PIN1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO2_PIN2, GPIO_PIN_SET);

            set_pwm(&htim3, TIM_CHANNEL_1, 5.5f);
            set_pwm(&htim3, TIM_CHANNEL_2, 9.5f);
            break;
            
        case PROPELLER_STOP:
        default:
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO1_PIN1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO1_PIN2, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO2_PIN1, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_LUO2_PIN2, GPIO_PIN_RESET);
            set_pwm(&htim3, TIM_CHANNEL_1, 7.5f);
            set_pwm(&htim3, TIM_CHANNEL_2, 7.5f);
            break;
    }
}
