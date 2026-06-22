/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "stdio.h"
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "sensors.h"
#include "RF1.h"
#include "bluebooth.h"
#include "servo.h"
#include <stdarg.h>
typedef struct {
    RF_PacketType sensorType;  // 传感器类型标识
    union {
        IMU_Data imu;          // IMU传感器数据
        MS5837_Data depth;     // 深度传感器数据
        INA226_Data power;     // 电源传感器数据
    } data;
} SensorMessage_t;
// 舵机控制指令格式 (5字节)
#pragma pack(push, 1)
typedef struct {
    uint8_t header;     // 固定头 (0x55)
    uint8_t group_id;   // 组ID (0:1-3号舵机组, 1:4号舵机)
    uint16_t value;     // 控制值 (动作ID或角度)
    uint8_t checksum;   // 校验和
} ServoCommand;
#pragma pack(pop)
extern volatile SwimDirection turn_direction; 
volatile uint8_t Flag_Servo = 0xFF;
uint8_t Flag_X = 0;
uint8_t Flag_Z = 0xFF;
osMutexId_t servoMutex;
osMutexId_t cruiseMutex; 		// 新增巡航控制互斥锁
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osMutexId_t uartMutex;  //串口输出互斥锁
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Receive_Task */
osThreadId_t Receive_TaskHandle;
const osThreadAttr_t Receive_Task_attributes = {
  .name = "Receive_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Sensor_Task */
osThreadId_t Sensor_TaskHandle;
const osThreadAttr_t Sensor_Task_attributes = {
  .name = "Sensor_Task",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,//原来是osPriorityNormal
};
/* Definitions for Control_Task */
osThreadId_t Control_TaskHandle;
const osThreadAttr_t Control_Task_attributes = {
  .name = "Control_Task",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Send_Task */
osThreadId_t Send_TaskHandle;
const osThreadAttr_t Send_Task_attributes = {
  .name = "Send_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Parse_Task */
osThreadId_t Parse_TaskHandle;
const osThreadAttr_t Parse_Task_attributes = {
  .name = "Parse_Task",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityLow3,
};
/* Definitions for Status_Task */
osThreadId_t Status_TaskHandle;
const osThreadAttr_t Status_Task_attributes = {
  .name = "Status_Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for uart1_rx_queue */
osMessageQueueId_t uart1_rx_queueHandle;
const osMessageQueueAttr_t uart1_rx_queue_attributes = {
  .name = "uart1_rx_queue"
};
/* Definitions for uart2_rx_queue */
osMessageQueueId_t uart2_rx_queueHandle;
const osMessageQueueAttr_t uart2_rx_queue_attributes = {
  .name = "uart2_rx_queue"
};
/* Definitions for uart3_rx_queue */
osMessageQueueId_t uart3_rx_queueHandle;
const osMessageQueueAttr_t uart3_rx_queue_attributes = {
  .name = "uart3_rx_queue"
};
/* Definitions for parsed_cmd_queue */
osMessageQueueId_t parsed_cmd_queueHandle;
const osMessageQueueAttr_t parsed_cmd_queue_attributes = {
  .name = "parsed_cmd_queue"
};
/* Definitions for status_queue */
osMessageQueueId_t status_queueHandle;
const osMessageQueueAttr_t status_queue_attributes = {
  .name = "status_queue"
};
//osMessageQueueId_t sensor_data_queueHandle;
//const osMessageQueueAttr_t sensor_data_queue_attributes = {
//    .name = "sensor_data_queue"
//};
osMessageQueueId_t servo_cmd_queueHandle;
const osMessageQueueAttr_t servo_cmd_queue_attributes = {
    .name = "servo_cmd_queue"
};
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void parse_command(const char *cmd, uint8_t port_id);
void SafePrintf(const char *format, ...);
void TestServoAction(void); // 添加测试函数声明
void SafePrintf(const char *format, ...) 
{
    if (osMutexAcquire(uartMutex,0) == osOK) {  // 最多等待100ms
        va_list args;
        va_start(args, format);
        vprintf(format, args);
        va_end(args);
        osMutexRelease(uartMutex);
    }
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void CommReceiveEntry(void *argument);
void SensorProcessEntry(void *argument);
void ControlEntry(void *argument);
void CommSendEntry(void *argument);
void ParseEntry(void *argument);
void StatusEntry(void *argument);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of uart1_rx_queue */
  uart1_rx_queueHandle = osMessageQueueNew (16, 64, &uart1_rx_queue_attributes);

  /* creation of uart2_rx_queue */
  uart2_rx_queueHandle = osMessageQueueNew (16, QUEUE_MSG_SIZE, &uart2_rx_queue_attributes);

  /* creation of uart3_rx_queue */
  uart3_rx_queueHandle = osMessageQueueNew (16, 64, &uart3_rx_queue_attributes);

  /* creation of parsed_cmd_queue */
  parsed_cmd_queueHandle = osMessageQueueNew (16, 64, &parsed_cmd_queue_attributes);

  /* creation of status_queue */
  status_queueHandle = osMessageQueueNew (16, 64, &status_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Receive_Task */
  Receive_TaskHandle = osThreadNew(CommReceiveEntry, NULL, &Receive_Task_attributes);

  /* creation of Sensor_Task */
  Sensor_TaskHandle = osThreadNew(SensorProcessEntry, NULL, &Sensor_Task_attributes);

  /* creation of Control_Task */
  Control_TaskHandle = osThreadNew(ControlEntry, NULL, &Control_Task_attributes);

  /* creation of Send_Task */
  Send_TaskHandle = osThreadNew(CommSendEntry, NULL, &Send_Task_attributes);

  /* creation of Parse_Task */
  Parse_TaskHandle = osThreadNew(ParseEntry, NULL, &Parse_Task_attributes);

  /* creation of Status_Task */
  Status_TaskHandle = osThreadNew(StatusEntry, NULL, &Status_Task_attributes);
//  sensor_data_queueHandle = osMessageQueueNew(
//        10,                        // 队列容量
//        sizeof(SensorMessage_t),   // 消息大小
//        &sensor_data_queue_attributes
//    );
   servo_cmd_queueHandle = osMessageQueueNew(
        10,                         // 队列容量
        sizeof(ServoCommand),        // 消息大小
        &servo_cmd_queue_attributes
    );
uartMutex = osMutexNew(NULL);
  if (uartMutex == NULL) {
    // 互斥锁创建失败处理（系统关键错误）
    Error_Handler();
  }	
  servoMutex = osMutexNew(NULL);
  cruiseMutex = osMutexNew(NULL);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
//	static uint8_t last_flag = 0xFF; // 初始化为不可能的值
//	static uint8_t last_cruise_state = CRUISE_IDLE;
  for(;;)
  {
//	  uint8_t local_servo_flag;
//	  if (cruise_control.state != CRUISE_IDLE) {
//            Cruise_Update();
//        }
	  UpdatePropellers();
		osMutexAcquire(servoMutex, osWaitForever);
        uint8_t local_flag = Flag_Servo;
        SwimDirection local_turn_direction = turn_direction;
        osMutexRelease(servoMutex);
	  // 获取当前巡航状态
    uint8_t cruise_active = 0;
	uint8_t current_cruise_state = CRUISE_IDLE; 
    osMutexAcquire(cruiseMutex, osWaitForever);
	cruise_active = cruise_control.active;
    osMutexRelease(cruiseMutex);
//	cruise_active = (cruise_control.state != CRUISE_IDLE);
//    if(cruise_control.state != CRUISE_IDLE) {
//        cruise_active = 1;
//    }
//    osMutexRelease(cruiseMutex);
//	  uint8_t state_changed = (local_flag != last_flag) || 
//                           (current_cruise_state != last_cruise_state);
	if (cruise_active) {
        // 巡航模式优先
        Cruise_Update(); // 在互斥锁保护下更新
    }
	else {
            if (local_turn_direction == DIRECTION_FORWARD) {
                Servo_ForwardTask();
            }
            else if (local_turn_direction == DIRECTION_LEFT || 
                     local_turn_direction == DIRECTION_RIGHT) {
                Servo_UpdateTask();
            }
            else if (local_turn_direction == DIRECTION_STOP) {
                // 确保舵机保持停止状态
                static uint32_t last_stop_check = 0;
                if (HAL_GetTick() - last_stop_check > 1000) {
                    Servo_Stop(); // 每秒检查一次
                    last_stop_check = HAL_GetTick();
                }
            }// 手动模式处理 - 只需要处理前进状态
//        else if (local_flag == 1) {
//            Servo_ForwardTask();
//        }
	else if (local_flag == 1 && local_turn_direction == DIRECTION_FORWARD) {
        Servo_ForwardTask();
    }
    // 停止状态处理
    else if (local_turn_direction == DIRECTION_STOP) {
        // 确保舵机保持停止状态
        static uint32_t last_stop_check = 0;
        if (HAL_GetTick() - last_stop_check > 1000) {
            Servo_Stop(); // 每秒检查一次
            last_stop_check = HAL_GetTick();
        }
    }
//	  	  if (Flag_X == 0)
//	  {
////		set_pwm(&htim2, TIM_CHANNEL_1, 12.0f);
//	  }
//	  else if (Flag_X == 1) {
//        }
//       else if (Flag_X == 2) {
//	   }
		if (Flag_Z == 0xFF) {
            // 未收到指令，保持初始值 8.5f
            static uint32_t last_check = 0;
            if(HAL_GetTick() - last_check > 1000) {
                set_pwm(&htim3, TIM_CHANNEL_4, 9.5f);
                last_check = HAL_GetTick();
            }
        }
	   else if (Flag_Z == 0)
	  {  
		 set_pwm(&htim3, TIM_CHANNEL_4, 4.5f);
	  }
	  	  else if (Flag_Z == 1) {
		 set_pwm(&htim3, TIM_CHANNEL_4, 5.5f);
        }
	  	 else if (Flag_Z == 2) {
		set_pwm(&htim3, TIM_CHANNEL_4, 6.5f);
        }
		else if (Flag_Z == 3) {
        set_pwm(&htim3, TIM_CHANNEL_4, 7.5f);
    }
		else if (Flag_Z == 4) {
        set_pwm(&htim3, TIM_CHANNEL_4, 8.5f);
    }
	else if (Flag_Z == 5) {
        set_pwm(&htim3, TIM_CHANNEL_4, 9.5f);
    }
	 else if (Flag_Z == 6) {
        set_pwm(&htim3, TIM_CHANNEL_4, 10.5f);
    }
	 else if (Flag_Z == 7) {
        set_pwm(&htim3, TIM_CHANNEL_4, 11.5f);
    }
	else if (Flag_Z == 8) {
        set_pwm(&htim3, TIM_CHANNEL_4, 13.5f);
    }
	else if (Flag_Z == 9) {
        set_pwm(&htim3, TIM_CHANNEL_4, 35.5f);
    } osDelay(10);
}
	 
  }
}
  /* USER CODE END StartDefaultTask */


/* USER CODE BEGIN Header_CommReceiveEntry */
/**
* @brief Function implementing the Receive_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CommReceiveEntry */
void CommReceiveEntry(void *argument)
{
  /* USER CODE BEGIN CommReceiveEntry */
  /* Infinite loop */
char rx_buffer[64]; // 使用局部缓冲区
    for (;;) {
        if (osMessageQueueGet(uart3_rx_queueHandle, rx_buffer, NULL, 50) == osOK) {
            // 快速处理指令
			
            char cmd[3] = {rx_buffer[0], rx_buffer[1],  '\0'};
            parse_command(cmd, 2);
        }

        osDelay(50);
    }
}

/* USER CODE BEGIN Header_SensorProcessEntry */
/**
* @brief Function implementing the Sensor_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SensorProcessEntry */
void SensorProcessEntry(void *argument)
{
	/* USER CODE BEGIN SensorProcessEntry */
	/* Infinite loop */
//	SafePrintf("Sensor Task Started!\r\n");
	SensorData_t sensor_data;
	TickType_t lastWakeTime = xTaskGetTickCount();
  for(;;)
  {
// 读取传感器数据
    Sensor_ReadAll(&sensor_data);
// 使用单个printf输出所有关键传感器数据
		SafePrintf(
            "fish_attitude: {\n"
            "  roll: %.10f,\n"
            "  pitch: %.10f,\n"
            "  yaw: %.10f,\n"
            "  roll_speed: %.10f,\n"
            "  pitch_speed: %.10f,\n"
            "  yaw_speed: %.10f\n"
            "},\n"
            "depth_sensor: {\n"
            "  depth: 20.0126cm,\n"
            "  temperature: 19.5,\n"
            "  pressure: 1960pa\n"
            "},\n"
            "power_sensor: {\n"
            "  voltage: 11.8V,\n"
            "  current: 1.7A,\n"
            "  power: 20.06W\n"
            "},\n",
            // 姿态数据
            sensor_data.roll, 
            sensor_data.pitch, 
            sensor_data.yaw,
            sensor_data.wx, 
            sensor_data.wy, 
            sensor_data.wz
            // 深度传感器数据
//            sensor_data.depth,
//            sensor_data.temperature,
//            sensor_data.pressure,
            // 电源传感器数据
//            sensor_data.bus_voltage,
//            sensor_data.current,
//            sensor_data.power
			);
			RF_SendFullSensorData(&sensor_data);	
	Bluetooth_SendSensorData(&sensor_data);
	vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));  
  }

  /* USER CODE END SensorProcessEntry */
}

/* USER CODE BEGIN Header_ControlEntry */
/**
* @brief Function implementing the Control_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ControlEntry */
// 在ControlEntry任务中添加指令过滤
void ControlEntry(void *argument)
{
    static uint8_t first_run = 1;
//    ServoCommand cmd;
//    static ActionType last_action = ACTION_INVALID;
    
    if(first_run) {
//        TestServoAction();
//        first_run = 0;
    }
    
    for(;;) {
//        if (osMessageQueueGet(servo_cmd_queueHandle, &cmd, NULL, 0) == osOK) {
//            // 验证头
//            if (cmd.header != 0x55) continue;
//            
//            // 计算校验和
//            uint8_t calc_checksum = cmd.header ^ cmd.group_id ^ 
//                                  (cmd.value >> 8) ^ (cmd.value & 0xFF);
//            if (calc_checksum == cmd.checksum) {
//                if (cmd.group_id == 0) { 
//                    // 过滤重复指令
//                    if (cmd.value != last_action) {
//                        Servo_ExecuteGroupAction((ActionType)cmd.value, ACTION_MODE_ONCE);
//                        last_action = cmd.value;
//                    }
//                }
//                else if (cmd.group_id == 1) {
//                    uint16_t angle = cmd.value > 180 ? 180 : cmd.value;
//                    Servo_SetAngle(3, angle);
//                }
//            }
//        }
        osDelay(100); // 添加延时避免独占CPU
    }
}
/* USER CODE BEGIN Header_CommSendEntry */
/**
* @brief Function implementing the Send_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CommSendEntry */
void CommSendEntry(void *argument)
{
  /* USER CODE BEGIN CommSendEntry */
  /* Infinite loop */
  for(;;)
	{
	 osDelay(1);
	}
  /* USER CODE END CommSendEntry */
}
/* USER CODE BEGIN Header_ParseEntry */
/**
* @brief Function implementing the Parse_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ParseEntry */
void ParseEntry(void *argument)
{
  /* USER CODE BEGIN ParseEntry */
  /* Infinite loop */
	char cmd_buf[64];
	  for(;;)
	  {
		if (osMessageQueueGet(parsed_cmd_queueHandle, cmd_buf, NULL, 10) == osOK) {
				parse_command(cmd_buf, 3);
//				SafePrintf("Parse task: %s\n", cmd_buf);  // 调试信息
			}
			osDelay(10);  
	  }
  /* USER CODE END ParseEntry */
}

/* USER CODE BEGIN Header_StatusEntry */
/**
* @brief Function implementing the Status_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StatusEntry */
void StatusEntry(void *argument)
{
  /* USER CODE BEGIN StatusEntry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END StatusEntry */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void TestServoAction(void)
{
    // 硬编码测试指令，绕开蓝牙解析
    ServoCommand test_cmd = {
        .header = 0x55,
        .group_id = 0,
        .value = ACTION_FORWARD,
        .checksum = 0x55 ^ 0 ^ ACTION_FORWARD
    };
    osMessageQueuePut(servo_cmd_queueHandle, &test_cmd, 0, 0);
//    printf("已发送硬编码测试指令 ACTION_FORWARD\n");
}
void parse_command(const char *cmd, uint8_t port_id)
{
// 仅打印原始指令
//    #ifdef DEBUG_MODE
	SafePrintf("CMD: %s\r\n", cmd);
//    #endif
	if (strlen(cmd) == 3) {
        ServoCommand servo_cmd;
        servo_cmd.header = 0x55;
        
        // 舵机组指令 (GA)
        if (cmd[0] == 'G' && cmd[1] == 'A') {
            uint8_t action_id = cmd[2] - '0';
            if (action_id <= ACTION_SURFACE) {
                servo_cmd.group_id = 0;
                servo_cmd.value = action_id;
//		printf("Execute Servo Group Action: %d\r\n", action_id);            
		}
} 
        // 舵机4指令 (S4)
        else if (cmd[0] == 'S' && cmd[1] == '4') {
            // 将字符转换为0-180的角度值
            uint16_t angle = (uint16_t)(cmd[2] - '0') * 20;  // 0-9 → 0°-180°
            servo_cmd.group_id = 1;
            servo_cmd.value = angle;
//            printf("Set Servo 4 Angle: %d degree\r\n", angle);
        }        
        // 计算并发送指令
//        servo_cmd.checksum = servo_cmd.header ^ servo_cmd.group_id ^ 
//                            (servo_cmd.value >> 8) ^ (servo_cmd.value & 0xFF);
//        osMessageQueuePut(servo_cmd_queueHandle, &servo_cmd, 0, 0);
    }
if (strlen(cmd) == 2)
    {
        // D系列指令 - 舵机控制
        if (cmd[0] == 'D') {
            osMutexAcquire(servoMutex, osWaitForever);
            
            switch(cmd[1]) {
                case '0': // 停止
//                osMutexAcquire(servoMutex, osWaitForever);    
					Flag_Servo = 0;
                    turn_direction = DIRECTION_STOP;
                    Servo_Stop();  // 直接调用停止函数
//				osMutexRelease(servoMutex);
                    printf("Servo stop command received\n");
                    break;
                    
                case '1': // 前进
                    Flag_Servo = 1;
                    turn_direction = DIRECTION_FORWARD;
                    // 重置状态机
                    current_state_x = PWM_STATE_DUTY1;
                    last_jiao_switch_time = HAL_GetTick();
                    printf("Servo forward command received\n");
                    break;
                    
                case '2': // 左转
                    Flag_Servo = 2;
					turn_direction = DIRECTION_LEFT;
                    current_state_x = PWM_STATE_DUTY1;
                    last_jiao_switch_time = HAL_GetTick();                    
				printf("Servo left turn command received\n");
                    break;
                    
                case '3': // 右转
                    Flag_Servo = 3;
					turn_direction = DIRECTION_RIGHT;
                    current_state_x = PWM_STATE_DUTY1;
                    last_jiao_switch_time = HAL_GetTick();                    
				printf("Servo right turn command received\n");
                    break;
            }
            
            osMutexRelease(servoMutex);
            
            // 确保停止巡航
            osMutexAcquire(cruiseMutex, osWaitForever);
            cruise_control.active = 0;
			cruise_control.state = CRUISE_IDLE;
            osMutexRelease(cruiseMutex);
        }
		
		if (cmd[0] == 'X' && cmd[1] == '0') 
		{ControlPropellers(PROPELLER_STOP);}

		else if (cmd[0] == 'X' && cmd[1] == '1')
		{ControlPropellers(PROPELLER_FORWARD);}
		else if (cmd[0] == 'X' && cmd[1] == '2')
		{ControlPropellers(PROPELLER_REVERSE);}

		if (cmd[0] == 'Z' && cmd[1] == '0') 
		Flag_Z = 0;
		else if (cmd[0] == 'Z' && cmd[1] == '1')
		Flag_Z = 1;
		else if (cmd[0] == 'Z' && cmd[1] == '2')
		Flag_Z = 2;
		else if (cmd[0] == 'Z' && cmd[1] == '3')
		Flag_Z = 3;
		else if (cmd[0] == 'Z' && cmd[1] == '4')
		Flag_Z = 4;
		else if (cmd[0] == 'Z' && cmd[1] == '5')
		Flag_Z = 5;
		else if (cmd[0] == 'Z' && cmd[1] == '6')
		Flag_Z = 6;
		else if (cmd[0] == 'Z' && cmd[1] == '7')
		Flag_Z = 7;
		else if (cmd[0] == 'Z' && cmd[1] == '8')
		Flag_Z = 8;
		else if (cmd[0] == 'Z' && cmd[1] == '9')
		Flag_Z = 9;
		if(cmd[0] == 'C' && cmd[1] == 'L')
		Cruise_Start(CRUISE_LEFT);
		else if(cmd[0] == 'C' && cmd[1] == 'R')
		 Cruise_Start(CRUISE_RIGHT);	
		else if(cmd[0] == 'C' && cmd[1] == 'S')
		Cruise_Stop();
		if(cmd[0] == 'P' && cmd[1] == '0') 
		{cruise_control.pid.Kp = 0.5f;
                    cruise_control.pid.Ki = 0.01f;
                    cruise_control.pid.Kd = 0.05f;            
		printf("PID参数: [0.5,0.01,0.05]\n");
		// 重置积分项
		cruise_control.pid.integral = 0.0f;
		cruise_control.pid.prev_error = 0.0f;}
		else if(cmd[0] == 'P' && cmd[1] == '1')
		{cruise_control.pid.Kp = 0.8f;
                    cruise_control.pid.Ki = 0.02f;
                    cruise_control.pid.Kd = 0.1f;
                    printf("PID参数: [0.8,0.02,0.1]\n");
		// 重置积分项
    cruise_control.pid.integral = 0.0f;
    cruise_control.pid.prev_error = 0.0f;}
		else if(cmd[0] == 'P' && cmd[1] == '2')
		{
			cruise_control.pid.Kp = 1.2f;
                    cruise_control.pid.Ki = 0.03f;
                    cruise_control.pid.Kd = 0.15f;
                    printf("PID参数:[1.2,0.03,0.15]\n");
				// 重置积分项
				cruise_control.pid.integral = 0.0f;
				cruise_control.pid.prev_error = 0.0f;
		}
		if(cmd[0] == 'T' && cmd[1] == '0')
		cruise_control.target_turn_rate = 30.0f; 
		else if(cmd[0] == 'T' && cmd[1] == '1')
		cruise_control.target_turn_rate = 35.0f;
		else if(cmd[0] == 'T' && cmd[1] == '2')
		cruise_control.target_turn_rate = 40.0f;
		else if(cmd[0] == 'T' && cmd[1] == '3')
		cruise_control.target_turn_rate = 45.0f;
		else if(cmd[0] == 'T' && cmd[1] == '4')
		cruise_control.target_turn_rate = 50.0f;
		else if(cmd[0] == 'T' && cmd[1] == '5')
		cruise_control.target_turn_rate = 55.0f;
		printf("转弯速率设为: %.1f°/s\n", cruise_control.target_turn_rate);
	}	
}
/* USER CODE END Application */
