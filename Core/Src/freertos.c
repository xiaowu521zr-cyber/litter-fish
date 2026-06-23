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

/* Definitions for Sensor_Task */
osThreadId_t Sensor_TaskHandle;
const osThreadAttr_t Sensor_Task_attributes = {
  .name = "Sensor_Task",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityNormal,//原来是osPriorityNormal
};

/* Definitions for Parse_Task */
osThreadId_t Parse_TaskHandle;
const osThreadAttr_t Parse_Task_attributes = {
  .name = "Parse_Task",
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
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
osMessageQueueId_t servo_cmd_queueHandle;
const osMessageQueueAttr_t servo_cmd_queue_attributes = {
    .name = "servo_cmd_queue"
};
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void parse_command(const char *cmd, uint8_t port_id);
void SafePrintf(const char *format, ...);
void SafePrintf(const char *format, ...) 
{
    if (osMutexAcquire(uartMutex,100) == osOK) {  // 最多等待100ms
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
void CommSendEntry(void *argument);
//void ParseEntry(void *argument);
//void StatusEntry(void *argument);
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

  /* creation of Sensor_Task */
  Sensor_TaskHandle = osThreadNew(SensorProcessEntry, NULL, &Sensor_Task_attributes);

//  /* creation of Parse_Task */
//  Parse_TaskHandle = osThreadNew(ParseEntry, NULL, &Parse_Task_attributes);

//  /* creation of Status_Task */
//  Status_TaskHandle = osThreadNew(StatusEntry, NULL, &Status_Task_attributes);

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
  for(;;)
  {
		UpdatePropellers();
		osMutexAcquire(servoMutex, osWaitForever);
        uint8_t local_flag = Flag_Servo;
        SwimDirection local_turn_direction = turn_direction;
        osMutexRelease(servoMutex);
		// 获取当前巡航状态
		uint8_t cruise_active = 0;
		osMutexAcquire(cruiseMutex, osWaitForever);
		cruise_active = cruise_control.active;
		osMutexRelease(cruiseMutex);
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
            }
		}
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
  /* USER CODE END StartDefaultTask */


/* USER CODE BEGIN Header_CommReceiveEntry */

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
            "depth_sensor: {"
            "  depth: %.4fcm,"
            "  temperature: %.2f,"
            "  pressure: %.2fpa"
            "},\n"
            "power_sensor: {"
            "  voltage: %umV,"
            "  current: %umA,"
            "  power: %umW""},\n",
            // 姿态数据
            sensor_data.roll, 
            sensor_data.pitch, 
            sensor_data.yaw,
            sensor_data.wx, 
            sensor_data.wy, 
            sensor_data.wz,
            // 深度传感器数据
            sensor_data.depth,
            sensor_data.temperature,
            sensor_data.pressure,
          // 电源传感器数据
            sensor_data.bus_voltage,
            sensor_data.current,
            sensor_data.power
			);
	vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));  
  }

  /* USER CODE END SensorProcessEntry */
}

/* USER CODE BEGIN Header_CommSendEntry */

/* USER CODE END Header_CommSendEntry */
void parse_command(const char *cmd, uint8_t port_id)
{
// 仅打印原始指令

	SafePrintf("CMD: %s\r\n", cmd);

	if (strlen(cmd) == 2)
    {
        // D系列指令 - 舵机控制
        if (cmd[0] == 'D') {
            osMutexAcquire(servoMutex, osWaitForever);
            
            switch(cmd[1]) {
                case '0': // 停止
					Flag_Servo = 0;
                    turn_direction = DIRECTION_STOP;
                    Servo_Stop();  // 直接调用停止函数
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
