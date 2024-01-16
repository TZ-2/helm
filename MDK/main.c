#include "stm32f4xx.h"                  // Device header
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"
#include "rabbit.h"


#define START_TASK_PRIO 10
#define START_STK_SIZE  128
static TaskHandle_t StartTask_Handler;	
void START_TASK(void *pvParameters);

#define CHASSIS_TASK_PRIO 20
#define CHASSIS_STK_SIZE  128
static TaskHandle_t ChassisTask_Handler;
void CHASSIS_TASK(void *pvParameters);

int Flag_right;
int Flag_init;

int main()
{
	system_init();       //≥ı ºªØ
	
	xTaskCreate((TaskFunction_t)START_TASK,
					  (const char*   )"START_TASK",
						(uint16_t      )START_STK_SIZE,
						(void*         )NULL,
						(UBaseType_t   )START_TASK_PRIO,
						(TaskHandle_t* )&StartTask_Handler);
	vTaskStartScheduler();	
						
	while(1);
}

void START_TASK(void *pvParameters)
{
	taskENTER_CRITICAL();
	Flag_right = 0;
	Flag_init = 0;
	xTaskCreate((TaskFunction_t)CHASSIS_TASK,
						(const char*   )"CHASSIS_TASK",
						(uint16_t      )CHASSIS_STK_SIZE,
						(void*         )NULL,
						(UBaseType_t   )CHASSIS_TASK_PRIO,
						(TaskHandle_t* )&ChassisTask_Handler);
	vTaskDelete(StartTask_Handler);
	taskEXIT_CRITICAL();
}

