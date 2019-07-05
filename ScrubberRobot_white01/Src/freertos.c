/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "soft.h"
#include "iwdg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
u8 stop_run=0;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
osThreadId myTask03Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

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

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityIdle, 0, 128);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* definition and creation of myTask03 */
  osThreadDef(myTask03, StartTask03, osPriorityIdle, 0, 128);
  myTask03Handle = osThreadCreate(osThread(myTask03), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
	static u32 left_electrode=0;
	static u32 right_electrode=0;
	
    /* Infinite loop */
    for(;;)
    {
        osDelay(10);
		//��⵽��缫Ƭ���뵽λ
		if (((account_current(3)<2000) && (account_current(4)<2000))	//�Ӵ��缫��⣬�����缫ͬʱ�Ӵ���֪ͨ����ֹͣ
			|| (left_electrode>50) || (right_electrode>50))		//�����缫�Ӵ�����500ms��֪ͨ����ֹͣ
        {
					HAL_GPIO_WritePin (GPIOE, GPIO_PIN_11, GPIO_PIN_SET);  	//�ṩ5v�źţ�֪ͨ���ػ�ֹͣ�ƶ�����ʼ���
								
					HAL_GPIO_WritePin (GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);					//MOTOR3_DIR �� �͵�ƽ��Ч
					HAL_GPIO_WritePin (GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);					//MOTOR3_DIR ��
					HAL_GPIO_WritePin (GPIOE, GPIO_PIN_14, GPIO_PIN_SET);					//MOTOR3_DIR ��
					
					osDelay(10000);
					if ((account_current(3)<2000) && (account_current(4)<2000))
					{
						HAL_GPIO_WritePin (GPIOD, GPIO_PIN_5, GPIO_PIN_SET);					//M2 POWER    changing on
					}
			
        }
		
		if ((account_current(3)>2000) && (account_current(4)>2000))
		{
			HAL_GPIO_WritePin (GPIOE, GPIO_PIN_11, GPIO_PIN_RESET);  	//�ṩ0v�źţ�֪ͨ���ػ����ڳ��״̬
			
			left_electrode=0;
			right_electrode=0;
			
			HAL_GPIO_WritePin (GPIOB, GPIO_PIN_3, GPIO_PIN_SET);					//MOTOR3_DIR �� �͵�ƽ��Ч
			HAL_GPIO_WritePin (GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);					//MOTOR3_DIR ��
			HAL_GPIO_WritePin (GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);					//MOTOR3_DIR ��
			
			HAL_GPIO_WritePin (GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);					//M2 POWER    changing OFF
		}
		
		if (account_current(3)<2000)
		{
			left_electrode++;
		}
		if (account_current(4)<2000)
		{
			right_electrode++;
		}
		
		if (account_current(2)<2000)		 //��ص�ѹ���㣬С��3.5v
		{
			HAL_GPIO_WritePin (GPIOE, GPIO_PIN_11, GPIO_PIN_SET);  					//�ṩ5v�źţ�֪ͨ���ػ�ֹͣ�ƶ�
			HAL_GPIO_WritePin (GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); 					//�رճ�ˮ��
			HAL_GPIO_WritePin (GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);					//FAN1 POWER OFF ��ŷ�1
			HAL_GPIO_WritePin (GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);					//FAN2 POWER OFF ��ŷ�2
			HAL_GPIO_WritePin (GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);					//MM POWER OFF ˢ�̵��
			osDelay(200);
			HAL_GPIO_WritePin (GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);					//M1 POWER OFF ��ˮ���
			osDelay(200);
			HAL_GPIO_WritePin (GPIOB, GPIO_PIN_13, GPIO_PIN_SET);					//LMOTOR_DIR �Ƹ˵����ת,�ߵ�ƽ��Ч
			HAL_GPIO_WritePin (GPIOD, GPIO_PIN_14, GPIO_PIN_SET);					//LMOTOR_PLS �Ƹ˵����ת �͵�ƽ��Ч
			osDelay(5000);
			HAL_GPIO_WritePin (GPIOB, GPIO_PIN_13, GPIO_PIN_SET);					//LMOTOR_DIR �Ƹ˵����ת,�ߵ�ƽ��Ч
			HAL_GPIO_WritePin (GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);					//LMOTOR_PLS �Ƹ˵����ת �͵�ƽ��Ч
			HAL_GPIO_WritePin (GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);					//M2 POWER OFF �رչ��ػ�
			
			stop_run=1;
			while(1)
			{
				HAL_GPIO_WritePin (GPIOB, GPIO_PIN_3, GPIO_PIN_SET);					//MOTOR3_DIR �� �͵�ƽ��Ч
				HAL_GPIO_WritePin (GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);					//MOTOR3_DIR ��
				HAL_GPIO_WritePin (GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);					//MOTOR3_DIR ��
				osDelay(500);
				HAL_GPIO_WritePin (GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);					//MOTOR3_DIR �� �͵�ƽ��Ч
				HAL_GPIO_WritePin (GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);					//MOTOR3_DIR ��
				HAL_GPIO_WritePin (GPIOE, GPIO_PIN_14, GPIO_PIN_SET);					//MOTOR3_DIR ��
				osDelay(500);	
			}
		}
		
		
    }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
    static u32 clear_stop_time=0,move_base_ring_old=0;
    static uint8_t flagworking = 0;
	static u32 chouwu=0;
    /* Infinite loop */
    for(;;)
    {
        osDelay(200);
		while(stop_run)		//ͣ��
		{
			osDelay(1000);
		}
        if((flagworking == 0) && (s_and.error_code == 0))
        {
            if(move_base_ring_old < move_base_ring) //move base starts moving
            {
                if((move_base_ring - move_base_ring_old) >= 5) //at least 20 pluses per 200ms
                {
					
					HAL_GPIO_WritePin (GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);					//MOTOR3_DIR �� �͵�ƽ��Ч
					HAL_GPIO_WritePin (GPIOC, GPIO_PIN_7, GPIO_PIN_SET);					//MOTOR3_DIR ��
					HAL_GPIO_WritePin (GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);					//MOTOR3_DIR ��
					
                    HAL_GPIO_WritePin (GPIOE, GPIO_PIN_12, GPIO_PIN_SET);					//FAN1 POWER OFF ��ŷ�1
					HAL_GPIO_WritePin (GPIOE, GPIO_PIN_15, GPIO_PIN_SET);						//FAN2 POWER OFF ��ŷ�2
					HAL_GPIO_WritePin (GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);					//LMOTOR_DIR �Ƹ˵����ת,�ߵ�ƽ��Ч
					HAL_GPIO_WritePin (GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);					//LMOTOR_PLS �Ƹ˵����ת �͵�ƽ��Ч
					osDelay(3000);
                    HAL_GPIO_WritePin (GPIOD, GPIO_PIN_9, GPIO_PIN_SET);					//MM POWER OFF ˢ�̵��
					osDelay(200);	
					HAL_GPIO_WritePin (GPIOD, GPIO_PIN_6, GPIO_PIN_SET);					//M1 POWER OFF ��ˮ���

                    clear_stop_time = 30;
                    flagworking = 1;
                }
            }
        }
        else//(flagworking != 0)
        {
			if((move_base_ring - move_base_ring_old) < 2) //no more than 20 pluses per 200ms
				clear_stop_time--; 
			else 
				clear_stop_time = 30;

            if(clear_stop_time <= 0) //error
            {
				HAL_GPIO_WritePin (GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);					//FAN1 POWER OFF ��ŷ�1
				HAL_GPIO_WritePin (GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);					//FAN2 POWER OFF ��ŷ�2
				HAL_GPIO_WritePin (GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);					//MM POWER OFF ˢ�̵��
				osDelay(200);
				HAL_GPIO_WritePin (GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);					//M1 POWER OFF ��ˮ���
				osDelay(200);
                move_base_ring = 0;
                move_base_ring_old = 0;
                clear_stop_time = 0;
                flagworking = 0;
				HAL_GPIO_WritePin (GPIOB, GPIO_PIN_13, GPIO_PIN_SET);					//LMOTOR_DIR �Ƹ˵����ת,�ߵ�ƽ��Ч
				HAL_GPIO_WritePin (GPIOD, GPIO_PIN_14, GPIO_PIN_SET);					//LMOTOR_PLS �Ƹ˵����ת �͵�ƽ��Ч
				osDelay(5000);
				HAL_GPIO_WritePin (GPIOB, GPIO_PIN_13, GPIO_PIN_SET);					//LMOTOR_DIR �Ƹ˵����ת,�ߵ�ƽ��Ч
				HAL_GPIO_WritePin (GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);					//LMOTOR_PLS �Ƹ˵����ת �͵�ƽ��Ч
				
				
				HAL_GPIO_WritePin (GPIOB, GPIO_PIN_3, GPIO_PIN_SET);					//MOTOR3_DIR �� �͵�ƽ��Ч
				HAL_GPIO_WritePin (GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);					//MOTOR3_DIR ��
				HAL_GPIO_WritePin (GPIOE, GPIO_PIN_14, GPIO_PIN_RESET);					//MOTOR3_DIR ��
            }
			

			if((move_base_ring - move_base_ring_old) < 2) 
			{
				HAL_GPIO_WritePin (GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);					//FAN1 POWER OFF ��ŷ�1
				HAL_GPIO_WritePin (GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);					//FAN2 POWER OFF ��ŷ�2
			}
			else if((move_base_ring - move_base_ring_old) < 50)	//����ˮ��
			{
				HAL_GPIO_WritePin (GPIOE, GPIO_PIN_12, GPIO_PIN_SET);					//FAN1 POWER OFF ��ŷ�1	
				HAL_GPIO_WritePin (GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);					//FAN2 POWER OFF ��ŷ�2
			}
			else	//����ˮ��
			{
				HAL_GPIO_WritePin (GPIOE, GPIO_PIN_12, GPIO_PIN_SET);					//FAN1 POWER OFF ��ŷ�1	
				HAL_GPIO_WritePin (GPIOE, GPIO_PIN_15, GPIO_PIN_SET);					//FAN2 POWER OFF ��ŷ�2
			}
        }//(flagworking == 1)
		

		if ((HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_SET)	//��ˮ�俪ʼ������ʼ��ˮ����,�ߵ�ƽ��Ч
        &&(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_RESET))
				{
				HAL_GPIO_WritePin (GPIOD, GPIO_PIN_3, GPIO_PIN_SET);  	//�򿪳�ˮ��ѹ��
        chouwu=300;
				}
		
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET)  //������ɣ��͵�ƽ��Ч
		{
			chouwu=300;
		}
		else if(chouwu>0)
		{
			chouwu--;
		}
		if(chouwu==0)  	//�رճ�ˮ��ѹ��
		{
			HAL_GPIO_WritePin (GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); 
		}
		
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == GPIO_PIN_SET)  //��ˮ�����ߵ�ƽ��Ч
		{
			//��ʾ����
			chouwu=0;
			HAL_GPIO_WritePin (GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);  	//�رճ�ˮ��ѹ��
			//ˮ��������
		}
		
        move_base_ring_old = move_base_ring;
    }//for(;;)
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
    /* Infinite loop */
    for(;;)
    {
        HAL_IWDG_Refresh(&hiwdg);
        osDelay(200);
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_6);   //RUN LED FLASH
//		 Alarm();
//		  if(s_and.error_code ) //error
//            {
//				HAL_GPIO_WritePin (GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);					//FAN2 POWER OFF ��ŷ�
//				HAL_GPIO_WritePin (GPIOE, GPIO_PIN_15, GPIO_PIN_RESET);					//FAN2 POWER OFF û��ʹ��
//				HAL_GPIO_WritePin (GPIOB, GPIO_PIN_13, GPIO_PIN_SET);					//LMOTOR_DIR �Ƹ˵����ת
//				HAL_GPIO_WritePin (GPIOD, GPIO_PIN_14, GPIO_PIN_SET);					//LMOTOR_PLS �Ƹ˵����ת���͵�ƽ��Ч
//				HAL_GPIO_WritePin (GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);					//MM POWER OFF ˢ�̵��
//				HAL_GPIO_WritePin (GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);					//RM POWER OFF ��ѹ��
//				HAL_GPIO_WritePin (GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);					//M1 POWER OFF ��ˮ���
//				HAL_GPIO_WritePin (GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);					//M2 POWER OFF û��ʹ��
//				HAL_GPIO_WritePin (GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);					//M3 POWER OFF ��ʾ��	
//            }
    }
  /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
