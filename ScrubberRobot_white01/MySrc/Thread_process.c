/**
  ******************************************************************************
  * @file    Thread_process.c
  * @author  刘振宇
  * @version V1.0
  * @date    2018.3.5
  * @brief   这个文件是程序的处理逻辑和保护
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* USER CODE BEGIN */

/* Includes ------------------------------------------------------------------*/
#include "soft.h"

/* Define --------------------------------------------------------------------*/

/* Variables -----------------------------------------------------------------*/

const uint16 M_PROTECTCURRENT = 700; //1.8A  //273,//1A//292;//中扫保护电流 500MA，计算公式 1A*0.47R*4096/3.3
const uint16 M_STOPCURRENT = 546; //2A//369;//788;//中扫停止电流1350MA，计算公式 1A*0.47R*4096/3.3


/* Private variables */

/* Function prototypes -------------------------------------------------------*/

/* Private function prototypes */


uint16_t LM_CURR,MM_CURR,RM_CURR,FAN1_CURR,FAN2_CURR;
float lmc,mmc,rmc,fan1c,fan2c;

/* Hook prototypes */

/**
   * @brief
   * @param
   * @param
   *
   * @retval
	 * 1.灰尘盒检测未做硬件，这版本可利用风机反馈信号检测灰尘盒是否安装，安装到位后才启动中扫、边扫。
	 * 2.电流检测只做中扫，风机和边扫电机自带保护板，还可利用反馈信号检测是否异常
   */
	 
void Go_mode(void)
{

}

void Alarm(void)
{
		LM_CURR = account_current(LM_ADC_PC5);
	  lmc = LM_CURR * 0.005395;//0.00081 = 3.3/4096   0.005395 = 0.00081/(15*0.01)
		MM_CURR = account_current(MM_ADC_PB0);	
	  mmc = MM_CURR * 0.005395;//0.00081 = 3.3/4096   0.005395 = 0.00081/(15*0.01)	
		RM_CURR = account_current(RM_ADC_PB1);
	  rmc = RM_CURR * 0.005395;//0.00081 = 3.3/4096   0.005395 = 0.00081/(15*0.01)	
		FAN1_CURR = account_current(FAN1_ADC_PA7);	
    fan1c = FAN1_CURR * 0.005395;//0.00081 = 3.3/4096   0.005395 = 0.00081/(15*0.01)	
		FAN2_CURR = account_current(FAN2_ADC_PC4);	
    fan2c = FAN2_CURR * 0.005395;//0.00081 = 3.3/4096   0.005395 = 0.00081/(15*0.01)	  
	
    if(mmc > 2.5/*M_PROTECTCURRENT*/) //
    {
        s_and.error_code |= 0x10; //中扫电流异常
    }
    else
    {
        s_and.error_code &= ~0x10;
    }	

				
    if((r_vr.velocity != 0) || (rmc > 1.5))  //右刷转速异常
    {
        s_and.error_code |= 0x08;
    }
    else
        {
            s_and.error_code &= ~0x08;
        }
				
    if((l_vr.velocity != 0) || (lmc > 1.5))     //左刷转速异常
    {
        s_and.error_code |= 0x20;
    }
    else
        {
            s_and.error_code &= ~0x20;
        }

    if((fan1.velocity != 0) || (fan1c > 4))
    {
        s_and.error_code |= 0x40; //风机1转速异常
    }
    else
    {
        s_and.error_code &= ~0x40;
    }

    if((fan2.velocity != 0) || (fan2c > 4))
    {
        s_and.error_code |= 0x80; //风机2转速异常
    }
    else
    {
        s_and.error_code &= ~0x80;
    }
}
/* USER CODE END */


