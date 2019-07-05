/**
  ******************************************************************************
  * @file    Thread_process.c
  * @author  ������
  * @version V1.0
  * @date    2018.3.5
  * @brief   ����ļ��ǳ���Ĵ����߼��ͱ���
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

const uint16 M_PROTECTCURRENT = 700; //1.8A  //273,//1A//292;//��ɨ�������� 500MA�����㹫ʽ 1A*0.47R*4096/3.3
const uint16 M_STOPCURRENT = 546; //2A//369;//788;//��ɨֹͣ����1350MA�����㹫ʽ 1A*0.47R*4096/3.3


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
	 * 1.�ҳ��м��δ��Ӳ������汾�����÷�������źż��ҳ����Ƿ�װ����װ��λ���������ɨ����ɨ��
	 * 2.�������ֻ����ɨ������ͱ�ɨ����Դ������壬�������÷����źż���Ƿ��쳣
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
        s_and.error_code |= 0x10; //��ɨ�����쳣
    }
    else
    {
        s_and.error_code &= ~0x10;
    }	

				
    if((r_vr.velocity != 0) || (rmc > 1.5))  //��ˢת���쳣
    {
        s_and.error_code |= 0x08;
    }
    else
        {
            s_and.error_code &= ~0x08;
        }
				
    if((l_vr.velocity != 0) || (lmc > 1.5))     //��ˢת���쳣
    {
        s_and.error_code |= 0x20;
    }
    else
        {
            s_and.error_code &= ~0x20;
        }

    if((fan1.velocity != 0) || (fan1c > 4))
    {
        s_and.error_code |= 0x40; //���1ת���쳣
    }
    else
    {
        s_and.error_code &= ~0x40;
    }

    if((fan2.velocity != 0) || (fan2c > 4))
    {
        s_and.error_code |= 0x80; //���2ת���쳣
    }
    else
    {
        s_and.error_code &= ~0x80;
    }
}
/* USER CODE END */


