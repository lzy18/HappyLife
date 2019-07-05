/**
  ******************************************************************************
  * @file    Thread_MyDrive.c
  * @author  ������
  * @version V1.0
  * @date    2018.3.5
  * @brief   ����ļ�����һЩ��������
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
r_android r_and;

s_android s_and;

/* Private variables */

/* Function prototypes -------------------------------------------------------*/

/* Private function prototypes */

/* Hook prototypes */


/**
   * @brief ���㵥һͨ����ƽ��ֵ
   * @param ͨ��ֵ
   * @retval ��ѹƽ��ֵ
   */
uint16 account_current (uint8 channel)
{
    uint16 l;
    uint8  x;
    l = 0;
    for(x=0; x<8; x++)
    {
        l += uhADCxConvertedValue[channel + x*10] & 0XFFF;
    }
    l = l>>3;
    return l;

}

/**
   * @brief    ��䰲׿��ͨѶ����
   * @param    None
   * @param    None
   *
   * @retval   None
   */
void	USART3_send(void)						//?????
{
    u8 i = 0, p = 0;
    s_and.head1 = 0xaa;
    s_and.head2 = 0x55;


    for(i = 2; i < sizeof(s_android) - 1; i++)
    {
        p ^= *((u8*)&s_and + i);
    }
    s_and.check = p;

//    USART_ITConfig(USART3, USART_IT_TXE, ENABLE); //????
}

/* USER CODE END */

