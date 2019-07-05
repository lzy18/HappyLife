/**
  ******************************************************************************
  * @file    Thread_NeckControl.c
  * @author  ������
  * @version V2.1.0
  * @date    2017.9.12
  * @brief   ����ļ��ǳ���Ĵ����߼��ͱ���
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* USER CODE BEGIN */

/* Includes ------------------------------------------------------------------*/
//#include "main.h"
//#include "stm32f1xx_hal.h"
#include "cmsis_os.h"
#include "adc.h"
//#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "math.h"

#include "stm32f1xx_hal_def.h"

/* Define --------------------------------------------------------------------*/


#ifndef        __SOFT_H
#define        __SOFT_H

////////////////////////////////////////������һЩ����Ķ���//////////////////////
//#define PA0_MBrush        0         //��ˢ�������
//#define PA1_RBrushP2        1
//#define PA2_MBrushP1        2        //��ˢ������λ1
//#define PA3_MBrushP2        3
//#define PA4_LBrushP1        4           //���ˢ������λ1
//#define PA5_DCINADC         5           //���ڵ�ѹ���
//#define PA6_VCCBATADC       6         //��ص�ѹͨ��   
//#define PA7_IR1             7        //���Ᵽ����
//#define PB0_SBrushP1        8       //��ˢ������λ
//#define PB1_SBrushP2        9
//#define PC0_LBrushP2        10      //���ˢ������λ2
//#define PC1_R_IADC          11       //��ˢ��������
//#define PC3_L_IADC          12      //��ˢ��������
//#define PC4_IR2          13         //���Ᵽ����
//#define PC5_IR3          14         //���Ᵽ����

#define LM_ADC_PC5 0 //ADCIN15
#define MM_ADC_PB0 1 //ADCIN8
#define RM_ADC_PB1 2 //ADCIN9
#define FAN1_ADC_PA7 3 //ADCIN7
#define FAN2_ADC_PC4 4 //ADCIN14
#define AIN0_ADC_PA6 5 //ADCIN0
#define AIN1_ADC_PA5 6 //ADCIN1
#define AIN2_ADC_PA3 7 //ADCIN3
#define AIN3_ADC_PA4 8 //ADCIN4
#define AIN4_ADC_PA2 9 //ADCIN2



#define MAPAN	320    //1Ȧ*139���ٱ�*16�����̼���
#define MAX_SPEED 500 //mm
#define	BASE_DUTY 50		//��ʼDUTY
#define MAX_ACC_SPEED 80    //Ŀ����ԣ��о�ƽ��Ϊ׼
#define MAX_DUTY      3500      //0-4000

#define  order_speed 200



#define PA10_LEFTBRUSH_CP_HIGH       HAL_GPIO_WritePin (PA10_LeftBrush_CP_GPIO_Port, PA10_LeftBrush_CP_Pin,GPIO_PIN_SET)
#define PA10_LEFTBRUSH_CP_LOW       HAL_GPIO_WritePin (PA10_LeftBrush_CP_GPIO_Port, PA10_LeftBrush_CP_Pin,GPIO_PIN_RESET)
#define PE13_LEFTBURSH_EN     HAL_GPIO_WritePin (PE13_LEFTBURSH_EN_GPIO_Port,PE13_LEFTBURSH_EN_Pin,GPIO_PIN_SET)
#define PE13_LEFTBURSH_DIS     HAL_GPIO_WritePin (PE13_LEFTBURSH_EN_GPIO_Port,PE13_LEFTBURSH_EN_Pin,GPIO_PIN_RESET)
#define PE5_LEFTBRUSH_DIR_ELONGATE     HAL_GPIO_WritePin (PE5_leftBrush_DIR_GPIO_Port,PE5_leftBrush_DIR_Pin,GPIO_PIN_SET)
#define PE5_LEFTBRUSH_DIR_SHRINK     HAL_GPIO_WritePin (PE5_leftBrush_DIR_GPIO_Port,PE5_leftBrush_DIR_Pin,GPIO_PIN_RESET)

#define PA11_RIGHTBRUSH_CP_HIGH       HAL_GPIO_WritePin (PA11_rightBrush_CP_GPIO_Port, PA11_rightBrush_CP_Pin,GPIO_PIN_SET)
#define PA11_RIGHTBRUSH_CP_LOW       HAL_GPIO_WritePin (PA11_rightBrush_CP_GPIO_Port, PA11_rightBrush_CP_Pin,GPIO_PIN_RESET)
#define PE14_RIGHTBURSH_EN     HAL_GPIO_WritePin (PE14_RIGHTBRUSH_EN_GPIO_Port,PE14_RIGHTBRUSH_EN_Pin,GPIO_PIN_SET)
#define PE14_RIGHTBURSH_DIS     HAL_GPIO_WritePin (PE14_RIGHTBRUSH_EN_GPIO_Port,PE14_RIGHTBRUSH_EN_Pin,GPIO_PIN_RESET)
#define PE6_RIGHTBRUSH_DIR_ELONGATE     HAL_GPIO_WritePin (PE6_rightBrush_DIR_GPIO_Port,PE6_rightBrush_DIR_Pin,GPIO_PIN_SET)
#define PE6_RIGHTBRUSH_DIR_SHRINK     HAL_GPIO_WritePin (PE6_rightBrush_DIR_GPIO_Port,PE6_rightBrush_DIR_Pin,GPIO_PIN_RESET)

#define PE2_MBRUSHEN_HIGH       HAL_GPIO_WritePin (PE2_MBRUSHEN_GPIO_Port, PE2_MBRUSHEN_Pin,GPIO_PIN_SET)
#define PE2_MBRUSHEN_LOW       HAL_GPIO_WritePin (PE2_MBRUSHEN_GPIO_Port, PE2_MBRUSHEN_Pin,GPIO_PIN_RESET)
#define PC6_MBRUSH_P_HIGH       HAL_GPIO_WritePin (PC6_MBRUSH_P_GPIO_Port, PC6_MBRUSH_P_Pin,GPIO_PIN_SET)
#define PC6_MBRUSH_P_LOW       HAL_GPIO_WritePin (PC6_MBRUSH_P_GPIO_Port, PC6_MBRUSH_P_Pin,GPIO_PIN_RESET)
#define PC7_MBRUSH_N_HIGH       HAL_GPIO_WritePin (PC7_MBRUSH_N_GPIO_Port, PC7_MBRUSH_N_Pin,GPIO_PIN_SET)
#define PC7_MBRUSH_N_LOW       HAL_GPIO_WritePin (PC7_MBRUSH_N_GPIO_Port, PC7_MBRUSH_N_Pin,GPIO_PIN_RESET)

#define PE3_SBRUSHEN_HIGH       HAL_GPIO_WritePin (PE3_SBRUSHEN_GPIO_Port, PE3_SBRUSHEN_Pin,GPIO_PIN_SET)
#define PE3_SBRUSHEN_LOW       HAL_GPIO_WritePin (PE3_SBRUSHEN_GPIO_Port, PE3_SBRUSHEN_Pin,GPIO_PIN_RESET)
#define PC8_SBRUSH_P_HIGH       HAL_GPIO_WritePin (PC8_SBRUSH_P_GPIO_Port, PC8_SBRUSH_P_Pin,GPIO_PIN_SET)
#define PC8_SBRUSH_P_LOW       HAL_GPIO_WritePin (PC8_SBRUSH_P_GPIO_Port, PC8_SBRUSH_P_Pin,GPIO_PIN_RESET)
#define PC9_SBRUSH_N_HIGH       HAL_GPIO_WritePin (PC9_SBRUSH_N_GPIO_Port, PC9_SBRUSH_N_Pin,GPIO_PIN_SET)
#define PC9_SBRUSH_N_LOW       HAL_GPIO_WritePin (PC9_SBRUSH_N_GPIO_Port, PC9_SBRUSH_N_Pin,GPIO_PIN_RESET)

////////////////////////////////һЩ�꺯������//////////////////////////////

typedef  int16_t  s16;
typedef  int32_t  s32;

typedef  uint8_t  u8;
typedef  uint16_t  u16;
typedef  uint32_t  u32;

typedef  uint8_t  uint8;
typedef  uint16_t  uint16;
typedef  uint32_t  uint32;


typedef struct 		//���հ�׿���������ݰ�   12���ֽ�
{
    u8 head1;
    u8 head2;
    u8 order;		//���ֻ���ָ�1����ˢ����ת��2����ˢ�ӣ�3ת��ˢ�ӣ�4ͣתˢ�ӣ�5һ����ͣ����
    u8 reserved[4];
    u8 check;
} r_android;

typedef struct 			//�������ݰ���ANDROID				10���ֽ�
{
    u8 head1;			//0XAA
    u8 head2;			//0X55
    u8 error_code;      //��������ֽ�
    u8 dc_electricity_h;
    u8 dc_electricity_l;
    u8 bat_electricity_h;
    u8 bat_electricity_l;
    u8 reserved[2];
    u8 check;			//��λ�����У��
} s_android;


typedef struct	  //���ֵ��ٶ���Ϣ
{
    float  ek[3];       //���������ϵͳ�ٶ�ƫ��
    volatile    uint32  old_time;               //��������ǰ��ʱ��
    volatile    uint32  it_time;
    volatile    s32 ring;            //ת�ٵ�ǰֵ
    volatile    s32 old_ring;            //ת��ǰֵ
    float duty;     // ����ת�ٵ�ռ�ձ�
    s16 rap  ;       //�˰汾Ϊ���ת��
    float integral;
    float velocity;
} RAP;


/* Extern Variables -----------------------------------------------------------------*/

extern volatile uint32  giv_sys_time,clear_num,move_base_ring;

extern void Init_Ring(void);

extern u8  andr_order;

extern void USART3_send(void);

//extern IWDG_HandleTypeDef hiwdg;

extern r_android r_and;

extern s_android s_and;

extern u8 UART3ReceiveData[20];


/* Extern Function prototypes -------------------------------------------------------*/

extern uint16 account_current (uint8 channel);
extern void enable_rap(s16 v, s16 r);
extern void  comm_rap(void);
extern void Go_mode(void);
extern void Alarm(void);
extern uint16_t ultr_recvfinal[6];

extern RAP  m_vr, l_vr, r_vr, fan1, fan2;     //��������
extern void PIDSpeedController(RAP * rap);

extern void StartStep(void);
extern void StopStep(void);

extern void InitRap(RAP * rap);

extern uint16_t  uhADCxConvertedValue[80];//adת��������������

#endif

/* USER CODE END */
