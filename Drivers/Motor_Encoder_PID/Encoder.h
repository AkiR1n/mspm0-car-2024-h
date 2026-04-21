#ifndef __ENCODER_H
#define __ENCODER_H

#include "ti_msp_dl_config.h"
#include <stdint.h>


//电机基本参数
#define ENCODE_13X 13 		//编码器线数
#define JIANSUBI 28     	//减速比
#define QUADRATURE_MULTIPLIER 4 // 4倍频 (A/B相的上升沿和下降沿都计数)

// 编码器电机转一圈，输出轴（车轮）产生的总脉冲数
#define PULSES_PER_REVOLUTION (ENCODE_13X * JIANSUBI * QUADRATURE_MULTIPLIER) // 13 * 28 * 4 = 1456

#define PI  3.1415f
#define RR  32.5f    		//车轮半径单位mm

/*编码器端口读取宏定义*/
#define Read_Encoder_A1 (DL_GPIO_readPins(GPIO_ENCODER_PORT,GPIO_ENCODER_PIN_A1_PIN)==GPIO_ENCODER_PIN_A1_PIN)?0:1//左轮 A相
#define Read_Encoder_A2 (DL_GPIO_readPins(GPIO_ENCODER_PORT,GPIO_ENCODER_PIN_A2_PIN)==GPIO_ENCODER_PIN_A2_PIN)?0:1//左轮 B相
#define Read_Encoder_B1 (DL_GPIO_readPins(GPIO_ENCODER_PORT,GPIO_ENCODER_PIN_B1_PIN)==GPIO_ENCODER_PIN_B1_PIN)?0:1//右轮 A相
#define Read_Encoder_B2 (DL_GPIO_readPins(GPIO_ENCODER_PORT,GPIO_ENCODER_PIN_B2_PIN)==GPIO_ENCODER_PIN_B2_PIN)?0:1//右轮 B相


// 编码器相关函数
void Encoder_Init(void);                           // 初始化编码器
int32_t Encoder_GetCount(uint8_t motor_id);        // 获取编码器计数 (0=左电机, 1=右电机)
int32_t Encoder_GetSpeed_PPS(uint8_t motor_id);    // 获取编码器速度(每秒脉冲数 PPS)
uint32_t Encoder_GetSpeed_PPS_Abs(uint8_t motor_id);    // 获取编码器速度绝对值(PPS)
float Encoder_GetSpeed_RPS(uint8_t motor_id);    // 获取编码器速度(每秒转数 RPS)
float Encoder_GetSpeed_RPS_Abs(uint8_t motor_id);   // 获取编码器速度绝对值(RPS)
void Encoder_Reset(uint8_t motor_id);              // 重置编码器计数 (0=左电机, 1=右电机, 2=全部)
void Encoder_IRQHandler(void);                     // 编码器中断处理函数

// 定时器中断处理函数（用于速度计算）
void Encoder_Timer_IRQHandler(void);

#endif
