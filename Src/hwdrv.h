/**
 *
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HWDRV_H
#define __HWDRV_H

/* Includes ------------------------------------------------------------------*/

#define GPIO_sensor GPIOA
#define QTRSENSOR1 	GPIO_Pin_6
#define QTRSENSOR2 	GPIO_Pin_7
#define READSENSOR1 GPIO_sensor->IDR & QTRSENSOR1
#define READSENSOR2 GPIO_sensor->IDR & QTRSENSOR2

#define MOTORx		GPIOB
#define MOTORA1 	GPIO_Pin_6
#define MOTORA2 	GPIO_Pin_7
#define MOTORB1 	GPIO_Pin_8
#define MOTORB2 	GPIO_Pin_9

#define ROBOTADDR   98D3,32,30511C,10

/* PWM consts */
#define MINPWM      0
#define MAXPWM      127
#define MIDPWM      64
#define SPWM        90
#define SBPWM       34
/* ADC const */
#define MAXVAL  4095
#define MINVAL  0
#define MIDVAL  2045

/* Definition for new status register */
#define Status_STOP         0x00 // 0b00000000
#define Status_FORWARD      0x01 // 0b00000001 
#define Status_BACK         0x02 // 0b00000010    
#define Status_LEFT         0x03 // 0b00000011   
#define Status_RIGHT        0x04 // 0b00000100     
#define Status_SLEFT        0x05 // 0b00000101     
#define Status_SRIGHT       0x06 // 0b00000110   
#define Status_SBLEFT       0x07 // 0b00000111     
#define Status_SBRIGHT      0x08 // 0b00001000 
#define Status_WALL         0x10 // 0b00010000
#define Status_CALIB        0x20 // 0b00100000
#define Status_AUTO         0x40 // 0b01000000
#define Status_OK           0x80 // 0b10000000
#define ARM_CMD_NO          0x00 // No command
#define ARM_CMD_UP          0x01
#define ARM_CMD_DOWN        0x02
#define ARM_CMD_GRAB        0x04
#define ARM_CMD_RELEASE     0x08


#define TXNUM 6
#define RXNUM 7
#define FAILURES 4
#define CALIBRATENUM 10
#define STARTMARKER		0x81
#define STOPMARKER 		0x8F
#define NUM 16
#define TIMEOUT 10
#define ARRAYSIZE 2*5

typedef enum
{
    FALSE = 0,
    TRUE = 1
} BOOL_TypeDef;

typedef enum
{
    SW_NONE = 1,
    SW_UP,
    SW_DOWN,
    SW_LEFT,
    SW_RIGHT,
    SW_OK    
} KEY_TypeDef;

typedef enum
{
    DISPLAY_unknown = 1,
    DISPLAY_menu,
    DISPLAY_status,
    DISPLAY_btmodule
} DISPLAY_Typedef;

typedef struct
{
    uint8_t prevStatus;
    uint8_t currentStatus;
    uint8_t responseStatus;
    uint8_t armStatus;
    KEY_TypeDef prevKey;
    KEY_TypeDef currentKey;
    BOOL_TypeDef keyPressed;
    DISPLAY_Typedef prevPage;
    DISPLAY_Typedef currentPage;
} DATA_TypeDef;

#endif
