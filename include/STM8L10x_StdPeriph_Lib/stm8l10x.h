/**
  ******************************************************************************
  * @file    stm8l10x.h
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    30-September-2014
  * @brief   This file contains all HW registers definitions and memory mapping.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM8L10x_H
#define __STM8L10x_H

#include <stdint.h>

#ifdef __cplusplus
namespace STM8L10x_StdPeriph_Lib {
#endif

/******************************************************************************/
/*                   Library configuration section                            */
/******************************************************************************/

/* Check the used compiler */
#if defined(__CSMC__)
 #define _COSMIC_
#elif defined(__RCST7__)
 #define _RAISONANCE_
#elif defined(__ICCSTM8__)
 #define _IAR_
#else
 #error "Unsupported Compiler!"          /* Compiler defines not found */
#endif

#if !defined  USE_STDPERIPH_DRIVER
/* Comment the line below if you will not use the peripherals drivers.
   In this case, these drivers will not be included and the application code will be
   based on direct access to peripherals registers */
 #define USE_STDPERIPH_DRIVER
#endif

#if defined(_COSMIC_)
 #define FAR  @far
 #define NEAR @near
 #define TINY @tiny
 #define EEPROM @eeprom
 #define CONST  const
#elif defined(_RAISONANCE_)
 #define FAR  far
 #define NEAR data
 #define TINY page0
 #define EEPROM eeprom
 #define CONST  code
 #define MEMCPY memcpy /*!< Used with memory Models for code less than 64K */
#else /* _IAR_ */
 #define FAR  __far
 #define NEAR __near
 #define TINY __tiny
 #define EEPROM __eeprom
 #define CONST  const
#endif

/**
  * @brief Legacy definition
  */
#define __CONST  CONST

/*!< Used with memory Models for code smaller than 64K */
#define PointerAttr NEAR

/* Uncomment the line below to enable the FLASH functions execution from RAM */
#if !defined (RAM_EXECUTION)
/* #define RAM_EXECUTION  (1) */
#endif /* RAM_EXECUTION */

#ifdef RAM_EXECUTION
 #ifdef _COSMIC_
   #define IN_RAM(a) a
 #elif defined (_RAISONANCE_) /* __RCST7__ */
   #define IN_RAM(a) a inram
 #else /*_IAR_*/
  #define IN_RAM(a) __ramfunc a
 #endif /* _COSMIC_ */
#else 
  #define IN_RAM(a) a
#endif /* RAM_EXECUTION */

/*!< [31:16] STM8L10x Standard Peripheral Library main version */
#define __STM8L10X_STDPERIPH_VERSION_MAIN   ((uint8_t)0x01) /*!< [31:24] main version */                                  
#define __STM8L10X_STDPERIPH_VERSION_SUB1   ((uint8_t)0x02) /*!< [23:16] sub1 version */
#define __STM8L10X_STDPERIPH_VERSION_SUB2   ((uint8_t)0x01) /*!< [15:8]  sub2 version */
#define __STM8L10X_STDPERIPH_VERSION_RC     ((uint8_t)0x00) /*!< [7:0]  release candidate */ 
#define __STM8L10X_STDPERIPH_VERSION       ( (__STM8L10X_STDPERIPH_VERSION_MAIN << 24)\
                                          |(__STM8L10X_STDPERIPH_VERSION_SUB1 << 16)\
                                          |(__STM8L10X_STDPERIPH_VERSION_SUB2 << 8)\
                                          |(__STM8L10X_STDPERIPH_VERSION_RC))


/******************************************************************************/

/* Includes ------------------------------------------------------------------*/

/* Exported types and constants-----------------------------------------------*/

/** @addtogroup Exported_types
  * @{
  */

/**
 * IO definitions
 *
 * define access restrictions to peripheral registers
 */
#define     __I     volatile const            /*!< defines 'read only' permissions      */
#define     __O     volatile                  /*!< defines 'write only' permissions     */
#define     __IO    volatile                  /*!< defines 'read / write' permissions   */

//# use build-in stdint.h instead
#if 0
/*!< Signed integer types  */
typedef   signed char     int8_t;
typedef   signed short    int16_t;
typedef   signed long     int32_t;

/*!< Unsigned integer types  */
typedef unsigned char     uint8_t;
typedef unsigned short    uint16_t;
typedef unsigned long     uint32_t;

/*!< STM8L10x Standard Peripheral Library old types (maintained for legacy prupose) */

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;


typedef enum {FALSE = 0, TRUE = !FALSE} bool;

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus, BitStatus, BitAction;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

enum {
  U8_MAX = (255), 
  S8_MAX = (127), 
  S8_MIN = (-128), 
  U16_MAX = (65535u), 
  S16_MAX = (32767), 
  S16_MIN = (-32768), 
  // U32_MAX = (4294967295uL), 
  S32_MAX = (2147483647), 
  // S32_MIN = (-2147483648uL), 
};
/**
  * @}
  */
#endif

/** @addtogroup MAP_FILE_Exported_Types_and_Constants
  * @{
  */

/******************************************************************************/
/*                          IP registers structures                           */
/******************************************************************************/

/*----------------------------------------------------------------------------*/
/**
  * @brief General Purpose I/Os (GPIO)
  */
typedef struct GPIO_struct
{
  __IO uint8_t ODR; /*!< Output Data Register */
  __IO uint8_t IDR; /*!< Input Data Register */
  __IO uint8_t DDR; /*!< Data Direction Register */
  __IO uint8_t CR1; /*!< Configuration Register 1 */
  __IO uint8_t CR2; /*!< Configuration Register 2 */
}
GPIO_TypeDef;

/** @addtogroup GPIO_Registers_Reset_Value
  * @{
  */
enum {
  GPIO_ODR_RESET_VALUE = ((uint8_t)0x00), 
  GPIO_DDR_RESET_VALUE = ((uint8_t)0x00), 
  GPIO_CR1_RESET_VALUE = ((uint8_t)0x00), 
  GPIO_CR2_RESET_VALUE = ((uint8_t)0x00), 
};
/**
  * @}
  */
/*----------------------------------------------------------------------------*/

/**
  * @brief Auto Wake Up (AWU) peripheral registers.
  */
typedef struct AWU_struct
{
  __IO uint8_t CSR; /*!< AWU Control status register */
  __IO uint8_t APR; /*!< AWU Asynchronous prescalar buffer */
  __IO uint8_t TBR; /*!< AWU Time base selection register */
}
AWU_TypeDef;

/** @addtogroup AWU_Registers_Reset_Value
  * @{
  */

enum {
  AWU_CSR_RESET_VALUE = ((uint8_t)0x00), 
  AWU_APR_RESET_VALUE = ((uint8_t)0x3F), 
  AWU_TBR_RESET_VALUE = ((uint8_t)0x00), 
};
/**
  * @}
  */

/** @addtogroup AWU_Registers_Bits_Definition
  * @{
  */

enum {
  AWU_CSR_AWUF = ((uint8_t)0x20) /*!< Interrupt flag mask */, 
  AWU_CSR_AWUEN = ((uint8_t)0x10) /*!< Auto Wake-up enable mask */, 
  AWU_CSR_MSR = ((uint8_t)0x01) /*!< Measurement enable mask */, 

  AWU_APR_APR = ((uint8_t)0x3F) /*!< Asynchronous Prescaler divider mask */, 

  AWU_TBR_AWUTB = ((uint8_t)0x0F) /*!< Timebase selection mask */, 
};


/**
  * @}
  */


/*----------------------------------------------------------------------------*/
/**
  * @brief Beeper (BEEP) peripheral registers.
  */

typedef struct BEEP_struct
{
  __IO uint8_t CSR; /*!< BEEP Control status register */
}
BEEP_TypeDef;

/** @addtogroup BEEP_Registers_Reset_Value
  * @{
  */

enum {
  BEEP_CSR_RESET_VALUE = ((uint8_t)0x1F), 
};
/**
  * @}
  */

/** @addtogroup BEEP_Registers_Bits_Definition
  * @{
  */

enum {
  BEEP_CSR_BEEPSEL = ((uint8_t)0xC0) /*!< Beeper frequency selection mask */, 
  BEEP_CSR_BEEPEN = ((uint8_t)0x20) /*!< Beeper enable mask */, 
  BEEP_CSR_BEEPDIV = ((uint8_t)0x1F) /*!< Beeper Divider prescalar mask */, 
};
/**
  * @}
  */

/*----------------------------------------------------------------------------ok*/

/**
  * @brief Configuration Registers (CFG)
  */

typedef struct CFG_struct
{
  __IO uint8_t GCR; /*!< Global Configuration register */
}
CFG_TypeDef;

/** @addtogroup CFG_Registers_Reset_Value
  * @{
  */

enum {
  CFG_GCR_RESET_VALUE = ((uint8_t)0x00), 
};
/**
  * @}
  */

/** @addtogroup CFG_Registers_Bits_Definition
  * @{
  */

enum {
  CFG_GCR_SWD = ((uint8_t)0x01) /*!< Swim disable bit mask */, 
  CFG_GCR_AL = ((uint8_t)0x02) /*!< Activation Level bit mask */, 
};
/**
  * @}
  */

/**
  * @}
  */

/*----------------------------------------------------------------------------ok*/

/**
  * @brief Clock Controller (CLK)
  */
typedef struct CLK_struct
{
  __IO uint8_t CKDIVR;      /*!< Clock Divider Register */
  uint8_t RESERVED1;
  uint8_t RESERVED2;
  __IO uint8_t PCKENR;     /*!< Peripheral Clock Gating Register  */
  uint8_t RESERVED3;
  __IO uint8_t CCOR;       /*!< Configurable Clock Output Register */
}
CLK_TypeDef;

/** @addtogroup CLK_Registers_Reset_Value
  * @{
  */
enum {
  CLK_CKDIVR_RESET_VALUE = ((uint8_t)0x03), 
  CLK_PCKENR_RESET_VALUE = ((uint8_t)0x00), 
  CLK_CCOR_RESET_VALUE = ((uint8_t)0x00), 
};
/**
  * @}
  */

/** @addtogroup CLK_Registers_Bits_Definition
  * @{
  */

enum {
  CLK_CKDIVR_HSIDIV = ((uint8_t)0x03) /*!< clock prescaler */, 

  CLK_PCKENR_TIM2 = ((uint8_t)0x01) /*!< Peripheral Clock Enable 1, TIM2 */, 
  CLK_PCKENR_TIM3 = ((uint8_t)0x02) /*!< Peripheral Clock Enable 1, TIM3 */, 
  CLK_PCKENR_TIM4 = ((uint8_t)0x04) /*!< Peripheral Clock Enable 1, TIM4 */, 
  CLK_PCKENR_I2C = ((uint8_t)0x08) /*!< Peripheral Clock Enable 1, I2C */, 
  CLK_PCKENR_SPI = ((uint8_t)0x10) /*!< Peripheral Clock Enable 1, SPI */, 
  CLK_PCKENR_USART = ((uint8_t)0x20) /*!< Peripheral Clock Enable 1, USART */, 
  CLK_PCKENR_AWU = ((uint8_t)0x40) /*!< Peripheral Clock Enable 1, AWU */, 
  CLK_PCKENR_PXS = ((uint8_t)0x80) /*!< Peripheral Clock Enable 1, PXS */, 

  CLK_CCOR_CCOSEL = ((uint8_t)0x06) /*!< Configurable clock output selection */, 
  CLK_CCOR_CCOEN = ((uint8_t)0x01) /*!< Configurable clock output enable */, 
};
/**
  * @}
  */
/*----------------------------------------------------------------------------ok*/

/**
  * @brief Comparator interface (COMP)
  */
typedef struct COMP_struct
{
  __IO uint8_t CR;    /*!< Control register  */
  __IO uint8_t CSR;   /*!< Control status register   */
  __IO uint8_t CCS;   /*!< Comparator Channel Selection   */
}
COMP_TypeDef;

/** @addtogroup COMP_Registers_Reset_Value
  * @{
  */
enum {
  COMP_CR_RESET_VALUE = ((uint8_t)0x00), 
  COMP_CSR_RESET_VALUE = ((uint8_t)0x50), 
  COMP_CCS_RESET_VALUE = ((uint8_t)0x00), 
};
/**
  * @}
  */

/** @addtogroup COMP_Registers_Bits_Definition
  * @{
  */
enum {
/* CR*/
  COMP_CR_IC1BK = ((uint8_t)0x80) /*!< TIM2 input capture/break  Mask. */, 
  COMP_CR_CNFTIM = ((uint8_t)0x60) /*!< Connect output comparator configuration Mask. */, 
  COMP_CR_POL = ((uint8_t)0x10) /*!< Comparator Polarity Mask. */, 
  COMP_CR_COMPREF = ((uint8_t)0x08) /*!< Comparator Reference Mask. */, 
  COMP_CR_COMP2EN = ((uint8_t)0x04) /*!< Second Comparator Enable Mask. */, 
  COMP_CR_COMP1EN = ((uint8_t)0x02) /*!< First Comparator Enable Mask. */, 
  COMP_CR_BIASEN = ((uint8_t)0x01) /*!< Bias Enable Mask. */, 

/* CSR*/
  COMP_CSR_ITEN2 = ((uint8_t)0x80) /*!<  Second Comparator Interrupt Enable Mask. */, 
  COMP_CSR_CEF2 = ((uint8_t)0x40) /*!<  Second Comparator Ecevt Flag Mask. */, 
  COMP_CSR_ITEN1 = ((uint8_t)0x20) /*!<  First Comparator Interrupt Enable Mask. */, 
  COMP_CSR_CEF1 = ((uint8_t)0x10) /*!<  First Comparator Event Flag Mask. */, 
  COMP_CSR_COMP2OUT = ((uint8_t)0x02) /*!<  Second Comparator Output Mask. */, 
  COMP_CSR_COMP1OUT = ((uint8_t)0x01) /*!<  First Comparator Output Mask. */, 

/* CCS*/
  COMP_CCS_COMP2CH4 = ((uint8_t)0x80) /*!< Comparator 2 switch 4 Enable Mask. */, 
  COMP_CCS_COMP2CH3 = ((uint8_t)0x40) /*!< Comparator 2 switch 3 Enable Mask. */, 
  COMP_CCS_COMP2CH2 = ((uint8_t)0x20) /*!< Comparator 2 switch 2 Enable Mask. */, 
  COMP_CCS_COMP2CH1 = ((uint8_t)0x10) /*!< Comparator 2 switch 1 Enable Mask. */, 
  COMP_CCS_COMP1CH4 = ((uint8_t)0x08) /*!< Comparator 1 switch 4 Enable Mask. */, 
  COMP_CCS_COMP1CH3 = ((uint8_t)0x04) /*!< Comparator 1 switch 3 Enable Mask. */, 
  COMP_CCS_COMP1CH2 = ((uint8_t)0x02) /*!< Comparator 1 switch 2 Enable Mask. */, 
  COMP_CCS_COMP1CH1 = ((uint8_t)0x01) /*!< Comparator 1 switch 1 Enable Mask. */, 
};
/**
  * @}
  */

/*----------------------------------------------------------------------------ok*/

/**
  * @brief External Interrupt Controller (EXTI)
  */
typedef struct EXTI_struct
{
  __IO uint8_t CR1;  /*!<  The four LSB EXTI  pin sensitivity */
  __IO uint8_t CR2;  /*!<  The four MSB EXTI  pin sensitivity */
  __IO uint8_t CR3;  /*!<  EXTI port B & port D sensitivity */
  __IO uint8_t SR1;  /*!<  Pins Status flag register 1 */
  __IO uint8_t SR2;  /*!<  Ports Status flag register 2 */
  __IO uint8_t CONF; /*!<  Port interrupt selector */
}
EXTI_TypeDef;

/** @addtogroup EXTI_Registers_Reset_Value
  * @{
  */

enum {
  EXTI_CR1_RESET_VALUE = ((uint8_t)0x00), 
  EXTI_CR2_RESET_VALUE = ((uint8_t)0x00), 
  EXTI_CR3_RESET_VALUE = ((uint8_t)0x00), 
  EXTI_CONF_RESET_VALUE = ((uint8_t)0x00), 
  EXTI_SR1_RESET_VALUE = ((uint8_t)0x00), 
  EXTI_SR2_RESET_VALUE = ((uint8_t)0x00), 
};
/**
  * @}
  */

/** @addtogroup EXTI_Registers_Bits_Definition
  * @{
  */
enum {
/* CR1 */
  EXTI_CR1_P3IS = ((uint8_t)0xC0) /*!< EXTI Pin 3 external interrupt sensitivity bit Mask */, 
  EXTI_CR1_P2IS = ((uint8_t)0x30) /*!< EXTI Pin 2 external interrupt sensitivity bit Mask */, 
  EXTI_CR1_P1IS = ((uint8_t)0x0C) /*!< EXTI Pin 1  external interrupt sensitivity bit Mask */, 
  EXTI_CR1_P0IS = ((uint8_t)0x03) /*!< EXTI Pin 0 external interrupt sensitivity bit Mask */, 

/* CR2 */
  EXTI_CR2_P7IS = ((uint8_t)0xC0) /*!< EXTI Pin 7 external interrupt sensitivity bit Mask */, 
  EXTI_CR2_P6IS = ((uint8_t)0x30) /*!< EXTI Pin 6 external interrupt sensitivity bit Mask */, 
  EXTI_CR2_P5IS = ((uint8_t)0x0C) /*!< EXTI Pin 5  external interrupt sensitivity bit Mask */, 
  EXTI_CR2_P4IS = ((uint8_t)0x03) /*!< EXTI Pin 4 external interrupt sensitivity bit Mask */, 

/* CR3 */
  EXTI_CR3_PBIS = ((uint8_t)0x03) /*!< EXTI PORTB external interrupt sensitivity bits Mask */, 
  EXTI_CR3_PDIS = ((uint8_t)0x0C) /*!< EXTI PORTD external interrupt sensitivity bits Mask */, 

/* CONF */
  EXTI_CONF_PBL = ((uint8_t)0x01) /*!< EXTI PORTB low interrupt selector bit Mask */, 
  EXTI_CONF_PBH = ((uint8_t)0x02) /*!< EXTI PORTB high interrupt selector bit Mask */, 
  EXTI_CONF_PDL = ((uint8_t)0x04) /*!< EXTI PORTD low interrupt selector bit Mask */, 
  EXTI_CONF_PDH = ((uint8_t)0x08) /*!< EXTI PORTD high interrupt selector bit Mask */, 
};
/**
  * @}
  */

/*----------------------------------------------------------------------------ok*/

/**
  * @brief FLASH and Data EEPROM
  */
typedef struct FLASH_struct
{
  __IO uint8_t CR1;        /*!< Flash control register 1 */
  __IO uint8_t CR2;        /*!< Flash control register 2 */
  __IO uint8_t PUKR;       /*!< Flash program memory unprotection register */
  __IO uint8_t DUKR;       /*!< Data EEPROM unprotection register */
  __IO uint8_t IAPSR;      /*!< Flash in-application programming status register */
}
FLASH_TypeDef;

/** @addtogroup FLASH_Registers_Reset_Value
  * @{
  */
enum {
  FLASH_CR1_RESET_VALUE = ((uint8_t)0x00), 
  FLASH_CR2_RESET_VALUE = ((uint8_t)0x00), 
  FLASH_PUKR_RESET_VALUE = ((uint8_t)0xAE), 
  FLASH_DUKR_RESET_VALUE = ((uint8_t)0x56), 
  FLASH_IAPSR_RESET_VALUE = ((uint8_t)0x40), 
};

/**
  * @}
  */

/** @addtogroup FLASH_Registers_Bits_Definition
  * @{
  */
enum {
  FLASH_CR1_IE = ((uint8_t)0x02) /*!< Flash Interrupt enable Mask */, 
  FLASH_CR1_FIX = ((uint8_t)0x01) /*!< Fix programming time Mask */, 

  FLASH_CR2_WPRG = ((uint8_t)0x40) /*!< Word write once Mask */, 
  FLASH_CR2_ERASE = ((uint8_t)0x20) /*!< Erase block Mask */, 
  FLASH_CR2_FPRG = ((uint8_t)0x10) /*!< Fast programming mode Mask */, 
  FLASH_CR2_PRG = ((uint8_t)0x01) /*!< Program block Mask */, 

  FLASH_IAPSR_HVOFF = ((uint8_t)0x40) /*!< End of high voltage flag Mask */, 
  FLASH_IAPSR_DUL = ((uint8_t)0x08) /*!< Data EEPROM unlocked flag Mask */, 
  FLASH_IAPSR_EOP = ((uint8_t)0x04) /*!< End of operation flag Mask */, 
  FLASH_IAPSR_PUL = ((uint8_t)0x02) /*!< Program memory unlocked flag Mask */, 
  FLASH_IAPSR_WR_PG_DIS = ((uint8_t)0x01) /*!< Write attempted to protected page Mask */, 

  FLASH_PUKR_PUK = ((uint8_t)0xFF) /*!< Flash Program memory unprotection mask */, 

  FLASH_DUKR_DUK = ((uint8_t)0xFF) /*!< Data EEPROM unprotection mask */, 
};
/**
  * @}
  */

/*----------------------------------------------------------------------------*/

/**
  * @brief Inter-Integrated Circuit (I2C)
  */
typedef struct I2C_struct
{
  __IO uint8_t CR1;    /*!< I2C control register 1 */
  __IO uint8_t CR2;    /*!< I2C control register 2 */
  __IO uint8_t FREQR;  /*!< I2C frequency register */
  __IO uint8_t OARL;   /*!< I2C own address register LSB */
  __IO uint8_t OARH;   /*!< I2C own address register MSB */
  uint8_t RESERVED;
  __IO uint8_t DR;     /*!< I2C data register */
  __IO uint8_t SR1;    /*!< I2C status register 1 */
  __IO uint8_t SR2;    /*!< I2C status register 2 */
  __IO uint8_t SR3;    /*!< I2C status register 3 */
  __IO uint8_t ITR;    /*!< I2C interrupt register */
  __IO uint8_t CCRL;   /*!< I2C clock control register low */
  __IO uint8_t CCRH;   /*!< I2C clock control register high */
  __IO uint8_t TRISER; /*!< I2C maximum rise time register */
}
I2C_TypeDef;

/** @addtogroup I2C_Registers_Reset_Value
  * @{
  */
enum {
  I2C_CR1_RESET_VALUE = ((uint8_t)0x00), 
  I2C_CR2_RESET_VALUE = ((uint8_t)0x00), 
  I2C_FREQR_RESET_VALUE = ((uint8_t)0x00), 
  I2C_OARL_RESET_VALUE = ((uint8_t)0x00), 
  I2C_OARH_RESET_VALUE = ((uint8_t)0x00), 
  I2C_DR_RESET_VALUE = ((uint8_t)0x00), 
  I2C_SR1_RESET_VALUE = ((uint8_t)0x00), 
  I2C_SR2_RESET_VALUE = ((uint8_t)0x00), 
  I2C_SR3_RESET_VALUE = ((uint8_t)0x00), 
  I2C_ITR_RESET_VALUE = ((uint8_t)0x00), 
  I2C_CCRL_RESET_VALUE = ((uint8_t)0x00), 
  I2C_CCRH_RESET_VALUE = ((uint8_t)0x00), 
  I2C_TRISER_RESET_VALUE = ((uint8_t)0x02), 
};
/**
  * @}
  */

/** @addtogroup I2C_Registers_Bits_Definition
  * @{
  */
enum {
  I2C_CR1_NOSTRETCH = ((uint8_t)0x80) /*!< Clock Stretching Disable (Slave mode) */, 
  I2C_CR1_ENGC = ((uint8_t)0x40) /*!< General Call Enable */, 
  I2C_CR1_PE = ((uint8_t)0x01) /*!< Peripheral Enable */, 

  I2C_CR2_SWRST = ((uint8_t)0x80) /*!< Software Reset */, 
  I2C_CR2_POS = ((uint8_t)0x08) /*!< Acknowledge */, 
  I2C_CR2_ACK = ((uint8_t)0x04) /*!< Acknowledge Enable */, 
  I2C_CR2_STOP = ((uint8_t)0x02) /*!< Stop Generation */, 
  I2C_CR2_START = ((uint8_t)0x01) /*!< Start Generation */, 

  I2C_FREQR_FREQ = ((uint8_t)0x3F) /*!< Peripheral Clock Frequency */, 

  I2C_OARL_ADD = ((uint8_t)0xFE) /*!< Interface Address bits [7..1] */, 
  I2C_OARL_ADD0 = ((uint8_t)0x01) /*!< Interface Address bit0 */, 

  I2C_OARH_ADDMODE = ((uint8_t)0x80) /*!< Addressing Mode (Slave mode) */, 
  I2C_OARH_ADDCONF = ((uint8_t)0x40) /*!< Address mode configuration */, 
  I2C_OARH_ADD = ((uint8_t)0x06) /*!< Interface Address bits [9..8] */, 

  I2C_DR_DR = ((uint8_t)0xFF) /*!< Data Register */, 

  I2C_SR1_TXE = ((uint8_t)0x80) /*!< Data Register Empty (transmitters) */, 
  I2C_SR1_RXNE = ((uint8_t)0x40) /*!< Data Register not Empty (receivers) */, 
  I2C_SR1_STOPF = ((uint8_t)0x10) /*!< Stop detection (Slave mode) */, 
  I2C_SR1_ADD10 = ((uint8_t)0x08) /*!< 10-bit header sent (Master mode) */, 
  I2C_SR1_BTF = ((uint8_t)0x04) /*!< Byte Transfer Finished */, 
  I2C_SR1_ADDR = ((uint8_t)0x02) /*!< Address sent (master mode)/matched (slave mode) */, 
  I2C_SR1_SB = ((uint8_t)0x01) /*!< Start Bit (Master mode) */, 

  I2C_SR2_WUFH = ((uint8_t)0x20) /*!< Wake-up from Halt */, 
  I2C_SR2_OVR = ((uint8_t)0x08) /*!< Overrun/Underrun */, 
  I2C_SR2_AF = ((uint8_t)0x04) /*!< Acknowledge Failure */, 
  I2C_SR2_ARLO = ((uint8_t)0x02) /*!< Arbitration Lost (master mode) */, 
  I2C_SR2_BERR = ((uint8_t)0x01) /*!< Bus Error */, 

  I2C_SR3_GENCALL = ((uint8_t)0x10) /*!< General Call Header (Slave mode) */, 
  I2C_SR3_TRA = ((uint8_t)0x04) /*!< Transmitter/Receiver */, 
  I2C_SR3_BUSY = ((uint8_t)0x02) /*!< Bus Busy */, 
  I2C_SR3_MSL = ((uint8_t)0x01) /*!< Master/Slave */, 

  I2C_ITR_ITBUFEN = ((uint8_t)0x04) /*!< Buffer Interrupt Enable */, 
  I2C_ITR_ITEVTEN = ((uint8_t)0x02) /*!< Event Interrupt Enable */, 
  I2C_ITR_ITERREN = ((uint8_t)0x01) /*!< Error Interrupt Enable */, 

  I2C_CCRL_CCR = ((uint8_t)0xFF) /*!< Clock Control Register (Master mode) */, 

  I2C_CCRH_FS = ((uint8_t)0x80) /*!< Master Mode Selection */, 
  I2C_CCRH_DUTY = ((uint8_t)0x40) /*!< Fast Mode Duty Cycle */, 
  I2C_CCRH_CCR = ((uint8_t)0x0F) /*!< Clock Control Register in Fast/Standard mode (Master mode) bits [11..8] */, 

  I2C_TRISER_TRISE = ((uint8_t)0x3F) /*!< Maximum Rise Time in Fast/Standard mode (Master mode) */, 
};
/**
  * @}
  */

/*----------------------------------------------------------------------------*/

/**
  * @brief IR digital interface (IRTIM)
  */
typedef struct IRTIM_struct
{
  __IO uint8_t CR; /*!< control register */
}
IRTIM_TypeDef;
/** @addtogroup IRTIM_Registers_Reset_Value
  * @{
  */
enum {
  IRTIM_CR_RESET_VALUE = ((uint8_t)0x00), 
};

/**
* @}
*/

/** @addtogroup IRTIM_Registers_Bits_Definition
  * @{
  */
/* CR*/
enum {
  IRTIM_CR_EN = ((uint8_t)0x01) /*!< IRTIM_OUT enable Mask. */, 
  IRTIM_CR_HSEN = ((uint8_t)0x02) /*!< High sink open drain buffer enable Mask */, 
};
/**
  * @}
  */

/*----------------------------------------------------------------------------*/

/**
  * @brief Interrupt Controller (ITC)
  */
typedef struct ITC_struct
{
  __IO uint8_t ISPR1; /*!<  Interrupt Software Priority register 1 */
  __IO uint8_t ISPR2; /*!<  Interrupt Software Priority register 2 */
  __IO uint8_t ISPR3; /*!<  Interrupt Software Priority register 3 */
  __IO uint8_t ISPR4; /*!<  Interrupt Software Priority register 4 */
  __IO uint8_t ISPR5; /*!<  Interrupt Software Priority register 5 */
  __IO uint8_t ISPR6; /*!<  Interrupt Software Priority register 6 */
  __IO uint8_t ISPR7; /*!<  Interrupt Software Priority register 7 */
  __IO uint8_t ISPR8; /*!< Interrupt Software Priority register 8 */
}
ITC_TypeDef;

/** @addtogroup ITC_Registers_Reset_Value
  * @{
  */
enum {
  ITC_SPR1_RESET_VALUE = ((uint8_t)0xFF) /*!< Reset value of Software Priority registers 1 */, 
  ITC_SPR2_RESET_VALUE = ((uint8_t)0xFF) /*!< Reset value of Software Priority registers 2 */, 
  ITC_SPR3_RESET_VALUE = ((uint8_t)0xFF) /*!< Reset value of Software Priority registers 3 */, 
  ITC_SPR4_RESET_VALUE = ((uint8_t)0xFF) /*!< Reset value of Software Priority registers 4 */, 
  ITC_SPR5_RESET_VALUE = ((uint8_t)0xFF) /*!< Reset value of Software Priority registers 5 */, 
  ITC_SPR6_RESET_VALUE = ((uint8_t)0xFF) /*!< Reset value of Software Priority registers 6 */, 
  ITC_SPR7_RESET_VALUE = ((uint8_t)0xFF) /*!< Reset value of Software Priority registers 7 */, 
  ITC_SPR8_RESET_VALUE = ((uint8_t)0xFF) /*!< Reset value of Software Priority registers 8 */, 
};
/**
  * @}
  */

/*----------------------------------------------------------------------------*/

/**
  * @brief Internal Low Speed Watchdog (IWDG)
  */
typedef struct IWDG_struct
{
  __IO uint8_t KR;  /*!<  Low Speed Watchdog Key Register */
  __IO uint8_t PR;  /*!<  Low Speed Watchdog Prescaler Register */
  __IO uint8_t RLR; /*!<  Low Speed Watchdog Reload Register */
}
IWDG_TypeDef;

/** @addtogroup IWDG_Registers_Reset_Value
  * @{
  */
enum {
  IWDG_RLR_RESET_VALUE = ((uint8_t)0xFF)  /*! <Reload Register Default Value */, 
  IWDG_PR_RESET_VALUE = ((uint8_t)0x00)  /*! <Prescaler Register Default Value */, 
};
/**
  * @}
  */
/*----------------------------------------------------------------------------*/


/**
  * @brief Wait For Event (WFE) peripheral registers.
  */
/** @addtogroup WFE_Registers
  * @{
  */
typedef struct WFE_struct
{
  __IO uint8_t CR1;   /*!<  Wait for event control register 1 */
  __IO uint8_t CR2;   /*!<  Wait for event control register 2 */
}
WFE_TypeDef;
/**
  * @}
  */
/** @addtogroup WFE_Registers_Reset_Value
  * @{
  */

enum {
  WFE_CR1_RESET_VALUE = ((uint8_t)0x00) /*!< Reset value wait for event control register1*/, 
  WFE_CR2_RESET_VALUE = ((uint8_t)0x00) /*!< Reset value wait for event control register2*/, 
};
/**
  * @}
  */

/** @addtogroup WFE_Registers_Bits_Definition
  * @{
  */

enum {
  WFE_CR1_EXTI_EV3 = ((uint8_t)0x80) /*!< External interrupt event 3*/, 
  WFE_CR1_EXTI_EV2 = ((uint8_t)0x40) /*!< External interrupt event 2*/, 
  WFE_CR1_EXTI_EV1 = ((uint8_t)0x20) /*!< External interrupt event 1*/, 
  WFE_CR1_EXTI_EV0 = ((uint8_t)0x10) /*!< External interrupt event 0*/, 
  WFE_CR1_TIM2_EV1 = ((uint8_t)0x02) /*!< TIM2 event 1*/, 
  WFE_CR1_TIM2_EV0 = ((uint8_t)0x01) /*!< TIM2 event 0*/, 

  WFE_CR2_EXTI_EVD = ((uint8_t)0x20) /*!< External interrupt event on Port D*/, 
  WFE_CR2_EXTI_EVB = ((uint8_t)0x10) /*!< External interrupt event on Port B*/, 
  WFE_CR2_EXTI_EV7 = ((uint8_t)0x08) /*!< External interrupt event 7*/, 
  WFE_CR2_EXTI_EV6 = ((uint8_t)0x04) /*!< External interrupt event 6*/, 
  WFE_CR2_EXTI_EV5 = ((uint8_t)0x02) /*!< External interrupt event 5*/, 
  WFE_CR2_EXTI_EV4 = ((uint8_t)0x01) /*!< External interrupt event 4*/, 
};
/**
  * @}
  */

/*----------------------------------------------------------------------------*/
/**
  * @brief Option Bytes (OPT)
  */
typedef struct OPT_struct
{
  __IO uint8_t OPT0_LOCKBYTE;   /*!< Option byte 0 */
  __IO uint8_t OPT1_PCODESIZE;  /*!< Option byte 1 */
  __IO uint8_t OPT2_BOOTSIZE;   /*!< Option byte 2 */
  __IO uint8_t OPT3_DATASIZE;   /*!< Option byte 3 */
}
OPT_TypeDef;

/**
  * @}
  */

/*----------------------------------------------------------------------------*/

/**
  * @brief Reset Controller (RST)
  */
typedef struct RST_struct
{
  __IO uint8_t CR;  /*!< Multiplex Reset Pad */
  __IO uint8_t SR;  /*!< Reset status register */

}
RST_TypeDef;
/**
  * @}
  */

/** @addtogroup RST_Registers_Reset_Value
  * @{
  */

enum {
  RST_CR_RESET_VALUE = ((uint8_t)0x00) /*!< Reset pin configuration register  1 reset value */, 
  RST_SR_RESET_VALUE = ((uint8_t)0x01) /*!< Reset status register 2 reset value */, 
};
/**
  * @}
  */

/** @addtogroup RST_Registers_Bits_Definition
  * @{
  */
enum {
  RST_SR_SWIMF = ((uint8_t)0x08) /*!< SWIM reset flag mask */, 
  RST_SR_ILLOPF = ((uint8_t)0x04) /*!< Illegal opcode reset flag mask */, 
  RST_SR_IWDGF = ((uint8_t)0x02) /*!< Independent Watchdog reset flag mask */, 
  RST_SR_PORF = ((uint8_t)0x01) /*!< Power On Reset (POR) flag mask */, 
};
/**
  * @}
  */
/*----------------------------------------------------------------------------ok*/

/**
  * @brief Serial Peripheral Interface (SPI)
  */
typedef struct SPI_struct
{
  __IO uint8_t CR1;    /*!< SPI control register 1 */
  __IO uint8_t CR2;    /*!< SPI control register 2 */
  __IO uint8_t ICR;    /*!< SPI interrupt control register */
  __IO uint8_t SR;     /*!< SPI status register */
  __IO uint8_t DR;     /*!< SPI data I/O register */
}
SPI_TypeDef;

/** @addtogroup SPI_Registers_Reset_Value
  * @{
  */

enum {
  SPI_CR1_RESET_VALUE = ((uint8_t)0x00) /*!< Control Register 1 reset value */, 
  SPI_CR2_RESET_VALUE = ((uint8_t)0x00) /*!< Control Register 2 reset value */, 
  SPI_ICR_RESET_VALUE = ((uint8_t)0x00) /*!< Interrupt Control Register reset value */, 
  SPI_SR_RESET_VALUE = ((uint8_t)0x02) /*!< Status Register reset value */, 
  SPI_DR_RESET_VALUE = ((uint8_t)0x00) /*!< Data Register reset value */, 
};
/**
  * @}
  */

/** @addtogroup SPI_Registers_Bits_Definition
  * @{
  */
enum {
  SPI_CR1_LSBFIRST = ((uint8_t)0x80) /*!< Frame format mask */, 
  SPI_CR1_SPE = ((uint8_t)0x40) /*!< Enable bits mask */, 
  SPI_CR1_BR = ((uint8_t)0x38) /*!< Baud rate control mask */, 
  SPI_CR1_MSTR = ((uint8_t)0x04) /*!< Master Selection mask */, 
  SPI_CR1_CPOL = ((uint8_t)0x02) /*!< Clock Polarity mask */, 
  SPI_CR1_CPHA = ((uint8_t)0x01) /*!< Clock Phase mask */, 

  SPI_CR2_BDM = ((uint8_t)0x80) /*!< Bi-directional data mode enable mask */, 
  SPI_CR2_BDOE = ((uint8_t)0x40) /*!< Output enable in bi-directional mode mask */, 
  SPI_CR2_RXONLY = ((uint8_t)0x04) /*!< Receive only mask */, 
  SPI_CR2_SSM = ((uint8_t)0x02) /*!< Software slave management mask */, 
  SPI_CR2_SSI = ((uint8_t)0x01) /*!< Internal slave select mask */, 

  SPI_ICR_TXIE = ((uint8_t)0x80) /*!< Tx buffer empty interrupt enable mask */, 
  SPI_ICR_RXIE = ((uint8_t)0x40) /*!< Rx buffer empty interrupt enable mask */, 
  SPI_ICR_ERRIE = ((uint8_t)0x20) /*!< Error interrupt enable mask */, 
  SPI_ICR_WKIE = ((uint8_t)0x10) /*!< Wake-up interrupt enable mask */, 

  SPI_SR_BSY = ((uint8_t)0x80) /*!< Busy flag */, 
  SPI_SR_OVR = ((uint8_t)0x40) /*!< Overrun flag */, 
  SPI_SR_MODF = ((uint8_t)0x20) /*!< Mode fault */, 
  SPI_SR_WKUP = ((uint8_t)0x08) /*!< Wake-Up flag */, 
  SPI_SR_TXE = ((uint8_t)0x02) /*!< Transmit buffer empty */, 
  SPI_SR_RXNE = ((uint8_t)0x01) /*!< Receive buffer not empty */, 
};
/**
  * @}
  */

/*----------------------------------------------------------------------------*/

/**
  * @brief 16 bit timer with complementary PWM outputs (TIM2 & TIM3)
  */
typedef struct TIM_struct
{
  __IO uint8_t CR1;  /*!< control register 1   */
  __IO uint8_t CR2;  /*!< control register 2   */
  __IO uint8_t SMCR;  /*!< Synchro mode control register */
  __IO uint8_t ETR;  /*!< external trigger register */
  __IO uint8_t IER;  /*!< interrupt enable register*/
  __IO uint8_t SR1;  /*!< status register 1   */
  __IO uint8_t SR2;  /*!< status register 2   */
  __IO uint8_t EGR;  /*!< event generation register */
  __IO uint8_t CCMR1;  /*!< CC mode register 1      */
  __IO uint8_t CCMR2;  /*!< CC mode register 2      */
  __IO uint8_t CCER1;  /*!< CC enable register 1     */
  __IO uint8_t CNTRH;  /*!< counter high       */
  __IO uint8_t CNTRL;  /*!< counter low       */
  __IO uint8_t PSCR;  /*!< prescaler     */
  __IO uint8_t ARRH;  /*!< auto-reload register high  */
  __IO uint8_t ARRL;  /*!< auto-reload register low    */
  __IO uint8_t CCR1H;  /*!< capture/compare register 1 high   */
  __IO uint8_t CCR1L;  /*!< capture/compare register 1 low     */
  __IO uint8_t CCR2H;  /*!< capture/compare register 2 high   */
  __IO uint8_t CCR2L;  /*!< capture/compare register 2 low     */
  __IO uint8_t BKR;  /*!< Break Register */
  __IO uint8_t OISR;  /*!< Output idle register */
}
TIM_TypeDef;

/** @addtogroup TIM2_TIM3_Registers_Reset_Value
  * @{
  */
enum {
  TIM_CR1_RESET_VALUE = ((uint8_t)0x00), 
  TIM_CR2_RESET_VALUE = ((uint8_t)0x00), 
  TIM_SMCR_RESET_VALUE = ((uint8_t)0x00), 
  TIM_ETR_RESET_VALUE = ((uint8_t)0x00), 
  TIM_IER_RESET_VALUE = ((uint8_t)0x00), 
  TIM_SR1_RESET_VALUE = ((uint8_t)0x00), 
  TIM_SR2_RESET_VALUE = ((uint8_t)0x00), 
  TIM_EGR_RESET_VALUE = ((uint8_t)0x00), 
  TIM_CCMR1_RESET_VALUE = ((uint8_t)0x00), 
  TIM_CCMR2_RESET_VALUE = ((uint8_t)0x00), 

  TIM_CCER1_RESET_VALUE = ((uint8_t)0x00), 

  TIM_CNTRH_RESET_VALUE = ((uint8_t)0x00), 
  TIM_CNTRL_RESET_VALUE = ((uint8_t)0x00), 

  TIM_PSCR_RESET_VALUE = ((uint8_t)0x00), 
  TIM_ARRH_RESET_VALUE = ((uint8_t)0xFF), 
  TIM_ARRL_RESET_VALUE = ((uint8_t)0xFF), 

  TIM_CCR1H_RESET_VALUE = ((uint8_t)0x00), 
  TIM_CCR1L_RESET_VALUE = ((uint8_t)0x00), 
  TIM_CCR2H_RESET_VALUE = ((uint8_t)0x00), 
  TIM_CCR2L_RESET_VALUE = ((uint8_t)0x00), 

  TIM_BKR_RESET_VALUE = ((uint8_t)0x00), 
  TIM_OISR_RESET_VALUE = ((uint8_t)0x00), 
};
/**
  * @}
  */

/** @addtogroup TIM2_TIM3_Registers_Bits_Definition
  * @{
  */
enum {
/* CR1*/
  TIM_CR1_ARPE = ((uint8_t)0x80) /*!< Auto-Reload Preload Enable Mask. */, 
  TIM_CR1_CMS = ((uint8_t)0x60) /*!< Center-aligned Mode Selection Mask. */, 
  TIM_CR1_DIR = ((uint8_t)0x10) /*!< Direction Mask. */, 
  TIM_CR1_OPM = ((uint8_t)0x08) /*!< One Pulse Mode Mask. */, 
  TIM_CR1_URS = ((uint8_t)0x04) /*!< Update Request Source Mask. */, 
  TIM_CR1_UDIS = ((uint8_t)0x02) /*!< Update Disable Mask. */, 
  TIM_CR1_CEN = ((uint8_t)0x01) /*!< Counter Enable Mask. */, 

/* CR2*/
  TIM_CR2_TI1S = ((uint8_t)0x80) /*!< TI1S Selection Mask. */, 
  TIM_CR2_MMS = ((uint8_t)0x70) /*!< MMS Selection Mask. */, 

/* SMCR*/
  TIM_SMCR_MSM = ((uint8_t)0x80) /*!< Master/Slave Mode Mask. */, 
  TIM_SMCR_TS = ((uint8_t)0x70) /*!< Trigger Selection Mask. */, 
  TIM_SMCR_SMS = ((uint8_t)0x07) /*!< Slave Mode Selection Mask. */, 


/* ETR*/
  TIM_ETR_ECE = ((uint8_t)0x40)/*!< External Clock Mask. */, 
  TIM_ETR_ETP = ((uint8_t)0x80) /*!< External Trigger Polarity Mask. */, 
  TIM_ETR_ETPS = ((uint8_t)0x30) /*!< External Trigger Prescaler Mask. */, 
  TIM_ETR_ETF = ((uint8_t)0x0F) /*!< External Trigger Filter Mask. */, 


/* IER*/
  TIM_IER_BIE = ((uint8_t)0x80) /*!< Break Interrupt Enable Mask. */, 
  TIM_IER_TIE = ((uint8_t)0x40) /*!< Trigger Interrupt Enable Mask. */, 
  TIM_IER_CC2IE = ((uint8_t)0x04) /*!< Capture/Compare 2 Interrupt Enable Mask. */, 
  TIM_IER_CC1IE = ((uint8_t)0x02) /*!< Capture/Compare 1 Interrupt Enable Mask. */, 
  TIM_IER_UIE = ((uint8_t)0x01) /*!< Update Interrupt Enable Mask. */, 

/* SR1*/
  TIM_SR1_BIF = ((uint8_t)0x80) /*!< Break Interrupt Flag Mask. */, 
  TIM_SR1_TIF = ((uint8_t)0x40) /*!< Trigger Interrupt Flag Mask. */, 
  TIM_SR1_CC2IF = ((uint8_t)0x04) /*!< Capture/Compare 2 Interrupt Flag Mask. */, 
  TIM_SR1_CC1IF = ((uint8_t)0x02) /*!< Capture/Compare 1 Interrupt Flag Mask. */, 
  TIM_SR1_UIF = ((uint8_t)0x01) /*!< Update Interrupt Flag Mask. */, 

/* SR2*/
  TIM_SR2_CC2OF = ((uint8_t)0x04) /*!< Capture/Compare 2 Overcapture Flag Mask. */, 
  TIM_SR2_CC1OF = ((uint8_t)0x02) /*!< Capture/Compare 1 Overcapture Flag Mask. */, 

/*EGR*/
  TIM_EGR_BG = ((uint8_t)0x80) /*!< Break Generation Mask. */, 
  TIM_EGR_TG = ((uint8_t)0x40) /*!< Trigger Generation Mask. */, 
  TIM_EGR_CC2G = ((uint8_t)0x04) /*!< Capture/Compare 2 Generation Mask. */, 
  TIM_EGR_CC1G = ((uint8_t)0x02) /*!< Capture/Compare 1 Generation Mask. */, 
  TIM_EGR_UG = ((uint8_t)0x01) /*!< Update Generation Mask. */, 

/*CCMR*/
  TIM_CCMR_ICxPSC = ((uint8_t)0x0C) /*!< Input Capture x Prescaler Mask. */, 
  TIM_CCMR_ICxF = ((uint8_t)0xF0) /*!< Input Capture x Filter Mask. */, 
  TIM_CCMR_OCM = ((uint8_t)0x70) /*!< Output Compare x Mode Mask. */, 
  TIM_CCMR_OCxPE = ((uint8_t)0x08) /*!< Output Compare x Preload Enable Mask. */, 
  TIM_CCMR_OCxFE = ((uint8_t)0x04) /*!< Output Compare x Fast Enable Mask. */, 
  TIM_CCMR_CCxS = ((uint8_t)0x03) /*!< Capture/Compare x Selection Mask. */, 
  TIM_CCMR_TIxDirect_Set = ((uint8_t)0x01), 

/*CCER1*/
  TIM_CCER1_CC2P = ((uint8_t)0x20) /*!< Capture/Compare 2 output Polarity Mask. */, 
  TIM_CCER1_CC2E = ((uint8_t)0x10) /*!< Capture/Compare 2 output enable Mask. */, 
  TIM_CCER1_CC1P = ((uint8_t)0x02) /*!< Capture/Compare 1 output Polarity Mask. */, 
  TIM_CCER1_CC1E = ((uint8_t)0x01) /*!< Capture/Compare 1 output enable Mask. */, 

/*CNTR*/
  TIM_CNTRH_CNT = ((uint8_t)0xFF) /*!< Counter Value (MSB) Mask. */, 
  TIM_CNTRL_CNT = ((uint8_t)0xFF) /*!< Counter Value (LSB) Mask. */, 

/*PSCR*/
  TIM_PSC_PSC = ((uint8_t)0x03) /*!< Prescaler Value  Mask. */, 

/*ARR*/
  TIM_ARRH_ARR = ((uint8_t)0xFF) /*!< Autoreload Value (MSB) Mask. */, 
  TIM_ARRL_ARR = ((uint8_t)0xFF) /*!< Autoreload Value (LSB) Mask. */, 


/*CCR1*/
  TIM_CCR1H_CCR1 = ((uint8_t)0xFF) /*!< Capture/Compare 1 Value (MSB) Mask. */, 
  TIM_CCR1L_CCR1 = ((uint8_t)0xFF) /*!< Capture/Compare 1 Value (LSB) Mask. */, 

/*CCR2*/
  TIM_CCR2H_CCR2 = ((uint8_t)0xFF) /*!< Capture/Compare 2 Value (MSB) Mask. */, 
  TIM_CCR2L_CCR2 = ((uint8_t)0xFF) /*!< Capture/Compare 2 Value (LSB) Mask. */, 


/*BKR*/
  TIM_BKR_MOE = ((uint8_t)0x80) /*!< Main Output Enable Mask. */, 
  TIM_BKR_AOE = ((uint8_t)0x40) /*!< Automatic Output Enable Mask. */, 
  TIM_BKR_BKP = ((uint8_t)0x20) /*!< Break Polarity Mask. */, 
  TIM_BKR_BKE = ((uint8_t)0x10) /*!< Break Enable Mask. */, 
  TIM_BKR_OSSI = ((uint8_t)0x04) /*!< Off-State Selection for Idle mode Mask. */, 
  TIM_BKR_LOCK = ((uint8_t)0x03) /*!<Lock Configuration Mask. */, 

/*OISR*/
  TIM_OISR_OIS2 = ((uint8_t)0x04) /*!< Output Idle state 2 (OC2 output) Mask. */, 
  TIM_OISR_OIS1 = ((uint8_t)0x01) /*!< Output Idle state 1 (OC1 output) Mask. */, 
};
/**
  * @}
  */


/*----------------------------------------------------------------------------*/

/**
  * @brief 8-bit system or Low End Small Timer (TIM4)
  */
typedef struct TIM4_struct
{
  __IO uint8_t CR1;  /*!< control register 1 */
  __IO uint8_t CR2;  /*!< control register 2 */
  __IO uint8_t SMCR;  /*!< Synchro mode control register */
  __IO uint8_t IER;  /*!< interrupt enable register  */
  __IO uint8_t SR1;  /*!< status register 1    */
  __IO uint8_t EGR;  /*!< event generation register */
  __IO uint8_t CNTR;  /*!< counter register  */
  __IO uint8_t PSCR;  /*!< prescaler register */
  __IO uint8_t ARR;  /*!< auto-reload register */
}
TIM4_TypeDef;
/** @addtogroup TIM4_Registers_Reset_Value
  * @{
  */
enum {
  TIM4_CR1_RESET_VALUE = ((uint8_t)0x00), 
  TIM4_CR2_RESET_VALUE = ((uint8_t)0x00), 
  TIM4_SMCR_RESET_VALUE = ((uint8_t)0x00), 
  TIM4_IER_RESET_VALUE = ((uint8_t)0x00), 
  TIM4_SR1_RESET_VALUE = ((uint8_t)0x00), 
  TIM4_EGR_RESET_VALUE = ((uint8_t)0x00), 
  TIM4_CNTR_RESET_VALUE = ((uint8_t)0x00), 
  TIM4_PSCR_RESET_VALUE = ((uint8_t)0x00), 
  TIM4_ARR_RESET_VALUE = ((uint8_t)0xFF), 
};
/**
* @}
*/

/** @addtogroup TIM4_Registers_Bits_Definition
  * @{
  */
enum {
/* CR1*/
  TIM4_CR1_ARPE = ((uint8_t)0x80) /*!< Auto-Reload Preload Enable Mask. */, 
  TIM4_CR1_OPM = ((uint8_t)0x08) /*!< One Pulse Mode Mask. */, 
  TIM4_CR1_URS = ((uint8_t)0x04) /*!< Update Request Source Mask. */, 
  TIM4_CR1_UDIS = ((uint8_t)0x02) /*!< Update Disable Mask. */, 
  TIM4_CR1_CEN = ((uint8_t)0x01) /*!< Counter Enable Mask. */, 

/* CR2*/
  TIM4_CR2_MMS = ((uint8_t)0x70) /*!< MMS Selection Mask. */, 

/* SMCR*/
  TIM4_SMCR_TS = ((uint8_t)0x70) /*!< Trigger Selection Mask. */, 
  TIM4_SMCR_SMS = ((uint8_t)0x07) /*!< Slave Mode Selection Mask. */, 
  TIM4_SMCR_MSM = ((uint8_t)0x80) /*!< Master/Slave Mode Mask. */, 

/* IER*/
  TIM4_IER_TIE = ((uint8_t)0x40) /*!< Trigger Interrupt Enable Mask. */, 
  TIM4_IER_UIE = ((uint8_t)0x01) /*!< Update Interrupt Enable Mask. */, 

/* SR1*/
  TIM4_SR1_TIF = ((uint8_t)0x40) /*!< Trigger Interrupt Flag Mask. */, 
  TIM4_SR1_UIF = ((uint8_t)0x01) /*!< Update Interrupt Flag Mask. */, 

/* EGR*/
  TIM4_EGR_TG = ((uint8_t)0x40) /*!< Trigger Generation Mask. */, 
  TIM4_EGR_UG = ((uint8_t)0x01) /*!< Update Generation Mask. */, 

/* CNTR*/
  TIM4_CNTR_CNT = ((uint8_t)0xFF) /*!<Counter Value (LSB) Mask. */, 

/* PSCR*/
  TIM4_PSCR_PSC = ((uint8_t)0x0F) /*!<Prescaler Value  Mask. */, 

/* ARR*/
  TIM4_ARR_ARR = ((uint8_t)0xFF) /*!<Autoreload Value Mask. */, 
};
/**
  * @}
  */

/*----------------------------------------------------------------------------*/

/**
  * @brief USART
  */
typedef struct USART_struct
{
  __IO uint8_t SR;  /*!<  USART status register  */
  __IO uint8_t DR;  /*!<  USART data register     */
  __IO uint8_t BRR1;  /*!<  USART baud rate register   */
  __IO uint8_t BRR2;  /*!<  USART DIV mantissa[11:8] SCIDIV fraction */
  __IO uint8_t CR1;  /*!<  USART control register 1     */
  __IO uint8_t CR2;  /*!<  USART control register 2     */
  __IO uint8_t CR3;  /*!<  USART control register 3      */
  __IO uint8_t CR4;  /*!< USART control register 4      */
  __IO uint8_t CR5;  /*!<  USART control register 5       */
  __IO uint8_t GTR;  /*!<  USART guard time register     */
  __IO uint8_t PSCR;  /*!<  USART prescaler register     */
}
USART_TypeDef;


/** @addtogroup USART_Registers_Reset_Value
  * @{
  */
enum {
  USART_SR_RESET_VALUE = ((uint8_t)0xC0), 
  USART_BRR1_RESET_VALUE = ((uint8_t)0x00), 
  USART_BRR2_RESET_VALUE = ((uint8_t)0x00), 
  USART_CR1_RESET_VALUE = ((uint8_t)0x00), 
  USART_CR2_RESET_VALUE = ((uint8_t)0x00), 
  USART_CR3_RESET_VALUE = ((uint8_t)0x00), 
  USART_CR4_RESET_VALUE = ((uint8_t)0x00), 
};
/**
  * @}
  */

/** @addtogroup USART_Registers_Bits_Definition
  * @{
  */
enum {
  USART_SR_TXE = ((uint8_t)0x80) /*!< Transmit Data Register Empty Mask         */, 
  USART_SR_TC = ((uint8_t)0x40) /*!< Transmission Complete Mask                */, 
  USART_SR_RXNE = ((uint8_t)0x20) /*!< Read Data Register Not Empty Mask         */, 
  USART_SR_IDLE = ((uint8_t)0x10) /*!< IDLE line detected Mask                   */, 
  USART_SR_OR = ((uint8_t)0x08) /*!< OverRun error Mask                        */, 
  USART_SR_NF = ((uint8_t)0x04) /*!< Noise Flag Mask                           */, 
  USART_SR_FE = ((uint8_t)0x02) /*!< Framing Error Mask                        */, 
  USART_SR_PE = ((uint8_t)0x01) /*!< Parity Error Mask                         */, 
  USART_BRR1_DIVM = ((uint8_t)0xFF) /*!< LSB mantissa of USARTDIV [7:0] Mask       */, 
  USART_BRR2_DIVM = ((uint8_t)0xF0) /*!< MSB mantissa of USARTDIV [11:8] Mask      */, 
  USART_BRR2_DIVF = ((uint8_t)0x0F) /*!< Fraction bits of USARTDIV [3:0] Mask      */, 
  USART_CR1_R8 = ((uint8_t)0x80) /*!< Receive Data bit 8                        */, 
  USART_CR1_T8 = ((uint8_t)0x40) /*!< Transmit data bit 8                       */, 
  USART_CR1_USARTD = ((uint8_t)0x20)  /*!< USART Disable (for low power consumption) */, 
  USART_CR1_M = ((uint8_t)0x10) /*!< Word length Mask                          */, 
  USART_CR1_WAKE = ((uint8_t)0x08) /*!< Wake-up method Mask                       */, 
  USART_CR1_PCEN = ((uint8_t)0x04) /*!< Parity Control Enable Mask                */, 
  USART_CR1_PS = ((uint8_t)0x02) /*!< USART Parity Selection                 */, 
  USART_CR1_PIEN = ((uint8_t)0x01) /*!< USART Parity Interrupt Enable Mask        */, 
  USART_CR2_TIEN = ((uint8_t)0x80) /*!< Transmitter Interrupt Enable Mask         */, 
  USART_CR2_TCIEN = ((uint8_t)0x40) /*!< TransmissionComplete Interrupt Enable Mask*/, 
  USART_CR2_RIEN = ((uint8_t)0x20) /*!< Receiver Interrupt Enable Mask            */, 
  USART_CR2_ILIEN = ((uint8_t)0x10) /*!< IDLE Line Interrupt Enable Mask           */, 
  USART_CR2_TEN = ((uint8_t)0x08) /*!< Transmitter Enable Mask                   */, 
  USART_CR2_REN = ((uint8_t)0x04) /*!< Receiver Enable Mask                      */, 
  USART_CR2_RWU = ((uint8_t)0x02) /*!< Receiver Wake-Up Mask                     */, 
  USART_CR2_SBK = ((uint8_t)0x01) /*!< Send Break Mask                     */, 
  USART_CR3_Reserved = ((uint8_t)0x80) /*!< RESERVED bit Mask                        */, 
  USART_CR3_STOP = ((uint8_t)0x30) /*!< STOP bits [1:0] Mask                      */, 
  USART_CR3_CLKEN = ((uint8_t)0x08) /*!< Clock Enable Mask                         */, 
  USART_CR3_CPOL = ((uint8_t)0x04) /*!< Clock Polarity Mask                       */, 
  USART_CR3_CPHA = ((uint8_t)0x02) /*!< Clock Phase Mask                          */, 
  USART_CR3_LBCL = ((uint8_t)0x01) /*!< Last Bit Clock pulse Mask                 */, 
  USART_CR4_Reserved = ((uint8_t)0x80) /*!< RESERVED bit Mask                        */, 
  USART_CR4_ADD = ((uint8_t)0x0F) /*!< Address of the USART node Mask            */, 
};
/**
  * @}
  */
/*----------------------------------------------------------------------------*/

/******************************************************************************/
/*                          Peripherals Base Address                          */
/******************************************************************************/


enum {
  GPIOA_BaseAddress = 0x5000, 
  GPIOB_BaseAddress = 0x5005, 
  GPIOC_BaseAddress = 0x500A, 
  GPIOD_BaseAddress = 0x500F, 

  FLASH_BaseAddress = 0x5050, 

  USART_BaseAddress = 0x5230, 

  TIM2_BaseAddress = 0x5250, 
  TIM3_BaseAddress = 0x5280, 
  TIM4_BaseAddress = 0x52E0, 

  IRTIM_BaseAddress = 0x52FF, 

  COMP_BaseAddress = 0x5300, 

  EXTI_BaseAddress = 0x50A0, 

  WFE_BaseAddress = 0x50A6, 

  RST_BaseAddress = 0x50B0, 

  CLK_BaseAddress = 0x50C0, 

  IWDG_BaseAddress = 0x50E0, 

  AWU_BaseAddress = 0x50F0, 

  BEEP_BaseAddress = 0x50F3, 

  SPI_BaseAddress = 0x5200, 

  I2C_BaseAddress = 0x5210, 

  OPT_BaseAddress = 0x4800, 

  ITC_BaseAddress = 0x7F70, 

  DM_BaseAddress = 0x7F90, 
};

/******************************************************************************/
/*                          Peripherals declarations                          */
/******************************************************************************/


#define GPIOA ((GPIO_TypeDef *) GPIOA_BaseAddress)
#define GPIOB ((GPIO_TypeDef *) GPIOB_BaseAddress)
#define GPIOC ((GPIO_TypeDef *) GPIOC_BaseAddress)
#define GPIOD ((GPIO_TypeDef *) GPIOD_BaseAddress)
#define FLASH ((FLASH_TypeDef *) FLASH_BaseAddress)
#define EXTI ((EXTI_TypeDef *) EXTI_BaseAddress)
#define RST ((RST_TypeDef *) RST_BaseAddress)
#define CLK ((CLK_TypeDef *) CLK_BaseAddress)
#define IWDG ((IWDG_TypeDef *) IWDG_BaseAddress)
#define WFE ((WFE_TypeDef *) WFE_BaseAddress)
#define AWU ((AWU_TypeDef *) AWU_BaseAddress)
#define BEEP ((BEEP_TypeDef *) BEEP_BaseAddress)
#define SPI ((SPI_TypeDef *) SPI_BaseAddress)
#define I2C ((I2C_TypeDef *) I2C_BaseAddress)
#define USART ((USART_TypeDef *) USART_BaseAddress)
#define TIM2 ((TIM_TypeDef *) TIM2_BaseAddress)
#define TIM3 ((TIM_TypeDef *) TIM3_BaseAddress)
#define TIM4 ((TIM4_TypeDef *) TIM4_BaseAddress)
#define IRTIM ((IRTIM_TypeDef *) IRTIM_BaseAddress)
#define ITC ((ITC_TypeDef *) ITC_BaseAddress)
#define CFG ((CFG_TypeDef *) CFG_BaseAddress)
#define DM ((DM_TypeDef *) DM_BaseAddress)
#define COMP ((COMP_TypeDef *) COMP_BaseAddress)
#define OPT ((OPT_TypeDef *) OPT_BaseAddress)



#ifdef USE_STDPERIPH_DRIVER
 #include "stm8l10x_conf.h"
#endif

/* Exported macro --------------------------------------------------------------*/

/*============================== Interrupts ====================================*/
#ifdef _RAISONANCE_
 #include <intrins.h>
 static inline void enableInterrupts() { ::_rim_(); } /* enable interrupts */
 static inline void disableInterrupts() { ::_sim_(); } /* disable interrupts */
 static inline void rim() { ::_rim_(); } /* enable interrupts */
 static inline void sim() { ::_sim_(); } /* disable interrupts */
 static inline void nop() { ::_nop_(); } /* No Operation */
 static inline void trap() { ::_trap_(); } /* Trap (soft IT) */
 static inline void wfi() { ::_wfi_(); } /* Wait For Interrupt */
 static inline void wfe() { ::_wfe_(); } /* Wait For Event */
 static inline void halt() { ::_halt_(); } /* Halt */
#elif defined(_COSMIC_)
 static inline void enableInterrupts() { {_asm("rim\n");}; } /* enable interrupts */
 static inline void disableInterrupts() { {_asm("sim\n");}; } /* disable interrupts */
 static inline void rim() { {_asm("rim\n");}; } /* enable interrupts */
 static inline void sim() { {_asm("sim\n");}; } /* disable interrupts */
 static inline void nop() { {_asm("nop\n");}; } /* No Operation */
 static inline void trap() { {_asm("trap\n");}; } /* Trap (soft IT) */
 static inline void wfi() { {_asm("wfi\n");}; } /* Wait For Interrupt */
 static inline void wfe() { {_asm("wfe\n");}; } /* Wait For Event */
 static inline void halt() { {_asm("halt\n");}; } /* Halt */
#else /*_IAR_*/
 #include <intrinsics.h>
 static inline void enableInterrupts() { __enable_interrupt(); } /* enable interrupts */
 static inline void disableInterrupts() { __disable_interrupt(); } /* disable interrupts */
 static inline void rim() { __enable_interrupt(); } /* enable interrupts */
 static inline void sim() { __disable_interrupt(); } /* disable interrupts */
 static inline void nop() { __no_operation(); } /* No Operation */
 static inline void trap() { __trap(); } /* Trap (soft IT) */
 static inline void wfi() { __wait_for_interrupt(); } /* Wait For Interrupt */
 static inline void wfe() { __wait_for_event(); } /* Wait For Event */
 static inline void halt() { __halt(); } /* Halt */
#endif /*_RAISONANCE_*/

/*============================== Interrupt vector Handling ========================*/

#ifdef _COSMIC_
#define INTERRUPT_HANDLER(a,b) @far @interrupt void a(void)
#define INTERRUPT_HANDLER_TRAP(a) void @far @interrupt a(void)
#endif

#ifdef _RAISONANCE_
#define INTERRUPT_HANDLER(a,b) void a(void) interrupt b
#define INTERRUPT_HANDLER_TRAP(a) void a(void) trap
#endif

#ifdef _IAR_
#define STRINGVECTOR(x) #x
#define VECTOR_ID(x) STRINGVECTOR( vector = (x) )
#define INTERRUPT_HANDLER( a, b )  \
  _Pragma( VECTOR_ID( (b)+2 ) )        \
  __interrupt void (a)( void )
#define INTERRUPT_HANDLER_TRAP(a) \
  _Pragma( VECTOR_ID( 1 ) ) \
  __interrupt void (a) (void)
#endif

/*============================== Interrupt Handler declaration ========================*/

#ifdef _COSMIC_
#define INTERRUPT @far @interrupt
#endif

#ifdef _IAR_
#define INTERRUPT __interrupt
#endif


#if 0
/*============================== Handling bits ====================================*/
/*-----------------------------------------------------------------------------
Method : I
Description : Handle the bit from the character variables.
Comments :    The different parameters of commands are
              - VAR : Name of the character variable where the bit is located.
              - Place : Bit position in the variable (7 6 5 4 3 2 1 0)
              - Value : Can be 0 (reset bit) or not 0 (set bit)
              The "MskBit" command allows to select some bits in a source
              variables and copy it in a destination var (return the value).
              The "ValBit" command returns the value of a bit in a char
              variable: the bit is reseted if it returns 0 else the bit is set.
              This method generates not an optimised code yet.
-----------------------------------------------------------------------------*/
#define SetBit(VAR,Place)         ( (VAR) |= (uint8_t)((uint8_t)1<<(uint8_t)(Place)) )
#define ClrBit(VAR,Place)         ( (VAR) &= (uint8_t)((uint8_t)((uint8_t)1<<(uint8_t)(Place))^(uint8_t)0xFF) )

#define ChgBit(VAR,Place)         ( (VAR) ^= (uint8_t)((uint8_t)1<<(uint8_t)(Place)) )
#define AffBit(VAR,Place,Value)   ((Value) ? \
                                   ((VAR) |= ((uint8_t)1<<(Place))) : \
                                   ((VAR) &= (((uint8_t)1<<(Place))^(uint8_t)0xFF)))
#define MskBit(Dest,Msk,Src)      ( (Dest) = ((Msk) & (Src)) | ((~(Msk)) & (Dest)) )

#define ValBit(VAR,Place)         ((uint8_t)(VAR) & (uint8_t)((uint8_t)1<<(uint8_t)(Place)))

#define BYTE_0(n)                 ((uint8_t)((n) & (uint8_t)0xFF))        /*!< Returns the low byte of the 32-bit value */
#define BYTE_1(n)                 ((uint8_t)(BYTE_0((n) >> (uint8_t)8)))  /*!< Returns the second byte of the 32-bit value */
#define BYTE_2(n)                 ((uint8_t)(BYTE_0((n) >> (uint8_t)16))) /*!< Returns the third byte of the 32-bit value */
#define BYTE_3(n)                 ((uint8_t)(BYTE_0((n) >> (uint8_t)24))) /*!< Returns the high byte of the 32-bit value */

/*============================== Assert Macros ====================================*/
#define IS_STATE_VALUE(STATE) \
  (((STATE) == SET) || \
   ((STATE) == RESET))

/*-----------------------------------------------------------------------------
Method : II
Description : Handle directly the bit.
Comments :    The idea is to handle directly with the bit name. For that, it is
              necessary to have RAM area descriptions (example: HW register...)
              and the following command line for each area.
              This method generates the most optimized code.
-----------------------------------------------------------------------------*/

#define AREA 0x00     /* The area of bits begins at address 0x10. */

#define BitClr(BIT)  ( *((unsigned char *) (AREA+(BIT)/8)) &= (~(1<<(7-(BIT)%8))) )
#define BitSet(BIT)  ( *((unsigned char *) (AREA+(BIT)/8)) |= (1<<(7-(BIT)%8)) )
#define BitVal(BIT)  ( *((unsigned char *) (AREA+(BIT)/8)) & (1<<(7-(BIT)%8)) )

#endif

#endif /* __STM8L10x_H */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

#ifdef __cplusplus
} //# namespace STM8L10x_StdPeriph_Lib
#endif

