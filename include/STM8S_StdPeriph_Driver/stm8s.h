/**
	******************************************************************************
	* @file    stm8s.h
	* @author  MCD Application Team
	* @version V2.3.0
	* @date    16-June-2017
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

/*
	used regexpes

	#define AWU ((AWU_TypeDef *) AWU_BaseAddress)
	->
	static __IO AWU_TypeDef* const AWU = ((AWU_TypeDef *) AWU_BaseAddress);
	#define STM8S_STDPERIPH_LIB__AWU (STM8S_STDPERIPH_LIB__NS_PREFIX AWU)
	
	^(\s*)\#define\s+(\w+)\s+(\(\(\s*(\w+)\s+\*\s*\)\s*\w+\))\s*$
	\1static __IO \4* const \2 = \3;\n\1#define STM8S_STDPERIPH_LIB__\2 (STM8S_STDPERIPH_LIB__NS_PREFIX \2)
		
	#define TIM4_ARR_ARR ((uint8_t)0xFF)
	->
	enum {
		TIM4_ARR_ARR = ((uint8_t)0xFF),
	};
	
	^(\s*)\#define\s+(\w+)\s+(.+)$
	\1\t\2 = \3,
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM8S_H
#define __STM8S_H

// #define __STDC_CONSTANT_MACROS
#include <stdint.h>


#ifdef __cplusplus
namespace STM8S_StdPeriph_Lib {
#define STM8S_STDPERIPH_LIB__NS_PREFIX ::STM8S_StdPeriph_Lib::
#else
#define STM8S_STDPERIPH_LIB__NS_PREFIX 
#endif


/** @addtogroup STM8S_StdPeriph_Driver
	* @{
	*/

/* Uncomment the line below according to the target STM8S or STM8A device used in your
	 application. */

 /* #define STM8S208 */      /*!< STM8S High density devices with CAN */
 /* #define STM8S207 */      /*!< STM8S High density devices without CAN */
 /* #define STM8S007 */      /*!< STM8S Value Line High density devices */
 /* #define STM8AF52Ax */    /*!< STM8A High density devices with CAN */
 /* #define STM8AF62Ax */    /*!< STM8A High density devices without CAN */
 /* #define STM8S105 */      /*!< STM8S Medium density devices */
 /* #define STM8S005 */      /*!< STM8S Value Line Medium density devices */
 /* #define STM8AF626x */    /*!< STM8A Medium density devices */
 /* #define STM8AF622x */    /*!< STM8A Low density devices */
 /* #define STM8S103 */      /*!< STM8S Low density devices */
 /* #define STM8S003 */      /*!< STM8S Value Line Low density devices */
 /* #define STM8S903 */      /*!< STM8S Low density devices */
 /* #define STM8S001 */      /*!< STM8S Value Line Low denisty devices */

/*   Tip: To avoid modifying this file each time you need to switch between these
				devices, you can define the device in your toolchain compiler preprocessor.

	- High-Density STM8A devices are the STM8AF52xx STM8AF6269/8x/Ax,
		STM8AF51xx, and STM8AF6169/7x/8x/9x/Ax microcontrollers where the Flash memory
		density ranges between 32 to 128 Kbytes
	- Medium-Density STM8A devices are the STM8AF622x/4x, STM8AF6266/68,
		STM8AF612x/4x, and STM8AF6166/68 microcontrollers where the Flash memory
		density ranges between 8 to 32 Kbytes
	- High-Density STM8S devices are the STM8S207xx, STM8S007 and STM8S208xx microcontrollers
		where the Flash memory density ranges between 32 to 128 Kbytes.
	- Medium-Density STM8S devices are the STM8S105x and STM8S005 microcontrollers
		where the Flash memory density ranges between 16 to 32-Kbytes.
	- Low-Density STM8A devices are the STM8AF622x microcontrollers where the Flash
		density is 8 Kbytes.
	- Low-Density STM8S devices are the STM8S103xx, STM8S003, STM8S903xx and STM8S001 microcontrollers
		where the Flash density is 8 Kbytes. */

#if !defined (STM8S208) && !defined (STM8S207) && !defined (STM8S105) && \
		!defined (STM8S103) && !defined (STM8S903) && !defined (STM8AF52Ax) && \
		!defined (STM8AF62Ax) && !defined (STM8AF626x) && !defined (STM8S007) && \
		!defined (STM8S003)&& !defined (STM8S005) && !defined(STM8S001) && !defined (STM8AF622x)
 #error "Please select first the target STM8S/A device used in your application (in stm8s.h file)"
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

/**
	* @brief  In the following line adjust the value of External High Speed oscillator (HSE)
	 used in your application

	 Tip: To avoid modifying this file each time you need to use different HSE, you
				can define the HSE value in your toolchain compiler preprocessor.
	*/
#if !defined  HSE_Value
 #if defined (STM8S208) || defined (STM8S207) || defined (STM8S007) || defined (STM8AF52Ax) || \
		 defined (STM8AF62Ax) || defined (STM8AF622x)
	 enum HSE_VALUE_enum { HSE_VALUE = ((uint32_t)24000000) /* Value of the External oscillator in Hz*/, };
 #else
		enum HSE_VALUE_enum { HSE_VALUE = ((uint32_t)16000000) /* Value of the External oscillator in Hz*/, };
 #endif /* STM8S208 || STM8S207 || STM8S007 || STM8AF62Ax || STM8AF52Ax || STM8AF622x */
#endif /* HSE_Value */

/**
	* @brief  Definition of Device on-chip RC oscillator frequencies
	*/
enum xSI_VALUE_enum {
	HSI_VALUE = ((uint32_t)16000000) /*!< Typical Value of the HSI in Hz */,
	LSI_VALUE = ((uint32_t)128000)   /*!< Typical Value of the LSI in Hz */,
};

#ifdef _COSMIC_
 #define FAR  @far
 #define NEAR @near
 #define TINY @tiny
 #define STM8S_STDPERIPH_LIB__EEPROM @eeprom
 #define STM8S_STDPERIPH_LIB__CONST  const
#elif defined (_RAISONANCE_) /* __RCST7__ */
 #define FAR  far
 #define NEAR data
 #define TINY page0
 #define STM8S_STDPERIPH_LIB__EEPROM eeprom
 #define STM8S_STDPERIPH_LIB__CONST  code
 #if defined (STM8S208) || defined (STM8S207) || defined (STM8S007) || defined (STM8AF52Ax) || \
		 defined (STM8AF62Ax)
	 /*!< Used with memory Models for code higher than 64K */
	#define STM8S_STDPERIPH_LIB__MEMCPY fmemcpy
 #else /* STM8S903, STM8S103, STM8S001, STM8S003, STM8S105, STM8AF626x, STM8AF622x */
	/*!< Used with memory Models for code less than 64K */
	#define STM8S_STDPERIPH_LIB__MEMCPY memcpy
 #endif /* STM8S208 or STM8S207 or STM8S007 or STM8AF62Ax or STM8AF52Ax */
#else /*_IAR_*/
 #define FAR  __far
 #define NEAR __near
 #define TINY __tiny
 #define STM8S_STDPERIPH_LIB__EEPROM __eeprom
 #define STM8S_STDPERIPH_LIB__CONST  const
#endif /* __CSMC__ */

/* For FLASH routines, select whether pointer will be declared as near (2 bytes,
	 to handle code smaller than 64KB) or far (3 bytes, to handle code larger
	 than 64K) */

#if defined (STM8S105) || defined (STM8S005) || defined (STM8S103) || defined (STM8S003) || \
		defined (STM8S001) || defined (STM8S903) || defined (STM8AF626x) || defined (STM8AF622x)
/*!< Used with memory Models for code smaller than 64K */
 #define PointerAttr NEAR
 #define MemoryAddressCast uint16_t
#else /* STM8S208 or STM8S207 or STM8AF62Ax or STM8AF52Ax */
/*!< Used with memory Models for code higher than 64K */
 #define PointerAttr FAR
 #define MemoryAddressCast uint32_t
#endif /* STM8S105 or STM8S103 or STM8S003 or STM8S001 or STM8S903 or STM8AF626x or STM8AF622x */

/* Uncomment the line below to enable the FLASH functions execution from RAM */
#if !defined (RAM_EXECUTION)
/* #define RAM_EXECUTION  (1) */
#endif /* RAM_EXECUTION */

#ifdef RAM_EXECUTION
 #ifdef _COSMIC_
	 #define STM8S_STDPERIPH_LIB__IN_RAM(a) a
 #elif defined (_RAISONANCE_) /* __RCST7__ */
	 #define STM8S_STDPERIPH_LIB__IN_RAM(a) a inram
 #else /*_IAR_*/
	#define STM8S_STDPERIPH_LIB__IN_RAM(a) __ramfunc a
 #endif /* _COSMIC_ */
#else
	#define STM8S_STDPERIPH_LIB__IN_RAM(a) a
#endif /* RAM_EXECUTION */

/*!< [31:16] STM8S Standard Peripheral Library main version V2.3.0*/
#define __STM8S_STDPERIPH_VERSION_MAIN   ((uint8_t)0x02) /*!< [31:24] main version */
#define __STM8S_STDPERIPH_VERSION_SUB1   ((uint8_t)0x03) /*!< [23:16] sub1 version */
#define __STM8S_STDPERIPH_VERSION_SUB2   ((uint8_t)0x00) /*!< [15:8]  sub2 version */
#define __STM8S_STDPERIPH_VERSION_RC     ((uint8_t)0x00) /*!< [7:0]  release candidate */
#define __STM8S_STDPERIPH_VERSION       ( (__STM8S_STDPERIPH_VERSION_MAIN << 24)\
																					|(__STM8S_STDPERIPH_VERSION_SUB1 << 16)\
																					|(__STM8S_STDPERIPH_VERSION_SUB2 << 8)\
																					|(__STM8S_STDPERIPH_VERSION_RC))

/******************************************************************************/

/* Includes ------------------------------------------------------------------*/

/* Exported types and constants ----------------------------------------------*/

/** @addtogroup Exported_types
	* @{
	*/

#ifndef STM8S_STDPERIPH_LIB__VOLATILE
	#define STM8S_STDPERIPH_LIB__VOLATILE volatile
#endif

/**
 * IO definitions
 *
 * define access restrictions to peripheral registers
 */

#pragma push_macro("__I")
#undef __I
#define     __I     STM8S_STDPERIPH_LIB__VOLATILE const   /*!< defines 'read only' permissions     */

#pragma push_macro("__O")
#undef __O
#define     __O     STM8S_STDPERIPH_LIB__VOLATILE         /*!< defines 'write only' permissions    */

#pragma push_macro("__IO")
#undef __IO
#define     __IO    STM8S_STDPERIPH_LIB__VOLATILE         /*!< defines 'read / write' permissions  */

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

/*!< STM8 Standard Peripheral Library old types (maintained for legacy purpose) */

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;


// typedef enum {FALSE = 0, TRUE = !FALSE} bool;

typedef enum FlagStatus {RESET = 0, SET = !RESET} FlagStatus, ITStatus, BitStatus, BitAction;

typedef enum FunctionalState {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONALSTATE_OK(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum ErrorStatus {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

enum IntLimit {
	U8_MAX = (255),
	S8_MAX = (127),
	S8_MIN = (-128),
	U16_MAX = (65535u),
	S16_MAX = (32767),
	S16_MIN = (-32768),
	#if !defined(_IAR_) 
		U32_MAX = (4294967295uL),
		S32_MIN = (-2147483648L),
	#endif
	S32_MAX = (2147483647L),
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

/**
	* @brief  General Purpose I/Os (GPIO)
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

/**
	* @}
	*/

/** @addtogroup GPIO_Registers_Reset_Value
	* @{
	*/

enum GPIO_Registers_Reset_Value {
	GPIO_ODR_RESET_VALUE = ((uint8_t)0x00),
	GPIO_DDR_RESET_VALUE = ((uint8_t)0x00),
	GPIO_CR1_RESET_VALUE = ((uint8_t)0x00),
	GPIO_CR2_RESET_VALUE = ((uint8_t)0x00),
};
/**
	* @}
	*/

/*----------------------------------------------------------------------------*/
#if defined(STM8S105) || defined(STM8S005) || defined(STM8S103) || defined(STM8S003) || \
		defined(STM8S001) || defined(STM8S903) || defined(STM8AF626x) || defined(STM8AF622x)
/**
	* @brief  Analog to Digital Converter (ADC1)
	*/
 typedef struct ADC1_struct
 {
	__IO uint8_t DB0RH;         /*!< ADC1 Data Buffer Register (MSB)  */
	__IO uint8_t DB0RL;         /*!< ADC1 Data Buffer Register (LSB)  */
	__IO uint8_t DB1RH;         /*!< ADC1 Data Buffer Register (MSB)  */
	__IO uint8_t DB1RL;         /*!< ADC1 Data Buffer Register (LSB)  */
	__IO uint8_t DB2RH;         /*!< ADC1 Data Buffer Register (MSB)  */
	__IO uint8_t DB2RL;         /*!< ADC1 Data Buffer Register (LSB)  */
	__IO uint8_t DB3RH;         /*!< ADC1 Data Buffer Register (MSB)  */
	__IO uint8_t DB3RL;         /*!< ADC1 Data Buffer Register (LSB)  */
	__IO uint8_t DB4RH;         /*!< ADC1 Data Buffer Register (MSB)  */
	__IO uint8_t DB4RL;         /*!< ADC1 Data Buffer Register (LSB)  */
	__IO uint8_t DB5RH;         /*!< ADC1 Data Buffer Register (MSB)  */
	__IO uint8_t DB5RL;         /*!< ADC1 Data Buffer Register (LSB)  */
	__IO uint8_t DB6RH;         /*!< ADC1 Data Buffer Register (MSB)  */
	__IO uint8_t DB6RL;         /*!< ADC1 Data Buffer Register (LSB)  */
	__IO uint8_t DB7RH;         /*!< ADC1 Data Buffer Register (MSB)  */
	__IO uint8_t DB7RL;         /*!< ADC1 Data Buffer Register (LSB)  */
	__IO uint8_t DB8RH;         /*!< ADC1 Data Buffer Register (MSB)  */
	__IO uint8_t DB8RL;         /*!< ADC1 Data Buffer Register (LSB)  */
	__IO uint8_t DB9RH;         /*!< ADC1 Data Buffer Register (MSB)  */
	__IO uint8_t DB9RL;         /*!< ADC1 Data Buffer Register (LSB)  */
	uint8_t RESERVED[12];       /*!< Reserved byte */
	__IO uint8_t CSR;           /*!< ADC1 control status register */
	__IO uint8_t CR1;           /*!< ADC1 configuration register 1 */
	__IO uint8_t CR2;           /*!< ADC1 configuration register 2 */
	__IO uint8_t CR3;           /*!< ADC1 configuration register 3  */
	__IO uint8_t DRH;           /*!< ADC1 Data high */
	__IO uint8_t DRL;           /*!< ADC1 Data low */
	__IO uint8_t TDRH;          /*!< ADC1 Schmitt trigger disable register high */
	__IO uint8_t TDRL;          /*!< ADC1 Schmitt trigger disable register low */
	__IO uint8_t HTRH;          /*!< ADC1 high threshold register High*/
	__IO uint8_t HTRL;          /*!< ADC1 high threshold register Low*/
	__IO uint8_t LTRH;          /*!< ADC1 low threshold register high */
	__IO uint8_t LTRL;          /*!< ADC1 low threshold register low */
	__IO uint8_t AWSRH;         /*!< ADC1 watchdog status register high */
	__IO uint8_t AWSRL;         /*!< ADC1 watchdog status register low */
	__IO uint8_t AWCRH;         /*!< ADC1 watchdog control register high */
	__IO uint8_t AWCRL;         /*!< ADC1 watchdog control register low */
 }
ADC1_TypeDef;

/** @addtogroup ADC1_Registers_Reset_Value
	* @{
	*/
enum ADC1_Registers_Reset_Value {
 	ADC1_CSR_RESET_VALUE = ((uint8_t)0x00),
 	ADC1_CR1_RESET_VALUE = ((uint8_t)0x00),
 	ADC1_CR2_RESET_VALUE = ((uint8_t)0x00),
 	ADC1_CR3_RESET_VALUE = ((uint8_t)0x00),
 	ADC1_TDRL_RESET_VALUE = ((uint8_t)0x00),
 	ADC1_TDRH_RESET_VALUE = ((uint8_t)0x00),
 	ADC1_HTRL_RESET_VALUE = ((uint8_t)0x03),
 	ADC1_HTRH_RESET_VALUE = ((uint8_t)0xFF),
 	ADC1_LTRH_RESET_VALUE = ((uint8_t)0x00),
 	ADC1_LTRL_RESET_VALUE = ((uint8_t)0x00),
 	ADC1_AWCRH_RESET_VALUE = ((uint8_t)0x00),
 	ADC1_AWCRL_RESET_VALUE = ((uint8_t)0x00),
};
/**
	* @}
	*/

/** @addtogroup ADC1_Registers_Bits_Definition
	* @{
	*/
enum ADC1_Registers_Bits_Definition {
 	ADC1_CSR_EOC = ((uint8_t)0x80) /*!< End of Conversion mask */,
 	ADC1_CSR_AWD = ((uint8_t)0x40) /*!< Analog Watch Dog Status mask */,
 	ADC1_CSR_EOCIE = ((uint8_t)0x20) /*!< Interrupt Enable for EOC mask */,
 	ADC1_CSR_AWDIE = ((uint8_t)0x10) /*!< Analog Watchdog interrupt enable mask */,
 	ADC1_CSR_CH = ((uint8_t)0x0F) /*!< Channel selection bits mask */,

 	ADC1_CR1_SPSEL = ((uint8_t)0x70) /*!< Prescaler selection mask */,
 	ADC1_CR1_CONT = ((uint8_t)0x02) /*!< Continuous conversion mask */,
 	ADC1_CR1_ADON = ((uint8_t)0x01) /*!< A/D Converter on/off mask */,

 	ADC1_CR2_EXTTRIG = ((uint8_t)0x40) /*!< External trigger enable mask */,
 	ADC1_CR2_EXTSEL = ((uint8_t)0x30) /*!< External event selection mask */,
 	ADC1_CR2_ALIGN = ((uint8_t)0x08) /*!< Data Alignment mask */,
 	ADC1_CR2_SCAN = ((uint8_t)0x02) /*!< Scan mode mask */,

 	ADC1_CR3_DBUF = ((uint8_t)0x80) /*!< Data Buffer Enable mask */,
 	ADC1_CR3_OVR = ((uint8_t)0x40) /*!< Overrun Status Flag mask */,
};
#endif /* (STM8S105) ||(STM8S103) || (STM8S005) ||(STM8S003) || (STM8S001) || (STM8S903) || (STM8AF626x) || (STM8AF622x) */
/**
	* @}
	*/

/*----------------------------------------------------------------------------*/
/**
	* @brief  Analog to Digital Converter (ADC2)
	*/
#if defined(STM8S208) || defined(STM8S207) || defined (STM8S007) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
 typedef struct ADC2_struct
 {
	__IO uint8_t CSR;        /*!< ADC2 control status register */
	__IO uint8_t CR1;        /*!< ADC2 configuration register 1 */
	__IO uint8_t CR2;        /*!< ADC2 configuration register 2 */
	uint8_t RESERVED;        /*!< Reserved byte */
	__IO uint8_t DRH;        /*!< ADC2 Data high */
	__IO uint8_t DRL;        /*!< ADC2 Data low */
	__IO uint8_t TDRH;       /*!< ADC2 Schmitt trigger disable register high  */
	__IO uint8_t TDRL;       /*!< ADC2 Schmitt trigger disable register low */
 }
 ADC2_TypeDef;

/** @addtogroup ADC2_Registers_Reset_Value
	* @{
	*/
enum ADC2_Registers_Reset_Value {
 	ADC2_CSR_RESET_VALUE = ((uint8_t)0x00),
 	ADC2_CR1_RESET_VALUE = ((uint8_t)0x00),
 	ADC2_CR2_RESET_VALUE = ((uint8_t)0x00),
 	ADC2_TDRL_RESET_VALUE = ((uint8_t)0x00),
 	ADC2_TDRH_RESET_VALUE = ((uint8_t)0x00),
};
/**
	* @}
	*/

/** @addtogroup ADC2_Registers_Bits_Definition
	* @{
	*/
enum ADC2_Registers_Bits_Definition {
 	ADC2_CSR_EOC = ((uint8_t)0x80) /*!< End of Conversion mask */,
 	ADC2_CSR_EOCIE = ((uint8_t)0x20) /*!< Interrupt Enable for EOC mask */,
 	ADC2_CSR_CH = ((uint8_t)0x0F) /*!< Channel selection bits mask */,

 	ADC2_CR1_SPSEL = ((uint8_t)0x70) /*!< Prescaler selection mask */,
 	ADC2_CR1_CONT = ((uint8_t)0x02) /*!< Continuous conversion mask */,
 	ADC2_CR1_ADON = ((uint8_t)0x01) /*!< A/D Converter on/off mask */,

 	ADC2_CR2_EXTTRIG = ((uint8_t)0x40) /*!< External trigger enable mask */,
 	ADC2_CR2_EXTSEL = ((uint8_t)0x30) /*!< External event selection mask */,
 	ADC2_CR2_ALIGN = ((uint8_t)0x08) /*!< Data Alignment mask */,
};
#endif /* (STM8S208) ||(STM8S207) || defined (STM8S007) || (STM8AF62Ax) || (STM8AF52Ax) */
/**
	* @}
	*/

/*----------------------------------------------------------------------------*/

/**
	* @brief  Auto Wake Up (AWU) peripheral registers.
	*/
typedef struct AWU_struct
{
	__IO uint8_t CSR; /*!< AWU Control status register */
	__IO uint8_t APR; /*!< AWU Asynchronous prescaler buffer */
	__IO uint8_t TBR; /*!< AWU Time base selection register */
}
AWU_TypeDef;

/** @addtogroup AWU_Registers_Reset_Value
	* @{
	*/
enum AWU_Registers_Reset_Value {
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
enum AWU_Registers_Bits_Definition {
	AWU_CSR_AWUF = ((uint8_t)0x20) /*!< Interrupt flag mask */,
	AWU_CSR_AWUEN = ((uint8_t)0x10) /*!< Auto Wake-up enable mask */,
	AWU_CSR_MSR = ((uint8_t)0x01) /*!< LSI Measurement enable mask */,

	AWU_APR_APR = ((uint8_t)0x3F) /*!< Asynchronous Prescaler divider mask */,

	AWU_TBR_AWUTB = ((uint8_t)0x0F) /*!< Timebase selection mask */,
};
/**
	* @}
	*/

/*----------------------------------------------------------------------------*/
/**
	* @brief  Beeper (BEEP) peripheral registers.
	*/

typedef struct BEEP_struct
{
	__IO uint8_t CSR; /*!< BEEP Control status register */
}
BEEP_TypeDef;

/** @addtogroup BEEP_Registers_Reset_Value
	* @{
	*/
enum BEEP_Registers_Reset_Value {
	BEEP_CSR_RESET_VALUE = ((uint8_t)0x1F),
};
/**
	* @}
	*/

/** @addtogroup BEEP_Registers_Bits_Definition
	* @{
	*/
enum BEEP_Registers_Bits_Definition {
	BEEP_CSR_BEEPSEL = ((uint8_t)0xC0) /*!< Beeper frequency selection mask */,
	BEEP_CSR_BEEPEN = ((uint8_t)0x20) /*!< Beeper enable mask */,
	BEEP_CSR_BEEPDIV = ((uint8_t)0x1F) /*!< Beeper Divider prescalar mask */,
};
/**
	* @}
	*/

/*----------------------------------------------------------------------------*/
/**
	* @brief  Clock Controller (CLK)
	*/
typedef struct CLK_struct
{
	__IO uint8_t ICKR;     /*!< Internal Clocks Control Register */
	__IO uint8_t ECKR;     /*!< External Clocks Control Register */
	uint8_t RESERVED;      /*!< Reserved byte */
	__IO uint8_t CMSR;     /*!< Clock Master Status Register */
	__IO uint8_t SWR;      /*!< Clock Master Switch Register */
	__IO uint8_t SWCR;     /*!< Switch Control Register */
	__IO uint8_t CKDIVR;   /*!< Clock Divider Register */
	__IO uint8_t PCKENR1;  /*!< Peripheral Clock Gating Register 1 */
	__IO uint8_t CSSR;     /*!< Clock Security System Register */
	__IO uint8_t CCOR;     /*!< Configurable Clock Output Register */
	__IO uint8_t PCKENR2;  /*!< Peripheral Clock Gating Register 2 */
	uint8_t RESERVED1;     /*!< Reserved byte */
	__IO uint8_t HSITRIMR; /*!< HSI Calibration Trimmer Register */
	__IO uint8_t SWIMCCR;  /*!< SWIM clock control register */
}
CLK_TypeDef;

/** @addtogroup CLK_Registers_Reset_Value
	* @{
	*/
enum CLK_Registers_Reset_Value {
	CLK_ICKR_RESET_VALUE = ((uint8_t)0x01),
	CLK_ECKR_RESET_VALUE = ((uint8_t)0x00),
	CLK_CMSR_RESET_VALUE = ((uint8_t)0xE1),
	CLK_SWR_RESET_VALUE = ((uint8_t)0xE1),
	CLK_SWCR_RESET_VALUE = ((uint8_t)0x00),
	CLK_CKDIVR_RESET_VALUE = ((uint8_t)0x18),
	CLK_PCKENR1_RESET_VALUE = ((uint8_t)0xFF),
	CLK_PCKENR2_RESET_VALUE = ((uint8_t)0xFF),
	CLK_CSSR_RESET_VALUE = ((uint8_t)0x00),
	CLK_CCOR_RESET_VALUE = ((uint8_t)0x00),
	CLK_HSITRIMR_RESET_VALUE = ((uint8_t)0x00),
	CLK_SWIMCCR_RESET_VALUE = ((uint8_t)0x00),
};
/**
	* @}
	*/

/** @addtogroup CLK_Registers_Bits_Definition
	* @{
	*/
enum CLK_Registers_Bits_Definition {
	CLK_ICKR_SWUAH = ((uint8_t)0x20) /*!< Slow Wake-up from Active Halt/Halt modes */,
	CLK_ICKR_LSIRDY = ((uint8_t)0x10) /*!< Low speed internal oscillator ready */,
	CLK_ICKR_LSIEN = ((uint8_t)0x08) /*!< Low speed internal RC oscillator enable */,
	CLK_ICKR_FHWU = ((uint8_t)0x04) /*!< Fast Wake-up from Active Halt/Halt mode */,
	CLK_ICKR_HSIRDY = ((uint8_t)0x02) /*!< High speed internal RC oscillator ready */,
	CLK_ICKR_HSIEN = ((uint8_t)0x01) /*!< High speed internal RC oscillator enable */,

	CLK_ECKR_HSERDY = ((uint8_t)0x02) /*!< High speed external crystal oscillator ready */,
	CLK_ECKR_HSEEN = ((uint8_t)0x01) /*!< High speed external crystal oscillator enable */,

	CLK_CMSR_CKM = ((uint8_t)0xFF) /*!< Clock master status bits */,

	CLK_SWR_SWI = ((uint8_t)0xFF) /*!< Clock master selection bits */,

	CLK_SWCR_SWIF = ((uint8_t)0x08) /*!< Clock switch interrupt flag */,
	CLK_SWCR_SWIEN = ((uint8_t)0x04) /*!< Clock switch interrupt enable */,
	CLK_SWCR_SWEN = ((uint8_t)0x02) /*!< Switch start/stop */,
	CLK_SWCR_SWBSY = ((uint8_t)0x01) /*!< Switch busy flag*/,

	CLK_CKDIVR_HSIDIV = ((uint8_t)0x18) /*!< High speed internal clock prescaler */,
	CLK_CKDIVR_CPUDIV = ((uint8_t)0x07) /*!< CPU clock prescaler */,

	CLK_PCKENR1_TIM1 = ((uint8_t)0x80) /*!< Timer 1 clock enable */ ,
	CLK_PCKENR1_TIM3 = ((uint8_t)0x40) /*!< Timer 3 clock enable */,
	CLK_PCKENR1_TIM2 = ((uint8_t)0x20) /*!< Timer 2 clock enable */,
	CLK_PCKENR1_TIM5 = ((uint8_t)0x20) /*!< Timer 5 clock enable */,
	CLK_PCKENR1_TIM4 = ((uint8_t)0x10) /*!< Timer 4 clock enable */,
	CLK_PCKENR1_TIM6 = ((uint8_t)0x10) /*!< Timer 6 clock enable */,
	CLK_PCKENR1_UART3 = ((uint8_t)0x08) /*!< UART3 clock enable */,
	CLK_PCKENR1_UART2 = ((uint8_t)0x08) /*!< UART2 clock enable */,
	CLK_PCKENR1_UART1 = ((uint8_t)0x04) /*!< UART1 clock enable */,
	CLK_PCKENR1_SPI = ((uint8_t)0x02) /*!< SPI clock enable */,
	CLK_PCKENR1_I2C = ((uint8_t)0x01) /*!< I2C clock enable */,

	CLK_PCKENR2_CAN = ((uint8_t)0x80) /*!< CAN clock enable */,
	CLK_PCKENR2_ADC = ((uint8_t)0x08) /*!< ADC clock enable */,
	CLK_PCKENR2_AWU = ((uint8_t)0x04) /*!< AWU clock enable */,

	CLK_CSSR_CSSD = ((uint8_t)0x08) /*!< Clock security system detection */,
	CLK_CSSR_CSSDIE = ((uint8_t)0x04) /*!< Clock security system detection interrupt enable */,
	CLK_CSSR_AUX = ((uint8_t)0x02) /*!< Auxiliary oscillator connected to master clock */,
	CLK_CSSR_CSSEN = ((uint8_t)0x01) /*!< Clock security system enable */,

	CLK_CCOR_CCOBSY = ((uint8_t)0x40) /*!< Configurable clock output busy */,
	CLK_CCOR_CCORDY = ((uint8_t)0x20) /*!< Configurable clock output ready */,
	CLK_CCOR_CCOSEL = ((uint8_t)0x1E) /*!< Configurable clock output selection */,
	CLK_CCOR_CCOEN = ((uint8_t)0x01) /*!< Configurable clock output enable */,

	CLK_HSITRIMR_HSITRIM = ((uint8_t)0x07) /*!< High speed internal oscillator trimmer */,

	CLK_SWIMCCR_SWIMDIV = ((uint8_t)0x01) /*!< SWIM Clock Dividing Factor */,
};
/**
	* @}
	*/

/*----------------------------------------------------------------------------*/
/**
	* @brief  16-bit timer with complementary PWM outputs (TIM1)
	*/

typedef struct TIM1_struct
{
	__IO uint8_t CR1;   /*!< control register 1 */
	__IO uint8_t CR2;   /*!< control register 2 */
	__IO uint8_t SMCR;  /*!< Synchro mode control register */
	__IO uint8_t ETR;   /*!< external trigger register */
	__IO uint8_t IER;   /*!< interrupt enable register*/
	__IO uint8_t SR1;   /*!< status register 1 */
	__IO uint8_t SR2;   /*!< status register 2 */
	__IO uint8_t EGR;   /*!< event generation register */
	__IO uint8_t CCMR1; /*!< CC mode register 1 */
	__IO uint8_t CCMR2; /*!< CC mode register 2 */
	__IO uint8_t CCMR3; /*!< CC mode register 3 */
	__IO uint8_t CCMR4; /*!< CC mode register 4 */
	__IO uint8_t CCER1; /*!< CC enable register 1 */
	__IO uint8_t CCER2; /*!< CC enable register 2 */
	__IO uint8_t CNTRH; /*!< counter high */
	__IO uint8_t CNTRL; /*!< counter low */
	__IO uint8_t PSCRH; /*!< prescaler high */
	__IO uint8_t PSCRL; /*!< prescaler low */
	__IO uint8_t ARRH;  /*!< auto-reload register high */
	__IO uint8_t ARRL;  /*!< auto-reload register low */
	__IO uint8_t RCR;   /*!< Repetition Counter register */
	__IO uint8_t CCR1H; /*!< capture/compare register 1 high */
	__IO uint8_t CCR1L; /*!< capture/compare register 1 low */
	__IO uint8_t CCR2H; /*!< capture/compare register 2 high */
	__IO uint8_t CCR2L; /*!< capture/compare register 2 low */
	__IO uint8_t CCR3H; /*!< capture/compare register 3 high */
	__IO uint8_t CCR3L; /*!< capture/compare register 3 low */
	__IO uint8_t CCR4H; /*!< capture/compare register 3 high */
	__IO uint8_t CCR4L; /*!< capture/compare register 3 low */
	__IO uint8_t BKR;   /*!< Break Register */
	__IO uint8_t DTR;   /*!< dead-time register */
	__IO uint8_t OISR;  /*!< Output idle register */
}
TIM1_TypeDef;

/** @addtogroup TIM1_Registers_Reset_Value
	* @{
	*/
enum TIM1_Registers_Reset_Value {
	TIM1_CR1_RESET_VALUE = ((uint8_t)0x00),
	TIM1_CR2_RESET_VALUE = ((uint8_t)0x00),
	TIM1_SMCR_RESET_VALUE = ((uint8_t)0x00),
	TIM1_ETR_RESET_VALUE = ((uint8_t)0x00),
	TIM1_IER_RESET_VALUE = ((uint8_t)0x00),
	TIM1_SR1_RESET_VALUE = ((uint8_t)0x00),
	TIM1_SR2_RESET_VALUE = ((uint8_t)0x00),
	TIM1_EGR_RESET_VALUE = ((uint8_t)0x00),
	TIM1_CCMR1_RESET_VALUE = ((uint8_t)0x00),
	TIM1_CCMR2_RESET_VALUE = ((uint8_t)0x00),
	TIM1_CCMR3_RESET_VALUE = ((uint8_t)0x00),
	TIM1_CCMR4_RESET_VALUE = ((uint8_t)0x00),
	TIM1_CCER1_RESET_VALUE = ((uint8_t)0x00),
	TIM1_CCER2_RESET_VALUE = ((uint8_t)0x00),
	TIM1_CNTRH_RESET_VALUE = ((uint8_t)0x00),
	TIM1_CNTRL_RESET_VALUE = ((uint8_t)0x00),
	TIM1_PSCRH_RESET_VALUE = ((uint8_t)0x00),
	TIM1_PSCRL_RESET_VALUE = ((uint8_t)0x00),
	TIM1_ARRH_RESET_VALUE = ((uint8_t)0xFF),
	TIM1_ARRL_RESET_VALUE = ((uint8_t)0xFF),
	TIM1_RCR_RESET_VALUE = ((uint8_t)0x00),
	TIM1_CCR1H_RESET_VALUE = ((uint8_t)0x00),
	TIM1_CCR1L_RESET_VALUE = ((uint8_t)0x00),
	TIM1_CCR2H_RESET_VALUE = ((uint8_t)0x00),
	TIM1_CCR2L_RESET_VALUE = ((uint8_t)0x00),
	TIM1_CCR3H_RESET_VALUE = ((uint8_t)0x00),
	TIM1_CCR3L_RESET_VALUE = ((uint8_t)0x00),
	TIM1_CCR4H_RESET_VALUE = ((uint8_t)0x00),
	TIM1_CCR4L_RESET_VALUE = ((uint8_t)0x00),
	TIM1_BKR_RESET_VALUE = ((uint8_t)0x00),
	TIM1_DTR_RESET_VALUE = ((uint8_t)0x00),
	TIM1_OISR_RESET_VALUE = ((uint8_t)0x00),
};
/**
	* @}
	*/

/** @addtogroup TIM1_Registers_Bits_Definition
	* @{
	*/
enum TIM1_Registers_Bits_Definition {
/* CR1*/
	TIM1_CR1_ARPE = ((uint8_t)0x80) /*!< Auto-Reload Preload Enable mask. */,
	TIM1_CR1_CMS = ((uint8_t)0x60) /*!< Center-aligned Mode Selection mask. */,
	TIM1_CR1_DIR = ((uint8_t)0x10) /*!< Direction mask. */,
	TIM1_CR1_OPM = ((uint8_t)0x08) /*!< One Pulse Mode mask. */,
	TIM1_CR1_URS = ((uint8_t)0x04) /*!< Update Request Source mask. */,
	TIM1_CR1_UDIS = ((uint8_t)0x02) /*!< Update DIsable mask. */,
	TIM1_CR1_CEN = ((uint8_t)0x01) /*!< Counter Enable mask. */,
/* CR2*/
	TIM1_CR2_TI1S = ((uint8_t)0x80) /*!< TI1S Selection mask. */,
	TIM1_CR2_MMS = ((uint8_t)0x70) /*!< MMS Selection mask. */,
	TIM1_CR2_COMS = ((uint8_t)0x04) /*!< Capture/Compare Control Update Selection mask. */,
	TIM1_CR2_CCPC = ((uint8_t)0x01) /*!< Capture/Compare Preloaded Control mask. */,
/* SMCR*/
	TIM1_SMCR_MSM = ((uint8_t)0x80) /*!< Master/Slave Mode mask. */,
	TIM1_SMCR_TS = ((uint8_t)0x70) /*!< Trigger Selection mask. */,
	TIM1_SMCR_SMS = ((uint8_t)0x07) /*!< Slave Mode Selection mask. */,
/*ETR*/
	TIM1_ETR_ETP = ((uint8_t)0x80) /*!< External Trigger Polarity mask. */,
	TIM1_ETR_ECE = ((uint8_t)0x40)/*!< External Clock mask. */,
	TIM1_ETR_ETPS = ((uint8_t)0x30) /*!< External Trigger Prescaler mask. */,
	TIM1_ETR_ETF = ((uint8_t)0x0F) /*!< External Trigger Filter mask. */,
/*IER*/
	TIM1_IER_BIE = ((uint8_t)0x80) /*!< Break Interrupt Enable mask. */,
	TIM1_IER_TIE = ((uint8_t)0x40) /*!< Trigger Interrupt Enable mask. */,
	TIM1_IER_COMIE = ((uint8_t)0x20) /*!<  Commutation Interrupt Enable mask.*/,
	TIM1_IER_CC4IE = ((uint8_t)0x10) /*!< Capture/Compare 4 Interrupt Enable mask. */,
	TIM1_IER_CC3IE = ((uint8_t)0x08) /*!< Capture/Compare 3 Interrupt Enable mask. */,
	TIM1_IER_CC2IE = ((uint8_t)0x04) /*!< Capture/Compare 2 Interrupt Enable mask. */,
	TIM1_IER_CC1IE = ((uint8_t)0x02) /*!< Capture/Compare 1 Interrupt Enable mask. */,
	TIM1_IER_UIE = ((uint8_t)0x01) /*!< Update Interrupt Enable mask. */,
/*SR1*/
	TIM1_SR1_BIF = ((uint8_t)0x80) /*!< Break Interrupt Flag mask. */,
	TIM1_SR1_TIF = ((uint8_t)0x40) /*!< Trigger Interrupt Flag mask. */,
	TIM1_SR1_COMIF = ((uint8_t)0x20) /*!< Commutation Interrupt Flag mask. */,
	TIM1_SR1_CC4IF = ((uint8_t)0x10) /*!< Capture/Compare 4 Interrupt Flag mask. */,
	TIM1_SR1_CC3IF = ((uint8_t)0x08) /*!< Capture/Compare 3 Interrupt Flag mask. */,
	TIM1_SR1_CC2IF = ((uint8_t)0x04) /*!< Capture/Compare 2 Interrupt Flag mask. */,
	TIM1_SR1_CC1IF = ((uint8_t)0x02) /*!< Capture/Compare 1 Interrupt Flag mask. */,
	TIM1_SR1_UIF = ((uint8_t)0x01) /*!< Update Interrupt Flag mask. */,
/*SR2*/
	TIM1_SR2_CC4OF = ((uint8_t)0x10) /*!< Capture/Compare 4 Overcapture Flag mask. */,
	TIM1_SR2_CC3OF = ((uint8_t)0x08) /*!< Capture/Compare 3 Overcapture Flag mask. */,
	TIM1_SR2_CC2OF = ((uint8_t)0x04) /*!< Capture/Compare 2 Overcapture Flag mask. */,
	TIM1_SR2_CC1OF = ((uint8_t)0x02) /*!< Capture/Compare 1 Overcapture Flag mask. */,
/*EGR*/
	TIM1_EGR_BG = ((uint8_t)0x80) /*!< Break Generation mask. */,
	TIM1_EGR_TG = ((uint8_t)0x40) /*!< Trigger Generation mask. */,
	TIM1_EGR_COMG = ((uint8_t)0x20) /*!< Capture/Compare Control Update Generation mask. */,
	TIM1_EGR_CC4G = ((uint8_t)0x10) /*!< Capture/Compare 4 Generation mask. */,
	TIM1_EGR_CC3G = ((uint8_t)0x08) /*!< Capture/Compare 3 Generation mask. */,
	TIM1_EGR_CC2G = ((uint8_t)0x04) /*!< Capture/Compare 2 Generation mask. */,
	TIM1_EGR_CC1G = ((uint8_t)0x02) /*!< Capture/Compare 1 Generation mask. */,
	TIM1_EGR_UG = ((uint8_t)0x01) /*!< Update Generation mask. */,
/*CCMR*/
	TIM1_CCMR_ICxPSC = ((uint8_t)0x0C) /*!< Input Capture x Prescaler mask. */,
	TIM1_CCMR_ICxF = ((uint8_t)0xF0) /*!< Input Capture x Filter mask. */,
	TIM1_CCMR_OCM = ((uint8_t)0x70) /*!< Output Compare x Mode mask. */,
	TIM1_CCMR_OCxPE = ((uint8_t)0x08) /*!< Output Compare x Preload Enable mask. */,
	TIM1_CCMR_OCxFE = ((uint8_t)0x04) /*!< Output Compare x Fast Enable mask. */,
	TIM1_CCMR_CCxS = ((uint8_t)0x03) /*!< Capture/Compare x Selection mask. */,

	CCMR_TIxDirect_Set = ((uint8_t)0x01),
/*CCER1*/
	TIM1_CCER1_CC2NP = ((uint8_t)0x80) /*!< Capture/Compare 2 Complementary output Polarity mask. */,
	TIM1_CCER1_CC2NE = ((uint8_t)0x40) /*!< Capture/Compare 2 Complementary output enable mask. */,
	TIM1_CCER1_CC2P = ((uint8_t)0x20) /*!< Capture/Compare 2 output Polarity mask. */,
	TIM1_CCER1_CC2E = ((uint8_t)0x10) /*!< Capture/Compare 2 output enable mask. */,
	TIM1_CCER1_CC1NP = ((uint8_t)0x08) /*!< Capture/Compare 1 Complementary output Polarity mask. */,
	TIM1_CCER1_CC1NE = ((uint8_t)0x04) /*!< Capture/Compare 1 Complementary output enable mask. */,
	TIM1_CCER1_CC1P = ((uint8_t)0x02) /*!< Capture/Compare 1 output Polarity mask. */,
	TIM1_CCER1_CC1E = ((uint8_t)0x01) /*!< Capture/Compare 1 output enable mask. */,
/*CCER2*/
	TIM1_CCER2_CC4P = ((uint8_t)0x20) /*!< Capture/Compare 4 output Polarity mask. */,
	TIM1_CCER2_CC4E = ((uint8_t)0x10) /*!< Capture/Compare 4 output enable mask. */,
	TIM1_CCER2_CC3NP = ((uint8_t)0x08) /*!< Capture/Compare 3 Complementary output Polarity mask. */,
	TIM1_CCER2_CC3NE = ((uint8_t)0x04) /*!< Capture/Compare 3 Complementary output enable mask. */,
	TIM1_CCER2_CC3P = ((uint8_t)0x02) /*!< Capture/Compare 3 output Polarity mask. */,
	TIM1_CCER2_CC3E = ((uint8_t)0x01) /*!< Capture/Compare 3 output enable mask. */,
/*CNTRH*/
	TIM1_CNTRH_CNT = ((uint8_t)0xFF) /*!< Counter Value (MSB) mask. */,
/*CNTRL*/
	TIM1_CNTRL_CNT = ((uint8_t)0xFF) /*!< Counter Value (LSB) mask. */,
/*PSCH*/
	TIM1_PSCH_PSC = ((uint8_t)0xFF) /*!< Prescaler Value (MSB) mask. */,
/*PSCL*/
	TIM1_PSCL_PSC = ((uint8_t)0xFF) /*!< Prescaler Value (LSB) mask. */,
/*ARR*/
	TIM1_ARRH_ARR = ((uint8_t)0xFF) /*!< Autoreload Value (MSB) mask. */,
	TIM1_ARRL_ARR = ((uint8_t)0xFF) /*!< Autoreload Value (LSB) mask. */,
/*RCR*/
	TIM1_RCR_REP = ((uint8_t)0xFF) /*!< Repetition Counter Value mask. */,
/*CCR1*/
	TIM1_CCR1H_CCR1 = ((uint8_t)0xFF) /*!< Capture/Compare 1 Value (MSB) mask. */,
	TIM1_CCR1L_CCR1 = ((uint8_t)0xFF) /*!< Capture/Compare 1 Value (LSB) mask. */,
/*CCR2*/
	TIM1_CCR2H_CCR2 = ((uint8_t)0xFF) /*!< Capture/Compare 2 Value (MSB) mask. */,
	TIM1_CCR2L_CCR2 = ((uint8_t)0xFF) /*!< Capture/Compare 2 Value (LSB) mask. */,
/*CCR3*/
	TIM1_CCR3H_CCR3 = ((uint8_t)0xFF) /*!< Capture/Compare 3 Value (MSB) mask. */,
	TIM1_CCR3L_CCR3 = ((uint8_t)0xFF) /*!< Capture/Compare 3 Value (LSB) mask. */,
/*CCR4*/
	TIM1_CCR4H_CCR4 = ((uint8_t)0xFF) /*!< Capture/Compare 4 Value (MSB) mask. */,
	TIM1_CCR4L_CCR4 = ((uint8_t)0xFF) /*!< Capture/Compare 4 Value (LSB) mask. */,
/*BKR*/
	TIM1_BKR_MOE = ((uint8_t)0x80) /*!< Main Output Enable mask. */,
	TIM1_BKR_AOE = ((uint8_t)0x40) /*!< Automatic Output Enable mask. */,
	TIM1_BKR_BKP = ((uint8_t)0x20) /*!< Break Polarity mask. */,
	TIM1_BKR_BKE = ((uint8_t)0x10) /*!< Break Enable mask. */,
	TIM1_BKR_OSSR = ((uint8_t)0x08) /*!< Off-State Selection for Run mode mask. */,
	TIM1_BKR_OSSI = ((uint8_t)0x04) /*!< Off-State Selection for Idle mode mask. */,
	TIM1_BKR_LOCK = ((uint8_t)0x03) /*!< Lock Configuration mask. */,
/*DTR*/
	TIM1_DTR_DTG = ((uint8_t)0xFF) /*!< Dead-Time Generator set-up mask. */,
/*OISR*/
	TIM1_OISR_OIS4 = ((uint8_t)0x40) /*!< Output Idle state 4 (OC4 output) mask. */,
	TIM1_OISR_OIS3N = ((uint8_t)0x20) /*!< Output Idle state 3 (OC3N output) mask. */,
	TIM1_OISR_OIS3 = ((uint8_t)0x10) /*!< Output Idle state 3 (OC3 output) mask. */,
	TIM1_OISR_OIS2N = ((uint8_t)0x08) /*!< Output Idle state 2 (OC2N output) mask. */,
	TIM1_OISR_OIS2 = ((uint8_t)0x04) /*!< Output Idle state 2 (OC2 output) mask. */,
	TIM1_OISR_OIS1N = ((uint8_t)0x02) /*!< Output Idle state 1 (OC1N output) mask. */,
	TIM1_OISR_OIS1 = ((uint8_t)0x01) /*!< Output Idle state 1 (OC1 output) mask. */,
};
/**
	* @}
	*/

/*----------------------------------------------------------------------------*/
/**
	* @brief  16-bit timer (TIM2)
	*/

typedef struct TIM2_struct
{
	__IO uint8_t CR1;   /*!< control register 1 */
#if defined(STM8S103) || defined(STM8S003) || defined(STM8S001)
	uint8_t RESERVED1; /*!< Reserved register */
	uint8_t RESERVED2; /*!< Reserved register */
#endif
	__IO uint8_t IER;   /*!< interrupt enable register */
	__IO uint8_t SR1;   /*!< status register 1 */
	__IO uint8_t SR2;   /*!< status register 2 */
	__IO uint8_t EGR;   /*!< event generation register */
	__IO uint8_t CCMR1; /*!< CC mode register 1 */
	__IO uint8_t CCMR2; /*!< CC mode register 2 */
	__IO uint8_t CCMR3; /*!< CC mode register 3 */
	__IO uint8_t CCER1; /*!< CC enable register 1 */
	__IO uint8_t CCER2; /*!< CC enable register 2 */
	__IO uint8_t CNTRH; /*!< counter high */
	__IO uint8_t CNTRL; /*!< counter low */
	__IO uint8_t PSCR;  /*!< prescaler register */
	__IO uint8_t ARRH;  /*!< auto-reload register high */
	__IO uint8_t ARRL;  /*!< auto-reload register low */
	__IO uint8_t CCR1H; /*!< capture/compare register 1 high */
	__IO uint8_t CCR1L; /*!< capture/compare register 1 low */
	__IO uint8_t CCR2H; /*!< capture/compare register 2 high */
	__IO uint8_t CCR2L; /*!< capture/compare register 2 low */
	__IO uint8_t CCR3H; /*!< capture/compare register 3 high */
	__IO uint8_t CCR3L; /*!< capture/compare register 3 low */
}
TIM2_TypeDef;

/** @addtogroup TIM2_Registers_Reset_Value
	* @{
	*/
enum TIM2_Registers_Reset_Value {
	TIM2_CR1_RESET_VALUE = ((uint8_t)0x00),
	TIM2_IER_RESET_VALUE = ((uint8_t)0x00),
	TIM2_SR1_RESET_VALUE = ((uint8_t)0x00),
	TIM2_SR2_RESET_VALUE = ((uint8_t)0x00),
	TIM2_EGR_RESET_VALUE = ((uint8_t)0x00),
	TIM2_CCMR1_RESET_VALUE = ((uint8_t)0x00),
	TIM2_CCMR2_RESET_VALUE = ((uint8_t)0x00),
	TIM2_CCMR3_RESET_VALUE = ((uint8_t)0x00),
	TIM2_CCER1_RESET_VALUE = ((uint8_t)0x00),
	TIM2_CCER2_RESET_VALUE = ((uint8_t)0x00),
	TIM2_CNTRH_RESET_VALUE = ((uint8_t)0x00),
	TIM2_CNTRL_RESET_VALUE = ((uint8_t)0x00),
	TIM2_PSCR_RESET_VALUE = ((uint8_t)0x00),
	TIM2_ARRH_RESET_VALUE = ((uint8_t)0xFF),
	TIM2_ARRL_RESET_VALUE = ((uint8_t)0xFF),
	TIM2_CCR1H_RESET_VALUE = ((uint8_t)0x00),
	TIM2_CCR1L_RESET_VALUE = ((uint8_t)0x00),
	TIM2_CCR2H_RESET_VALUE = ((uint8_t)0x00),
	TIM2_CCR2L_RESET_VALUE = ((uint8_t)0x00),
	TIM2_CCR3H_RESET_VALUE = ((uint8_t)0x00),
	TIM2_CCR3L_RESET_VALUE = ((uint8_t)0x00),
};
/**
	* @}
	*/

/** @addtogroup TIM2_Registers_Bits_Definition
	* @{
	*/
enum TIM2_Registers_Bits_Definition {
/*CR1*/
	TIM2_CR1_ARPE = ((uint8_t)0x80) /*!< Auto-Reload Preload Enable mask. */,
	TIM2_CR1_OPM = ((uint8_t)0x08) /*!< One Pulse Mode mask. */,
	TIM2_CR1_URS = ((uint8_t)0x04) /*!< Update Request Source mask. */,
	TIM2_CR1_UDIS = ((uint8_t)0x02) /*!< Update DIsable mask. */,
	TIM2_CR1_CEN = ((uint8_t)0x01) /*!< Counter Enable mask. */,
/*IER*/
	TIM2_IER_CC3IE = ((uint8_t)0x08) /*!< Capture/Compare 3 Interrupt Enable mask. */,
	TIM2_IER_CC2IE = ((uint8_t)0x04) /*!< Capture/Compare 2 Interrupt Enable mask. */,
	TIM2_IER_CC1IE = ((uint8_t)0x02) /*!< Capture/Compare 1 Interrupt Enable mask. */,
	TIM2_IER_UIE = ((uint8_t)0x01) /*!< Update Interrupt Enable mask. */,
/*SR1*/
	TIM2_SR1_CC3IF = ((uint8_t)0x08) /*!< Capture/Compare 3 Interrupt Flag mask. */,
	TIM2_SR1_CC2IF = ((uint8_t)0x04) /*!< Capture/Compare 2 Interrupt Flag mask. */,
	TIM2_SR1_CC1IF = ((uint8_t)0x02) /*!< Capture/Compare 1 Interrupt Flag mask. */,
	TIM2_SR1_UIF = ((uint8_t)0x01) /*!< Update Interrupt Flag mask. */,
/*SR2*/
	TIM2_SR2_CC3OF = ((uint8_t)0x08) /*!< Capture/Compare 3 Overcapture Flag mask. */,
	TIM2_SR2_CC2OF = ((uint8_t)0x04) /*!< Capture/Compare 2 Overcapture Flag mask. */,
	TIM2_SR2_CC1OF = ((uint8_t)0x02) /*!< Capture/Compare 1 Overcapture Flag mask. */,
/*EGR*/
	TIM2_EGR_CC3G = ((uint8_t)0x08) /*!< Capture/Compare 3 Generation mask. */,
	TIM2_EGR_CC2G = ((uint8_t)0x04) /*!< Capture/Compare 2 Generation mask. */,
	TIM2_EGR_CC1G = ((uint8_t)0x02) /*!< Capture/Compare 1 Generation mask. */,
	TIM2_EGR_UG = ((uint8_t)0x01) /*!< Update Generation mask. */,
/*CCMR*/
	TIM2_CCMR_ICxPSC = ((uint8_t)0x0C) /*!< Input Capture x Prescaler mask. */,
	TIM2_CCMR_ICxF = ((uint8_t)0xF0) /*!< Input Capture x Filter mask. */,
	TIM2_CCMR_OCM = ((uint8_t)0x70) /*!< Output Compare x Mode mask. */,
	TIM2_CCMR_OCxPE = ((uint8_t)0x08) /*!< Output Compare x Preload Enable mask. */,
	TIM2_CCMR_CCxS = ((uint8_t)0x03) /*!< Capture/Compare x Selection mask. */,
/*CCER1*/
	TIM2_CCER1_CC2P = ((uint8_t)0x20) /*!< Capture/Compare 2 output Polarity mask. */,
	TIM2_CCER1_CC2E = ((uint8_t)0x10) /*!< Capture/Compare 2 output enable mask. */,
	TIM2_CCER1_CC1P = ((uint8_t)0x02) /*!< Capture/Compare 1 output Polarity mask. */,
	TIM2_CCER1_CC1E = ((uint8_t)0x01) /*!< Capture/Compare 1 output enable mask. */,
/*CCER2*/
	TIM2_CCER2_CC3P = ((uint8_t)0x02) /*!< Capture/Compare 3 output Polarity mask. */,
	TIM2_CCER2_CC3E = ((uint8_t)0x01) /*!< Capture/Compare 3 output enable mask. */,
/*CNTR*/
	TIM2_CNTRH_CNT = ((uint8_t)0xFF) /*!< Counter Value (MSB) mask. */,
	TIM2_CNTRL_CNT = ((uint8_t)0xFF) /*!< Counter Value (LSB) mask. */,
/*PSCR*/
	TIM2_PSCR_PSC = ((uint8_t)0xFF) /*!< Prescaler Value (MSB) mask. */,
/*ARR*/
	TIM2_ARRH_ARR = ((uint8_t)0xFF) /*!< Autoreload Value (MSB) mask. */,
	TIM2_ARRL_ARR = ((uint8_t)0xFF) /*!< Autoreload Value (LSB) mask. */,
/*CCR1*/
	TIM2_CCR1H_CCR1 = ((uint8_t)0xFF) /*!< Capture/Compare 1 Value (MSB) mask. */,
	TIM2_CCR1L_CCR1 = ((uint8_t)0xFF) /*!< Capture/Compare 1 Value (LSB) mask. */,
/*CCR2*/
	TIM2_CCR2H_CCR2 = ((uint8_t)0xFF) /*!< Capture/Compare 2 Value (MSB) mask. */,
	TIM2_CCR2L_CCR2 = ((uint8_t)0xFF) /*!< Capture/Compare 2 Value (LSB) mask. */,
/*CCR3*/
	TIM2_CCR3H_CCR3 = ((uint8_t)0xFF) /*!< Capture/Compare 3 Value (MSB) mask. */,
	TIM2_CCR3L_CCR3 = ((uint8_t)0xFF) /*!< Capture/Compare 3 Value (LSB) mask. */,
};
/**
	* @}
	*/

/*----------------------------------------------------------------------------*/
/**
	* @brief  16-bit timer (TIM3)
	*/
typedef struct TIM3_struct
{
	__IO uint8_t CR1;   /*!< control register 1 */
	__IO uint8_t IER;   /*!< interrupt enable register */
	__IO uint8_t SR1;   /*!< status register 1 */
	__IO uint8_t SR2;   /*!< status register 2 */
	__IO uint8_t EGR;   /*!< event generation register */
	__IO uint8_t CCMR1; /*!< CC mode register 1 */
	__IO uint8_t CCMR2; /*!< CC mode register 2 */
	__IO uint8_t CCER1; /*!< CC enable register 1 */
	__IO uint8_t CNTRH; /*!< counter high */
	__IO uint8_t CNTRL; /*!< counter low */
	__IO uint8_t PSCR;  /*!< prescaler register */
	__IO uint8_t ARRH;  /*!< auto-reload register high */
	__IO uint8_t ARRL;  /*!< auto-reload register low */
	__IO uint8_t CCR1H; /*!< capture/compare register 1 high */
	__IO uint8_t CCR1L; /*!< capture/compare register 1 low */
	__IO uint8_t CCR2H; /*!< capture/compare register 2 high */
	__IO uint8_t CCR2L; /*!< capture/compare register 2 low */
}
TIM3_TypeDef;

/** @addtogroup TIM3_Registers_Reset_Value
	* @{
	*/

enum TIM3_Registers_Reset_Value {
	TIM3_CR1_RESET_VALUE = ((uint8_t)0x00),
	TIM3_IER_RESET_VALUE = ((uint8_t)0x00),
	TIM3_SR1_RESET_VALUE = ((uint8_t)0x00),
	TIM3_SR2_RESET_VALUE = ((uint8_t)0x00),
	TIM3_EGR_RESET_VALUE = ((uint8_t)0x00),
	TIM3_CCMR1_RESET_VALUE = ((uint8_t)0x00),
	TIM3_CCMR2_RESET_VALUE = ((uint8_t)0x00),
	TIM3_CCER1_RESET_VALUE = ((uint8_t)0x00),
	TIM3_CNTRH_RESET_VALUE = ((uint8_t)0x00),
	TIM3_CNTRL_RESET_VALUE = ((uint8_t)0x00),
	TIM3_PSCR_RESET_VALUE = ((uint8_t)0x00),
	TIM3_ARRH_RESET_VALUE = ((uint8_t)0xFF),
	TIM3_ARRL_RESET_VALUE = ((uint8_t)0xFF),
	TIM3_CCR1H_RESET_VALUE = ((uint8_t)0x00),
	TIM3_CCR1L_RESET_VALUE = ((uint8_t)0x00),
	TIM3_CCR2H_RESET_VALUE = ((uint8_t)0x00),
	TIM3_CCR2L_RESET_VALUE = ((uint8_t)0x00),
};
/**
	* @}
	*/

/** @addtogroup TIM3_Registers_Bits_Definition
	* @{
	*/
enum TIM3_Registers_Bits_Definition {
/*CR1*/
	TIM3_CR1_ARPE = ((uint8_t)0x80) /*!< Auto-Reload Preload Enable mask. */,
	TIM3_CR1_OPM = ((uint8_t)0x08) /*!< One Pulse Mode mask. */,
	TIM3_CR1_URS = ((uint8_t)0x04) /*!< Update Request Source mask. */,
	TIM3_CR1_UDIS = ((uint8_t)0x02) /*!< Update DIsable mask. */,
	TIM3_CR1_CEN = ((uint8_t)0x01) /*!< Counter Enable mask. */,
/*IER*/
	TIM3_IER_CC2IE = ((uint8_t)0x04) /*!< Capture/Compare 2 Interrupt Enable mask. */,
	TIM3_IER_CC1IE = ((uint8_t)0x02) /*!< Capture/Compare 1 Interrupt Enable mask. */,
	TIM3_IER_UIE = ((uint8_t)0x01) /*!< Update Interrupt Enable mask. */,
/*SR1*/
	TIM3_SR1_CC2IF = ((uint8_t)0x04) /*!< Capture/Compare 2 Interrupt Flag mask. */,
	TIM3_SR1_CC1IF = ((uint8_t)0x02) /*!< Capture/Compare 1 Interrupt Flag mask. */,
	TIM3_SR1_UIF = ((uint8_t)0x01) /*!< Update Interrupt Flag mask. */,
/*SR2*/
	TIM3_SR2_CC2OF = ((uint8_t)0x04) /*!< Capture/Compare 2 Overcapture Flag mask. */,
	TIM3_SR2_CC1OF = ((uint8_t)0x02) /*!< Capture/Compare 1 Overcapture Flag mask. */,
/*EGR*/
	TIM3_EGR_CC2G = ((uint8_t)0x04) /*!< Capture/Compare 2 Generation mask. */,
	TIM3_EGR_CC1G = ((uint8_t)0x02) /*!< Capture/Compare 1 Generation mask. */,
	TIM3_EGR_UG = ((uint8_t)0x01) /*!< Update Generation mask. */,
/*CCMR*/
	TIM3_CCMR_ICxPSC = ((uint8_t)0x0C) /*!< Input Capture x Prescaler mask. */,
	TIM3_CCMR_ICxF = ((uint8_t)0xF0) /*!< Input Capture x Filter mask. */,
	TIM3_CCMR_OCM = ((uint8_t)0x70) /*!< Output Compare x Mode mask. */,
	TIM3_CCMR_OCxPE = ((uint8_t)0x08) /*!< Output Compare x Preload Enable mask. */,
	TIM3_CCMR_CCxS = ((uint8_t)0x03) /*!< Capture/Compare x Selection mask. */,
/*CCER1*/
	TIM3_CCER1_CC2P = ((uint8_t)0x20) /*!< Capture/Compare 2 output Polarity mask. */,
	TIM3_CCER1_CC2E = ((uint8_t)0x10) /*!< Capture/Compare 2 output enable mask. */,
	TIM3_CCER1_CC1P = ((uint8_t)0x02) /*!< Capture/Compare 1 output Polarity mask. */,
	TIM3_CCER1_CC1E = ((uint8_t)0x01) /*!< Capture/Compare 1 output enable mask. */,
/*CNTR*/
	TIM3_CNTRH_CNT = ((uint8_t)0xFF) /*!< Counter Value (MSB) mask. */,
	TIM3_CNTRL_CNT = ((uint8_t)0xFF) /*!< Counter Value (LSB) mask. */,
/*PSCR*/
	TIM3_PSCR_PSC = ((uint8_t)0xFF) /*!< Prescaler Value (MSB) mask. */,
/*ARR*/
	TIM3_ARRH_ARR = ((uint8_t)0xFF) /*!< Autoreload Value (MSB) mask. */,
	TIM3_ARRL_ARR = ((uint8_t)0xFF) /*!< Autoreload Value (LSB) mask. */,
/*CCR1*/
	TIM3_CCR1H_CCR1 = ((uint8_t)0xFF) /*!< Capture/Compare 1 Value (MSB) mask. */,
	TIM3_CCR1L_CCR1 = ((uint8_t)0xFF) /*!< Capture/Compare 1 Value (LSB) mask. */,
/*CCR2*/
	TIM3_CCR2H_CCR2 = ((uint8_t)0xFF) /*!< Capture/Compare 2 Value (MSB) mask. */,
	TIM3_CCR2L_CCR2 = ((uint8_t)0xFF) /*!< Capture/Compare 2 Value (LSB) mask. */,
};
/**
	* @}
	*/

/*----------------------------------------------------------------------------*/
/**
	* @brief  8-bit system timer (TIM4)
	*/

typedef struct TIM4_struct
{
	__IO uint8_t CR1;  /*!< control register 1 */
#if defined(STM8S103) || defined(STM8S003) || defined(STM8S001)
	uint8_t RESERVED1; /*!< Reserved register */
	uint8_t RESERVED2; /*!< Reserved register */
#endif
	__IO uint8_t IER;  /*!< interrupt enable register */
	__IO uint8_t SR1;  /*!< status register 1 */
	__IO uint8_t EGR;  /*!< event generation register */
	__IO uint8_t CNTR; /*!< counter register */
	__IO uint8_t PSCR; /*!< prescaler register */
	__IO uint8_t ARR;  /*!< auto-reload register */
}
TIM4_TypeDef;

/** @addtogroup TIM4_Registers_Reset_Value
	* @{
	*/
enum TIM4_Registers_Reset_Value {
	TIM4_CR1_RESET_VALUE = ((uint8_t)0x00),
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
enum TIM4_Registers_Bits_Definition {
/*CR1*/
	TIM4_CR1_ARPE = ((uint8_t)0x80) /*!< Auto-Reload Preload Enable mask. */,
	TIM4_CR1_OPM = ((uint8_t)0x08) /*!< One Pulse Mode mask. */,
	TIM4_CR1_URS = ((uint8_t)0x04) /*!< Update Request Source mask. */,
	TIM4_CR1_UDIS = ((uint8_t)0x02) /*!< Update DIsable mask. */,
	TIM4_CR1_CEN = ((uint8_t)0x01) /*!< Counter Enable mask. */,
/*IER*/
	TIM4_IER_UIE = ((uint8_t)0x01) /*!< Update Interrupt Enable mask. */,
/*SR1*/
	TIM4_SR1_UIF = ((uint8_t)0x01) /*!< Update Interrupt Flag mask. */,
/*EGR*/
	TIM4_EGR_UG = ((uint8_t)0x01) /*!< Update Generation mask. */,
/*CNTR*/
	TIM4_CNTR_CNT = ((uint8_t)0xFF) /*!< Counter Value (LSB) mask. */,
/*PSCR*/
	TIM4_PSCR_PSC = ((uint8_t)0x07) /*!< Prescaler Value  mask. */,
/*ARR*/
	TIM4_ARR_ARR = ((uint8_t)0xFF) /*!< Autoreload Value mask. */,
};

/**
	* @}
	*/

/*----------------------------------------------------------------------------*/
/**
	* @brief  16-bit timer with synchro module (TIM5)
	*/

typedef struct TIM5_struct
{
	__IO uint8_t CR1;       /*!<TIM5 Control Register 1                */
	__IO uint8_t CR2;       /*!<TIM5 Control Register 2                */
	__IO uint8_t SMCR;      /*!<TIM5 Slave Mode Control Register       */
	__IO uint8_t IER;       /*!<TIM5 Interrupt Enable Register         */
	__IO uint8_t SR1;       /*!<TIM5 Status Register 1                 */
	__IO uint8_t SR2;       /*!<TIM5 Status Register 2                 */
	__IO uint8_t EGR;       /*!<TIM5 Event Generation Register         */
	__IO uint8_t CCMR1;     /*!<TIM5 Capture/Compare Mode Register 1   */
	__IO uint8_t CCMR2;     /*!<TIM5 Capture/Compare Mode Register 2   */
	__IO uint8_t CCMR3;     /*!<TIM5 Capture/Compare Mode Register 3   */
	__IO uint8_t CCER1;     /*!<TIM5 Capture/Compare Enable Register 1 */
	__IO uint8_t CCER2;     /*!<TIM5 Capture/Compare Enable Register 2 */
	__IO uint8_t CNTRH;     /*!<TIM5 Counter High                      */
	__IO uint8_t CNTRL;     /*!<TIM5 Counter Low                       */
	__IO uint8_t PSCR;      /*!<TIM5 Prescaler Register                */
	__IO uint8_t ARRH;      /*!<TIM5 Auto-Reload Register High         */
	__IO uint8_t ARRL;      /*!<TIM5 Auto-Reload Register Low          */
	__IO uint8_t CCR1H;     /*!<TIM5 Capture/Compare Register 1 High   */
	__IO uint8_t CCR1L;     /*!<TIM5 Capture/Compare Register 1 Low    */
	__IO uint8_t CCR2H;     /*!<TIM5 Capture/Compare Register 2 High   */
	__IO uint8_t CCR2L;     /*!<TIM5 Capture/Compare Register 2 Low    */
	__IO uint8_t CCR3H;     /*!<TIM5 Capture/Compare Register 3 High   */
	__IO uint8_t CCR3L;     /*!<TIM5 Capture/Compare Register 3 Low    */
}TIM5_TypeDef;

/** @addtogroup TIM5_Registers_Reset_Value
	* @{
	*/
enum TIM5_Registers_Reset_Value {
	TIM5_CR1_RESET_VALUE = ((uint8_t)0x00),
	TIM5_CR2_RESET_VALUE = ((uint8_t)0x00),
	TIM5_SMCR_RESET_VALUE = ((uint8_t)0x00),
	TIM5_IER_RESET_VALUE = ((uint8_t)0x00),
	TIM5_SR1_RESET_VALUE = ((uint8_t)0x00),
	TIM5_SR2_RESET_VALUE = ((uint8_t)0x00),
	TIM5_EGR_RESET_VALUE = ((uint8_t)0x00),
	TIM5_CCMR1_RESET_VALUE = ((uint8_t)0x00),
	TIM5_CCMR2_RESET_VALUE = ((uint8_t)0x00),
	TIM5_CCMR3_RESET_VALUE = ((uint8_t)0x00),
	TIM5_CCER1_RESET_VALUE = ((uint8_t)0x00),
	TIM5_CCER2_RESET_VALUE = ((uint8_t)0x00),
	TIM5_CNTRH_RESET_VALUE = ((uint8_t)0x00),
	TIM5_CNTRL_RESET_VALUE = ((uint8_t)0x00),
	TIM5_PSCR_RESET_VALUE = ((uint8_t)0x00),
	TIM5_ARRH_RESET_VALUE = ((uint8_t)0xFF),
	TIM5_ARRL_RESET_VALUE = ((uint8_t)0xFF),
	TIM5_CCR1H_RESET_VALUE = ((uint8_t)0x00),
	TIM5_CCR1L_RESET_VALUE = ((uint8_t)0x00),
	TIM5_CCR2H_RESET_VALUE = ((uint8_t)0x00),
	TIM5_CCR2L_RESET_VALUE = ((uint8_t)0x00),
	TIM5_CCR3H_RESET_VALUE = ((uint8_t)0x00),
	TIM5_CCR3L_RESET_VALUE = ((uint8_t)0x00),
};

/**
	* @}
	*/

/** @addtogroup TIM5_Registers_Bits_Definition
	* @{
	*/
enum TIM5_Registers_Bits_Definition {
/* CR1*/
	TIM5_CR1_ARPE = ((uint8_t)0x80) /*!< Auto-Reload Preload Enable mask. */,
	TIM5_CR1_OPM = ((uint8_t)0x08) /*!< One Pulse Mode mask. */,
	TIM5_CR1_URS = ((uint8_t)0x04) /*!< Update Request Source mask. */,
	TIM5_CR1_UDIS = ((uint8_t)0x02) /*!< Update DIsable mask. */,
	TIM5_CR1_CEN = ((uint8_t)0x01) /*!< Counter Enable mask. */,
/* CR2*/
	TIM5_CR2_TI1S = ((uint8_t)0x80) /*!< TI1S Selection Mask. */,
	TIM5_CR2_MMS = ((uint8_t)0x70) /*!< MMS Selection Mask. */,
/* SMCR*/
	TIM5_SMCR_MSM = ((uint8_t)0x80) /*!< Master/Slave Mode Mask. */,
	TIM5_SMCR_TS = ((uint8_t)0x70) /*!< Trigger Selection Mask. */,
	TIM5_SMCR_SMS = ((uint8_t)0x07) /*!< Slave Mode Selection Mask. */,
/*IER*/
	TIM5_IER_TIE = ((uint8_t)0x40) /*!< Trigger Interrupt Enable mask. */,
	TIM5_IER_CC3IE = ((uint8_t)0x08) /*!< Capture/Compare 3 Interrupt Enable mask. */,
	TIM5_IER_CC2IE = ((uint8_t)0x04) /*!< Capture/Compare 2 Interrupt Enable mask. */,
	TIM5_IER_CC1IE = ((uint8_t)0x02) /*!< Capture/Compare 1 Interrupt Enable mask. */,
	TIM5_IER_UIE = ((uint8_t)0x01) /*!< Update Interrupt Enable mask. */,
/*SR1*/
	TIM5_SR1_TIF = ((uint8_t)0x40) /*!< Trigger Interrupt Flag mask. */,
	TIM5_SR1_CC3IF = ((uint8_t)0x08) /*!< Capture/Compare 3 Interrupt Flag mask. */,
	TIM5_SR1_CC2IF = ((uint8_t)0x04) /*!< Capture/Compare 2 Interrupt Flag mask. */,
	TIM5_SR1_CC1IF = ((uint8_t)0x02) /*!< Capture/Compare 1 Interrupt Flag mask. */,
	TIM5_SR1_UIF = ((uint8_t)0x01) /*!< Update Interrupt Flag mask. */,
/*SR2*/
	TIM5_SR2_CC3OF = ((uint8_t)0x08) /*!< Capture/Compare 3 Overcapture Flag mask. */,
	TIM5_SR2_CC2OF = ((uint8_t)0x04) /*!< Capture/Compare 2 Overcapture Flag mask. */,
	TIM5_SR2_CC1OF = ((uint8_t)0x02) /*!< Capture/Compare 1 Overcapture Flag mask. */,
/*EGR*/
	TIM5_EGR_TG = ((uint8_t)0x40) /*!< Trigger Generation mask. */,
	TIM5_EGR_CC3G = ((uint8_t)0x08) /*!< Capture/Compare 3 Generation mask. */,
	TIM5_EGR_CC2G = ((uint8_t)0x04) /*!< Capture/Compare 2 Generation mask. */,
	TIM5_EGR_CC1G = ((uint8_t)0x02) /*!< Capture/Compare 1 Generation mask. */,
	TIM5_EGR_UG = ((uint8_t)0x01) /*!< Update Generation mask. */,
/*CCMR*/
	TIM5_CCMR_ICxPSC = ((uint8_t)0x0C) /*!< Input Capture x Prescaler mask. */,
	TIM5_CCMR_ICxF = ((uint8_t)0xF0) /*!< Input Capture x Filter mask. */,
	TIM5_CCMR_OCM = ((uint8_t)0x70) /*!< Output Compare x Mode mask. */,
	TIM5_CCMR_OCxPE = ((uint8_t)0x08) /*!< Output Compare x Preload Enable mask. */,
	TIM5_CCMR_CCxS = ((uint8_t)0x03) /*!< Capture/Compare x Selection mask. */,
/*CCER1*/
	TIM5_CCER1_CC2P = ((uint8_t)0x20) /*!< Capture/Compare 2 output Polarity mask. */,
	TIM5_CCER1_CC2E = ((uint8_t)0x10) /*!< Capture/Compare 2 output enable mask. */,
	TIM5_CCER1_CC1P = ((uint8_t)0x02) /*!< Capture/Compare 1 output Polarity mask. */,
	TIM5_CCER1_CC1E = ((uint8_t)0x01) /*!< Capture/Compare 1 output enable mask. */,
/*CCER2*/
	TIM5_CCER2_CC3P = ((uint8_t)0x02) /*!< Capture/Compare 3 output Polarity mask. */,
	TIM5_CCER2_CC3E = ((uint8_t)0x01) /*!< Capture/Compare 3 output enable mask. */,
/*CNTR*/
	TIM5_CNTRH_CNT = ((uint8_t)0xFF) /*!< Counter Value (MSB) mask. */,
	TIM5_CNTRL_CNT = ((uint8_t)0xFF) /*!< Counter Value (LSB) mask. */,
/*PSCR*/
	TIM5_PSCR_PSC = ((uint8_t)0xFF) /*!< Prescaler Value (MSB) mask. */,
/*ARR*/
	TIM5_ARRH_ARR = ((uint8_t)0xFF) /*!< Autoreload Value (MSB) mask. */,
	TIM5_ARRL_ARR = ((uint8_t)0xFF) /*!< Autoreload Value (LSB) mask. */,
/*CCR1*/
	TIM5_CCR1H_CCR1 = ((uint8_t)0xFF) /*!< Capture/Compare 1 Value (MSB) mask. */,
	TIM5_CCR1L_CCR1 = ((uint8_t)0xFF) /*!< Capture/Compare 1 Value (LSB) mask. */,
/*CCR2*/
	TIM5_CCR2H_CCR2 = ((uint8_t)0xFF) /*!< Capture/Compare 2 Value (MSB) mask. */,
	TIM5_CCR2L_CCR2 = ((uint8_t)0xFF) /*!< Capture/Compare 2 Value (LSB) mask. */,
/*CCR3*/
	TIM5_CCR3H_CCR3 = ((uint8_t)0xFF) /*!< Capture/Compare 3 Value (MSB) mask. */,
	TIM5_CCR3L_CCR3 = ((uint8_t)0xFF) /*!< Capture/Compare 3 Value (LSB) mask. */,
/*CCMR*/
	TIM5_CCMR_TIxDirect_Set = ((uint8_t)0x01),
};
/**
	* @}
	*/

/*----------------------------------------------------------------------------*/
/**
	* @brief  8-bit system timer  with synchro module(TIM6)
	*/

typedef struct TIM6_struct
{
		__IO uint8_t CR1; 	/*!< control register 1 */
		__IO uint8_t CR2; 	/*!< control register 2 */
		__IO uint8_t SMCR; 	/*!< Synchro mode control register */
		__IO uint8_t IER; 	/*!< interrupt enable register  */
		__IO uint8_t SR1; 	/*!< status register 1    */
		__IO uint8_t EGR; 	/*!< event generation register */
		__IO uint8_t CNTR; 	/*!< counter register  */
		__IO uint8_t PSCR; 	/*!< prescaler register */
		__IO uint8_t ARR; 	/*!< auto-reload register */
}
TIM6_TypeDef;

/** @addtogroup TIM6_Registers_Reset_Value
	* @{
	*/
enum TIM6_Registers_Reset_Value {
	TIM6_CR1_RESET_VALUE = ((uint8_t)0x00),
	TIM6_CR2_RESET_VALUE = ((uint8_t)0x00),
	TIM6_SMCR_RESET_VALUE = ((uint8_t)0x00),
	TIM6_IER_RESET_VALUE = ((uint8_t)0x00),
	TIM6_SR1_RESET_VALUE = ((uint8_t)0x00),
	TIM6_EGR_RESET_VALUE = ((uint8_t)0x00),
	TIM6_CNTR_RESET_VALUE = ((uint8_t)0x00),
	TIM6_PSCR_RESET_VALUE = ((uint8_t)0x00),
	TIM6_ARR_RESET_VALUE = ((uint8_t)0xFF),
};

/**
* @}
*/

/** @addtogroup TIM6_Registers_Bits_Definition
	* @{
	*/
enum TIM6_Registers_Bits_Definition {
/* CR1*/
	TIM6_CR1_ARPE = ((uint8_t)0x80) /*!< Auto-Reload Preload Enable Mask. */,
	TIM6_CR1_OPM = ((uint8_t)0x08) /*!< One Pulse Mode Mask. */,
	TIM6_CR1_URS = ((uint8_t)0x04) /*!< Update Request Source Mask. */,
	TIM6_CR1_UDIS = ((uint8_t)0x02) /*!< Update DIsable Mask. */,
	TIM6_CR1_CEN = ((uint8_t)0x01) /*!< Counter Enable Mask. */,
/* CR2*/
	TIM6_CR2_MMS = ((uint8_t)0x70) /*!< MMS Selection Mask. */,
/* SMCR*/
	TIM6_SMCR_MSM = ((uint8_t)0x80) /*!< Master/Slave Mode Mask. */,
	TIM6_SMCR_TS = ((uint8_t)0x70) /*!< Trigger Selection Mask. */,
	TIM6_SMCR_SMS = ((uint8_t)0x07) /*!< Slave Mode Selection Mask. */,
/* IER*/
	TIM6_IER_TIE = ((uint8_t)0x40) /*!< Trigger Interrupt Enable Mask. */,
	TIM6_IER_UIE = ((uint8_t)0x01) /*!< Update Interrupt Enable Mask. */,
/* SR1*/
	TIM6_SR1_TIF = ((uint8_t)0x40) /*!< Trigger Interrupt Flag mask. */,
	TIM6_SR1_UIF = ((uint8_t)0x01) /*!< Update Interrupt Flag Mask. */,
/* EGR*/
	TIM6_EGR_TG = ((uint8_t)0x40) /*!< Trigger Generation mask. */,
	TIM6_EGR_UG = ((uint8_t)0x01) /*!< Update Generation Mask. */,
/* CNTR*/
	TIM6_CNTR_CNT = ((uint8_t)0xFF) /*!<Counter Value (LSB) Mask. */,
/* PSCR*/
	TIM6_PSCR_PSC = ((uint8_t)0x07) /*!<Prescaler Value  Mask. */,

	TIM6_ARR_ARR = ((uint8_t)0xFF) /*!<Autoreload Value Mask. */,
};
/**
	* @}
	*/
/*----------------------------------------------------------------------------*/
/**
	* @brief  Inter-Integrated Circuit (I2C)
	*/

typedef struct I2C_struct
{
	__IO uint8_t CR1;       /*!< I2C control register 1 */
	__IO uint8_t CR2;       /*!< I2C control register 2 */
	__IO uint8_t FREQR;     /*!< I2C frequency register */
	__IO uint8_t OARL;      /*!< I2C own address register LSB */
	__IO uint8_t OARH;      /*!< I2C own address register MSB */
	uint8_t RESERVED1;      /*!< Reserved byte */
	__IO uint8_t DR;        /*!< I2C data register */
	__IO uint8_t SR1;       /*!< I2C status register 1 */
	__IO uint8_t SR2;       /*!< I2C status register 2 */
	__IO uint8_t SR3;       /*!< I2C status register 3 */
	__IO uint8_t ITR;       /*!< I2C interrupt register */
	__IO uint8_t CCRL;      /*!< I2C clock control register low */
	__IO uint8_t CCRH;      /*!< I2C clock control register high */
	__IO uint8_t TRISER;    /*!< I2C maximum rise time register */
	uint8_t RESERVED2;      /*!< Reserved byte */
}
I2C_TypeDef;

/** @addtogroup I2C_Registers_Reset_Value
	* @{
	*/
enum I2C_Registers_Reset_Value {
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
enum I2C_Registers_Bits_Definition {
	I2C_CR1_NOSTRETCH = ((uint8_t)0x80) /*!< Clock Stretching Disable (Slave mode) */,
	I2C_CR1_ENGC = ((uint8_t)0x40) /*!< General Call Enable */,
	I2C_CR1_PE = ((uint8_t)0x01) /*!< Peripheral Enable */,

	I2C_CR2_SWRST = ((uint8_t)0x80)     /*!< Software Reset */,
	I2C_CR2_POS = ((uint8_t)0x08)     /*!< Acknowledge */,
	I2C_CR2_ACK = ((uint8_t)0x04)     /*!< Acknowledge Enable */,
	I2C_CR2_STOP = ((uint8_t)0x02)     /*!< Stop Generation */,
	I2C_CR2_START = ((uint8_t)0x01)     /*!< Start Generation */,

	I2C_FREQR_FREQ = ((uint8_t)0x3F)    /*!< Peripheral Clock Frequency */,

	I2C_OARL_ADD = ((uint8_t)0xFE)     /*!< Interface Address bits [7..1] */,
	I2C_OARL_ADD0 = ((uint8_t)0x01)     /*!< Interface Address bit0 */,

	I2C_OARH_ADDMODE = ((uint8_t)0x80)  /*!< Addressing Mode (Slave mode) */,
	I2C_OARH_ADDCONF = ((uint8_t)0x40)  /*!< Address Mode Configuration */,
	I2C_OARH_ADD = ((uint8_t)0x06)  /*!< Interface Address bits [9..8] */,

	I2C_DR_DR = ((uint8_t)0xFF)  /*!< Data Register */,

	I2C_SR1_TXE = ((uint8_t)0x80)  /*!< Data Register Empty (transmitters) */,
	I2C_SR1_RXNE = ((uint8_t)0x40)  /*!< Data Register not Empty (receivers) */,
	I2C_SR1_STOPF = ((uint8_t)0x10)  /*!< Stop detection (Slave mode) */,
	I2C_SR1_ADD10 = ((uint8_t)0x08)  /*!< 10-bit header sent (Master mode) */,
	I2C_SR1_BTF = ((uint8_t)0x04)  /*!< Byte Transfer Finished */,
	I2C_SR1_ADDR = ((uint8_t)0x02)  /*!< Address sent (master mode)/matched (slave mode) */,
	I2C_SR1_SB = ((uint8_t)0x01)  /*!< Start Bit (Master mode) */,

	I2C_SR2_WUFH = ((uint8_t)0x20)  /*!< Wake-up from Halt */,
	I2C_SR2_OVR = ((uint8_t)0x08)  /*!< Overrun/Underrun */,
	I2C_SR2_AF = ((uint8_t)0x04)  /*!< Acknowledge Failure */,
	I2C_SR2_ARLO = ((uint8_t)0x02)  /*!< Arbitration Lost (master mode) */,
	I2C_SR2_BERR = ((uint8_t)0x01)  /*!< Bus Error */,

	I2C_SR3_GENCALL = ((uint8_t)0x10)  /*!< General Call Header (Slave mode) */,
	I2C_SR3_TRA = ((uint8_t)0x04)  /*!< Transmitter/Receiver */,
	I2C_SR3_BUSY = ((uint8_t)0x02)  /*!< Bus Busy */,
	I2C_SR3_MSL = ((uint8_t)0x01)  /*!< Master/Slave */,

	I2C_ITR_ITBUFEN = ((uint8_t)0x04)  /*!< Buffer Interrupt Enable */,
	I2C_ITR_ITEVTEN = ((uint8_t)0x02)  /*!< Event Interrupt Enable */,
	I2C_ITR_ITERREN = ((uint8_t)0x01)  /*!< Error Interrupt Enable */,

	I2C_CCRL_CCR = ((uint8_t)0xFF)  /*!< Clock Control Register (Master mode) */,

	I2C_CCRH_FS = ((uint8_t)0x80)  /*!< Master Mode Selection */,
	I2C_CCRH_DUTY = ((uint8_t)0x40)  /*!< Fast Mode Duty Cycle */,
	I2C_CCRH_CCR = ((uint8_t)0x0F)  /*!< Clock Control Register in Fast/Standard mode (Master mode) bits [11..8] */,

	I2C_TRISER_TRISE = ((uint8_t)0x3F)  /*!< Maximum Rise Time in Fast/Standard mode (Master mode) */,
};

/**
	* @}
	*/

/*----------------------------------------------------------------------------*/
/**
	* @brief  Interrupt Controller (ITC)
	*/

typedef struct ITC_struct
{
	__IO uint8_t ISPR1; /*!< Interrupt Software Priority register 1 */
	__IO uint8_t ISPR2; /*!< Interrupt Software Priority register 2 */
	__IO uint8_t ISPR3; /*!< Interrupt Software Priority register 3 */
	__IO uint8_t ISPR4; /*!< Interrupt Software Priority register 4 */
	__IO uint8_t ISPR5; /*!< Interrupt Software Priority register 5 */
	__IO uint8_t ISPR6; /*!< Interrupt Software Priority register 6 */
	__IO uint8_t ISPR7; /*!< Interrupt Software Priority register 7 */
	__IO uint8_t ISPR8; /*!< Interrupt Software Priority register 8 */
}
ITC_TypeDef;

/** @addtogroup ITC_Registers_Reset_Value
	* @{
	*/
enum ITC_Registers_Reset_Value {
	ITC_SPRX_RESET_VALUE = ((uint8_t)0xFF) /*!< Reset value of Software Priority registers */,
};

/**
	* @}
	*/

/** @addtogroup CPU_Registers_Bits_Definition
	* @{
	*/
enum CPU_Registers_Bits_Definition {
	CPU_CC_I1I0 = ((uint8_t)0x28) /*!< Condition Code register, I1 and I0 bits mask */,
};

/**
	* @}
	*/

/*----------------------------------------------------------------------------*/
/**
	* @brief  External Interrupt Controller (EXTI)
	*/

typedef struct EXTI_struct
{
	__IO uint8_t CR1; /*!< External Interrupt Control Register for PORTA to PORTD */
	__IO uint8_t CR2; /*!< External Interrupt Control Register for PORTE and TLI */
}
EXTI_TypeDef;

/** @addtogroup EXTI_Registers_Reset_Value
	* @{
	*/
enum EXTI_Registers_Reset_Value {
	EXTI_CR1_RESET_VALUE = ((uint8_t)0x00),
	EXTI_CR2_RESET_VALUE = ((uint8_t)0x00),
};

/**
	* @}
	*/

/** @addtogroup EXTI_Registers_Bits_Definition
	* @{
	*/
enum EXTI_Registers_Bits_Definition {
	EXTI_CR1_PDIS = ((uint8_t)0xC0) /*!< PORTD external interrupt sensitivity bits mask */,
	EXTI_CR1_PCIS = ((uint8_t)0x30) /*!< PORTC external interrupt sensitivity bits mask */,
	EXTI_CR1_PBIS = ((uint8_t)0x0C) /*!< PORTB external interrupt sensitivity bits mask */,
	EXTI_CR1_PAIS = ((uint8_t)0x03) /*!< PORTA external interrupt sensitivity bits mask */,

	EXTI_CR2_TLIS = ((uint8_t)0x04) /*!< Top level interrupt sensitivity bit mask */,
	EXTI_CR2_PEIS = ((uint8_t)0x03) /*!< PORTE external interrupt sensitivity bits mask */,
};

/**
	* @}
	*/



/*----------------------------------------------------------------------------*/
/**
	* @brief  FLASH program and Data memory (FLASH)
	*/

typedef struct FLASH_struct
{
	__IO uint8_t CR1;       /*!< Flash control register 1 */
	__IO uint8_t CR2;       /*!< Flash control register 2 */
	__IO uint8_t NCR2;      /*!< Flash complementary control register 2 */
	__IO uint8_t FPR;       /*!< Flash protection register */
	__IO uint8_t NFPR;      /*!< Flash complementary protection register */
	__IO uint8_t IAPSR;     /*!< Flash in-application programming status register */
	uint8_t RESERVED1;      /*!< Reserved byte */
	uint8_t RESERVED2;      /*!< Reserved byte */
	__IO uint8_t PUKR;      /*!< Flash program memory unprotection register */
	uint8_t RESERVED3;      /*!< Reserved byte */
	__IO uint8_t DUKR;      /*!< Data EEPROM unprotection register */
}
FLASH_TypeDef;

/** @addtogroup FLASH_Registers_Reset_Value
	* @{
	*/
enum FLASH_Registers_Reset_Value {
	FLASH_CR1_RESET_VALUE = ((uint8_t)0x00),
	FLASH_CR2_RESET_VALUE = ((uint8_t)0x00),
	FLASH_NCR2_RESET_VALUE = ((uint8_t)0xFF),
	FLASH_IAPSR_RESET_VALUE = ((uint8_t)0x40),
	FLASH_PUKR_RESET_VALUE = ((uint8_t)0x00),
	FLASH_DUKR_RESET_VALUE = ((uint8_t)0x00),
};

/**
	* @}
	*/

/** @addtogroup FLASH_Registers_Bits_Definition
	* @{
	*/
enum FLASH_Registers_Bits_Definition {
	FLASH_CR1_HALT = ((uint8_t)0x08) /*!< Standby in Halt mode mask */,
	FLASH_CR1_AHALT = ((uint8_t)0x04) /*!< Standby in Active Halt mode mask */,
	FLASH_CR1_IE = ((uint8_t)0x02) /*!< Flash Interrupt enable mask */,
	FLASH_CR1_FIX = ((uint8_t)0x01) /*!< Fix programming time mask */,

	FLASH_CR2_OPT = ((uint8_t)0x80) /*!< Select option byte mask */,
	FLASH_CR2_WPRG = ((uint8_t)0x40) /*!< Word Programming mask */,
	FLASH_CR2_ERASE = ((uint8_t)0x20) /*!< Erase block mask */,
	FLASH_CR2_FPRG = ((uint8_t)0x10) /*!< Fast programming mode mask */,
	FLASH_CR2_PRG = ((uint8_t)0x01) /*!< Program block mask */,

	FLASH_NCR2_NOPT = ((uint8_t)0x80) /*!< Select option byte mask */,
	FLASH_NCR2_NWPRG = ((uint8_t)0x40) /*!< Word Programming mask */,
	FLASH_NCR2_NERASE = ((uint8_t)0x20) /*!< Erase block mask */,
	FLASH_NCR2_NFPRG = ((uint8_t)0x10) /*!< Fast programming mode mask */,
	FLASH_NCR2_NPRG = ((uint8_t)0x01) /*!< Program block mask */,

	FLASH_IAPSR_HVOFF = ((uint8_t)0x40) /*!< End of high voltage flag mask */,
	FLASH_IAPSR_DUL = ((uint8_t)0x08) /*!< Data EEPROM unlocked flag mask */,
	FLASH_IAPSR_EOP = ((uint8_t)0x04) /*!< End of operation flag mask */,
	FLASH_IAPSR_PUL = ((uint8_t)0x02) /*!< Flash Program memory unlocked flag mask */,
	FLASH_IAPSR_WR_PG_DIS = ((uint8_t)0x01) /*!< Write attempted to protected page mask */,

	FLASH_PUKR_PUK = ((uint8_t)0xFF) /*!< Flash Program memory unprotection mask */,

	FLASH_DUKR_DUK = ((uint8_t)0xFF) /*!< Data EEPROM unprotection mask */,
};

/**
	* @}
	*/

/*----------------------------------------------------------------------------*/
/**
	* @brief  Option Bytes (OPT)
	*/
typedef struct OPT_struct
{
	__IO uint8_t OPT0;  /*!< Option byte 0: Read-out protection (not accessible in IAP mode) */
	__IO uint8_t OPT1;  /*!< Option byte 1: User boot code */
	__IO uint8_t NOPT1; /*!< Complementary Option byte 1 */
	__IO uint8_t OPT2;  /*!< Option byte 2: Alternate function remapping */
	__IO uint8_t NOPT2; /*!< Complementary Option byte 2 */
	__IO uint8_t OPT3;  /*!< Option byte 3: Watchdog option */
	__IO uint8_t NOPT3; /*!< Complementary Option byte 3 */
	__IO uint8_t OPT4;  /*!< Option byte 4: Clock option */
	__IO uint8_t NOPT4; /*!< Complementary Option byte 4 */
	__IO uint8_t OPT5;  /*!< Option byte 5: HSE clock startup */
	__IO uint8_t NOPT5; /*!< Complementary Option byte 5 */
	uint8_t RESERVED1;  /*!< Reserved Option byte*/
	uint8_t RESERVED2; /*!< Reserved Option byte*/
	__IO uint8_t OPT7;  /*!< Option byte 7: flash wait states */
	__IO uint8_t NOPT7; /*!< Complementary Option byte 7 */
}
OPT_TypeDef;

/*----------------------------------------------------------------------------*/
/**
	* @brief  Independent Watchdog (IWDG)
	*/

typedef struct IWDG_struct
{
	__IO uint8_t KR;  /*!< Key Register */
	__IO uint8_t PR;  /*!< Prescaler Register */
	__IO uint8_t RLR; /*!< Reload Register */
}
IWDG_TypeDef;

/** @addtogroup IWDG_Registers_Reset_Value
	* @{
	*/

enum IWDG_Registers_Reset_Value {
	IWDG_PR_RESET_VALUE = ((uint8_t)0x00),
	IWDG_RLR_RESET_VALUE = ((uint8_t)0xFF),
};

/**
	* @}
	*/

/*----------------------------------------------------------------------------*/
/**
	* @brief  Window Watchdog (WWDG)
	*/

typedef struct WWDG_struct
{
	__IO uint8_t CR; /*!< Control Register */
	__IO uint8_t WR; /*!< Window Register */
}
WWDG_TypeDef;

/** @addtogroup WWDG_Registers_Reset_Value
	* @{
	*/
enum WWDG_Registers_Reset_Value {
	WWDG_CR_RESET_VALUE = ((uint8_t)0x7F),
	WWDG_WR_RESET_VALUE = ((uint8_t)0x7F),
};

/**
	* @}
	*/

/** @addtogroup WWDG_Registers_Bits_Definition
	* @{
	*/
enum WWDG_Registers_Bits_Definition {
	WWDG_CR_WDGA = ((uint8_t)0x80) /*!< WDGA bit mask */,
	WWDG_CR_T6 = ((uint8_t)0x40) /*!< T6 bit mask */,
	WWDG_CR_T = ((uint8_t)0x7F) /*!< T bits mask */,

	WWDG_WR_MSB = ((uint8_t)0x80) /*!< MSB bit mask */,
	WWDG_WR_W = ((uint8_t)0x7F) /*!< W bits mask */,
};

/**
	* @}
	*/

/*----------------------------------------------------------------------------*/
/**
	* @brief  Reset Controller (RST)
	*/

typedef struct RST_struct
{
	__IO uint8_t SR; /*!< Reset status register */
}
RST_TypeDef;

/** @addtogroup RST_Registers_Bits_Definition
	* @{
	*/
enum RST_Registers_Bits_Definition {
	RST_SR_EMCF = ((uint8_t)0x10) /*!< EMC reset flag bit mask */,
	RST_SR_SWIMF = ((uint8_t)0x08) /*!< SWIM reset flag bit mask */,
	RST_SR_ILLOPF = ((uint8_t)0x04) /*!< Illegal opcode reset flag bit mask */,
	RST_SR_IWDGF = ((uint8_t)0x02) /*!< IWDG reset flag bit mask */,
	RST_SR_WWDGF = ((uint8_t)0x01) /*!< WWDG reset flag bit mask */,
};

/**
	* @}
	*/

/*----------------------------------------------------------------------------*/
/**
	* @brief  Serial Peripheral Interface (SPI)
	*/

typedef struct SPI_struct
{
	__IO uint8_t CR1;    /*!< SPI control register 1 */
	__IO uint8_t CR2;    /*!< SPI control register 2 */
	__IO uint8_t ICR;    /*!< SPI interrupt control register */
	__IO uint8_t SR;     /*!< SPI status register */
	__IO uint8_t DR;     /*!< SPI data I/O register */
	__IO uint8_t CRCPR;  /*!< SPI CRC polynomial register */
	__IO uint8_t RXCRCR; /*!< SPI Rx CRC register */
	__IO uint8_t TXCRCR; /*!< SPI Tx CRC register */
}
SPI_TypeDef;

/** @addtogroup SPI_Registers_Reset_Value
	* @{
	*/
enum SPI_Registers_Reset_Value {
	SPI_CR1_RESET_VALUE = ((uint8_t)0x00) /*!< Control Register 1 reset value */,
	SPI_CR2_RESET_VALUE = ((uint8_t)0x00) /*!< Control Register 2 reset value */,
	SPI_ICR_RESET_VALUE = ((uint8_t)0x00) /*!< Interrupt Control Register reset value */,
	SPI_SR_RESET_VALUE = ((uint8_t)0x02) /*!< Status Register reset value */,
	SPI_DR_RESET_VALUE = ((uint8_t)0x00) /*!< Data Register reset value */,
	SPI_CRCPR_RESET_VALUE = ((uint8_t)0x07) /*!< Polynomial Register reset value */,
	SPI_RXCRCR_RESET_VALUE = ((uint8_t)0x00) /*!< RX CRC Register reset value */,
	SPI_TXCRCR_RESET_VALUE = ((uint8_t)0x00) /*!< TX CRC Register reset value */,
};

/**
	* @}
	*/

/** @addtogroup SPI_Registers_Bits_Definition
	* @{
	*/
enum SPI_Registers_Bits_Definition {
	SPI_CR1_LSBFIRST = ((uint8_t)0x80) /*!< Frame format mask */,
	SPI_CR1_SPE = ((uint8_t)0x40) /*!< Enable bits mask */,
	SPI_CR1_BR = ((uint8_t)0x38) /*!< Baud rate control mask */,
	SPI_CR1_MSTR = ((uint8_t)0x04) /*!< Master Selection mask */,
	SPI_CR1_CPOL = ((uint8_t)0x02) /*!< Clock Polarity mask */,
	SPI_CR1_CPHA = ((uint8_t)0x01) /*!< Clock Phase mask */,

	SPI_CR2_BDM = ((uint8_t)0x80) /*!< Bi-directional data mode enable mask */,
	SPI_CR2_BDOE = ((uint8_t)0x40) /*!< Output enable in bi-directional mode mask */,
	SPI_CR2_CRCEN = ((uint8_t)0x20) /*!< Hardware CRC calculation enable mask */,
	SPI_CR2_CRCNEXT = ((uint8_t)0x10) /*!< Transmit CRC next mask */,
	SPI_CR2_RXONLY = ((uint8_t)0x04) /*!< Receive only mask */,
	SPI_CR2_SSM = ((uint8_t)0x02) /*!< Software slave management mask */,
	SPI_CR2_SSI = ((uint8_t)0x01) /*!< Internal slave select mask */,

	SPI_ICR_TXEI = ((uint8_t)0x80) /*!< Tx buffer empty interrupt enable mask */,
	SPI_ICR_RXEI = ((uint8_t)0x40) /*!< Rx buffer empty interrupt enable mask */,
	SPI_ICR_ERRIE = ((uint8_t)0x20) /*!< Error interrupt enable mask */,
	SPI_ICR_WKIE = ((uint8_t)0x10) /*!< Wake-up interrupt enable mask */,

	SPI_SR_BSY = ((uint8_t)0x80) /*!< Busy flag */,
	SPI_SR_OVR = ((uint8_t)0x40) /*!< Overrun flag */,
	SPI_SR_MODF = ((uint8_t)0x20) /*!< Mode fault */,
	SPI_SR_CRCERR = ((uint8_t)0x10) /*!< CRC error flag */,
	SPI_SR_WKUP = ((uint8_t)0x08) /*!< Wake-Up flag */,
	SPI_SR_TXE = ((uint8_t)0x02) /*!< Transmit buffer empty */,
	SPI_SR_RXNE = ((uint8_t)0x01) /*!< Receive buffer not empty */,
};

/**
	* @}
	*/

/*----------------------------------------------------------------------------*/
/**
	* @brief  Universal Synchronous Asynchronous Receiver Transmitter (UART1)
	*/

typedef struct UART1_struct
{
	__IO uint8_t SR;   /*!< UART1 status register */
	__IO uint8_t DR;   /*!< UART1 data register */
	__IO uint8_t BRR1; /*!< UART1 baud rate register */
	__IO uint8_t BRR2; /*!< UART1 DIV mantissa[11:8] SCIDIV fraction */
	__IO uint8_t CR1;  /*!< UART1 control register 1 */
	__IO uint8_t CR2;  /*!< UART1 control register 2 */
	__IO uint8_t CR3;  /*!< UART1 control register 3 */
	__IO uint8_t CR4;  /*!< UART1 control register 4 */
	__IO uint8_t CR5;  /*!< UART1 control register 5 */
	__IO uint8_t GTR;  /*!< UART1 guard time register */
	__IO uint8_t PSCR; /*!< UART1 prescaler register */
}
UART1_TypeDef;

/** @addtogroup UART1_Registers_Reset_Value
	* @{
	*/
enum UART1_Registers_Reset_Value {
	UART1_SR_RESET_VALUE = ((uint8_t)0xC0),
	UART1_BRR1_RESET_VALUE = ((uint8_t)0x00),
	UART1_BRR2_RESET_VALUE = ((uint8_t)0x00),
	UART1_CR1_RESET_VALUE = ((uint8_t)0x00),
	UART1_CR2_RESET_VALUE = ((uint8_t)0x00),
	UART1_CR3_RESET_VALUE = ((uint8_t)0x00),
	UART1_CR4_RESET_VALUE = ((uint8_t)0x00),
	UART1_CR5_RESET_VALUE = ((uint8_t)0x00),
	UART1_GTR_RESET_VALUE = ((uint8_t)0x00),
	UART1_PSCR_RESET_VALUE = ((uint8_t)0x00),
};

/**
	* @}
	*/

/** @addtogroup UART1_Registers_Bits_Definition
	* @{
	*/
enum UART1_Registers_Bits_Definition {
	UART1_SR_TXE = ((uint8_t)0x80) /*!< Transmit Data Register Empty mask */,
	UART1_SR_TC = ((uint8_t)0x40) /*!< Transmission Complete mask */,
	UART1_SR_RXNE = ((uint8_t)0x20) /*!< Read Data Register Not Empty mask */,
	UART1_SR_IDLE = ((uint8_t)0x10) /*!< IDLE line detected mask */,
	UART1_SR_OR = ((uint8_t)0x08) /*!< OverRun error mask */,
	UART1_SR_NF = ((uint8_t)0x04) /*!< Noise Flag mask */,
	UART1_SR_FE = ((uint8_t)0x02) /*!< Framing Error mask */,
	UART1_SR_PE = ((uint8_t)0x01) /*!< Parity Error mask */,

	UART1_BRR1_DIVM = ((uint8_t)0xFF) /*!< LSB mantissa of UART1DIV [7:0] mask */,

	UART1_BRR2_DIVM = ((uint8_t)0xF0) /*!< MSB mantissa of UART1DIV [11:8] mask */,
	UART1_BRR2_DIVF = ((uint8_t)0x0F) /*!< Fraction bits of UART1DIV [3:0] mask */,

	UART1_CR1_R8 = ((uint8_t)0x80) /*!< Receive Data bit 8 */,
	UART1_CR1_T8 = ((uint8_t)0x40) /*!< Transmit data bit 8 */,
	UART1_CR1_UARTD = ((uint8_t)0x20) /*!< UART1 Disable (for low power consumption) */,
	UART1_CR1_M = ((uint8_t)0x10) /*!< Word length mask */,
	UART1_CR1_WAKE = ((uint8_t)0x08) /*!< Wake-up method mask */,
	UART1_CR1_PCEN = ((uint8_t)0x04) /*!< Parity Control Enable mask */,
	UART1_CR1_PS = ((uint8_t)0x02) /*!< UART1 Parity Selection */,
	UART1_CR1_PIEN = ((uint8_t)0x01) /*!< UART1 Parity Interrupt Enable mask */,

	UART1_CR2_TIEN = ((uint8_t)0x80) /*!< Transmitter Interrupt Enable mask */,
	UART1_CR2_TCIEN = ((uint8_t)0x40) /*!< Transmission Complete Interrupt Enable mask */,
	UART1_CR2_RIEN = ((uint8_t)0x20) /*!< Receiver Interrupt Enable mask */,
	UART1_CR2_ILIEN = ((uint8_t)0x10) /*!< IDLE Line Interrupt Enable mask */,
	UART1_CR2_TEN = ((uint8_t)0x08) /*!< Transmitter Enable mask */,
	UART1_CR2_REN = ((uint8_t)0x04) /*!< Receiver Enable mask */,
	UART1_CR2_RWU = ((uint8_t)0x02) /*!< Receiver Wake-Up mask */,
	UART1_CR2_SBK = ((uint8_t)0x01) /*!< Send Break mask */,

	UART1_CR3_LINEN = ((uint8_t)0x40) /*!< Alternate Function output mask */,
	UART1_CR3_STOP = ((uint8_t)0x30) /*!< STOP bits [1:0] mask */,
	UART1_CR3_CKEN = ((uint8_t)0x08) /*!< Clock Enable mask */,
	UART1_CR3_CPOL = ((uint8_t)0x04) /*!< Clock Polarity mask */,
	UART1_CR3_CPHA = ((uint8_t)0x02) /*!< Clock Phase mask */,
	UART1_CR3_LBCL = ((uint8_t)0x01) /*!< Last Bit Clock pulse mask */,

	UART1_CR4_LBDIEN = ((uint8_t)0x40) /*!< LIN Break Detection Interrupt Enable mask */,
	UART1_CR4_LBDL = ((uint8_t)0x20) /*!< LIN Break Detection Length mask */,
	UART1_CR4_LBDF = ((uint8_t)0x10) /*!< LIN Break Detection Flag mask */,
	UART1_CR4_ADD = ((uint8_t)0x0F) /*!< Address of the UART1 node mask */,

	UART1_CR5_SCEN = ((uint8_t)0x20) /*!< Smart Card Enable mask */,
	UART1_CR5_NACK = ((uint8_t)0x10) /*!< Smart Card Nack Enable mask */,
	UART1_CR5_HDSEL = ((uint8_t)0x08) /*!< Half-Duplex Selection mask */,
	UART1_CR5_IRLP = ((uint8_t)0x04) /*!< Irda Low Power Selection mask */,
	UART1_CR5_IREN = ((uint8_t)0x02) /*!< Irda Enable mask */,
};

/**
	* @}
	*/

/*----------------------------------------------------------------------------*/
/**
	* @brief  Universal Synchronous Asynchronous Receiver Transmitter (UART2)
	*/

typedef struct UART2_struct
{
	__IO uint8_t SR;   /*!< UART1 status register */
	__IO uint8_t DR;   /*!< UART1 data register */
	__IO uint8_t BRR1; /*!< UART1 baud rate register */
	__IO uint8_t BRR2; /*!< UART1 DIV mantissa[11:8] SCIDIV fraction */
	__IO uint8_t CR1;  /*!< UART1 control register 1 */
	__IO uint8_t CR2;  /*!< UART1 control register 2 */
	__IO uint8_t CR3;  /*!< UART1 control register 3 */
	__IO uint8_t CR4;  /*!< UART1 control register 4 */
	__IO uint8_t CR5;  /*!< UART1 control register 5 */
	__IO uint8_t CR6;  /*!< UART1 control register 6 */
	__IO uint8_t GTR;  /*!< UART1 guard time register */
	__IO uint8_t PSCR; /*!< UART1 prescaler register */
}
UART2_TypeDef;

/** @addtogroup UART2_Registers_Reset_Value
	* @{
	*/
enum UART2_Registers_Reset_Value {
	UART2_SR_RESET_VALUE = ((uint8_t)0xC0),
	UART2_BRR1_RESET_VALUE = ((uint8_t)0x00),
	UART2_BRR2_RESET_VALUE = ((uint8_t)0x00),
	UART2_CR1_RESET_VALUE = ((uint8_t)0x00),
	UART2_CR2_RESET_VALUE = ((uint8_t)0x00),
	UART2_CR3_RESET_VALUE = ((uint8_t)0x00),
	UART2_CR4_RESET_VALUE = ((uint8_t)0x00),
	UART2_CR5_RESET_VALUE = ((uint8_t)0x00),
	UART2_CR6_RESET_VALUE = ((uint8_t)0x00),
	UART2_GTR_RESET_VALUE = ((uint8_t)0x00),
	UART2_PSCR_RESET_VALUE = ((uint8_t)0x00),
};

/**
	* @}
	*/

/** @addtogroup UART2_Registers_Bits_Definition
	* @{
	*/
enum UART2_Registers_Bits_Definition {
	UART2_SR_TXE = ((uint8_t)0x80) /*!< Transmit Data Register Empty mask */,
	UART2_SR_TC = ((uint8_t)0x40) /*!< Transmission Complete mask */,
	UART2_SR_RXNE = ((uint8_t)0x20) /*!< Read Data Register Not Empty mask */,
	UART2_SR_IDLE = ((uint8_t)0x10) /*!< IDLE line detected mask */,
	UART2_SR_OR = ((uint8_t)0x08) /*!< OverRun error mask */,
	UART2_SR_NF = ((uint8_t)0x04) /*!< Noise Flag mask */,
	UART2_SR_FE = ((uint8_t)0x02) /*!< Framing Error mask */,
	UART2_SR_PE = ((uint8_t)0x01) /*!< Parity Error mask */,

	UART2_BRR1_DIVM = ((uint8_t)0xFF) /*!< LSB mantissa of UART2DIV [7:0] mask */,

	UART2_BRR2_DIVM = ((uint8_t)0xF0) /*!< MSB mantissa of UART2DIV [11:8] mask */,
	UART2_BRR2_DIVF = ((uint8_t)0x0F) /*!< Fraction bits of UART2DIV [3:0] mask */,

	UART2_CR1_R8 = ((uint8_t)0x80) /*!< Receive Data bit 8 */,
	UART2_CR1_T8 = ((uint8_t)0x40) /*!< Transmit data bit 8 */,
	UART2_CR1_UARTD = ((uint8_t)0x20) /*!< UART2 Disable (for low power consumption) */,
	UART2_CR1_M = ((uint8_t)0x10) /*!< Word length mask */,
	UART2_CR1_WAKE = ((uint8_t)0x08) /*!< Wake-up method mask */,
	UART2_CR1_PCEN = ((uint8_t)0x04) /*!< Parity Control Enable mask */,
	UART2_CR1_PS = ((uint8_t)0x02) /*!< UART2 Parity Selection */,
	UART2_CR1_PIEN = ((uint8_t)0x01) /*!< UART2 Parity Interrupt Enable mask */,

	UART2_CR2_TIEN = ((uint8_t)0x80) /*!< Transmitter Interrupt Enable mask */,
	UART2_CR2_TCIEN = ((uint8_t)0x40) /*!< Transmission Complete Interrupt Enable mask */,
	UART2_CR2_RIEN = ((uint8_t)0x20) /*!< Receiver Interrupt Enable mask */,
	UART2_CR2_ILIEN = ((uint8_t)0x10) /*!< IDLE Line Interrupt Enable mask */,
	UART2_CR2_TEN = ((uint8_t)0x08) /*!< Transmitter Enable mask */,
	UART2_CR2_REN = ((uint8_t)0x04) /*!< Receiver Enable mask */,
	UART2_CR2_RWU = ((uint8_t)0x02) /*!< Receiver Wake-Up mask */,
	UART2_CR2_SBK = ((uint8_t)0x01) /*!< Send Break mask */,

	UART2_CR3_LINEN = ((uint8_t)0x40) /*!< Alternate Function output mask */,
	UART2_CR3_STOP = ((uint8_t)0x30) /*!< STOP bits [1:0] mask */,
	UART2_CR3_CKEN = ((uint8_t)0x08) /*!< Clock Enable mask */,
	UART2_CR3_CPOL = ((uint8_t)0x04) /*!< Clock Polarity mask */,
	UART2_CR3_CPHA = ((uint8_t)0x02) /*!< Clock Phase mask */,
	UART2_CR3_LBCL = ((uint8_t)0x01) /*!< Last Bit Clock pulse mask */,

	UART2_CR4_LBDIEN = ((uint8_t)0x40) /*!< LIN Break Detection Interrupt Enable mask */,
	UART2_CR4_LBDL = ((uint8_t)0x20) /*!< LIN Break Detection Length mask */,
	UART2_CR4_LBDF = ((uint8_t)0x10) /*!< LIN Break Detection Flag mask */,
	UART2_CR4_ADD = ((uint8_t)0x0F) /*!< Address of the UART2 node mask */,

	UART2_CR5_SCEN = ((uint8_t)0x20) /*!< Smart Card Enable mask */,
	UART2_CR5_NACK = ((uint8_t)0x10) /*!< Smart Card Nack Enable mask */,
	UART2_CR5_IRLP = ((uint8_t)0x04) /*!< Irda Low Power Selection mask */,
	UART2_CR5_IREN = ((uint8_t)0x02) /*!< Irda Enable mask */,

	UART2_CR6_LDUM = ((uint8_t)0x80) /*!< LIN Divider Update Method */,
	UART2_CR6_LSLV = ((uint8_t)0x20) /*!< LIN Slave Enable */,
	UART2_CR6_LASE = ((uint8_t)0x10) /*!< LIN Auto synchronization Enable */,
	UART2_CR6_LHDIEN = ((uint8_t)0x04) /*!< LIN Header Detection Interrupt Enable */,
	UART2_CR6_LHDF = ((uint8_t)0x02) /*!< LIN Header Detection Flag */,
	UART2_CR6_LSF = ((uint8_t)0x01) /*!< LIN Synch Field */,
};

/**
	* @}
	*/


/*----------------------------------------------------------------------------*/
/**
	* @brief  LIN Universal Asynchronous Receiver Transmitter (UART3)
	*/

typedef struct UART3_struct
{
	__IO uint8_t SR;       /*!< status register */
	__IO uint8_t DR;       /*!< data register */
	__IO uint8_t BRR1;     /*!< baud rate register */
	__IO uint8_t BRR2;     /*!< DIV mantissa[11:8] SCIDIV fraction */
	__IO uint8_t CR1;      /*!< control register 1 */
	__IO uint8_t CR2;      /*!< control register 2 */
	__IO uint8_t CR3;      /*!< control register 3 */
	__IO uint8_t CR4;      /*!< control register 4 */
	uint8_t RESERVED; /*!< Reserved byte */
	__IO uint8_t CR6;      /*!< control register 5 */
}
UART3_TypeDef;

/** @addtogroup UART3_Registers_Reset_Value
	* @{
	*/
enum UART3_Registers_Reset_Value {
	UART3_SR_RESET_VALUE = ((uint8_t)0xC0),
	UART3_BRR1_RESET_VALUE = ((uint8_t)0x00),
	UART3_BRR2_RESET_VALUE = ((uint8_t)0x00),
	UART3_CR1_RESET_VALUE = ((uint8_t)0x00),
	UART3_CR2_RESET_VALUE = ((uint8_t)0x00),
	UART3_CR3_RESET_VALUE = ((uint8_t)0x00),
	UART3_CR4_RESET_VALUE = ((uint8_t)0x00),
	UART3_CR6_RESET_VALUE = ((uint8_t)0x00),
};

/**
	* @}
	*/

/** @addtogroup UART3_Registers_Bits_Definition
	* @{
	*/
enum UART3_Registers_Bits_Definition {
	UART3_SR_TXE = ((uint8_t)0x80) /*!< Transmit Data Register Empty mask */,
	UART3_SR_TC = ((uint8_t)0x40) /*!< Transmission Complete mask */,
	UART3_SR_RXNE = ((uint8_t)0x20) /*!< Read Data Register Not Empty mask */,
	UART3_SR_IDLE = ((uint8_t)0x10) /*!< IDLE line detected mask */,
	UART3_SR_OR = ((uint8_t)0x08) /*!< OverRun error mask */,
	UART3_SR_NF = ((uint8_t)0x04) /*!< Noise Flag mask */,
	UART3_SR_FE = ((uint8_t)0x02) /*!< Framing Error mask */,
	UART3_SR_PE = ((uint8_t)0x01) /*!< Parity Error mask */,

	UART3_BRR1_DIVM = ((uint8_t)0xFF) /*!< LSB mantissa of UARTDIV [7:0] mask */,

	UART3_BRR2_DIVM = ((uint8_t)0xF0) /*!< MSB mantissa of UARTDIV [11:8] mask */,
	UART3_BRR2_DIVF = ((uint8_t)0x0F) /*!< Fraction bits of UARTDIV [3:0] mask */,

	UART3_CR1_R8 = ((uint8_t)0x80) /*!< Receive Data bit 8 */,
	UART3_CR1_T8 = ((uint8_t)0x40) /*!< Transmit data bit 8 */,
	UART3_CR1_UARTD = ((uint8_t)0x20) /*!< UART Disable (for low power consumption) */,
	UART3_CR1_M = ((uint8_t)0x10) /*!< Word length mask */,
	UART3_CR1_WAKE = ((uint8_t)0x08) /*!< Wake-up method mask */,
	UART3_CR1_PCEN = ((uint8_t)0x04) /*!< Parity control enable mask */,
	UART3_CR1_PS = ((uint8_t)0x02) /*!< Parity selection bit mask */,
	UART3_CR1_PIEN = ((uint8_t)0x01) /*!< Parity interrupt enable bit mask */,

	UART3_CR2_TIEN = ((uint8_t)0x80) /*!< Transmitter Interrupt Enable mask */,
	UART3_CR2_TCIEN = ((uint8_t)0x40) /*!< Transmission Complete Interrupt Enable mask */,
	UART3_CR2_RIEN = ((uint8_t)0x20) /*!< Receiver Interrupt Enable mask */,
	UART3_CR2_ILIEN = ((uint8_t)0x10) /*!< IDLE Line Interrupt Enable mask */,
	UART3_CR2_TEN = ((uint8_t)0x08) /*!< Transmitter Enable mask */,
	UART3_CR2_REN = ((uint8_t)0x04) /*!< Receiver Enable mask */,
	UART3_CR2_RWU = ((uint8_t)0x02) /*!< Receiver Wake-Up mask */,
	UART3_CR2_SBK = ((uint8_t)0x01) /*!< Send Break mask */,

	UART3_CR3_LINEN = ((uint8_t)0x40) /*!< Alternate Function output mask */,
	UART3_CR3_STOP = ((uint8_t)0x30) /*!< STOP bits [1:0] mask */,

	UART3_CR4_LBDIEN = ((uint8_t)0x40) /*!< LIN Break Detection Interrupt Enable mask */,
	UART3_CR4_LBDL = ((uint8_t)0x20) /*!< LIN Break Detection Length mask */,
	UART3_CR4_LBDF = ((uint8_t)0x10) /*!< LIN Break Detection Flag mask */,
	UART3_CR4_ADD = ((uint8_t)0x0F) /*!< Address of the UART3 node mask */,

	UART3_CR6_LDUM = ((uint8_t)0x80) /*!< LIN Divider Update Method */,
	UART3_CR6_LSLV = ((uint8_t)0x20) /*!< LIN Slave Enable */,
	UART3_CR6_LASE = ((uint8_t)0x10) /*!< LIN Auto synchronization Enable */,
	UART3_CR6_LHDIEN = ((uint8_t)0x04) /*!< LIN Header Detection Interrupt Enable */,
	UART3_CR6_LHDF = ((uint8_t)0x02) /*!< LIN Header Detection Flag */,
	UART3_CR6_LSF = ((uint8_t)0x01) /*!< LIN Synch Field */,
};

/**
	* @}
	*/

/*----------------------------------------------------------------------------*/
/**
	* @brief Universal Synchronous Asynchronous Receiver Transmitter (UART4)
	*/
#if defined(STM8AF622x)
typedef struct UART4_struct
{
	__IO uint8_t SR;   /*!< UART4 status register */
	__IO uint8_t DR;   /*!< UART4 data register */
	__IO uint8_t BRR1; /*!< UART4 baud rate register */
	__IO uint8_t BRR2; /*!< UART4 DIV mantissa[11:8] SCIDIV fraction */
	__IO uint8_t CR1;  /*!< UART4 control register 1 */
	__IO uint8_t CR2;  /*!< UART4 control register 2 */
	__IO uint8_t CR3;  /*!< UART4 control register 3 */
	__IO uint8_t CR4;  /*!< UART4 control register 4 */
	__IO uint8_t CR5;  /*!< UART4 control register 5 */
	__IO uint8_t CR6;  /*!< UART4 control register 6 */
	__IO uint8_t GTR;  /*!< UART4 guard time register */
	__IO uint8_t PSCR; /*!< UART4 prescaler register */
}
UART4_TypeDef;

/** @addtogroup UART4_Registers_Reset_Value
	* @{
	*/
enum UART4_Registers_Reset_Value {
	UART4_SR_RESET_VALUE = ((uint8_t)0xC0),
	UART4_BRR1_RESET_VALUE = ((uint8_t)0x00),
	UART4_BRR2_RESET_VALUE = ((uint8_t)0x00),
	UART4_CR1_RESET_VALUE = ((uint8_t)0x00),
	UART4_CR2_RESET_VALUE = ((uint8_t)0x00),
	UART4_CR3_RESET_VALUE = ((uint8_t)0x00),
	UART4_CR4_RESET_VALUE = ((uint8_t)0x00),
	UART4_CR5_RESET_VALUE = ((uint8_t)0x00),
	UART4_CR6_RESET_VALUE = ((uint8_t)0x00),
	UART4_GTR_RESET_VALUE = ((uint8_t)0x00),
	UART4_PSCR_RESET_VALUE = ((uint8_t)0x00),
};

/**
	* @}
	*/

/** @addtogroup UART4_Registers_Bits_Definition
	* @{
	*/
enum UART4_Registers_Bits_Definition {
	UART4_SR_TXE = ((uint8_t)0x80) /*!< Transmit Data Register Empty mask */,
	UART4_SR_TC = ((uint8_t)0x40) /*!< Transmission Complete mask */,
	UART4_SR_RXNE = ((uint8_t)0x20) /*!< Read Data Register Not Empty mask */,
	UART4_SR_IDLE = ((uint8_t)0x10) /*!< IDLE line detected mask */,
	UART4_SR_OR = ((uint8_t)0x08) /*!< OverRun error mask */,
	UART4_SR_NF = ((uint8_t)0x04) /*!< Noise Flag mask */,
	UART4_SR_FE = ((uint8_t)0x02) /*!< Framing Error mask */,
	UART4_SR_PE = ((uint8_t)0x01) /*!< Parity Error mask */,

	UART4_BRR1_DIVM = ((uint8_t)0xFF) /*!< LSB mantissa of UART4DIV [7:0] mask */,

	UART4_BRR2_DIVM = ((uint8_t)0xF0) /*!< MSB mantissa of UART4DIV [11:8] mask */,
	UART4_BRR2_DIVF = ((uint8_t)0x0F) /*!< Fraction bits of UART4DIV [3:0] mask */,

	UART4_CR1_R8 = ((uint8_t)0x80) /*!< Receive Data bit 8 */,
	UART4_CR1_T8 = ((uint8_t)0x40) /*!< Transmit data bit 8 */,
	UART4_CR1_UARTD = ((uint8_t)0x20) /*!< UART4 Disable (for low power consumption) */,
	UART4_CR1_M = ((uint8_t)0x10) /*!< Word length mask */,
	UART4_CR1_WAKE = ((uint8_t)0x08) /*!< Wake-up method mask */,
	UART4_CR1_PCEN = ((uint8_t)0x04) /*!< Parity Control Enable mask */,
	UART4_CR1_PS = ((uint8_t)0x02) /*!< UART4 Parity Selection */,
	UART4_CR1_PIEN = ((uint8_t)0x01) /*!< UART4 Parity Interrupt Enable mask */,

	UART4_CR2_TIEN = ((uint8_t)0x80) /*!< Transmitter Interrupt Enable mask */,
	UART4_CR2_TCIEN = ((uint8_t)0x40) /*!< Transmission Complete Interrupt Enable mask */,
	UART4_CR2_RIEN = ((uint8_t)0x20) /*!< Receiver Interrupt Enable mask */,
	UART4_CR2_ILIEN = ((uint8_t)0x10) /*!< IDLE Line Interrupt Enable mask */,
	UART4_CR2_TEN = ((uint8_t)0x08) /*!< Transmitter Enable mask */,
	UART4_CR2_REN = ((uint8_t)0x04) /*!< Receiver Enable mask */,
	UART4_CR2_RWU = ((uint8_t)0x02) /*!< Receiver Wake-Up mask */,
	UART4_CR2_SBK = ((uint8_t)0x01) /*!< Send Break mask */,

	UART4_CR3_LINEN = ((uint8_t)0x40) /*!< Alternate Function output mask */,
	UART4_CR3_STOP = ((uint8_t)0x30) /*!< STOP bits [1:0] mask */,
	UART4_CR3_CKEN = ((uint8_t)0x08) /*!< Clock Enable mask */,
	UART4_CR3_CPOL = ((uint8_t)0x04) /*!< Clock Polarity mask */,
	UART4_CR3_CPHA = ((uint8_t)0x02) /*!< Clock Phase mask */,
	UART4_CR3_LBCL = ((uint8_t)0x01) /*!< Last Bit Clock pulse mask */,

	UART4_CR4_LBDIEN = ((uint8_t)0x40) /*!< LIN Break Detection Interrupt Enable mask */,
	UART4_CR4_LBDL = ((uint8_t)0x20) /*!< LIN Break Detection Length mask */,
	UART4_CR4_LBDF = ((uint8_t)0x10) /*!< LIN Break Detection Flag mask */,
	UART4_CR4_ADD = ((uint8_t)0x0F) /*!< Address of the UART4 node mask */,

	UART4_CR5_SCEN = ((uint8_t)0x20) /*!< Smart Card Enable mask */,
	UART4_CR5_NACK = ((uint8_t)0x10) /*!< Smart Card Nack Enable mask */,
	UART4_CR5_HDSEL = ((uint8_t)0x08) /*!< Half-Duplex Selection mask */,
	UART4_CR5_IRLP = ((uint8_t)0x04) /*!< Irda Low Power Selection mask */,
	UART4_CR5_IREN = ((uint8_t)0x02) /*!< Irda Enable mask */,

	UART4_CR6_LDUM = ((uint8_t)0x80) /*!< LIN Divider Update Method */,
	UART4_CR6_LSLV = ((uint8_t)0x20) /*!< LIN Slave Enable */,
	UART4_CR6_LASE = ((uint8_t)0x10) /*!< LIN Auto synchronization Enable */,
	UART4_CR6_LHDIEN = ((uint8_t)0x04) /*!< LIN Header Detection Interrupt Enable */,
	UART4_CR6_LHDF = ((uint8_t)0x02) /*!< LIN Header Detection Flag */,
	UART4_CR6_LSF = ((uint8_t)0x01) /*!< LIN Synch Field */,
};
#endif /* STM8AF622x */

/**
	* @}
	*/

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/**
	* @brief  Controller Area Network  (CAN)
	*/

typedef struct
{
	__IO uint8_t MCR;    /*!< CAN master control register */
	__IO uint8_t MSR;    /*!< CAN master status register */
	__IO uint8_t TSR;    /*!< CAN transmit status register */
	__IO uint8_t TPR;    /*!< CAN transmit priority register */
	__IO uint8_t RFR;    /*!< CAN receive FIFO register */
	__IO uint8_t IER;    /*!< CAN interrupt enable register */
	__IO uint8_t DGR;    /*!< CAN diagnosis register */
	__IO uint8_t PSR;    /*!< CAN page selection register */

	union
	{
		struct
		{
			__IO uint8_t MCSR;
			__IO uint8_t MDLCR;
			__IO uint8_t MIDR1;
			__IO uint8_t MIDR2;
			__IO uint8_t MIDR3;
			__IO uint8_t MIDR4;
			__IO uint8_t MDAR1;
			__IO uint8_t MDAR2;
			__IO uint8_t MDAR3;
			__IO uint8_t MDAR4;
			__IO uint8_t MDAR5;
			__IO uint8_t MDAR6;
			__IO uint8_t MDAR7;
			__IO uint8_t MDAR8;
			__IO uint8_t MTSRL;
			__IO uint8_t MTSRH;
		}TxMailbox;

		struct
		{
			__IO uint8_t FR01;
			__IO uint8_t FR02;
			__IO uint8_t FR03;
			__IO uint8_t FR04;
			__IO uint8_t FR05;
			__IO uint8_t FR06;
			__IO uint8_t FR07;
			__IO uint8_t FR08;

			__IO uint8_t FR09;
			__IO uint8_t FR10;
			__IO uint8_t FR11;
			__IO uint8_t FR12;
			__IO uint8_t FR13;
			__IO uint8_t FR14;
			__IO uint8_t FR15;
			__IO uint8_t FR16;
		}Filter;

		struct
		{
			__IO uint8_t F0R1;
			__IO uint8_t F0R2;
			__IO uint8_t F0R3;
			__IO uint8_t F0R4;
			__IO uint8_t F0R5;
			__IO uint8_t F0R6;
			__IO uint8_t F0R7;
			__IO uint8_t F0R8;

			__IO uint8_t F1R1;
			__IO uint8_t F1R2;
			__IO uint8_t F1R3;
			__IO uint8_t F1R4;
			__IO uint8_t F1R5;
			__IO uint8_t F1R6;
			__IO uint8_t F1R7;
			__IO uint8_t F1R8;
		}Filter01;

		struct
		{
			__IO uint8_t F2R1;
			__IO uint8_t F2R2;
			__IO uint8_t F2R3;
			__IO uint8_t F2R4;
			__IO uint8_t F2R5;
			__IO uint8_t F2R6;
			__IO uint8_t F2R7;
			__IO uint8_t F2R8;

			__IO uint8_t F3R1;
			__IO uint8_t F3R2;
			__IO uint8_t F3R3;
			__IO uint8_t F3R4;
			__IO uint8_t F3R5;
			__IO uint8_t F3R6;
			__IO uint8_t F3R7;
			__IO uint8_t F3R8;
		}Filter23;

		struct
		{
			__IO uint8_t F4R1;
			__IO uint8_t F4R2;
			__IO uint8_t F4R3;
			__IO uint8_t F4R4;
			__IO uint8_t F4R5;
			__IO uint8_t F4R6;
			__IO uint8_t F4R7;
			__IO uint8_t F4R8;

			__IO uint8_t F5R1;
			__IO uint8_t F5R2;
			__IO uint8_t F5R3;
			__IO uint8_t F5R4;
			__IO uint8_t F5R5;
			__IO uint8_t F5R6;
			__IO uint8_t F5R7;
			__IO uint8_t F5R8;
		} Filter45;

		struct
		{
			__IO uint8_t ESR;
			__IO uint8_t EIER;
			__IO uint8_t TECR;
			__IO uint8_t RECR;
			__IO uint8_t BTR1;
			__IO uint8_t BTR2;
			uint8_t Reserved1[2];
			__IO uint8_t FMR1;
			__IO uint8_t FMR2;
			__IO uint8_t FCR1;
			__IO uint8_t FCR2;
			__IO uint8_t FCR3;
			uint8_t Reserved2[3];
		}Config;

		struct
		{
			__IO uint8_t MFMI;
			__IO uint8_t MDLCR;
			__IO uint8_t MIDR1;
			__IO uint8_t MIDR2;
			__IO uint8_t MIDR3;
			__IO uint8_t MIDR4;
			__IO uint8_t MDAR1;
			__IO uint8_t MDAR2;
			__IO uint8_t MDAR3;
			__IO uint8_t MDAR4;
			__IO uint8_t MDAR5;
			__IO uint8_t MDAR6;
			__IO uint8_t MDAR7;
			__IO uint8_t MDAR8;
			__IO uint8_t MTSRL;
			__IO uint8_t MTSRH;
		}RxFIFO;
	}Page;
} CAN_TypeDef;

/** @addtogroup CAN_Registers_Bits_Definition
	* @{
	*/
/******************************* Common ***************************************/
enum CAN_Registers_Bits_Definition {
/* CAN Master Control Register bits */
	CAN_MCR_INRQ = ((uint8_t)0x01),
	CAN_MCR_SLEEP = ((uint8_t)0x02),
	CAN_MCR_TXFP = ((uint8_t)0x04),
	CAN_MCR_RFLM = ((uint8_t)0x08),
	CAN_MCR_NART = ((uint8_t)0x10),
	CAN_MCR_AWUM = ((uint8_t)0x20),
	CAN_MCR_ABOM = ((uint8_t)0x40),
	CAN_MCR_TTCM = ((uint8_t)0x80),

/* CAN Master Status Register bits */
	CAN_MSR_INAK = ((uint8_t)0x01),
	CAN_MSR_SLAK = ((uint8_t)0x02),
	CAN_MSR_ERRI = ((uint8_t)0x04),
	CAN_MSR_WKUI = ((uint8_t)0x08),
	CAN_MSR_TX = ((uint8_t)0x10),
	CAN_MSR_RX = ((uint8_t)0x20),

/* CAN Transmit Status Register bits */
	CAN_TSR_RQCP0 = ((uint8_t)0x01),
	CAN_TSR_RQCP1 = ((uint8_t)0x02),
	CAN_TSR_RQCP2 = ((uint8_t)0x04),
	CAN_TSR_RQCP012 = ((uint8_t)0x07),
	CAN_TSR_TXOK0 = ((uint8_t)0x10),
	CAN_TSR_TXOK1 = ((uint8_t)0x20),
	CAN_TSR_TXOK2 = ((uint8_t)0x40),

	CAN_TPR_CODE0 = ((uint8_t)0x01),
	CAN_TPR_TME0 = ((uint8_t)0x04),
	CAN_TPR_TME1 = ((uint8_t)0x08),
	CAN_TPR_TME2 = ((uint8_t)0x10),
	CAN_TPR_LOW0 = ((uint8_t)0x20),
	CAN_TPR_LOW1 = ((uint8_t)0x40),
	CAN_TPR_LOW2 = ((uint8_t)0x80),
/* CAN Receive FIFO Register bits */
	CAN_RFR_FMP01 = ((uint8_t)0x03),
	CAN_RFR_FULL = ((uint8_t)0x08),
	CAN_RFR_FOVR = ((uint8_t)0x10),
	CAN_RFR_RFOM = ((uint8_t)0x20),

/* CAN Interrupt Register bits */
	CAN_IER_TMEIE = ((uint8_t)0x01),
	CAN_IER_FMPIE = ((uint8_t)0x02),
	CAN_IER_FFIE = ((uint8_t)0x04),
	CAN_IER_FOVIE = ((uint8_t)0x08),
	CAN_IER_WKUIE = ((uint8_t)0x80),


/* CAN diagnostic Register bits */
	CAN_DGR_LBKM = ((uint8_t)0x01),
	CAN_DGR_SLIM = ((uint8_t)0x02),
	CAN_DGR_SAMP = ((uint8_t)0x04),
	CAN_DGR_RX = ((uint8_t)0x08),
	CAN_DGR_TXM2E = ((uint8_t)0x10),


/* CAN page select Register bits */
	CAN_PSR_PS0 = ((uint8_t)0x01),
	CAN_PSR_PS1 = ((uint8_t)0x02),
	CAN_PSR_PS2 = ((uint8_t)0x04),

/******************** Tx MailBox & Fifo Page common bits **********************/
	CAN_MCSR_TXRQ = ((uint8_t)0x01),
	CAN_MCSR_ABRQ = ((uint8_t)0x02),
	CAN_MCSR_RQCP = ((uint8_t)0x04),
	CAN_MCSR_TXOK = ((uint8_t)0x08),
	CAN_MCSR_ALST = ((uint8_t)0x10),
	CAN_MCSR_TERR = ((uint8_t)0x20),

	CAN_MDLCR_DLC = ((uint8_t)0x0F),
	CAN_MDLCR_TGT = ((uint8_t)0x80),

	CAN_MIDR1_RTR = ((uint8_t)0x20),
	CAN_MIDR1_IDE = ((uint8_t)0x40),


/************************* Filter Page ****************************************/
/* CAN Error Status Register bits */
	CAN_ESR_EWGF = ((uint8_t)0x01),
	CAN_ESR_EPVF = ((uint8_t)0x02),
	CAN_ESR_BOFF = ((uint8_t)0x04),
	CAN_ESR_LEC0 = ((uint8_t)0x10),
	CAN_ESR_LEC1 = ((uint8_t)0x20),
	CAN_ESR_LEC2 = ((uint8_t)0x40),
	CAN_ESR_LEC = ((uint8_t)0x70),

/* CAN Error Status Register bits */
	CAN_EIER_EWGIE = ((uint8_t)0x01),
	CAN_EIER_EPVIE = ((uint8_t)0x02),
	CAN_EIER_BOFIE = ((uint8_t)0x04),
	CAN_EIER_LECIE = ((uint8_t)0x10),
	CAN_EIER_ERRIE = ((uint8_t)0x80)    ,

/* CAN transmit error counter Register bits(CAN_TECR) */
	CAN_TECR_TEC0 = ((uint8_t)0x01)    ,
	CAN_TECR_TEC1 = ((uint8_t)0x02)    ,
	CAN_TECR_TEC2 = ((uint8_t)0x04)    ,
	CAN_TECR_TEC3 = ((uint8_t)0x08)    ,
	CAN_TECR_TEC4 = ((uint8_t)0x10)    ,
	CAN_TECR_TEC5 = ((uint8_t)0x20)    ,
	CAN_TECR_TEC6 = ((uint8_t)0x40)    ,
	CAN_TECR_TEC7 = ((uint8_t)0x80)    ,

/* CAN RECEIVE error counter Register bits(CAN_TECR) */
	CAN_RECR_REC0 = ((uint8_t)0x01)    ,
	CAN_RECR_REC1 = ((uint8_t)0x02)    ,
	CAN_RECR_REC2 = ((uint8_t)0x04)    ,
	CAN_RECR_REC3 = ((uint8_t)0x08)    ,
	CAN_RECR_REC4 = ((uint8_t)0x10)    ,
	CAN_RECR_REC5 = ((uint8_t)0x20)    ,
	CAN_RECR_REC6 = ((uint8_t)0x40)    ,
	CAN_RECR_REC7 = ((uint8_t)0x80)    ,

/* CAN filter mode register bits (CAN_FMR) */
	CAN_FMR1_FML0 = ((uint8_t)0x01)    ,
	CAN_FMR1_FMH0 = ((uint8_t)0x02)    ,
	CAN_FMR1_FML1 = ((uint8_t)0x04)    ,
	CAN_FMR1_FMH1 = ((uint8_t)0x08)    ,
	CAN_FMR1_FML2 = ((uint8_t)0x10)    ,
	CAN_FMR1_FMH2 = ((uint8_t)0x20)    ,
	CAN_FMR1_FML3 = ((uint8_t)0x40)    ,
	CAN_FMR1_FMH3 = ((uint8_t)0x80)    ,

	CAN_FMR2_FML4 = ((uint8_t)0x01)    ,
	CAN_FMR2_FMH4 = ((uint8_t)0x02)    ,
	CAN_FMR2_FML5 = ((uint8_t)0x04)    ,
	CAN_FMR2_FMH5 = ((uint8_t)0x08)    ,

/* CAN filter Config register bits (CAN_FCR) */
	CAN_FCR1_FACT0 = ((uint8_t)0x01)    ,
	CAN_FCR1_FACT1 = ((uint8_t)0x10)    ,
	CAN_FCR2_FACT2 = ((uint8_t)0x01)    ,
	CAN_FCR2_FACT3 = ((uint8_t)0x10)    ,
	CAN_FCR3_FACT4 = ((uint8_t)0x01)    ,
	CAN_FCR3_FACT5 = ((uint8_t)0x10)    ,

	CAN_FCR1_FSC00 = ((uint8_t)0x02)    ,
	CAN_FCR1_FSC01 = ((uint8_t)0x04)    ,
	CAN_FCR1_FSC10 = ((uint8_t)0x20)    ,
	CAN_FCR1_FSC11 = ((uint8_t)0x40)    ,
	CAN_FCR2_FSC20 = ((uint8_t)0x02)    ,
	CAN_FCR2_FSC21 = ((uint8_t)0x04)    ,
	CAN_FCR2_FSC30 = ((uint8_t)0x20)    ,
	CAN_FCR2_FSC31 = ((uint8_t)0x40)    ,
	CAN_FCR3_FSC40 = ((uint8_t)0x02)    ,
	CAN_FCR3_FSC41 = ((uint8_t)0x04)    ,
	CAN_FCR3_FSC50 = ((uint8_t)0x20)    ,
	CAN_FCR3_FSC51 = ((uint8_t)0x40),
};

/**
	* @}
	*/

/** @addtogroup CAN_Registers_Reset_Value
	* @{
	*/
enum CAN_Registers_Reset_Value {
	CAN_MCR_RESET_VALUE = ((uint8_t)0x02),
	CAN_MSR_RESET_VALUE = ((uint8_t)0x02),
	CAN_TSR_RESET_VALUE = ((uint8_t)0x00),
	CAN_TPR_RESET_VALUE = ((uint8_t)0x0C),
	CAN_RFR_RESET_VALUE = ((uint8_t)0x00),
	CAN_IER_RESET_VALUE = ((uint8_t)0x00),
	CAN_DGR_RESET_VALUE = ((uint8_t)0x0C),
	CAN_PSR_RESET_VALUE = ((uint8_t)0x00),

	CAN_ESR_RESET_VALUE = ((uint8_t)0x00),
	CAN_EIER_RESET_VALUE = ((uint8_t)0x00),
	CAN_TECR_RESET_VALUE = ((uint8_t)0x00),
	CAN_RECR_RESET_VALUE = ((uint8_t)0x00),
	CAN_BTR1_RESET_VALUE = ((uint8_t)0x40),
	CAN_BTR2_RESET_VALUE = ((uint8_t)0x23),
	CAN_FMR1_RESET_VALUE = ((uint8_t)0x00),
	CAN_FMR2_RESET_VALUE = ((uint8_t)0x00),
	CAN_FCR_RESET_VALUE = ((uint8_t)0x00),

	CAN_MFMI_RESET_VALUE = ((uint8_t)0x00),
	CAN_MDLC_RESET_VALUE = ((uint8_t)0x00),
	CAN_MCSR_RESET_VALUE = ((uint8_t)0x00),
};

/**
	* @}
	*/

/**
	* @brief  Configuration Registers (CFG)
	*/

typedef struct CFG_struct
{
	__IO uint8_t GCR; /*!< Global Configuration register */
}
CFG_TypeDef;

/** @addtogroup CFG_Registers_Reset_Value
	* @{
	*/
enum CFG_Registers_Reset_Value {
	CFG_GCR_RESET_VALUE = ((uint8_t)0x00),
};

/**
	* @}
	*/

/** @addtogroup CFG_Registers_Bits_Definition
	* @{
	*/
enum CFG_Registers_Bits_Definition {
	CFG_GCR_SWD = ((uint8_t)0x01) /*!< Swim disable bit mask */,
	CFG_GCR_AL = ((uint8_t)0x02) /*!< Activation Level bit mask */,
};

/**
	* @}
	*/

/******************************************************************************/
/*                          Peripherals Base Address                          */
/******************************************************************************/

/** @addtogroup MAP_FILE_Base_Addresses
	* @{
	*/
enum MAP_FILE_Base_Addresses {
	OPT_BaseAddress = 0x4800,
	GPIOA_BaseAddress = 0x5000,
	GPIOB_BaseAddress = 0x5005,
	GPIOC_BaseAddress = 0x500A,
	GPIOD_BaseAddress = 0x500F,
	GPIOE_BaseAddress = 0x5014,
	GPIOF_BaseAddress = 0x5019,
	GPIOG_BaseAddress = 0x501E,
	GPIOH_BaseAddress = 0x5023,
	GPIOI_BaseAddress = 0x5028,
	FLASH_BaseAddress = 0x505A,
	EXTI_BaseAddress = 0x50A0,
	RST_BaseAddress = 0x50B3,
	CLK_BaseAddress = 0x50C0,
	WWDG_BaseAddress = 0x50D1,
	IWDG_BaseAddress = 0x50E0,
	AWU_BaseAddress = 0x50F0,
	BEEP_BaseAddress = 0x50F3,
	SPI_BaseAddress = 0x5200,
	I2C_BaseAddress = 0x5210,
	UART1_BaseAddress = 0x5230,
	UART2_BaseAddress = 0x5240,
	UART3_BaseAddress = 0x5240,
	UART4_BaseAddress = 0x5230,
	TIM1_BaseAddress = 0x5250,
	TIM2_BaseAddress = 0x5300,
	TIM3_BaseAddress = 0x5320,
	TIM4_BaseAddress = 0x5340,
	TIM5_BaseAddress = 0x5300,
	TIM6_BaseAddress = 0x5340,
	ADC1_BaseAddress = 0x53E0,
	ADC2_BaseAddress = 0x5400,
	CAN_BaseAddress = 0x5420,
	CFG_BaseAddress = 0x7F60,
	ITC_BaseAddress = 0x7F70,
	DM_BaseAddress = 0x7F90,
};

/**
	* @}
	*/

/******************************************************************************/
/*                          Peripherals declarations                          */
/******************************************************************************/

#if defined(STM8S105) || defined(STM8S005) || defined(STM8S103) || defined(STM8S003) || \
		defined(STM8S001) || defined(STM8S903) || defined(STM8AF626x) || defined(STM8AF622x)
 static __IO ADC1_TypeDef* const ADC1 = ((ADC1_TypeDef*) ADC1_BaseAddress);
 #define STM8S_STDPERIPH_LIB__ADC1 ((STM8S_STDPERIPH_LIB__NS_PREFIX ADC1_TypeDef *) ADC1_BaseAddress)
#endif /* (STM8S105)||(STM8S103)||(STM8S005)||(STM8S003)||(STM8S001)||(STM8S903)||(STM8AF626x)||(STM8AF622x)*/

#if defined(STM8S208) || defined(STM8S207) || defined (STM8S007) || defined (STM8AF52Ax) || \
		defined (STM8AF62Ax)
static __IO ADC2_TypeDef* const ADC2 = ((ADC2_TypeDef*) ADC2_BaseAddress);
#define STM8S_STDPERIPH_LIB__ADC2 ((ADC2_TypeDef *) ADC2_BaseAddress)
#endif /* (STM8S208) ||(STM8S207) || (STM8S007) || (STM8AF52Ax) || (STM8AF62Ax) */

static __IO AWU_TypeDef* const AWU = ((AWU_TypeDef *) AWU_BaseAddress);
#define STM8S_STDPERIPH_LIB__AWU (STM8S_STDPERIPH_LIB__NS_PREFIX AWU)

static __IO BEEP_TypeDef* const BEEP = ((BEEP_TypeDef *) BEEP_BaseAddress);
#define STM8S_STDPERIPH_LIB__BEEP (STM8S_STDPERIPH_LIB__NS_PREFIX BEEP)

#if defined (STM8S208) || defined (STM8AF52Ax)
 static __IO CAN_TypeDef* const CAN = ((CAN_TypeDef *) CAN_BaseAddress);
 #define STM8S_STDPERIPH_LIB__CAN (STM8S_STDPERIPH_LIB__NS_PREFIX CAN)
#endif /* (STM8S208) || (STM8AF52Ax) */

static __IO CLK_TypeDef* const CLK = ((CLK_TypeDef *) CLK_BaseAddress);
#define STM8S_STDPERIPH_LIB__CLK (STM8S_STDPERIPH_LIB__NS_PREFIX CLK)

static __IO EXTI_TypeDef* const EXTI = ((EXTI_TypeDef *) EXTI_BaseAddress);
#define STM8S_STDPERIPH_LIB__EXTI (STM8S_STDPERIPH_LIB__NS_PREFIX EXTI)

static __IO FLASH_TypeDef* const FLASH = ((FLASH_TypeDef *) FLASH_BaseAddress);
#define STM8S_STDPERIPH_LIB__FLASH (STM8S_STDPERIPH_LIB__NS_PREFIX FLASH)

static __IO OPT_TypeDef* const OPT = ((OPT_TypeDef *) OPT_BaseAddress);
#define STM8S_STDPERIPH_LIB__OPT (STM8S_STDPERIPH_LIB__NS_PREFIX OPT)

static __IO GPIO_TypeDef* const GPIOA = ((GPIO_TypeDef *) GPIOA_BaseAddress);
#define STM8S_STDPERIPH_LIB__GPIOA (STM8S_STDPERIPH_LIB__NS_PREFIX GPIOA)

static __IO GPIO_TypeDef* const GPIOB = ((GPIO_TypeDef *) GPIOB_BaseAddress);
#define STM8S_STDPERIPH_LIB__GPIOB (STM8S_STDPERIPH_LIB__NS_PREFIX GPIOB)

static __IO GPIO_TypeDef* const GPIOC = ((GPIO_TypeDef *) GPIOC_BaseAddress);
#define STM8S_STDPERIPH_LIB__GPIOC (STM8S_STDPERIPH_LIB__NS_PREFIX GPIOC)

static __IO GPIO_TypeDef* const GPIOD = ((GPIO_TypeDef *) GPIOD_BaseAddress);
#define STM8S_STDPERIPH_LIB__GPIOD (STM8S_STDPERIPH_LIB__NS_PREFIX GPIOD)

static __IO GPIO_TypeDef* const GPIOE = ((GPIO_TypeDef *) GPIOE_BaseAddress);
#define STM8S_STDPERIPH_LIB__GPIOE (STM8S_STDPERIPH_LIB__NS_PREFIX GPIOE)

static __IO GPIO_TypeDef* const GPIOF = ((GPIO_TypeDef *) GPIOF_BaseAddress);
#define STM8S_STDPERIPH_LIB__GPIOF (STM8S_STDPERIPH_LIB__NS_PREFIX GPIOF)

#if defined(STM8S207) || defined (STM8S007) || defined(STM8S208) || defined(STM8S105) || \
		defined(STM8S005) || defined (STM8AF52Ax) || defined (STM8AF62Ax) || defined (STM8AF626x)
 static __IO GPIO_TypeDef* const GPIOG = ((GPIO_TypeDef *) GPIOG_BaseAddress);
 #define STM8S_STDPERIPH_LIB__GPIOG (STM8S_STDPERIPH_LIB__NS_PREFIX GPIOG)
#endif /* (STM8S208) ||(STM8S207)  || (STM8S105) || (STM8AF52Ax) || (STM8AF62Ax) || (STM8AF626x) */

#if defined(STM8S207) || defined (STM8S007) || defined(STM8S208) || defined (STM8AF52Ax) || \
		defined (STM8AF62Ax)
 static __IO GPIO_TypeDef* const GPIOH = ((GPIO_TypeDef *) GPIOH_BaseAddress);
 #define STM8S_STDPERIPH_LIB__GPIOH (STM8S_STDPERIPH_LIB__NS_PREFIX GPIOH)
 static __IO GPIO_TypeDef* const GPIOI = ((GPIO_TypeDef *) GPIOI_BaseAddress);
 #define STM8S_STDPERIPH_LIB__GPIOI (STM8S_STDPERIPH_LIB__NS_PREFIX GPIOI)
#endif /* (STM8S208) ||(STM8S207) || (STM8AF62Ax) || (STM8AF52Ax) */

#ifdef STM8S_STDPERIPH_LIB__GPIOA 
	#define STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__A(X_ARG) X_ARG(A, (::STM8S_StdPeriph_Lib::GPIOA), (::STM8S_StdPeriph_Lib::GPIOA_BaseAddress))
#else
	#define STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__A(X_ARG)
#endif
#ifdef STM8S_STDPERIPH_LIB__GPIOB 
	#define STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__B(X_ARG) X_ARG(B, (::STM8S_StdPeriph_Lib::GPIOB), (::STM8S_StdPeriph_Lib::GPIOB_BaseAddress))
#else
	#define STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__B(X_ARG)
#endif
#ifdef STM8S_STDPERIPH_LIB__GPIOC 
	#define STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__C(X_ARG) X_ARG(C, (::STM8S_StdPeriph_Lib::GPIOC), (::STM8S_StdPeriph_Lib::GPIOC_BaseAddress))
#else
	#define STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__C(X_ARG)
#endif
#ifdef STM8S_STDPERIPH_LIB__GPIOD 
	#define STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__D(X_ARG) X_ARG(D, (::STM8S_StdPeriph_Lib::GPIOD), (::STM8S_StdPeriph_Lib::GPIOD_BaseAddress))
#else
	#define STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__D(X_ARG)
#endif
#ifdef STM8S_STDPERIPH_LIB__GPIOE 
	#define STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__E(X_ARG) X_ARG(E, (::STM8S_StdPeriph_Lib::GPIOE), (::STM8S_StdPeriph_Lib::GPIOE_BaseAddress))
#else
	#define STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__E(X_ARG)
#endif
#ifdef STM8S_STDPERIPH_LIB__GPIOF 
	#define STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__F(X_ARG) X_ARG(F, (::STM8S_StdPeriph_Lib::GPIOF), (::STM8S_StdPeriph_Lib::GPIOF_BaseAddress))
#else
	#define STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__F(X_ARG)
#endif
#ifdef STM8S_STDPERIPH_LIB__GPIOG 
	#define STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__G(X_ARG) X_ARG(G, (::STM8S_StdPeriph_Lib::GPIOG), (::STM8S_StdPeriph_Lib::GPIOG_BaseAddress))
#else
	#define STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__G(X_ARG)
#endif
#ifdef STM8S_STDPERIPH_LIB__GPIOH 
	#define STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__H(X_ARG) X_ARG(H, (::STM8S_StdPeriph_Lib::GPIOH), (::STM8S_StdPeriph_Lib::GPIOH_BaseAddress))
#else
	#define STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__H(X_ARG)
#endif
#ifdef STM8S_STDPERIPH_LIB__GPIOI 
	#define STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__I(X_ARG) X_ARG(I, (::STM8S_StdPeriph_Lib::GPIOI), (::STM8S_StdPeriph_Lib::GPIOI_BaseAddress))
#else
	#define STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__I(X_ARG)
#endif


#define STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X(X_ARG) \
	STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__A(X_ARG) \
	STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__B(X_ARG) \
	STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__C(X_ARG) \
	STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__D(X_ARG) \
	STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__E(X_ARG) \
	STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__F(X_ARG) \
	STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__G(X_ARG) \
	STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__H(X_ARG) \
	STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X__IMPL__I(X_ARG)


static __IO RST_TypeDef* const RST = ((RST_TypeDef *) RST_BaseAddress);
#define STM8S_STDPERIPH_LIB__RST (STM8S_STDPERIPH_LIB__NS_PREFIX RST)

static __IO WWDG_TypeDef* const WWDG = ((WWDG_TypeDef *) WWDG_BaseAddress);
#define STM8S_STDPERIPH_LIB__WWDG (STM8S_STDPERIPH_LIB__NS_PREFIX WWDG)
static __IO IWDG_TypeDef* const IWDG = ((IWDG_TypeDef *) IWDG_BaseAddress);
#define STM8S_STDPERIPH_LIB__IWDG (STM8S_STDPERIPH_LIB__NS_PREFIX IWDG)

static __IO SPI_TypeDef* const SPI = ((SPI_TypeDef *) SPI_BaseAddress);
#define STM8S_STDPERIPH_LIB__SPI (STM8S_STDPERIPH_LIB__NS_PREFIX SPI)
static __IO I2C_TypeDef* const I2C = ((I2C_TypeDef *) I2C_BaseAddress);
#define STM8S_STDPERIPH_LIB__I2C (STM8S_STDPERIPH_LIB__NS_PREFIX I2C)

#if defined(STM8S208) || defined(STM8S207) || defined (STM8S007) || defined(STM8S103) || \
		defined(STM8S003) || defined(STM8S001) || defined(STM8S903) || defined (STM8AF52Ax) || defined (STM8AF62Ax)
 static __IO UART1_TypeDef* const UART1 = ((UART1_TypeDef *) UART1_BaseAddress);
 #define STM8S_STDPERIPH_LIB__UART1 (STM8S_STDPERIPH_LIB__NS_PREFIX UART1)
#endif /* (STM8S208) ||(STM8S207)  || (STM8S103) || (STM8S001) || (STM8S903) || (STM8AF52Ax) || (STM8AF62Ax) */

#if defined (STM8S105) || defined (STM8S005) || defined (STM8AF626x)
 static __IO UART2_TypeDef* const UART2 = ((UART2_TypeDef *) UART2_BaseAddress);
 #define STM8S_STDPERIPH_LIB__UART2 (STM8S_STDPERIPH_LIB__NS_PREFIX UART2)
#endif /* STM8S105 || STM8S005 || STM8AF626x */

#if defined(STM8S208) ||defined(STM8S207) || defined (STM8S007) || defined (STM8AF52Ax) || \
		defined (STM8AF62Ax)
 static __IO UART3_TypeDef* const UART3 = ((UART3_TypeDef *) UART3_BaseAddress);
 #define STM8S_STDPERIPH_LIB__UART3 (STM8S_STDPERIPH_LIB__NS_PREFIX UART3)
#endif /* (STM8S208) ||(STM8S207) || (STM8AF62Ax) || (STM8AF52Ax) */

#if defined(STM8AF622x)
 static __IO UART4_TypeDef* const UART4 = ((UART4_TypeDef *) UART4_BaseAddress);
 #define STM8S_STDPERIPH_LIB__UART4 (STM8S_STDPERIPH_LIB__NS_PREFIX UART4)
#endif /* (STM8AF622x) */

static __IO TIM1_TypeDef* const TIM1 = ((TIM1_TypeDef *) TIM1_BaseAddress);
#define STM8S_STDPERIPH_LIB__TIM1 (STM8S_STDPERIPH_LIB__NS_PREFIX TIM1)

#if defined(STM8S208) || defined(STM8S207) || defined (STM8S007) || defined(STM8S103) || \
		defined(STM8S003) || defined(STM8S001) || defined(STM8S105) || defined(STM8S005) || defined (STM8AF52Ax) || \
		defined (STM8AF62Ax) || defined (STM8AF626x)
 static __IO TIM2_TypeDef* const TIM2 = ((TIM2_TypeDef *) TIM2_BaseAddress);
 #define STM8S_STDPERIPH_LIB__TIM2 (STM8S_STDPERIPH_LIB__NS_PREFIX TIM2)
#endif /* (STM8S208) ||(STM8S207)  || (STM8S103) || (STM8S001) || (STM8S105) || (STM8AF52Ax) || (STM8AF62Ax) || (STM8AF626x)*/

#if defined(STM8S208) || defined(STM8S207) || defined (STM8S007) || defined(STM8S105) || \
		defined(STM8S005) || defined (STM8AF52Ax) || defined (STM8AF62Ax) || defined (STM8AF626x)
 static __IO TIM3_TypeDef* const TIM3 = ((TIM3_TypeDef *) TIM3_BaseAddress);
 #define STM8S_STDPERIPH_LIB__TIM3 (STM8S_STDPERIPH_LIB__NS_PREFIX TIM3)
#endif /* (STM8S208) ||(STM8S207)  || (STM8S105) || (STM8AF62Ax) || (STM8AF52Ax) || (STM8AF626x)*/

#if defined(STM8S208) ||defined(STM8S207) || defined (STM8S007) || defined(STM8S103) || \
		defined(STM8S003) || defined(STM8S001) || defined(STM8S105) || defined(STM8S005) || defined (STM8AF52Ax) || \
		defined (STM8AF62Ax) || defined (STM8AF626x)
 static __IO TIM4_TypeDef* const TIM4 = ((TIM4_TypeDef *) TIM4_BaseAddress);
 #define STM8S_STDPERIPH_LIB__TIM4 (STM8S_STDPERIPH_LIB__NS_PREFIX TIM4)
#endif /* (STM8S208) ||(STM8S207)  || (STM8S103) || (STM8S001) || (STM8S105) || (STM8AF52Ax) || (STM8AF62Ax) || (STM8AF626x)*/

#if defined (STM8S903) || defined (STM8AF622x)
 static __IO TIM5_TypeDef* const TIM5 = ((TIM5_TypeDef *) TIM5_BaseAddress);
 #define STM8S_STDPERIPH_LIB__TIM5 (STM8S_STDPERIPH_LIB__NS_PREFIX TIM5)
 static __IO TIM6_TypeDef* const TIM6 = ((TIM6_TypeDef *) TIM6_BaseAddress);
 #define STM8S_STDPERIPH_LIB__TIM6 (STM8S_STDPERIPH_LIB__NS_PREFIX TIM6)
#endif /* (STM8S903) || (STM8AF622x) */

static __IO ITC_TypeDef* const ITC = ((ITC_TypeDef *) ITC_BaseAddress);
#define STM8S_STDPERIPH_LIB__ITC (STM8S_STDPERIPH_LIB__NS_PREFIX ITC)

static __IO CFG_TypeDef* const CFG = ((CFG_TypeDef *) CFG_BaseAddress);
#define STM8S_STDPERIPH_LIB__CFG (STM8S_STDPERIPH_LIB__NS_PREFIX CFG)

// static __IO DM_TypeDef* const DM = ((DM_TypeDef *) DM_BaseAddress);
// #define STM8S_STDPERIPH_LIB__DM (STM8S_STDPERIPH_LIB__NS_PREFIX DM)


#ifdef USE_STDPERIPH_DRIVER
 #include "stm8s_conf.h"
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
 static inline void halt() { ::_halt_(); } /* Halt */
#elif defined(_COSMIC_)
 static inline void enableInterrupts() { {_asm("rim\n");}; } /* enable interrupts */
 static inline void disableInterrupts() { {_asm("sim\n");}; } /* disable interrupts */
 static inline void rim() { {_asm("rim\n");}; } /* enable interrupts */
 static inline void sim() { {_asm("sim\n");}; } /* disable interrupts */
 static inline void nop() { {_asm("nop\n");}; } /* No Operation */
 static inline void trap() { {_asm("trap\n");}; } /* Trap (soft IT) */
 static inline void wfi() { {_asm("wfi\n");}; } /* Wait For Interrupt */
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
 static inline void halt() { __halt(); } /* Halt */
#endif /*_RAISONANCE_*/

/*============================== Interrupt vector Handling ========================*/

#ifdef _COSMIC_
 #define STM8S_STDPERIPH_LIB__INTERRUPT_HANDLER(a,b) @far @interrupt void a(void)
 #define STM8S_STDPERIPH_LIB__INTERRUPT_HANDLER_TRAP(a) void @far @interrupt a(void)
#endif /* _COSMIC_ */

#ifdef _RAISONANCE_
 #define STM8S_STDPERIPH_LIB__INTERRUPT_HANDLER(a,b) void a(void) interrupt b
 #define STM8S_STDPERIPH_LIB__INTERRUPT_HANDLER_TRAP(a) void a(void) trap
#endif /* _RAISONANCE_ */

#ifdef _IAR_
 #define STM8S_STDPERIPH_LIB__STRINGVECTOR(x) #x
 #define STM8S_STDPERIPH_LIB__VECTOR_ID(x) STM8S_STDPERIPH_LIB__STRINGVECTOR( vector = (x) )
 #define STM8S_STDPERIPH_LIB__INTERRUPT_HANDLER( a, b )  \
 _Pragma( STM8S_STDPERIPH_LIB__VECTOR_ID( (b)+2 ) )        \
 __interrupt void (a)( void )
 #define STM8S_STDPERIPH_LIB__INTERRUPT_HANDLER_TRAP(a) \
 _Pragma( STM8S_STDPERIPH_LIB__VECTOR_ID( 1 ) ) \
 __interrupt void (a) (void)
#endif /* _IAR_ */

/*============================== Interrupt Handler declaration ========================*/
#ifdef _COSMIC_
 #define STM8S_STDPERIPH_LIB__INTERRUPT @far @interrupt
#elif defined(_IAR_)
 #define STM8S_STDPERIPH_LIB__INTERRUPT __interrupt
#endif /* _COSMIC_ */

#if defined (STM8S103) || defined (STM8S003) || defined (STM8S001)
	#define STM8S_STDPERIPH_LIB__TLI_ISR                 0
	#define STM8S_STDPERIPH_LIB__AWU_ISR                 1
	#define STM8S_STDPERIPH_LIB__CLK_ISR                 2
	#define STM8S_STDPERIPH_LIB__EXTI0_ISR               3
	#define STM8S_STDPERIPH_LIB__EXTI1_ISR               4
	#define STM8S_STDPERIPH_LIB__EXTI2_ISR               5
	#define STM8S_STDPERIPH_LIB__EXTI3_ISR               6
	#define STM8S_STDPERIPH_LIB__EXTI4_ISR               7
	// #define STM8S_STDPERIPH_LIB__CAN_RX_ISR              8   /* dual use, device dependent */
	// #define STM8S_STDPERIPH_LIB__EXTI5_ISR               8   /* dual use, device dependent */
	// #define STM8S_STDPERIPH_LIB__CAN_TX_ISR              9
	#define STM8S_STDPERIPH_LIB__SPI_ISR                 10
	#define STM8S_STDPERIPH_LIB__TIM1_OVF_ISR            11
	#define STM8S_STDPERIPH_LIB__TIM1_CC_ISR             12
	#define STM8S_STDPERIPH_LIB__TIM2_OVF_ISR            13   /* dual use, device dependent */
	// #define STM8S_STDPERIPH_LIB__TIM5_OVF_ISR            13   /* dual use, device dependent */
	#define STM8S_STDPERIPH_LIB__TIM2_CC_ISR             14   /* dual use, device dependent */
	// #define STM8S_STDPERIPH_LIB__TIM5_CC_ISR             14   /* dual use, device dependent */
	// #define STM8S_STDPERIPH_LIB__TIM3_OVF_ISR            15
	// #define STM8S_STDPERIPH_LIB__TIM3_CC_ISR             16
	#define STM8S_STDPERIPH_LIB__UART1_TXC_ISR           17
	#define STM8S_STDPERIPH_LIB__UART1_RXC_ISR           18
	#define STM8S_STDPERIPH_LIB__I2C_ISR                 19
	// #define STM8S_STDPERIPH_LIB__UART2_3_4_TXC_ISR       20
	// #define STM8S_STDPERIPH_LIB__UART2_3_4_RXC_ISR       21
	#define STM8S_STDPERIPH_LIB__ADC1_ISR                22   /* dual use, device dependent */
	// #define STM8S_STDPERIPH_LIB__ADC2_ISR                22   /* dual use, device dependent */
	#define STM8S_STDPERIPH_LIB__TIM4_ISR                23   /* dual use, device dependent */
	// #define STM8S_STDPERIPH_LIB__TIM6_ISR                23   /* dual use, device dependent */
	#define STM8S_STDPERIPH_LIB__FLASH_ISR               24
#endif

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
							variable: the bit is reset if it returns 0 else the bit is set.
							This method generates not an optimised code yet.
-----------------------------------------------------------------------------*/
#if 0
//# TODO template it
#define SetBit(VAR,Place)         ( (VAR) |= (uint8_t)((uint8_t)1<<(uint8_t)(Place)) )
#define ClrBit(VAR,Place)         ( (VAR) &= (uint8_t)((uint8_t)((uint8_t)1<<(uint8_t)(Place))^(uint8_t)255) )

#define ChgBit(VAR,Place)         ( (VAR) ^= (uint8_t)((uint8_t)1<<(uint8_t)(Place)) )
#define AffBit(VAR,Place,Value)   ((Value) ? \
																	 ((VAR) |= ((uint8_t)1<<(Place))) : \
																	 ((VAR) &= (((uint8_t)1<<(Place))^(uint8_t)255)))
#define MskBit(Dest,Msk,Src)      ( (Dest) = ((Msk) & (Src)) | ((~(Msk)) & (Dest)) )

#define ValBit(VAR,Place)         ((uint8_t)(VAR) & (uint8_t)((uint8_t)1<<(uint8_t)(Place)))

#define BYTE_0(n)                 ((uint8_t)((n) & (uint8_t)0xFF))        /*!< Returns the low byte of the 32-bit value */
#define BYTE_1(n)                 ((uint8_t)(BYTE_0((n) >> (uint8_t)8)))  /*!< Returns the second byte of the 32-bit value */
#define BYTE_2(n)                 ((uint8_t)(BYTE_0((n) >> (uint8_t)16))) /*!< Returns the third byte of the 32-bit value */
#define BYTE_3(n)                 ((uint8_t)(BYTE_0((n) >> (uint8_t)24))) /*!< Returns the high byte of the 32-bit value */

#define UNUSED(x)                 ((void)(x))
/*============================== Assert Macros ====================================*/
#define IS_STATE_VALUE_OK(SensitivityValue) \
	(((SensitivityValue) == ENABLE) || \
	 ((SensitivityValue) == DISABLE))

/*-----------------------------------------------------------------------------
Method : II
Description : Handle directly the bit.
Comments :    The idea is to handle directly with the bit name. For that, it is
							necessary to have RAM area descriptions (example: HW register...)
							and the following command line for each area.
							This method generates the most optimized code.
-----------------------------------------------------------------------------*/


enum {
	AREA = 0x00     /* The area of bits begins at address 0x10. */,
};

// template<class T> static inline T BitClr(unsigned BIT) { return ( *((unsigned char *) (AREA+(BIT)/8)) &= (~(1<<(7-(BIT)%8))) ); }
#define BitClr(BIT)  ( *((unsigned char *) (AREA+(BIT)/8)) &= (~(1<<(7-(BIT)%8))) )
#define BitSet(BIT)  ( *((unsigned char *) (AREA+(BIT)/8)) |= (1<<(7-(BIT)%8)) )
#define BitVal(BIT)  ( *((unsigned char *) (AREA+(BIT)/8)) & (1<<(7-(BIT)%8)) )

#endif

#pragma pop_macro("__IO")
#pragma pop_macro("__O")
#pragma pop_macro("__I")


#ifdef __cplusplus
} //# namespace
#endif

/* Exported functions ------------------------------------------------------- */

#endif /* __STM8S_H */

/**
	* @}
	*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
