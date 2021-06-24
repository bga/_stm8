/*
	Copyright 2021 Bga <bga.email@gmail.com>

	Licensed under the Apache License, Version 2.0 (the "License");
	you may not use this file except in compliance with the License.
	You may obtain a copy of the License at

		http://www.apache.org/licenses/LICENSE-2.0

	Unless required by applicable law or agreed to in writing, software
	distributed under the License is distributed on an "AS IS" BASIS,
	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
	See the License for the specific language governing permissions and
	limitations under the License.
*/

#pragma once

#include <!hal/common.h>

namespace Bga { namespace Mcu { namespace Hal {

typedef FU16 Adc_Value;

enum {
	//# TODO calculate real RC time
	Adc_maxValue = 1024,
	Adc_maxChannelNo = 16,
};

void Adc_init() {
	using namespace ::STM8S_StdPeriph_Lib;
	/* right-align data */
	setBitMask(ADC1->CR2, ADC1_CR2_ALIGN);

	//# ADC clock = fMasterClock / 18
//	setBitMaskedValues(ADC1->CR2, 4, 0x07, 7);

	/* wake ADC from power down */
	setBitMask(ADC1->CR1, ADC1_CR1_ADON);
}

void inline Adc_initChannel(FU8 channelNo) {
	using namespace ::STM8S_StdPeriph_Lib;
	#if 0
	if(channelNo < 8) {
		setBit(ADC1->TDRL, channelNo);
	}
	else {
		setBit(ADC1->TDRH, channelNo - 8);
	}
	#else
		setBit(((U16 *)&ADC1->TDRH)[0], channelNo);
	#endif
}

void Adc_setChannel(FU8 channelNo) {
	using namespace ::STM8S_StdPeriph_Lib;
	setBitMaskedValues(ADC1->CSR, 0, 0x0F, channelNo);
}

void Adc_readStart() {
	using namespace ::STM8S_StdPeriph_Lib;
	setBitMask(ADC1->CR1, ADC1_CR1_ADON);
}

#if BGA__HAL__CONFIG__ENABLE_ENTROPY_GEN
	namespace ::Random {
		void Entropy_pushBit(Bool x);
	} //# namespace
#endif

Adc_Value Adc_read() {
	using namespace ::STM8S_StdPeriph_Lib;
	while(!(ADC1->CSR & ADC1_CSR_EOC));
	U8 adcL = ADC1->DRL;
	U8 adcH = ADC1->DRH;
	clearBitMask(ADC1->CSR, ADC1_CSR_EOC);

	#if BGA__HAL__CONFIG__ENABLE_ENTROPY_GEN
		#undef  BGA__HAL__HAS_ENTROPY_SOURCE
		#define BGA__HAL__HAS_ENTROPY_SOURCE

		::Bga::Random::Entropy_pushBit(adcL & 1);
	#endif

	return (adcL | (adcH << 8));
}

#if BGA__HAL_CONFIG__ENABLE_ENTROPY_GEN

#endif


Adc_Value Adc_readSync(FU8 channelNo) {
	Adc_setChannel(channelNo);
	Adc_readStart();
	return Adc_read();
}

} } } //# namespace
