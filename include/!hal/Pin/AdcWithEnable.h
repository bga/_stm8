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

#include "common.h"
#include <!hal/Adc.h>
#include <delay.h>


namespace Bga { namespace Mcu { namespace Hal {namespace Pin {

#pragma push_macro("Self")

#undef Self
#define Self AdcWithEnable

template<
	unsigned adcPortAddrArg, unsigned adcBitNoArg,
	unsigned adcChannelNoArg,
	unsigned delayTimeUsArg>
struct Self {
	enum {
		adcPortAddr = adcPortAddrArg,
		adcBitNo = adcBitNoArg,
		adcChannelNo = adcChannelNoArg,
		delayTimeUs = delayTimeUsArg,
	};

	static_assert_lte(adcBitNoArg, 7);
	static_assert_lte(adcChannelNo, Adc_maxChannelNo);


	PullHiZ<adcPortAddr, adcBitNo> m_adcPin;

	void init() {
		m_adcPin.init();
	}

	Adc_Value readSync() {
		// HiZ
		m_adcPin.hiZ();

		::delay_us(delayTimeUs);
		Adc_Value ret = Adc_readSync(adcChannelNo);

		m_adcPin.off();
		return ret;
	}
};

#pragma pop_macro("Self")

} } } } //# namespace
