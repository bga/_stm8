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

namespace Bga { namespace Mcu { namespace Hal {namespace Pin {

#pragma push_macro("Self")

#undef Self
#define Self PushPullHiZ

template<unsigned portAddrArg, unsigned bitNoArg>
struct Self {
	enum {
		portAddr = portAddrArg,
		bitNo = bitNoArg,
	};

	static_assert_lte(bitNo, 7);


	GPIO_TypeDef* const m_gpioPort = (GPIO_TypeDef*)portAddr;
	void init() {
	}
	void hiZ() {
		clearBit(m_gpioPort->DDR, bitNo);
		clearBit(m_gpioPort->CR1, bitNo);
		clearBit(m_gpioPort->ODR, bitNo);
	}
	void on() {
		setBit(m_gpioPort->DDR, bitNo);
		setBit(m_gpioPort->CR1, bitNo);
		setBit(m_gpioPort->ODR, bitNo);
	}
	void off() {
		setBit(m_gpioPort->DDR, bitNo);
		setBit(m_gpioPort->CR1, bitNo);
		clearBit(m_gpioPort->ODR, bitNo);
	}
	void setValue(bool v) {
		(!!v) ? on() : off(); 
	}
	void toggle() {
		setBit(m_gpioPort->DDR, bitNo);
		setBit(m_gpioPort->CR1, bitNo);
		toggleBit(m_gpioPort->ODR, bitNo);
	}
	Bool read() const {
		return this->m_gpioPort->IDR & _BV(this->bitNo);
	}
};
#ifndef NDEBUG
	template<>
	struct Self<swimBaseAddress, swimPinNo>: Ignore {  };
#endif // NDEBUG

#pragma pop_macro("Self")

} } } } //# namespace
