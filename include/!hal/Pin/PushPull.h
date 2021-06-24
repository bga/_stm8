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

#pragma push_macro("Self")

#undef Self
#define Self PushPull

namespace Bga { namespace Mcu { namespace Hal {namespace Pin {

namespace details {

template<unsigned portAddrArg, unsigned bitNoArg>
struct Self: details::Base<portAddrArg, bitNoArg>, InitJoiner<
	portAddrArg, 
	0, 0, 
	_BV(bitNoArg), 0, 
	_BV(bitNoArg), 0, 
	0, 0 
> {
	::STM8S_StdPeriph_Lib::GPIO_TypeDef* const m_gpioPort = (::STM8S_StdPeriph_Lib::GPIO_TypeDef*)portAddrArg;
	
	void init() {
		setBit(this->m_gpioPort->DDR, this->bitNo);
		setBit(this->m_gpioPort->CR1, this->bitNo);
	}
	void on() {
		setBit(this->m_gpioPort->ODR, this->bitNo);
	}
	void off() {
		clearBit(this->m_gpioPort->ODR, this->bitNo);
	}
	void setValue(bool v) {
		(!!v) ? this->on() : this->off(); 
	}
	void toggle() {
		toggleBit(this->m_gpioPort->ODR, this->bitNo);
	}
};

} //# namespace details

template<unsigned portAddrArg, unsigned bitNoArg>
struct Self: details::Wrap<portAddrArg, bitNoArg, details::Self> {  };

#pragma pop_macro("Self")

} } } } //# namespace
