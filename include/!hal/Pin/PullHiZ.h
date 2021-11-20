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
// #pragma push_macro("SelfNS")
// #pragma push_macro("Self_impl")

#undef Self
#define Self PullHiZ
// #define Self_impl BGA__CONCAT(Self, _impl)
// #undef SelfNS
// #define SelfNS BGA__CONCAT(Self, NS)

// namespace SelfNS { namespace details {
namespace details {

template<unsigned portAddrArg, unsigned bitNoArg>
struct Self: Base<portAddrArg, bitNoArg>, InitJoiner<
	portAddrArg, 
	0, 0, 
	0, 0, 
	0, _BV(bitNoArg), 
	0, 0 
> {
	::STM8S_StdPeriph_Lib::GPIO_TypeDef* const m_gpioPort = (::STM8S_StdPeriph_Lib::GPIO_TypeDef*)portAddrArg;
	
	void init() {
		clearBit(this->m_gpioPort->CR1, this->bitNo);
		clearBit(this->m_gpioPort->CR2, this->bitNo);
	}
	void hiZ() {
		clearBit(this->m_gpioPort->DDR, this->bitNo);
	}
	void off() {
		setBit(this->m_gpioPort->DDR, this->bitNo);
	}
	void toggle() {
		toggleBit(this->m_gpioPort->DDR, this->bitNo);
	}
	Bool read() const {
		return this->m_gpioPort->IDR & _BV(this->bitNo);
	}
};

} //# namespace details

template<unsigned portAddrArg, unsigned bitNoArg>
struct Self: details::Wrap<portAddrArg, bitNoArg, details::Self> {  };

// #pragma pop_macro("SelfNS")
// #pragma pop_macro("Self_impl")
#pragma pop_macro("Self")

} } } } //# namespace
