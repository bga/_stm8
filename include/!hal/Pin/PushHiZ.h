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
#define Self PushHiZ
// #define Self_impl BGA__CONCAT(Self, _impl)
// #undef SelfNS
// #define SelfNS BGA__CONCAT(Self, NS)

// namespace SelfNS { namespace details {
namespace details {

template<unsigned portAddrArg, unsigned bitNoArg>
struct Self: Base<portAddrArg, bitNoArg>, InitJoiner<
	portAddrArg, 
	_BV(bitNoArg), 0, 
	0, 0, 
	0, 0, 
	0, 0 
> {
	enum {
		portAddr = portAddrArg,
		bitNo = bitNoArg,
	};

	GPIO_TypeDef* const m_gpioPort = (::STM8S_StdPeriph_Lib::GPIO_TypeDef*)portAddr;
	void init() {
		setBit(m_gpioPort->ODR, bitNo);
	}
	void hiZ() {
		clearBit(m_gpioPort->DDR, bitNo);
	}
	void on() {
		setBit(m_gpioPort->DDR, bitNo);
	}
	void toggle() {
		toggleBit(m_gpioPort->DDR, bitNo);
	}
};

} //# namespace details

template<unsigned portAddrArg, unsigned bitNoArg>
struct Self: details::Wrap<portAddrArg, bitNoArg, details::Self> {  };

// #pragma pop_macro("SelfNS")
// #pragma pop_macro("Self_impl")
#pragma pop_macro("Self")

} } } } //# namespace
