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

#ifdef __IAR_SYSTEMS_ICC__ 
	#pragma diag_suppress=Pe382
#endif

namespace Bga { namespace Mcu { namespace Hal {namespace Pin {

enum {
	swimBaseAddress = ::STM8S_StdPeriph_Lib::GPIOD_BaseAddress, 
	swimPinNo = 1, 
};

namespace details {

#pragma push_macro("Self")
#undef Self
#define Self InitJoiner

#pragma push_macro("SelfNS")
#undef SelfNS
#define SelfNS BGA__CONCAT(Self, NS)

namespace SelfNS { namespace details { 

struct Base {
	#define X(portLetterArg, portGpioArg, portGpioBaseAddressArg) enum { \
		BGA__CONCAT3(port, portLetterArg, _odr_set) = 0,  \
		BGA__CONCAT3(port, portLetterArg, _odr_reset) = 0,  \
		BGA__CONCAT3(port, portLetterArg, _ddr_set) = 0,  \
		BGA__CONCAT3(port, portLetterArg, _ddr_reset) = 0,  \
		BGA__CONCAT3(port, portLetterArg, _cr1_set) = 0,  \
		BGA__CONCAT3(port, portLetterArg, _cr1_reset) = 0,  \
		BGA__CONCAT3(port, portLetterArg, _cr2_set) = 0,  \
		BGA__CONCAT3(port, portLetterArg, _cr2_reset) = 0, \
	};
	STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X(X)
	#undef X 

	#if 0
	#define X(portLetterArg, portGpioArg, portGpioBaseAddressArg) { \
		if(BGA__CONCAT3(port, portLetterArg, _odr_set) != 0) setMask(BGA__CONCAT(::STM8S_StdPeriph_Lib::GPIO, portLetterArg)->ODR, BGA__CONCAT3(port, portLetterArg, _odr_set));  \
		if(BGA__CONCAT3(port, portLetterArg, _odr_reset) != 0) clearMask(BGA__CONCAT(::STM8S_StdPeriph_Lib::GPIO, portLetterArg)->ODR, BGA__CONCAT3(port, portLetterArg, _odr_reset));  \
	}
	
	void init() {
		STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X(X)
	}
	#undef X
	#endif
};

} } //# namespace SelfNS::details

template<
	unsigned portAddrArg, 
	unsigned odrSetArg, unsigned odrResetArg, 
	unsigned ddrSetArg, unsigned ddrResetArg, 
	unsigned cr1SetArg, unsigned cr1ResetArg, 
	unsigned cr2SetArg, unsigned cr2ResetArg 
> struct Self;

#define X(portLetterArg, portGpioArg, portGpioBaseAddressArg) \
template< \
	unsigned odrSetArg, unsigned odrResetArg,  \
	unsigned ddrSetArg, unsigned ddrResetArg,  \
	unsigned cr1SetArg, unsigned cr1ResetArg,  \
	unsigned cr2SetArg, unsigned cr2ResetArg  \
> struct Self< \
	portGpioBaseAddressArg,  \
	odrSetArg, odrResetArg,  \
	ddrSetArg, ddrResetArg,  \
	cr1SetArg, cr1ResetArg,  \
	cr2SetArg, cr2ResetArg  \
>: SelfNS::details::Base { \
	enum { \
		BGA__CONCAT3(port, portLetterArg, _odr_set) = odrSetArg,  \
		BGA__CONCAT3(port, portLetterArg, _odr_reset) = odrResetArg,  \
		BGA__CONCAT3(port, portLetterArg, _ddr_set) = ddrSetArg,  \
		BGA__CONCAT3(port, portLetterArg, _ddr_reset) = ddrResetArg,  \
		BGA__CONCAT3(port, portLetterArg, _cr1_set) = cr1SetArg,  \
		BGA__CONCAT3(port, portLetterArg, _cr1_reset) = cr1ResetArg,  \
		BGA__CONCAT3(port, portLetterArg, _cr2_set) = cr2SetArg,  \
		BGA__CONCAT3(port, portLetterArg, _cr2_reset) = cr2ResetArg, \
	}; \
};
STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X(X)
#undef X

#pragma pop_macro("SelfNS")
#pragma pop_macro("Self")

template<unsigned portAddrArg, unsigned bitNoArg>
struct Base {
	enum {
		portAddr = portAddrArg,
		bitNo = bitNoArg,
	};
	static_assert_lte(bitNoArg, 7);
	#define X(portLetterArg, portGpioArg, portGpioBaseAddressArg) || x == portGpioBaseAddressArg
	static_assert_test(portAddrArg, false STM8S_STDPERIPH_LIB__FOR_EACH_GPIO_X(X));
	#undef X
};

template<unsigned portAddrArg, unsigned bitNoArg>
struct Ignore: Base<portAddrArg, bitNoArg> {
	void init() {
	}
	void hiZ() {
	}
	void on() {
	}
	void off() {
	}
	void setValue(bool v) {
	}
	void toggle() {
	}
	Bool read() const {
		return 0;
	}
};

template<
	unsigned portAddrArg, unsigned bitNoArg, 
	template<unsigned pinPortAddrArg, unsigned pinBitNoArg> class PinArg
> struct Wrap: PinArg<portAddrArg, bitNoArg> {  };

#ifndef NDEBUG
	template<
		template<unsigned pinPortAddrArg, unsigned pinBitNoArg> class PinArg
	> struct Wrap<swimBaseAddress, swimPinNo, PinArg>: Ignore<swimBaseAddress, swimPinNo> {  };
#endif

} //# namespace details

} } } } //# namespace
