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

struct Uart {
	
	#ifdef __IAR_SYSTEMS_ICC__ 
		#pragma diag_suppress=Pe382
	#endif
	::STM8S_StdPeriph_Lib::UART1_struct* const uartPort = (::STM8S_StdPeriph_Lib::UART1_struct*)::STM8S_StdPeriph_Lib::UART1_BaseAddress;
	
	inline void init(FU32 baudrate) {
		using namespace ::STM8S_StdPeriph_Lib;
		
		/* round to nearest integer */
		U16 div = (F_CPU + baudrate / 2) / baudrate;
		/* madness.. */
		uartPort->BRR2 = ((div >> 8) & 0xF0) + (div & 0x0F);
		uartPort->BRR1 = div >> 4;
		/* enable transmitter and receiver */
		uartPort->CR2 = UART1_CR2_TEN | UART1_CR2_REN;
	}

	inline void write(FU8 data) {
		using namespace ::STM8S_StdPeriph_Lib;
		uartPort->DR = data;
	}
	inline Bool writeIsReady() {
		using namespace ::STM8S_StdPeriph_Lib;
		return uartPort->SR & UART1_SR_TC;
	}
	inline Bool writeIsBusy() {
		return !writeIsReady();
	}
	inline void writeWait() {
		while(writeIsBusy());
	}
	inline void writeSync(FU8 data) {
		writeWait();
		write(data);
	}
	inline void writeSync(const char* str) {
		while(*str) {
			writeSync(*str);
			str++;
		}
	}

	inline Bool readIsReady() {
		using namespace ::STM8S_StdPeriph_Lib;
		return uartPort->SR & UART1_SR_RXNE;
	}
	inline Bool readIsBusy() {
		return !readIsReady();
	}
	inline void readWait() {
		while(readIsBusy());
	}
	inline FU8 readGetResult() {
		using namespace ::STM8S_StdPeriph_Lib;
		readWait();
		return uartPort->DR;
	}
	
};

} } } //# namespace
