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

static inline void enableInterrupts() { ::STM8S_StdPeriph_Driver::enableInterrupts(); } /* enable interrupts */
static inline void disableInterrupts() { ::STM8S_StdPeriph_Driver::disableInterrupts(); } /* disable interrupts */
static inline void nop() { ::STM8S_StdPeriph_Driver::nop(); } /* No Operation */
static inline void trap() { ::STM8S_StdPeriph_Driver::trap(); } /* Trap (soft IT) */
static inline void waitForInterrupt() { ::STM8S_StdPeriph_Driver::wfi(); } /* Wait For Interrupt */
static inline void halt() { ::STM8S_StdPeriph_Driver::halt(); } /* Halt */

} } }
