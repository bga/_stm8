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

#include <STM8S_StdPeriph_Driver/stm8s.h>
// #include <stm8s.h>

namespace Bga { namespace Mcu { namespace Hal {

#define BGA__MCU__HAL__ISR(vectorArg) STM8S_STDPERIPH_LIB__INTERRUPT_HANDLER(BGA__CONCAT3(_, vectorArg, _vector), vectorArg)

} } } //# namespace
