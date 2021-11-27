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

Bool Eeprom_isUnlocked() {
	using namespace ::STM8S_StdPeriph_Lib;
	return hasBitMask(FLASH->IAPSR, FLASH_IAPSR_DUL);
}
void Eeprom_unlock() {
	using namespace ::STM8S_StdPeriph_Lib;
	
	if(Eeprom_isUnlocked()) return;
	
	FLASH->DUKR = FLASH_DUKR_KEY1;
	FLASH->DUKR = FLASH_DUKR_KEY2;
	while(!hasBitMask(FLASH->IAPSR, FLASH_IAPSR_DUL));
}
inline void Eeprom_lock() {
	using namespace ::STM8S_StdPeriph_Lib;
	clearBitMask(FLASH->IAPSR, FLASH_IAPSR_DUL);
}

} } } //# namespace
