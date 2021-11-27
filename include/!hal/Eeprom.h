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

#include <!cpp/newKeywords.h>
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

void Eeprom_nextWriteEnable32bitMode() {
	using namespace ::STM8S_StdPeriph_Lib;
	setBitMask(FLASH->CR2, FLASH_CR2_WPRG);
	clearBitMask(FLASH->NCR2, FLASH_NCR2_NWPRG);
}

U8* Eeprom_memcpyUpdate(U8* dest, U8 const* src, Size n) {
	forInc(Size, i, 0, n) {
		if(dest[i] != src[i]) {
			Eeprom_unlock();
			dest[i] = src[i];
		}
	}
	Eeprom_lock();
	
	return dest;
}

void Eeprom_memcpyUpdate_ptr1AlignEnd4(U8 const*& src, U8*& dest, U8 const * const destEnd) {
	while(dest < destEnd && (Size(dest) & 3) != 0) {
		if(*dest != *src) {
			Eeprom_unlock();
			*dest = *src;
		};
		dest += 1;
		src += 1;
	}
	
//	return dest;
}
inline void Eeprom_memcpyUpdate_ptr4(U8 const*& src, U8*& dest, U8 const * const destEnd) {
	while(dest < destEnd) {
		if(*(U32 const*)dest != *(U32 const*)src) {
			Eeprom_unlock();
			Eeprom_nextWriteEnable32bitMode();
			*(U32 *)dest = *(U32 const*)src;
		};
		dest += 4;
		src += 4;
	}
	
//	return dest;
}

void Eeprom_readRaw(U8* ramDest, U8 const* eepromSrc, Size dataSize) {
	memcpy(ramDest, eepromSrc, dataSize);
}
template<class T>
void Eeprom_read(T& ramDest, const T& eepromSrc) {
	Eeprom_readRaw(reinterpret_cast<U8*>(&ramDest), reinterpret_cast<U8 const*>(&eepromSrc), sizeof(T));
}

inline void Eeprom_writeRaw(U8* eepromDest, U8 const* ramSrc, Size n) {
	Eeprom_memcpyUpdate(eepromDest, ramSrc, n);
}
template<class T>
inline void Eeprom_write(T& eepromDest, T const& ramSrc) {
	Eeprom_writeRaw(reinterpret_cast<U8*>(&eepromDest), reinterpret_cast<U8 const*>(&ramSrc), sizeof(T));
}

inline void Eeprom_writeRawFast32(U8* dest, U8 const* src, Size n) {
	U8 const* const destEnd = dest + n; 
	U8 const* const destEndAlign4 = (U8 *)(Size(destEnd) & ~3);
	
	Eeprom_memcpyUpdate_ptr1AlignEnd4(src, dest, destEnd);
	Eeprom_memcpyUpdate_ptr4(src, dest, destEndAlign4);
	Eeprom_memcpyUpdate_ptr1AlignEnd4(src, dest, destEnd);
	Eeprom_lock();
}
template<class T>
inline void Eeprom_writeFast32(T& eepromDest, T const& ramSrc) {
	Eeprom_writeRawFast32(reinterpret_cast<U8*>(&eepromDest), reinterpret_cast<U8 const*>(&ramSrc), sizeof(T));
}

} } } //# namespace
