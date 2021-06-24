#ifndef DELAY_H
#define DELAY_H

#ifndef F_CPU
#warning "F_CPU not defined, using 2MHz by default"
#define F_CPU 2000000UL
#endif

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


//# wrong, uint32_t overflow
// #define delay_usToLoopCyclesCount(x) ((( F_CPU * (xArg) / 1000000UL ) - 0) / 4)

//# ok,  but lets avoid non power of 2 div
//# #define delay_usToLoopCyclesCount(xArg) ((( F_CPU / 1000UL * (xArg) / 1000UL ) - 0 ) / 4)

#define delay_usToLoopCyclesCount(xArg) ((( F_CPU / 976UL * (xArg) / 1024UL ) - 0 ) / 4 /* ticka */)

#if defined(__SDCC_stm8)
  #pragma callee_saves delay_cycle
#endif


void delay_cycle(uint16_t ticksCount);

static inline void delay_us(uint_fast16_t us) {
	delay_cycle(delay_usToLoopCyclesCount(us));
}

void delay_ms(uint_fast16_t ms);

#ifdef __cplusplus
} // extern "C"
#endif


#endif /* DELAY_H */
