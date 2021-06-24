#include "delay.h"

#if defined(__ICCSTM8__)

  void delay_cycle(uint16_t ticksCount /* X register */) {
    asm(
      "bga_delay_cycle_loop: decw X\n"
      "nop\n"
      "jrne bga_delay_cycle_loop\n"
    );
  }
#elif defined(__SDCC_stm8)

  void delay_cycle(uint16_t ticksCount /* (0x03, sp) */) {
    __asm
      pushw x
      ldw x, (3, sp)
      00001$: decw x
      nop
      jrne 00001$
      popw x
    __endasm;
  } 
#else
	#error Unsupported compiler
#endif

void delay_ms(uint_fast16_t ms) {
	while(ms--) {
		delay_us(1000);
	}
}
