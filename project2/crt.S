#include <avr/io.h>
#include <avr/sfr_defs.h>

#define __zero_reg__ r1
   .macro       vector name
   .weak        \name
   .set         \name, __vector_not_set
   jmp          \name
   .endm

   /* initialize the interrupt vector table */
   /* vector 0 is the "reset" vector */
   .section .vectors,"ax",@progbits
   .global      __vectors
   .func        __vectors
__vectors:
   jmp          __init
   vector       __vector_1
   vector       __vector_2
   vector       __vector_3
        ...
   vector       __vector_37
   .endfunc

   .text
   .global      __vector_not_set
   .func        __vector_not_set
__vector_not_set:
   jmp          vectors
   .endfunc

   /* beginning of our executable code */
   .section .init0,"ax",@progbits
   .weak        __init
__init:
   .weak        __stack
   .weak        __heap_end
   .set         __heap_end, 0
   .set         __stack, RAMEND

   /* initialize a default stack at the end of RAM */
   .section .init2,"ax",@progbits
   clr          __zero_reg__
   out          _SFR_IO_ADDR(SREG), __zero_reg__
   ldi          r28,lo8(__stack)
   ldi          r29,hi8(__stack)
   out         _SFR_IO_ADDR(SPH), r29
   out         _SFR_IO_ADDR(SPL), r28

   /* copy initial global data values from FLASH into RAM */
   .section .init4,"ax",@progbits
   .global __do_copy_data
__do_copy_data:
   ldi         r17, hi8(__data_end)
   ldi         r26, lo8(__data_start)
   ldi         r27, hi8(__data_start)
   ldi         r30, lo8(__data_load_start)
   ldi         r31, hi8(__data_load_start)
   ldi         r16, hh8(__data_load_start)
   out         _SFR_IO_ADDR(RAMPZ), r16
   rjmp       .L__do_copy_data_start
.L__do_copy_data_loop:
   elpm       r0, Z+
   st         X+, r0
.L__do_copy_data_start:
   cpi        r26, lo8(__data_end)
   cpc        r27, r17
   brne       .L__do_copy_data_loop

   /* now, we are ready to call our application's main program */
   .section .init9,"ax",@progbits
   call       OS_Init       /* changed "main" to "OS_Init" */
   jmp        exit          /* if returns, loop forever */