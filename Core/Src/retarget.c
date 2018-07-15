#include <stdio.h>
#include <stm32f4xx.h>

#pragma import(__use_no_semihosting_swi)

#define ECHO_FGETC
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))

#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

//volatile int ITM_RxBuffer=0x5AA55AA5; /* Buffer to transmit data towards debug system. */
     
struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;


int fputc(int ch, FILE *f) {
  if (DEMCR & TRCENA) {
    while (ITM_Port32(0) == 0);
			ITM_Port8(0) = ch;
  }
  return(ch);
}


unsigned char backspace_called;
unsigned char last_char_read;
int r;

int fgetc(FILE *f)
{
    /* if we just backspaced, then return the backspaced character */
    /* otherwise output the next character in the stream */
    if (backspace_called == 1)
    {
      backspace_called = 0;
    }
    else {
        do {
            r = ITM_ReceiveChar();
        } while (r == -1);
        
        last_char_read = (unsigned char)r;

#ifdef ECHO_FGETC
        ITM_SendChar(r);
#endif
    }

    return last_char_read;
}

/*
	** The effect of __backspace() should be to return the last character
	** read from the stream, such that a subsequent fgetc() will
** return the same character again.
*/

int __backspace(FILE *f)
{
    backspace_called = 1;
    return 0;
}

int ferror(FILE *f) {
  /* Your implementation of ferror */
  return EOF;
}

void _ttywrch(int ch) {
  ITM_SendChar((uint32_t)ch);
}

void _sys_exit(int return_code) {
  while (1);    /* endless loop */
}
