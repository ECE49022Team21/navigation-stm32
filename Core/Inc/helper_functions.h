#ifndef HELPER_FUNCTIONS_HEADER_G
#define HELPER_FUNCTIONS_HEADER_G
#include "custom_typedef.h"

#define DO_PRINT_STATEMENTS 1

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
extern UART_HandleTypeDef huart3;

void print_float(float_t num);
void print_message();
#endif
