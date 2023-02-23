#include "stdio.h"
#include "main.h"
#include "helper_functions.h"
#include "navigation.h"
#include "custom_typedef.h"

PUTCHAR_PROTOTYPE
{
#if DO_PRINT_STATEMENTS
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
#endif
  return ch;
}

void print_float(float_t num) {
	int dec;
	printf("%d.", (int) num);
	dec = (num - (int) num)*1000000;
	if (dec < 0) {
		dec = 0-dec;
	}
	printf("%d", dec);
}

void print_message() {
	for (int i = 0; i < UART2_RX_DMA_BUFFER_SIZE; i++) {
		printf("%c", nav_rx_data[i]);
	}
}
