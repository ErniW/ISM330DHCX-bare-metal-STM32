#include "stm32f446xx.h"

#define BAUDRATE         115200

#define PA2_AF           (2 << 4)
#define PA2_AF_USART2_TX (7 << 8)

#define PA3_AF           (2 << 6)
#define PA3_AF_USART2_RX (7 << 12)

extern "C" int __io_putchar(int ch);

void tx_init();
void tx_send(char c);