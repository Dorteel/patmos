#include <machine/patmos.h>

void inline write(_iodev_ptr_t uart_base_ptr, char c) {
  while ((*uart_base_ptr & 0x01) == 0);
  *(uart_base_ptr+1) = c;
}

char inline read(_iodev_ptr_t uart_base_ptr) {
  while ((*uart_base_ptr & 0x02) == 0);
  return *(uart_base_ptr+1);
}


int main() {

	_iodev_ptr_t uart1_ptr = (_iodev_ptr_t) PATMOS_IO_UART;
	_iodev_ptr_t uart2_ptr = (_iodev_ptr_t) PATMOS_IO_UART2;
	_iodev_ptr_t uart3_ptr = (_iodev_ptr_t) PATMOS_IO_UART3;

	printf("---> Reading data <---\n");
	char uart_data;
	for (int i =0; i<100; i++) {
		uart_data = read(uart3_ptr);
		printf("%c",uart_data);
		write(uart3_ptr,uart_data);
	}
}
