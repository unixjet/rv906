#include <stdio.h>
#include <stdint.h>

// CSR access
#define read_csr(reg) ({ unsigned long __tmp; \
    asm volatile ("csrr %0, " #reg : "=r"(__tmp)); __tmp; })

// tohost for signaling test result
extern volatile uint64_t tohost;

// Trap handler for unexpected exceptions
void trap_handler(void) {
    tohost = 3;  // Signal failure
    while(1);
}

int main(int argc, char **argv)
{
	printf("argc = %d\n", argc);

	for (int i = 0; i < argc; i++) {
		printf("argv[%d] = %s\n", i, argv[i]);
	}

	if (argc > 1) {
		printf("Message: %s\n", argv[1]);
	} else {
		printf("hello, world\n");
	}

	return 0;
}
