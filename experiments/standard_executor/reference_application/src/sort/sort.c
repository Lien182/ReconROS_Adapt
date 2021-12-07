#include <stdint.h>

#define BLOCK_SIZE 2048


void sort_bubble(uint32_t * ram) {
	unsigned int i, j;
	uint32_t tmp;
	for (i = 0; i < BLOCK_SIZE; i++) {
		for (j = 0; j < BLOCK_SIZE - 1; j++) {
			if (ram[j] > ram[j + 1]) {
				tmp = ram[j];
				ram[j] = ram[j + 1];
				ram[j + 1] = tmp;
			}
		}
	}
}