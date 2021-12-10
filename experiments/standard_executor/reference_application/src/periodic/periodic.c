#include <stdio.h>
#include <sys/time.h>
#include <stdint.h>

#include "sha256.h"

uint32_t calc_periodic(uint32_t input)
{
	uint32_t ret = 0;
	BYTE buf[SHA256_BLOCK_SIZE];

	SHA256_CTX ctx;
	int idx;
	int pass = 1;

	sha256_init(&ctx);

	memcpy(buf, &input, 4);
	for(int i = 0; i < 4; i++)
	{
		sha256_update(&ctx, buf, SHA256_BLOCK_SIZE);
		sha256_final(&ctx, buf);
	}

	memcpy(&ret, buf, 4);

	return ret;
}
