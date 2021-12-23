#include "reconos_calls.h"
#include "reconos_thread.h"

#include "ap_int.h"
#include "ap_fixed.h"
#include "hls_math.h"

#include <sorter_msgs/srv/sort.h>

#define BLOCK_SIZE 2048

#define WAIT_INIT() uint32_t cntr = 0, waitflag = 0;

#define WAIT_HLS(x) {for (int ___i=0; ___i < x; ___i++)	{ \
		_Pragma("hls pipeline") \
		_Pragma("hls protocol fixed") \
		cntr = cntr + 1; \
		ap_wait(); \
		if (cntr == x) { \
			waitflag = 1; \
		}}}


void sort_bubble(uint32_t ram[BLOCK_SIZE]) {
	unsigned int i, j;
	uint32_t tmp;
	uint32_t prev_val;
	uint32_t next_val;
	for (i = 0; i < BLOCK_SIZE; i++) {
		prev_val = ram[0];
		for (j = 0; j < BLOCK_SIZE - 1; j++) {
			#pragma HLS pipeline
			next_val = ram[j + 1];
			if (prev_val > next_val) {
				ram[j] = next_val;
				ram[j + 1] = prev_val;
			}
			prev_val = next_val;
		}
	}
}

void sort_net(uint32_t ram[BLOCK_SIZE]) {
	unsigned int i, k, stage;
	uint32_t tmp;
	uint32_t prev_val;
	uint32_t next_val;

	for(stage = 1; stage <= BLOCK_SIZE; stage++){
		k = (stage % 2 == 1) ? 0 : 1;
		for(i = k; i < BLOCK_SIZE - 1; i += 2){
			#pragma HLS unroll factor=8
			#pragma HLS pipeline
			ram[i] = prev_val;
			ram[i + 1] = next_val;
			if (prev_val > ram[i + 1]) {
				ram[i] = next_val;
				ram[i + 1] = prev_val;
			}
		}
	}
}

THREAD_ENTRY() {
	RAM(uint32_t, BLOCK_SIZE, ram);
	#pragma HLS array_partition variable=ram cyclic factor=16 dim=0
	uint32_t addr, initdata;	
	uint32_t pMessage;
	uint32_t payload_addr[1];

	THREAD_INIT();
	initdata = GET_INIT_DATA();

	addr = OFFSETOF(sorter_msgs__srv__Sort_Request, unsorted.data) + initdata;

	MEM_READ(addr, payload_addr, 4);					//Get the address of the data
	MEM_READ(payload_addr[0], ram, BLOCK_SIZE * 4);

	sort_net(ram);
	MEM_WRITE(ram, payload_addr[0], BLOCK_SIZE * 4);

	ROS_SERVICESERVER_SEND_RESPONSE(rsort_srv,rsort_sort_srv_res);

	THREAD_EXIT();
}
