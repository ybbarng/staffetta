#include "sys/etimer.h"

PROCESS_THREAD(example_process, ev, data)
{
	static struct etimer et;
	PROCESS_BEGIN();

	uint32_t t = RTIMER_NOW();
	printf("t0: %lu\n", t);
	etimer_set(&et, CLOCK_SECOND);
	PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
	t = RTIMER_NOW();
	printf("t1: %lu\n", t);

	PROCESS_END();
}

	

