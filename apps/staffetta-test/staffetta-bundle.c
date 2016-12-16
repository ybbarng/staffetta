#include "staffetta.h"
#include "node-id.h"

static uint8_t round_stats;
static int loop_stats;

PROCESS(staffetta_test, "Staffetta test");
AUTOSTART_PROCESSES(&staffetta_test);
PROCESS(staffetta_print_stats_process, "Staffetta print stats");

PROCESS_THREAD(staffetta_print_stats_process, ev, data){
PROCESS_BEGIN();
    static struct etimer et;
    round_stats = PAKETS_PER_NODE;
    loop_stats = node_id;
    etimer_set(&et,CLOCK_SECOND*1+(random_rand()%(CLOCK_SECOND*2)));
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    while(1) {
		staffetta_print_stats();
		staffetta_add_data(round_stats++);
		etimer_set(&et,CLOCK_SECOND*3);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    }
    PROCESS_END();
}

PROCESS_THREAD(staffetta_test, ev, data){
    PROCESS_BEGIN();
    static struct etimer et;
    int staffetta_result;
    uint32_t wakeups,Tw, dc, T0;
    leds_init();
    leds_on(LEDS_GREEN);
    staffetta_init();
    random_init(node_id);
    watchdog_stop();
    leds_off(LEDS_GREEN);
//    process_start(&staffetta_print_stats_process, NULL);
    while(1){
		wakeups = getWakeups(); //Get wakeups/period from Staffetta
		dc = get_duty_cycle();
		Tw = ((CLOCK_SECOND*(10*BUDGET_PRECISION))/wakeups) * wakeups * (1000 - dc) / 1000; //Compute Tw
		//Tw = ((CLOCK_SECOND*(10*BUDGET_PRECISION))/wakeups) * wakeups; //Compute Tw
		Tw = ((Tw*3)/4) + (random_rand()%(Tw/2));
		printf("wakeups: %lu, dc: %lu, Tw: %lu\n", wakeups, dc, Tw);
		etimer_set(&et,Tw); //Add some randomness
		//etimer_set(&et,Tw); //Add some randomness
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		wakeups = getWakeups();
		dc = get_duty_cycle();

		T0 = RTIMER_NOW();
		Tw = ((CLOCK_SECOND*(10*BUDGET_PRECISION))/wakeups) * wakeups; //Compute Tw
		Tw = ((Tw*3)/4) + (random_rand()%(Tw/2));
//		printf ("T0: %lu, Tw: %lu, T0+Tw: %lu, wakeups: %lu, duty_cycle: %lu\n", T0, Tw, T0+Tw, wakeups, dc);
//		printf("ENERGEST_CONF_ON: %u, duty_cycle: %u\n", ENERGEST_CONF_ON, dc);

		while (RTIMER_CLOCK_LT (RTIMER_NOW(), T0 + Tw))
		{
			printf("send packet\n");
			staffetta_result = staffetta_send_packet(); //Perform a data exchange
			printf("result: %d\n", staffetta_result);
		}
		//TODO compute histogram of staffetta results
		printf("go to sleep\n");
		leds_off(LEDS_RED);
    }
    PROCESS_END();
}

