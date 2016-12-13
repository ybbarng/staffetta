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
    etimer_set(&et,CLOCK_SECOND*55+(random_rand()%(CLOCK_SECOND*10)));
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    while(1) {
		staffetta_print_stats();
		staffetta_add_data(round_stats++);
		etimer_set(&et,CLOCK_SECOND*240);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    }
    PROCESS_END();
}

PROCESS_THREAD(staffetta_test, ev, data){
    PROCESS_BEGIN();
    static struct etimer et;
    int staffetta_result;
    uint32_t wakeups,Tw;
    leds_init();
    leds_on(LEDS_GREEN);
    staffetta_init();
    random_init(node_id);
    watchdog_stop();
    leds_off(LEDS_GREEN);
    process_start(&staffetta_print_stats_process, NULL);
    while(1){
		wakeups = getWakeups(); //Get wakeups/period from Staffetta
		Tw = ((CLOCK_SECOND*(10*BUDGET_PRECISION))/wakeups); //Compute Tw
		etimer_set(&et,((Tw*3)/4) + (random_rand()%(Tw/2))); //Add some randomness
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		staffetta_result = staffetta_send_packet(); //Perform a data exchange
		//TODO compute histogram of staffetta results
		leds_off(LEDS_RED);
    }
    PROCESS_END();
}

