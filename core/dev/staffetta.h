/**
 * \file
 *         Staffetta protocol underneat a generic opportunistic data collection protocol [SenSys 2016]
 * \author
 *         Marco Cattani <m.cattani@gmail.com>
 */

#ifndef __STAFFETTA_H__
#define __STAFFETTA_H__

#include "contiki.h"
#include "dev/watchdog.h"
#include "dev/cc2420_const.h"
#include "dev/leds.h"
#include "dev/spi.h"
#include "sys/ctimer.h"
#include "lib/random.h"
#include <stdio.h>
#include <legacymsp430.h>
#include <stdlib.h>

/*------------------------- OPTIONS --------------------------------------------------*/

#define PAKETS_PER_NODE 	    6                 // Initial queue size
#define WITH_CRC 		          1                 // Check packet CRC
#define IS_SINK 		          (node_id == 1)    // Define condition to be a sink node
//#define IS_SINK 		        (node_id < 4)     // Mobile sink on flocklab
#define WITH_SELECT 		      1                 // enable 3-way handshake (in case of multiple forwarders, initiator can choose)

#define WITH_GRADIENT 		    0                 // ensure that messages follows a gradient to the sink (number of wakeups)
#define BCP_GRADIENT		      0                 // use the queue size as gradient (BCP)
#define ORW_GRADIENT		      1                 // use the expected duty cycle as gradient (ORW)
#define DYN_DC 			          0                 // Enable staffetta adaptative wakeups. If disabled, the wakeup of nodes will be fixed

#define FAST_FORWARD 		      1                 // forward as soon as you can (not dummy messages)
#define BUDGET_PRECISION 	    1                 //use fixed point precision to compute the number of wakeups
#define BUDGET 			          750               // how ho long the radio should stay ON every second (in ms * 10)
#define AVG_SIZE 		          5                 // windows size for averaging the rendezvous time
#define AVG_EDC_SIZE		      20                // averaging size for orw's metric EDC
#define WITH_RETX 		        0                 // retransmit a beacon ack if we receive another beacon
#define USE_BACKOFF 		      1                 // Before sending listen to the channel for a certain period
#define SLEEP_BACKOFF 		    0                 // After the backoff, if we receive a beacon instead on starting a communication we go to sleep
#define RSSI_FILTER 		      0                 // Filter beacons with RSSI lower that a threshold
#define RSSI_THRESHOLD 		    -90               // Minimum RSSI value for accepting a beacon
#define WITH_SINK_DELAY 	    1                 // Add a delay to the beacon ack of nodes that are not a sink (sink is always the first to answer to beacons)
#define DATA_SIZE 		        100               // Size of the packet queue
#define WITH_AGGREGATE		    0                 // todo?

/*-------------------------- MACROS -------------------------------------------------*/

#define MIN(a, b) ((a) < (b)? (a) : (b))
#define MAX(a, b) ((a) > (b)? (a) : (b))
#define CSCHEDULE_POWERCYCLE(rtime) cschedule_powercycle((1ul * CLOCK_SECOND * (rtime)) / RTIMER_ARCH_SECOND)
#define GOTO_IDLE(rtime) ctimer_set(&idle_timeout_ctimer,(1ul * CLOCK_SECOND * (rtime)) / RTIMER_ARCH_SECOND,(void (*)(void *))goto_idle, NULL)
#define STOP_IDLE() ctimer_stop(&idle_timeout_ctimer)
#define SEND_BACKOFF(rtime) ctimer_set(&backoff_ctimer,(1ul * CLOCK_SECOND * (rtime)) / RTIMER_ARCH_SECOND,(void (*)(void *))send_packet, NULL)
#define STOP_BACKOFF() ctimer_stop(&backoff_ctimer)
#define PRINTF(...)
#define BUSYWAIT_UNTIL(cond, max_time)                                  \
  do {                                                                  \
    rtimer_clock_t t0;                                                  \
    t0 = RTIMER_NOW();                                                  \
    while(!(cond) && RTIMER_CLOCK_LT(RTIMER_NOW(), t0 + (max_time)));   \
  } while(0)

/*------------------------- STATE --------------------------------------------------*/

enum mac_state{
disabled =        0,
enabled =         1,
idle =            1,
wait_to_send =    2,
wait_beacon_ack = 3,
sending_ack =     4,
beacon_sent =     5,
wait_select =     6,
select_received = 7
} ;

/*------------------------- PACKETS --------------------------------------------------*/

struct staffetta_hdr {
  uint8_t type;
  uint8_t dst;
  uint8_t data;
  uint8_t hop;
  uint8_t seq;
  uint8_t gradient;
  uint16_t wakeups;
};

#define RET_FAST_FORWARD 	     1
#define RET_NO_RX		           2
#define RET_EMPTY_QUEUE		     3
#define RET_ERRORS		         4
#define RET_WRONG_SELECT	     5
#define RET_FAIL_RX_BUFF	     6
#define RET_WRONG_TYPE		     7
#define RET_WRONG_CRC		       8
#define RET_WRONG_GRADIENT	   9

#define TYPE_BEACON       	   1
#define TYPE_BEACON_ACK   	   2
#define TYPE_SELECT       	   3

#define STAFFETTA_PKT_LEN 	   7
#define FOOTER_LEN		         2

#define PKT_LEN			           0
#define PKT_TYPE		           1
#define PKT_SRC			           2
#define PKT_DST			           3
#define PKT_SEQ			           4
#define PKT_TTL			           5
#define PKT_DATA		           6
#define PKT_GRADIENT		       7
#define PKT_RSSI		           8
#define PKT_CRC			           9 //last field + 2

#define FOOTER1_CRC_OK         0x80
#define FOOTER1_CORRELATION    0x7f

#define STAFFETTA_LEN_FIELD              packet[0]
#define STAFFETTA_HEADER_FIELD           packet[1]
#define STAFFETTA_DATA_FIELD             packet[2]
#define STAFFETTA_RELAY_CNT_FIELD        packet[packet_len_tmp - FOOTER_LEN]
#define STAFFETTA_RSSI_FIELD             packet[packet_len_tmp - 1]
#define STAFFETTA_CRC_FIELD              packet[packet_len_tmp]

/*------------------------- TIME --------------------------------------------------*/

#define PERIOD 			          RTIMER_ARCH_SECOND 		     // 1s
#define STROBE_TIME 		      PERIOD				             // 1s
#define STROBE_WAIT_TIME	    (RTIMER_ARCH_SECOND/700) 	 // 2ms
#define ON_TIME 		          (RTIMER_ARCH_SECOND/300) 	 // 3ms
#define OFF_TIME 		          (PERIOD-ON_TIME)		       // 995ms
#define BACKOFF_TIME 		      (ON_TIME)			             // 5ms

struct staffettamac_config {
  rtimer_clock_t on_time;
  rtimer_clock_t off_time;
  rtimer_clock_t strobe_time;
  rtimer_clock_t strobe_wait_time;
};

/*------------------------- FUNCTIONS --------------------------------------------------*/

int staffetta_send_packet(void);
uint32_t getWakeups(void);
void sink_listen(void);
void staffetta_print_stats(void);
void staffetta_add_data(uint8_t);
void staffetta_init(void);

#endif /* __STAFFETTA_H__ */
