//------------------- PLANT.C ---------------------- 

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <rtai_lxrt.h>
#include <rtai_shm.h>
#include <sys/io.h>
#include <signal.h>
#include "parameters.h"
#define CPUMAP 0x1

//emulates the plant to be controlled

static RT_TASK *main_Task;
static RT_TASK *loop_Task;
static int keep_on_running = 1;

static pthread_t main_thread;
static RTIME expected;
static RTIME sampl_interv;

static void endme(int dummy) {keep_on_running = 0;}

int* sensor;
int* actuator;

static void * main_loop(void * par) {
	
	if (!(loop_Task = rt_task_init_schmod(nam2num("PLANT"), 2, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT PERIODIC TASK\n");
		exit(1);
	}

	unsigned int iseed = (unsigned int)rt_get_time();
  	srand (iseed);

	(*sensor) = 100; (*actuator) = 0;
	expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(loop_Task, expected, sampl_interv);
	rt_make_hard_real_time();

	int count = 1;
	int noise = 0;
	int sens = 0;
	int when_to_decrease = 1 + (int)( 10.0 * rand() / ( RAND_MAX + 1.0 ));
	while (keep_on_running)
	{
		if (count%when_to_decrease==0) {
			if ((*sensor)>0) (*sensor)--; // the sensed data smoothly decreases...
			when_to_decrease = 1 + (int)( 10.0 * rand() / ( RAND_MAX + 1.0 ));
			count = when_to_decrease;
		}
		// add noise between -1 and 1	
		noise = -1 + (int)(3.0 * rand() / ( RAND_MAX + 1.0 ));
		sens += noise;
		if (sens>0) (*sensor) += noise;
		
		// reaction to control
		if (count%2 == 0) 
			if ((*actuator) == 1) 
				(*sensor)++;
			else if (((*actuator) == -1) && ((*sensor)>0)) 
				(*sensor)--;
		count++;
		rt_task_wait_period();
	}
	rt_task_delete(loop_Task);
	return 0;
}

int main(void)
{
	printf("The plant STARTED!\n");
 	signal(SIGINT, endme);

	if (!(main_Task = rt_task_init_schmod(nam2num("MNTSK"), 0, 0, 0, SCHED_FIFO, 0xF))) {
		printf("CANNOT INIT MAIN TASK\n");
		exit(1);
	}

	//attach to data shared with the controller
	sensor = rtai_malloc(SEN_SHM, sizeof(int));
	actuator = rtai_malloc(ACT_SHM, sizeof(int));
	
	if ((rt_is_hard_timer_running())) {
		printf("Skip hard real_timer setting...\n");
	} else {
		rt_set_oneshot_mode();
		start_rt_timer(0);
	}

	sampl_interv = nano2count(TICK_TIME);
	pthread_create(&main_thread, NULL, main_loop, NULL);
	while (keep_on_running) {
		printf("Value: %d\n",(*sensor));
		rt_sleep(10000000);
	}

    	stop_rt_timer();
	rt_shm_free(SEN_SHM);
	rt_shm_free(ACT_SHM);
	rt_task_delete(main_Task);
 	printf("Simple RT Task (user mode) STOPPED\n");
	return 0;
}




