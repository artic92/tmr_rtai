//------------------- CONTROLLER.C ---------------------- 

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <rtai_lxrt.h>
#include <rtai_shm.h>
#include <rtai_sem.h>
#include <rtai_msg.h>
#include <rtai_mbx.h>
#include <sys/io.h>
#include <signal.h>
#include "parameters.h"
#define CPUMAP 0x1

//emulates the controller

static RT_TASK *main_Task;
static RT_TASK *read_Task;
static RT_TASK *filter_Task;
static RT_TASK *control_Task[3];
static RT_TASK *voter_Task;
static RT_TASK *write_Task;
static RT_TASK *server_Task;
static int keep_on_running = 1;

static pthread_t read_thread;
static pthread_t filter_thread;
static pthread_t control_thread[3];
static pthread_t write_thread;
static pthread_t voter_thread;
static pthread_t server_thread;
static RTIME sampl_interv;

static void endme(int dummy) {keep_on_running = 0;}

int* sensor;
int* actuator;
int* reference;

int buffer[BUF_SIZE];
int head = 0;
int tail = 0;

// data to be monitored
int avg = 0;
int control = 0;

SEM* space_avail;
SEM* meas_avail;

//SHM contenente le informazioni passate dal gather al diag riguardanti lo stato del sistema
struct Stato* info;

//Semafori per il problema produttore-consumatore tra diag e gather sulla SHM info
SEM* mess_disp;
SEM* spazio_disp;

//Semaforo per gestire la mutua esclusione tra diag, acquire e filter sul buffer
//N.B.: Siccome il server deve gestire una sola richiesta aperiodica si è reso il gather come una funzione richiamata dal
//server stesso. Per questo motivo ci riferiremo al gather facendo riferimento al server_Task.
SEM* sem_s_a_f;

//Mailbox per notificare al DS della richiesta aperiodica da parte del diag
static MBX* mbx;

static void * acquire_loop(void * par) {
	
	if (!(read_Task = rt_task_init_schmod(nam2num("READER"), 1, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT SENSOR TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(read_Task, expected, sampl_interv);
	rt_make_hard_real_time();

	while (keep_on_running)
	{
		//Acquisizione in MUTUA ESCLUSIONE della risorsa
		rt_sem_wait(sem_s_a_f);
		
		// DATA ACQUISITION FROM PLANT
		rt_sem_wait(space_avail);
		
		buffer[head] = (*sensor);
		head = (head+1) % BUF_SIZE;
		rt_sem_signal(meas_avail);
		
		rt_sem_signal(sem_s_a_f);

		rt_task_wait_period();
	}
	rt_task_delete(read_Task);
	return 0;
}

static void * filter_loop(void * par) {

	if (!(filter_Task = rt_task_init_schmod(nam2num("FILTER"), 2, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT FILTER TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(filter_Task, expected, sampl_interv);
	rt_make_hard_real_time();

	int cnt = BUF_SIZE, i;
	unsigned int sum = 0;
	
	while (keep_on_running)
	{
		//Acquisizione in MUTUA ESCLUSIONE della risorsa
		rt_sem_wait(sem_s_a_f);
		
		// FILTERING (average)
		rt_sem_wait(meas_avail);

		sum += buffer[tail];
		tail = (tail+1) % BUF_SIZE;
	
		rt_sem_signal(space_avail);
		
		rt_sem_signal(sem_s_a_f);
		
		cnt--;

		if (cnt == 0) {
			cnt = BUF_SIZE;
			avg = sum/BUF_SIZE;
			sum = 0;
			// sends the average measure to the controllers
			//Si usano send non bloccanti per evitare il bloccaggio al crash di un controllore
			for(i = 0; i < 3; i++)
				rt_send_if(control_Task[i], avg);
		}
		rt_task_wait_period();
	}
	rt_task_delete(filter_Task);
	return 0;
}

static void * control_loop1(void * par) {
	
	int id = (int) par;

	if (!(control_Task[id] = rt_task_init_schmod((nam2num("CONTROL") + id), 3, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT CONTROL TASK %d\n", id);
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(control_Task[id], expected, sampl_interv);
	rt_make_hard_real_time();

	unsigned int plant_state = 0;
	int error = 0;
	unsigned int control_action = 0;
	
	while (keep_on_running)
	{
		// receiving the average plant state from the filter
		rt_receive(0, &plant_state);
		
		// computation of the control law
		error = (*reference) - plant_state;

		if (error > 0) control_action = 1;
		else if (error < 0) control_action = 2;
		else control_action = 3;

		//FAULT: stuck-at-N
		//control_action = 3;

		// sending the control action to the voter
		rt_send(voter_Task, control_action);
		
		//Invio dell'informazione al gather (rappresentato dal server_Task)
		rt_send_if(server_Task, control_action);

		rt_task_wait_period();

	}
	rt_task_delete(control_Task[id]);
	return 0;
}

static void * control_loop2(void * par) {
	
	int id = (int) par;

	if (!(control_Task[id] = rt_task_init_schmod((nam2num("CONTROL") + id), 3, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT CONTROL TASK %d\n", id);
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(control_Task[id], expected, sampl_interv);
	rt_make_hard_real_time();

	unsigned int plant_state = 0;
	int error = 0;
	
	while (keep_on_running)
	{
		// receiving the average plant state from the filter
		rt_receive(0, &plant_state);
		
		// computation of the control law
		error = (*reference) - plant_state;
		
		//FAULT: introduzione di una sleep di 10s
		//rt_sleep(nano2count(10000000000));

		if (error > 0) {
			rt_send(voter_Task,1);
			rt_send_if(server_Task,1);
		}
		else if (error < 0) {
			rt_send(voter_Task,2);
			rt_send_if(server_Task,2);
		}
		else {
			rt_send(voter_Task,3);
			rt_send_if(server_Task,3);
		}

		rt_task_wait_period();

	}
	rt_task_delete(control_Task[id]);
	return 0;
}

static void * control_loop3(void * par) {
	
	int id = (int) par;

	if (!(control_Task[id] = rt_task_init_schmod((nam2num("CONTROL") + id), 3, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT CONTROL TASK %d\n", id);
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(control_Task[id], expected, sampl_interv);
	rt_make_hard_real_time();

	unsigned int plant_state = 0;
	int error = 0;
	//FAULT: terminazione anticipata (decommentare sotto)
	//int terminazione_in_anticipo = 10;
	unsigned int control_action = 0;
	
	while (keep_on_running)
	{
		//FAULT: terminazione anticipata (decommentare il ciclo while)
		//while(terminazione_in_anticipo != 0){
		
		// receiving the average plant state from the filter
		rt_receive(0, &plant_state);
		
		// computation of the control law
		error = (*reference) - plant_state;

		if (error > 0) control_action = 1;
		else if (error < 0) control_action = 2;
		else control_action = 3;
		
		//FAULT: introduzione di una sleep di 5s
		//rt_sleep(nano2count(5000000000));
		
		//Invio dell'informazione al gather (rappresentato dal server_Task)
		rt_send_if(server_Task, control_action);

		// sending the control action to the voter
		rt_send(voter_Task, control_action);
		
		//N.B. Istruzione da decommentare per il FAULT di terminazione anticipata
		//terminazione_in_anticipo--;
		
		rt_task_wait_period();
		
		//}
		//break;
	}
	rt_task_delete(control_Task[id]);
	return 0;
}


static void * voter_loop(void * par){

	if (!(voter_Task = rt_task_init_schmod(nam2num("VOTER"), 6, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT VOTER TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(voter_Task, expected, sampl_interv);
	rt_make_hard_real_time();
	
	//Inizializzazione variabili utili per l'esecuzione
	unsigned int i = 0, decisione = 0, azioni_controllo[3] = {0,0,0};
	
	while (keep_on_running)
	{
		//Attende 10 ms per la ricezione delle decisioni di controllo ()
		for(i = 0; i < 3; i++)
			rt_receive_timed(0, &azioni_controllo[i], rt_get_time() + nano2count(10000000) );
		
		//Effettuazione della decisione
		if(azioni_controllo[0] == azioni_controllo[1] || azioni_controllo[0] == azioni_controllo[2])
			decisione = azioni_controllo[0];
		else if(azioni_controllo[1] == azioni_controllo[2])
			decisione = azioni_controllo[1];
		else //Decisioni tutte diverse -> invio valore di default
			decisione = 3;	
		
		//Invio della decisione
		rt_send(write_Task, decisione);
		
		//Invio asincrono al server per gather
		rt_send_if(server_Task, decisione);

		rt_task_wait_period();

	}
	
	rt_task_delete(voter_Task);
	return 0;
}

static void * actuator_loop(void * par) {

	if (!(write_Task = rt_task_init_schmod(nam2num("WRITE"), 7, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT ACTUATOR TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(write_Task, expected, sampl_interv);
	rt_make_hard_real_time();

	unsigned int control_action = 0;

	while (keep_on_running)
	{
		// receiving the control action from the voter
		rt_receive(0, &control_action);
		
		switch (control_action) {
			case 1: control = 1; break;
			case 2:	control = -1; break;
			case 3:	control = 0; break;
			default: control = 0;
		}
		
		(*actuator) = control;

		rt_task_wait_period();
	}
	rt_task_delete(write_Task);
	return 0;
}

static void gather_routine(void) {

	int i;
	rt_sem_wait(spazio_disp);
	//Acquisizione TEMPORIZZATA delle azioni di controllo dei controllori fino ad 1 ms
	for(i = 0; i < 3; i++){
		if(rt_receive_timed(control_Task[i], &(info->ctrl_signal[i]), rt_get_time()+nano2count(1000000)) == 0)
			(info->stato[i]) = 1; //FALLITO
		else
			(info->stato[i]) = 0; //ATTIVO
	}
	
	//Ricezione decisione dal voter
	rt_receive(voter_Task, &(info->decisione));
	
	//Acquisizione stato dal buffer (Acquisizione in MUTUA ESCLUSIONE della risorsa)
	rt_sem_wait(sem_s_a_f);
	
	for(i = 0; i < BUF_SIZE; i++)
		(info->valori_buf[i]) = buffer[i];
	
	rt_sem_signal(sem_s_a_f);
	rt_sem_signal(mess_disp);
}

static void * server_loop(void * par) {

	if (!(server_Task = rt_task_init_schmod(nam2num("SERVER"), 1, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT SERVER TASK\n");
		exit(1);
	}

	RTIME expected = rt_get_time() + sampl_interv; 
	rt_task_make_periodic(write_Task, expected, (sampl_interv/BUF_SIZE)); //NB: periodo = T/N
	rt_make_hard_real_time();

	int req;	
	
	while (keep_on_running)
	{
		//Attesa di 1 ms per le richieste aperiodiche
		rt_mbx_receive_timed(mbx, &req, sizeof(int), nano2count(1000000));
		if(req == 1)
			gather_routine();
		rt_task_wait_period();
	}
	rt_task_delete(server_Task);
	return 0;
}

int main(void){
	
	printf("The controller is STARTED!\n");
 	signal(SIGINT, endme);

	if (!(main_Task = rt_task_init_schmod(nam2num("MAINTSK"), 0, 0, 0, SCHED_FIFO, 0xF))) {
		printf("CANNOT INIT MAIN TASK\n");
		exit(1);
	}

	//attach to data shared with the controller
	sensor = rtai_malloc(SEN_SHM, sizeof(int));
	actuator = rtai_malloc(ACT_SHM, sizeof(int));
	reference = rtai_malloc(REFSENS, sizeof(int));
	
	(*reference) = 110;
	
	space_avail = rt_typed_sem_init(SPACE_SEM, BUF_SIZE, CNT_SEM | PRIO_Q);
	meas_avail = rt_typed_sem_init(MEAS_SEM, 0, CNT_SEM | PRIO_Q);
	
	info = rtai_malloc(nam2num("INFO_SHM"), sizeof(struct Stato));
	
	mbx = rt_typed_named_mbx_init("MAILBOX",sizeof(int),FIFO_Q);
	
	sem_s_a_f = rt_typed_sem_init(nam2num("MUTEX"), 1, PRIO_Q);
	spazio_disp = rt_typed_named_sem_init("SPAZIO_DISP_INFO", 1, CNT_SEM | PRIO_Q);
	mess_disp = rt_typed_named_sem_init("MESS_DISP_INFO", 0, CNT_SEM | PRIO_Q);
	
	if (rt_is_hard_timer_running()) {
		printf("Skip hard real_timer setting...\n");
	} else {
		rt_set_oneshot_mode();
		start_rt_timer(0);
	}

	sampl_interv = nano2count(CNTRL_TIME);
	
	// CONTROL THREADS 
	pthread_create(&read_thread, NULL, acquire_loop, NULL);
	pthread_create(&filter_thread, NULL, filter_loop, NULL);
	pthread_create(&control_thread[0], NULL, control_loop1, (void*) 0);
	pthread_create(&control_thread[1], NULL, control_loop2, (void*) 1);
	pthread_create(&control_thread[2], NULL, control_loop3, (void*) 2);
	pthread_create(&voter_thread, NULL, voter_loop, NULL);
	pthread_create(&write_thread, NULL, actuator_loop, NULL);
	pthread_create(&server_thread, NULL, server_loop, NULL);

	while (keep_on_running) {
		printf("Control: %d\n",(*actuator));
		rt_sleep(10000000);
	}

    	stop_rt_timer();
    	
    	rt_sem_delete(sem_s_a_f);
	rt_named_sem_delete(spazio_disp);
	rt_named_sem_delete(mess_disp);
    	
    	rt_mbx_delete(mbx);
    	
    	rt_shm_free(nam2num("INFO_SHM"));
    	
	rt_shm_free(SEN_SHM);
	rt_shm_free(ACT_SHM);
	rt_shm_free(REFSENS);
	
	rt_task_delete(main_Task);
 	printf("The controller is STOPPED\n");
	return 0;
}
