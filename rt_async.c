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
#include "parameters.h"

static RT_TASK* asyncTask;
static struct Stato* info;
static SEM* mess_disp;
static SEM* spazio_disp;
static MBX* mbx;

int main(void){

	if(!(asyncTask = rt_task_init(nam2num("DIAG"), 1, STACK_SIZE, 0))){
		printf("failed creating DIAG task\n");
		exit(1);
	}
	
	int req = 1, i;
	
	info = rtai_malloc(nam2num("INFO_SHM"), sizeof(struct Stato));
	spazio_disp = rt_typed_named_sem_init("SPAZIO_DISP_INFO", 1, CNT_SEM | PRIO_Q);
	mess_disp = rt_typed_named_sem_init("MESS_DISP_INFO", 0, CNT_SEM | PRIO_Q);
	mbx = rt_typed_named_mbx_init("MAILBOX", sizeof(int), FIFO_Q);
	
	//Invio della richiesta dello stato attraverso la mailbox
	rt_mbx_send(mbx, &req, sizeof(int));
	
	//Attesa dell'invio della risposta da parte del gather
	rt_sem_wait(mess_disp);
	
	//I dati sullo stato sono stati ricevuti e vengono stampati
	for(i = 0; i < 3; i++){
		if(info->stato[i] == 0){
			printf("CONTROLLORE %d = ATTIVO\n", i);
			printf("AZIONE DI CONTROLLO %d = %d\n", i, info->ctrl_signal[i]);
		}else
			printf("CONTROLLORE %d = FALLITO\n",i);
	}
	
	printf("DECISIONE VOTER = %d\n", info->decisione);
	printf("STATO BUFFER :\n");
	for(i = 0; i < BUF_SIZE; i++)
		printf("%d ", info->valori_buf[i]);
	printf("\n");
	
	//Sblocca il gather per la scrittura delle informazioni di stato di richieste successive
	rt_sem_signal(spazio_disp);
	
	//N.B. la deallocazione delle strutture ipc viene effettuata dal main di controller.c

	rt_task_delete(asyncTask);

	return 0;
}
