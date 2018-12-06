//---------------- PARAMETERS.H ----------------------- 

#define TICK_TIME 100000000
#define CNTRL_TIME 50000000
#define VOTER_TIME 1000000

#define TASK_PRIORITY 1

#define STACK_SIZE 10000

#define BUF_SIZE 10

#define SEN_SHM 121111
#define ACT_SHM 112112
#define REFSENS 111213

#define SPACE_SEM 1234444
#define MEAS_SEM 1234445

//Struttura utilizzata per il trasferimento delle info di stato tra diag e gather
struct Stato
{
	int ctrl_signal[3];
	int stato[3];
	int decisione;
	int valori_buf[BUF_SIZE];
	
};
