/*
  Copyright 2018 Technical University of Denmark, DTU Compute.
  All rights reserved.

  TTEthernet library

  Author: Maja Lund (maja_lala@hotmail.com)
*/

#include "tte.h"

unsigned int TTE_MAX_TRANS_DELAY;
unsigned int TTE_COMP_DELAY;
unsigned int TTE_PRECISION;

unsigned int integration_period; //clock cycles at 80Mhz (12.5 ns)
unsigned int integration_cycle; 
unsigned int cluster_period;
static unsigned long long start_time; //clock cycles at 80MHz (12.5 ns)
static unsigned long long timer_time; //clock cycles at 80MHz (12.5 ns)
static unsigned long long receive_pit; //clock cycles at 80MHz (12.5 ns)
static unsigned long long scheduled_pit; //clock cycles at 80MHz (12.5 ns)
signed long long prev_error;
unsigned char CT_marker[4];
unsigned char max_sched;
unsigned char mac[6];
unsigned long long send_times[2000];
int send_time_i=0;

struct VL{
   unsigned char max_queue;
   unsigned int startTime;
   unsigned int period;
   unsigned int* queue;
   unsigned int* sizeQueue;
   unsigned char addplace;
   unsigned char rmplace;
};

struct VL *VLarray;
unsigned char VLsize;
unsigned int *sched;
unsigned int *VLsched;
unsigned int startTick;
unsigned char schedplace;
void tte_clock_tick(void) __attribute__((naked));
void tte_clock_tick_log(void) __attribute__((naked));

unsigned char is_pcf(unsigned int addr){
	unsigned type_1 = mem_iord_byte(addr + 12);
	unsigned type_2 = mem_iord_byte(addr + 13);
	if(type_1 == 0x89 && type_2 == 0x1D){ 
		return 1;
	}
	return 0;
}

void tte_wait_for_message(unsigned long long * receive_point){
	  while ((eth_iord(0x04) & 0x4)==0){
	    *receive_point = *(_iodev_ptr_t)(__PATMOS_TIMER_LOCLK); //to avoid delay error
	  };
	  *receive_point = *(_iodev_ptr_t)(__PATMOS_TIMER_LOCLK);
}

void tte_clear_free_rx_buffer(unsigned int addr){
	eth_iowr(0x04, 0x00000004);
	unsigned cur_data = eth_iord(addr);
    	eth_iowr(addr, cur_data | (1<<15));
}

unsigned char tte_receive_log(unsigned int addr,unsigned long long rec_start,signed long long error[],int i){
	if(is_pcf(addr)){
	  if((mem_iord_byte(addr + 28)) == 0x2){
	    if(handle_integration_frame_log(addr,rec_start,error,i)){
              return 1;
            }
	    else{
              return 0;
            }
          }
	} else if (is_tte(addr)){
	  return 2;
        }
	return 3;
}

unsigned char tte_receive(unsigned int addr,unsigned long long rec_start){ //0 for failed pcf, 1 for success pcf, 2 for tte, 3 otherwise
	if(is_pcf(addr)){
	  if((mem_iord_byte(addr + 28)) == 0x2){ //integration frame
	    if(handle_integration_frame(addr,rec_start)){
	      return 1;  	
	    }
	    else{
              return 0;
            }
          }
	  //other PCF types currently return 3
	} else if (is_tte(addr)){
	  return 2;
        }
	return 3;
}

int handle_integration_frame_log(unsigned int addr,unsigned long long receive_pit,
  signed long long error[],int i){
	unsigned long long permanence_pit;
	unsigned long long sched_rec_pit;
	unsigned long long trans_clock;
	signed long long err;

	trans_clock = mem_iord_byte(addr + 34);
	trans_clock = (trans_clock << 8) | (mem_iord_byte(addr + 35));
	trans_clock = (trans_clock << 8) | (mem_iord_byte(addr + 36));
	trans_clock = (trans_clock << 8) | (mem_iord_byte(addr + 37));
	trans_clock = (trans_clock << 8) | (mem_iord_byte(addr + 38));
	trans_clock = (trans_clock << 8) | (mem_iord_byte(addr + 39));
	trans_clock = (trans_clock << 8) | (mem_iord_byte(addr + 40));
	trans_clock = (trans_clock << 8) | (mem_iord_byte(addr + 41));
	trans_clock = transClk_to_clk(trans_clock);

	integration_cycle = mem_iord_byte(addr + 14);
	integration_cycle = (integration_cycle << 8) | (mem_iord_byte(addr + 15));
	integration_cycle = (integration_cycle << 8) | (mem_iord_byte(addr + 16));
	integration_cycle = (integration_cycle << 8) | (mem_iord_byte(addr + 17));

	permanence_pit = receive_pit + (TTE_MAX_TRANS_DELAY-trans_clock); 

	if(start_time==0){
		start_time=permanence_pit-(2*TTE_MAX_TRANS_DELAY + TTE_COMP_DELAY);
	}

	sched_rec_pit = start_time + 2*TTE_MAX_TRANS_DELAY + TTE_COMP_DELAY;

	if(permanence_pit>(sched_rec_pit-TTE_PRECISION) &&
	    permanence_pit<(sched_rec_pit+TTE_PRECISION)){

		err=permanence_pit - sched_rec_pit;
		error[i]=err;
		start_time = start_time+err;

		if(integration_cycle==0){
		  schedplace=0;
		  timer_time = start_time+(CYCLES_PER_UNIT*startTick); 
		  arm_clock_timer(timer_time);
		}
		start_time += integration_period;

		return 1;
	}
	else{
	    start_time = 0;
	}
	return 0;
}

int handle_integration_frame(unsigned int addr,unsigned long long receive_pit){
	unsigned long long permanence_pit;
	unsigned long long sched_rec_pit;
	unsigned long long trans_clock;

	trans_clock = mem_iord_byte(addr + 34);
	trans_clock = (trans_clock << 8) | (mem_iord_byte(addr + 35));
	trans_clock = (trans_clock << 8) | (mem_iord_byte(addr + 36));
	trans_clock = (trans_clock << 8) | (mem_iord_byte(addr + 37));
	trans_clock = (trans_clock << 8) | (mem_iord_byte(addr + 38));
	trans_clock = (trans_clock << 8) | (mem_iord_byte(addr + 39));
	trans_clock = (trans_clock << 8) | (mem_iord_byte(addr + 40));
	trans_clock = (trans_clock << 8) | (mem_iord_byte(addr + 41));
	trans_clock = transClk_to_clk(trans_clock);

	integration_cycle = mem_iord_byte(addr + 14);
	integration_cycle = (integration_cycle << 8) | (mem_iord_byte(addr + 15));
	integration_cycle = (integration_cycle << 8) | (mem_iord_byte(addr + 16));
	integration_cycle = (integration_cycle << 8) | (mem_iord_byte(addr + 17));

	permanence_pit = receive_pit + (TTE_MAX_TRANS_DELAY-trans_clock); 

	if(start_time==0){
		start_time=permanence_pit-(2*TTE_MAX_TRANS_DELAY + TTE_COMP_DELAY);
	}

	sched_rec_pit = start_time + 2*TTE_MAX_TRANS_DELAY + TTE_COMP_DELAY;

	if(permanence_pit>(sched_rec_pit-TTE_PRECISION) &&
	    permanence_pit<(sched_rec_pit+TTE_PRECISION)){

		start_time = start_time+(permanence_pit - sched_rec_pit);

		if(integration_cycle==0){
		  schedplace=0;
		  timer_time = start_time+(CYCLES_PER_UNIT*startTick);
		  arm_clock_timer(timer_time);
		}
		start_time += integration_period;

		return 1;
	}
	else{
	    start_time = 0;
	}
	return 0;
}

unsigned long long transClk_to_clk (unsigned long long transClk){
        return ((transClk*10)>>16)*536871>>26; //*536871>>26 is almost equivalent to /125
}

unsigned char is_tte(unsigned int addr){
        #pragma loopbound min 0 max 4
	for(int i=0;i<4;i++){
	  if(mem_iord_byte(addr + i)!=CT_marker[i]){
	    return 0;
	  }
	}
	return 1;
}

void tte_initialize(unsigned int int_period, unsigned int cl_period, unsigned char CT[], unsigned char VLcount,
    unsigned int max_delay, unsigned int comp_delay, unsigned int precision){ 
	integration_period=int_period*CYCLES_PER_UNIT;
	cluster_period=cl_period;
	for(int i=0;i<4;i++){
	  CT_marker[i]=CT[i];
        }
	TTE_MAX_TRANS_DELAY = max_delay;
	TTE_COMP_DELAY = comp_delay;
	TTE_PRECISION = precision;

	unsigned long long macAdd = get_mac_address();
	for(int i=5;i>=0;i--){
	  mac[i]=macAdd & 0xFF;
	  macAdd = macAdd>>8;
	}
	eth_iowr(0x00, 0x0000A423); //like eth_mac_initialize, but with pro-bit set and fullduplex
	eth_iowr(0x08, 0x00000004); //generate interrupt on received frame

	VLarray = malloc((VLcount+1) * sizeof(struct VL));
	VLsize=VLcount+1;	
	tte_init_VL(VLcount, int_period, int_period); //dummy VL for incorporating PCF's in schedule
	return;
}

void tte_init_VL(unsigned char i, unsigned int start, unsigned int period){
	VLarray[i].startTime=start;
	VLarray[i].period=period;
	VLarray[i].addplace=0;
	VLarray[i].rmplace=0;
	VLarray[i].max_queue=(unsigned char) cluster_period/period;
	VLarray[i].queue=(unsigned int*)malloc(VLarray[i].max_queue*sizeof(unsigned int));
	VLarray[i].sizeQueue=(unsigned int*)malloc(VLarray[i].max_queue*sizeof(unsigned int));
	for(int j=0;j<VLarray[i].max_queue;j++){
  	  VLarray[i].queue[j]=0;
  	  VLarray[i].sizeQueue[j]=0;
	}
}

void tte_generate_schedule(){
	unsigned int VLcurrent[VLsize];
	unsigned int min=cluster_period;
	unsigned int VL;
	unsigned int current;
	max_sched=0;
	for(int i=0;i<VLsize;i++){
	  if(VLarray[i].startTime<min){
	    min=VLarray[i].startTime;
	    VL=i;
	  }
	  VLcurrent[i]=VLarray[i].startTime;
	  max_sched=max_sched+VLarray[i].max_queue;
	}
	startTick=min; 	current=min;
	VLcurrent[VL]=VLcurrent[VL]+VLarray[VL].period;
	sched=(unsigned int*)malloc(max_sched*sizeof(unsigned int));
	VLsched=(unsigned int*)malloc(max_sched*sizeof(unsigned int));
	VLsched[0]=VL;
	int index=0;
	while(1){
  	  min=cluster_period*2;
	  for(int i=0;i<VLsize;i++){
	    if(VLcurrent[i]<min){
	      min=VLcurrent[i];
	      VL=i;
	    }
	  }
	  sched[index]=min-current;
	  if(index<max_sched-1) VLsched[index+1]=VL;
	  current=min;
	  VLcurrent[VL]=VLcurrent[VL]+VLarray[VL].period;
	  if(min>=cluster_period) break;
	  index++;
	}
}

void tte_start_ticking(char log_sending,char enable_int, void (int_handler)(void)){
	tte_generate_schedule();
	void (*timer_handler)(void);
	if(log_sending){
	  timer_handler=&tte_clock_tick_log;
	}
	else{
	  timer_handler=&tte_clock_tick;
	}

	if(enable_int){
	  exc_register(16, int_handler); //ethmac interrupt, moved to 16 to have prio over timer
	  exc_register(17, timer_handler); //timer, note this is usually 16
	  intr_unmask(16);
	  intr_unmask(17);
	}
	else{
	  exc_register(16, timer_handler); //timer
	  intr_unmask(16);
	}
  	intr_clear_all_pending();
  	intr_enable();

  	start_time = 0; 
}

void tte_prepare_header(unsigned int tx_addr, unsigned char VL[], unsigned char ethType[]){
  #pragma loopbound min 4 max 4
  for(int i=0; i<4; i++){
    mem_iowr_byte(tx_addr + i, CT_marker[i]);
  }
  mem_iowr_byte(tx_addr + 4, VL[0]);
  mem_iowr_byte(tx_addr + 5, VL[1]);

  #pragma loopbound min 6 max 6
  for(int i=6; i<12; i++){
    mem_iowr_byte(tx_addr + i, mac[i-6]);
  }

  mem_iowr_byte(tx_addr + 12, ethType[0]);
  mem_iowr_byte(tx_addr + 13, ethType[1]);
}

void tte_prepare_test_data(unsigned int tx_addr, unsigned char VL[], unsigned char data, int length){
  unsigned char ethType[2];
  ethType[0]=((length-14)>>8) & 0xFF;
  ethType[1]=(length-14) & 0xFF;

  tte_prepare_header(tx_addr,VL,ethType);

  #pragma loopbound min 0 max 1500
  for(int i=14; i<length; i++){
    mem_iowr_byte(tx_addr + i, data);
  }
}

void tte_prepare_pcf(unsigned int addr,unsigned char VL[],unsigned char type){
	unsigned char pcfType[] = {0x89,0x1D};
	tte_prepare_header(addr, VL, pcfType);
	
	//integration cycle
	#pragma loopbound min 4 max 4
	for(int i=14; i<18; i++){
	  mem_iowr_byte(addr + i, 0x00);
	}
	//membership
	#pragma loopbound min 4 max 4
	for(int i=18; i<21; i++){
	  mem_iowr_byte(addr + i, 0x00);
	}
	mem_iowr_byte(addr + 21, 0x02); //audioclient membership
	//sync prio
	mem_iowr_byte(addr+26, 0x01);
	//sync domain
	mem_iowr_byte(addr+27, 0x01);
	//type
	mem_iowr_byte(addr+28, type); //integration frame 0x02,coldstart 0x04, coldstart ack 0x08
	//transparent clock
	#pragma loopbound min 8 max 8
	for(int i=34; i<42; i++){
	  mem_iowr_byte(addr + i, 0x00);
	}
}

void tte_stop_ticking(){
  for(int i=0;i<VLsize;i++){
    free(VLarray[i].queue);
    free(VLarray[i].sizeQueue);
  }
  free(VLarray);
}

char tte_schedule_send(unsigned int addr,unsigned int size,unsigned char i){
  if(VLarray[i].queue[VLarray[i].addplace]==0){
    VLarray[i].queue[VLarray[i].addplace]=addr;
    VLarray[i].sizeQueue[VLarray[i].addplace]=size;
    VLarray[i].addplace++;
    if(VLarray[i].addplace==VLarray[i].max_queue){
      VLarray[i].addplace=0;
    }
    return 1;
  } 
  return 0; //scheduling error
}

void tte_send_data(unsigned char i){ 
  int tx_addr=VLarray[i].queue[VLarray[i].rmplace];
  VLarray[i].queue[VLarray[i].rmplace]=0;
  eth_mac_send(tx_addr, VLarray[i].sizeQueue[VLarray[i].rmplace]);
  VLarray[i].rmplace++;
  if(VLarray[i].rmplace==VLarray[i].max_queue){
    VLarray[i].rmplace=0;
  }
  return;
}

void tte_clock_tick(void) {
  exc_prologue();
  timer_time += (CYCLES_PER_UNIT*sched[schedplace]);
  int i=VLsched[schedplace];
  schedplace++;
  if(schedplace<max_sched){ 
    if(VLsched[schedplace]==(VLsize-1)){
      timer_time = start_time;
    }
    arm_clock_timer(timer_time);
  }
  if(VLarray[i].queue[VLarray[i].rmplace]>0){
    tte_send_data(i);
  }
  exc_epilogue();
}

void tte_clock_tick_log(void) {
  exc_prologue();
  timer_time += (CYCLES_PER_UNIT*sched[schedplace]);
  int i=VLsched[schedplace];
  schedplace++;
  if(schedplace<max_sched){ 
    if(VLsched[schedplace]==(VLsize-1)){
      timer_time = start_time;
    }
    arm_clock_timer(timer_time);
  }
  if(VLarray[i].queue[VLarray[i].rmplace]>0){
    send_times[send_time_i]=get_cpu_cycles();
    send_time_i++;
    tte_send_data(i);
  }
  exc_epilogue();
}
