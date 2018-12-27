#include <app_multicore_cfg.h>
#include <ipc_msg.h>
#include "cr_start_m0.h"
#include "arm_math.h"
#include "math_helper.h"

#define IPCEX_QUEUE_SZ	64
#define PID_Update	0
#define F_sample	10
#define set_point 1

float32_t Coef_B[]={0.09, 0.01, -0.068};
float32_t Coef_A[]={1, -2.29, 1.85, -0.53};

float32_t mem[3]={0, 0, 0};

typedef struct {
	struct {
		uint16_t cpu;
		uint16_t pid;
	} id;
	struct {
		float32_t Kp;
		float32_t Ki;
		float32_t Kd;
	} pid;
} ipcex_msg_t;

union MyUnion{
	struct {
		float32_t planta_out;
		float32_t planta_in;
		float32_t error;
		uint16_t ch1;
	} variables;
	char buffer[14];
}send;

int Set_point = 512;

bool ADC_flag = false, PID_flag = false;

ipcex_msg_t ipcex_queue[IPCEX_QUEUE_SZ];
arm_pid_instance_f32 PID_inst;

void TIMER_setup(){

     Chip_TIMER_Init(LPC_TIMER3);

     Chip_RGU_TriggerReset(RGU_TIMER3_RST);

     while (Chip_RGU_InReset(RGU_TIMER3_RST)) {}

     uint32_t timerFreq = Chip_Clock_GetRate(CLK_MX_TIMER3);

     Chip_TIMER_Reset(LPC_TIMER3);

     Chip_TIMER_MatchEnableInt(LPC_TIMER3, 3);

     Chip_TIMER_SetMatch(LPC_TIMER3, 3, timerFreq/(2*F_sample));

     Chip_TIMER_ResetOnMatchEnable(LPC_TIMER3, 3);

     Chip_TIMER_ExtMatchControlSet(LPC_TIMER3, 1, TIMER_EXTMATCH_TOGGLE, 3);

     Chip_TIMER_Enable(LPC_TIMER3);
}

void ADC_setup(void) {

     static ADC_CLOCK_SETUP_T ADC0Setup;

     Chip_ADC_Init(LPC_ADC0, &ADC0Setup);

     Chip_ADC_EnableChannel(LPC_ADC0, ADC_CH1, ENABLE);

     Chip_ADC_Int_SetChannelCmd(LPC_ADC0, ADC_CH1, ENABLE);

     Chip_ADC_SetSampleRate(LPC_ADC0, &ADC0Setup, ADC_MAX_SAMPLE_RATE);

     Chip_ADC_SetResolution(LPC_ADC0, &ADC0Setup, ADC_10BITS);

     Chip_ADC_SetBurstCmd(LPC_ADC0, DISABLE);

     Chip_ADC_SetStartMode(LPC_ADC0, ADC_START_ON_CTOUT15, ADC_TRIGGERMODE_RISING);

}

void Serial_Send(char *buffer, char length){
	int i;
	Board_Led_Set(LEDB, true);
	for(i=(length-1);i>-1;i--){
		Board_UARTPutChar(buffer[i]);
	}
	Board_Led_Set(LEDB, false);
}

void ADC0_IRQHandler(void){
	Chip_ADC_ReadValue(LPC_ADC0, ADC_CH1, &send.variables.ch1);
	ADC_flag = true;
}

void M0APP_IRQHandler(void)
{
	ipcex_msg_t msg;
	Chip_CREG_ClearM0AppEvent();

	if (IPC_tryPopMsg(&msg) != QUEUE_VALID) {
		return;
	}

	switch (msg.id.pid) {
		case PID_Update:
			PID_inst.Kp = msg.pid.Kp;
			PID_inst.Ki = msg.pid.Ki;
			PID_inst.Kd = msg.pid.Kd;
			PID_flag = true;
			break;
	}

}

float planta(float * B, float * A, float planta_in){
    int i, length = sizeof(mem)/4;
    float aux[length], planta_out;

    for(i=0;i<length-1;i++){
        aux[i] = planta_in * B[i+1] + mem[i+1]  - A[i+1] * planta_in * B[0] - A[i+1] * mem[0];
    }
    aux[length-1] = - A[length] * planta_in * B[0] - A[length] * mem[0];

    planta_out = planta_in * B[0] + mem[0];

    for(i=0;i<length;i++){
        mem[i]=aux[i];
    }
    return planta_out;

}

void planta_reset(void){
	int i,length = sizeof(mem)/4;;
	for(i=0;i<length; i++){
		mem[i] = 0;
	}
}

int main(void) {

    SystemCoreClockUpdate();
    Board_Init();

    cr_start_m0(SLAVE_M0APP,&__core_m0app_START__);
	IPC_initMsgQueue(ipcex_queue, sizeof(ipcex_msg_t), IPCEX_QUEUE_SZ);

	ADC_setup();
	TIMER_setup();

	NVIC_EnableIRQ(ADC0_IRQn);
	NVIC_ClearPendingIRQ(ADC0_IRQn);

	NVIC_EnableIRQ(M0APP_IRQn);
	NVIC_ClearPendingIRQ(M0APP_IRQn);

	send.variables.planta_in = 0;
	send.variables.planta_out = 0;
	send.variables.error = 0;

    PID_inst.Kp = 2.2586;
    PID_inst.Ki = 0.2618;
    PID_inst.Kd = 1.3089;

    arm_pid_init_f32(&PID_inst, 1);

    while(1) {
    	if(ADC_flag){
    		ADC_flag = false;
    		send.variables.error = set_point - send.variables.planta_out;
    		send.variables.planta_in = arm_pid_f32(&PID_inst, send.variables.error);
    		send.variables.planta_out = planta(&Coef_B[0], &Coef_A[0], send.variables.planta_in);
    		Serial_Send(&send.buffer[0], 14);

    	}
    	if(PID_flag){
    		PID_flag = false;
    		send.variables.planta_in = 0;
    		send.variables.planta_out = 0;
    		send.variables.error = 0;
    	    arm_pid_init_f32(&PID_inst, 1);
    		planta_reset();
    	}
    }
}
