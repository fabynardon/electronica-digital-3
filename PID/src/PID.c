#include "board.h"
#include "arm_math.h"
#include "math_helper.h"

#define set_point 1

#define NUMERADOR 3
#define DENOMINADOR 4

float Coef_B[]={0.09, 0.01, -0.068};
float Coef_A[]={1, -2.29, 1.85, -0.53};

float X[NUMERADOR]={0, 0, 0};
float Y[DENOMINADOR]={0, 0, 0, 0};

union MyUnion{
	struct {
		float32_t planta_out;
		float32_t planta_in;
		float32_t error;
		uint16_t ch1;
	} variables;
	char buffer[14];
} send;

bool ADC_flag;

arm_pid_instance_f32 PID_inst;

#define TICKRATE_HZ	4

void TIMER_setup(){

     Chip_TIMER_Init(LPC_TIMER3);

     Chip_RGU_TriggerReset(RGU_TIMER3_RST);

     while (Chip_RGU_InReset(RGU_TIMER3_RST)) {}

     uint32_t timerFreq = Chip_Clock_GetRate(CLK_MX_TIMER3);

     Chip_TIMER_Reset(LPC_TIMER3);

     Chip_TIMER_MatchEnableInt(LPC_TIMER3, 3);

     Chip_TIMER_SetMatch(LPC_TIMER3, 3, (timerFreq / (2*TICKRATE_HZ)));

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

void ADC0_IRQHandler(void){
	Chip_ADC_ReadValue(LPC_ADC0, ADC_CH1, &send.variables.ch1);
	ADC_flag = true;
}

void shift(float *vec, float in, int length) {
    int i;
	for(i=length; i>0; i--) {
		vec[i] = vec[i-1];
	}
	vec[0]=in;
}

float filtro(float *A, float *B, float in) {
	int i;
	float aux = 0;

	shift(&X[0], in, NUMERADOR);

	for(i=0; i<NUMERADOR; i++) {
		aux += B[i] * X[i] * A[0];
	}

	for(i=1; i<DENOMINADOR; i++){
	    aux -= A[i] * Y[i];
	}

	shift(&Y[1], aux, NUMERADOR-1);
	return aux;
}

int main(void) {

    int i;

    SystemCoreClockUpdate();
    Board_Init();
	ADC_setup();
	TIMER_setup();

	NVIC_EnableIRQ(ADC0_IRQn);
	NVIC_ClearPendingIRQ(ADC0_IRQn);

    PID_inst.Kp = 1.1826;
    PID_inst.Ki = 0.6365;
    PID_inst.Kd = 1.1393;

    arm_pid_init_f32(&PID_inst, 1);

    while(1) {
    	if(ADC_flag == true){

    		ADC_flag = false;

    		send.variables.error = set_point - send.variables.planta_out;
    		send.variables.planta_in = arm_pid_f32(&PID_inst, send.variables.error);
    		send.variables.planta_out = filtro(&Coef_A[0], &Coef_B[0], send.variables.planta_in);

//    		send.variables.error = 1.1;
//    		send.variables.planta_in = 2.2;
//    		send.variables.planta_out = 3.3;

    		for(i=13;i>-1;i--){
    			Board_UARTPutChar(send.buffer[i]);
    		}
    	}
    }
}
