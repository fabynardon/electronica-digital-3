#include "board.h"

#define MUESTRAS 10

uint16_t ch1[MUESTRAS], ch2[MUESTRAS], ch3[MUESTRAS];
uint32_t muestras_cont_adc0, muestras_cont_adc1;
bool stop_flag_adc0, stop_flag_adc1;
char buffer[2*MUESTRAS];

/************ ADC setup ************/

void ADC_setup(void) {

     static ADC_CLOCK_SETUP_T ADC0Setup, ADC1Setup;

     Chip_ADC_Init(LPC_ADC0, &ADC0Setup);

     Chip_ADC_EnableChannel(LPC_ADC0, ADC_CH1, ENABLE);

     Chip_ADC_Int_SetChannelCmd(LPC_ADC0, ADC_CH1, ENABLE);

     Chip_ADC_SetSampleRate(LPC_ADC0, &ADC0Setup, ADC_MAX_SAMPLE_RATE);

     Chip_ADC_SetResolution(LPC_ADC0, &ADC0Setup, ADC_10BITS);

     Chip_ADC_SetBurstCmd(LPC_ADC0, DISABLE);

     Chip_ADC_SetStartMode(LPC_ADC0, ADC_START_ON_CTOUT15, ADC_TRIGGERMODE_RISING);

     Chip_ADC_Init(LPC_ADC1, &ADC1Setup);

     Chip_ADC_EnableChannel(LPC_ADC1, ADC_CH2, ENABLE);

     Chip_ADC_Int_SetChannelCmd(LPC_ADC1, ADC_CH2, ENABLE);

     Chip_ADC_SetSampleRate(LPC_ADC1, &ADC1Setup, ADC_MAX_SAMPLE_RATE);

     Chip_ADC_SetResolution(LPC_ADC1, &ADC1Setup, ADC_10BITS);

     Chip_ADC_SetBurstCmd(LPC_ADC1, DISABLE);

     Chip_ADC_SetStartMode(LPC_ADC1, ADC_START_ON_CTOUT15, ADC_TRIGGERMODE_RISING);

}

void ADC0_IRQHandler(void) {

	Chip_ADC_ReadValue(LPC_ADC0, ADC_CH1, &ch1[muestras_cont_adc0]);
	muestras_cont_adc0++;
 	if(muestras_cont_adc0 == MUESTRAS){
 		NVIC_DisableIRQ(ADC0_IRQn);
 		stop_flag_adc0 = 1;
 	}

}

void ADC1_IRQHandler(void) {

	Chip_ADC_ReadValue(LPC_ADC1, ADC_CH2, &ch2[muestras_cont_adc1]);
	muestras_cont_adc1++;
	if(muestras_cont_adc1 == MUESTRAS){
		NVIC_DisableIRQ(ADC1_IRQn);
		stop_flag_adc1 = 1;
	}
}

/************ TIMER setup ************/

#define TICKRATE_HZ	500

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

void Serial_Send(void){
	Board_Led_Set(LEDB, true);
	int i;
	for(i=0;i<MUESTRAS;i++){
		buffer[2*i+1]=(char) (ch1[i] & 0xFF);
		buffer[2*i]=(char) ((ch1[i] >> 8) & 0xFF);
	}
	for(i=0;i<(2*MUESTRAS);i++){
		Board_UARTPutChar(buffer[i]);
	}
	Board_Led_Set(LEDB, false);
}

/************ main() ************/

int main(void)
{
	char in_char=0;

	SystemCoreClockUpdate();

	Board_Init();

	ADC_setup();

	TIMER_setup();

//     NVIC_EnableIRQ(TIMER3_IRQn);
//     NVIC_ClearPendingIRQ(TIMER3_IRQn);

     while(1) {

    	 in_char=Board_UARTGetChar();
    	 if(in_char == 0xD){

    		 Board_Led_Set(LED3, true);
    		 NVIC_EnableIRQ(ADC0_IRQn);
    		 NVIC_ClearPendingIRQ(ADC0_IRQn);

    		 NVIC_EnableIRQ(ADC1_IRQn);
    		 NVIC_ClearPendingIRQ(ADC1_IRQn);

    		 muestras_cont_adc0 = 0;
    		 muestras_cont_adc1 = 0;
     	}
     	if( (stop_flag_adc0 == 1) && (stop_flag_adc1 == 1) ){
//    	if(stop_flag_adc0==1){
     		Board_Led_Set(LED3, false);
     		stop_flag_adc0 = 0;
     		stop_flag_adc1 = 0;
    		Serial_Send();
     	}
     }

     return 1;
}
