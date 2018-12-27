#include "board.h"

STATIC RINGBUFF_T txring, rxring;

#define UART_SRB_SIZE 128	/* Send */
#define UART_RRB_SIZE 32	/* Receive */

static uint8_t rxbuff[UART_RRB_SIZE], txbuff[UART_SRB_SIZE];

typedef struct {
   int Kp;
   int Ki;
   int Kd;
} PID_T;

void UART2_IRQHandler(void){

	Chip_UART_IRQRBHandler(LPC_USART2, &rxring, &txring);
}

int main(void)
{
	PID_T PID;

    SystemCoreClockUpdate();
    Board_Init();

    RingBuffer_Init(&rxring, rxbuff, 1, UART_RRB_SIZE);
    RingBuffer_Init(&txring, txbuff, 1, UART_SRB_SIZE);

    Chip_UART_SetupFIFOS(LPC_USART2, (UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_TRG_LEV3));
    Chip_UART_IntEnable(LPC_USART2, (UART_IER_RBRINT | UART_IER_RLSINT));

    NVIC_SetPriority(USART2_IRQn, 1);
    NVIC_EnableIRQ(USART2_IRQn);

 	while (1) {
 		if(RingBuffer_GetCount(&rxring) == 12){
 			Chip_UART_ReadRB(LPC_USART2, &rxring, &PID, 12);
 		}
 	}

     return 1;
}
