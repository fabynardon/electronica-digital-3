#include <ipc_msg.h>
#include <app_multicore_cfg.h>

#define IPCEX_QUEUE_SZ        64
#define PID_Update	0

#define UART_SRB_SIZE 128	/* Send */
#define UART_RRB_SIZE 32	/* Receive */

typedef struct {
	struct {
		uint16_t cpu;
		uint16_t pid;
	} id;
	struct {
		float Kp;
		float Ki;
		float Kd;
	} pid;
} ipcex_msg_t;

static ipcex_msg_t ipcex_queue[IPCEX_QUEUE_SZ];

static RINGBUFF_T txring, rxring;
static uint8_t rxbuff[UART_RRB_SIZE], txbuff[UART_SRB_SIZE];

//void M4_IRQHandler(void)
//{
//	int i = 0;
//	ipcex_msg_t msg;
//	Chip_CREG_ClearM4Event();
//
//	if (IPC_tryPopMsg(&msg) != QUEUE_VALID) {
//		return;
//	}
//
//	switch (msg.id.pid) {
//	case PID_BLINKY:
//		while (msg.data0) {
//			Chip_GPIO_SetPinState(LPC_GPIO_PORT,0,14, !Chip_GPIO_GetPinState(LPC_GPIO_PORT, 0, 14));
//			//Board_LED_Set(i, msg.data1 & 1);
//			msg.data0 >>= 1;
//			msg.data1 >>= 1;
//		}
//		break;
//
//	//default:
//		/* Not for us just ignore */
//		//printf("M0 Ignoring unknown message!");
//	}
//
//}

void UART2_IRQHandler(void){

	Chip_UART_IRQRBHandler(LPC_USART2, &rxring, &txring);
}

int main(void)
{
	ipcex_msg_t msg;

	IPC_initMsgQueue(ipcex_queue, sizeof(ipcex_msg_t), IPCEX_QUEUE_SZ);
	//NVIC_EnableIRQ(M4_IRQn);

    RingBuffer_Init(&rxring, rxbuff, 1, UART_RRB_SIZE);
    RingBuffer_Init(&txring, txbuff, 1, UART_SRB_SIZE);

    Chip_UART_SetupFIFOS(LPC_USART2, (UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_TRG_LEV3));
    Chip_UART_IntEnable(LPC_USART2, (UART_IER_RBRINT | UART_IER_RLSINT));

    NVIC_SetPriority(USART2_IRQn, 1);
    NVIC_EnableIRQ(USART2_IRQn);

    Board_Led_Set(LED3, true);
 	while (1) {
 		if(RingBuffer_GetCount(&rxring) == 12){

 			msg.id.cpu = CPUID_M4;
 			msg.id.pid = PID_Update;
 			Chip_UART_ReadRB(LPC_USART2, &rxring, &msg.pid, 12);
 			IPC_tryPushMsg(msg.id.cpu, &msg);

 		}
 	}

 	return 1;
}
