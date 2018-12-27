#include <ipc_msg.h>
#include <app_multicore_cfg.h>

#define IPCEX_QUEUE_SZ        64
#define PID_Update	0

static volatile uint8_t dmaChannelNum;

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

static ipcex_msg_t msg;
static ipcex_msg_t ipcex_queue[IPCEX_QUEUE_SZ];

void DMA_IRQHandler(void){

	if (Chip_GPDMA_Interrupt(LPC_GPDMA, dmaChannelNum) == SUCCESS) {
		IPC_tryPushMsg(msg.id.cpu, &msg);
		Chip_GPDMA_Transfer(LPC_GPDMA, dmaChannelNum, GPDMA_CONN_UART2_Rx, (uint32_t) &msg.pid, GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA, 12);
	}
}

int main(void){

	msg.id.cpu = CPUID_M4;

	msg.id.pid = PID_Update;

	IPC_initMsgQueue(ipcex_queue, sizeof(ipcex_msg_t), IPCEX_QUEUE_SZ);

    Chip_UART_SetupFIFOS(LPC_USART2, (UART_FCR_FIFO_EN | UART_FCR_RX_RS | UART_FCR_TX_RS | UART_FCR_TRG_LEV3 | UART_FCR_DMAMODE_SEL));

	Chip_GPDMA_Init(LPC_GPDMA);

    NVIC_EnableIRQ(DMA_IRQn);

	dmaChannelNum = Chip_GPDMA_GetFreeChannel(LPC_GPDMA, GPDMA_CONN_UART2_Rx);

	Chip_GPDMA_Transfer(LPC_GPDMA, dmaChannelNum, GPDMA_CONN_UART2_Rx, (uint32_t) &msg.pid, GPDMA_TRANSFERTYPE_P2M_CONTROLLER_DMA, 12);

 	while (1) {
 		__WFI();
 	}

 	return 1;
}
