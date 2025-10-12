#include "chip.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"

const uint32_t OscRateIn = 12000000; // 外部オシレータの周波数（12MHz）
const uint32_t ExtRateIn = 0;        // 外部クロックの周波数

#define LED_PORT 0
#define LED_PIN 1

// // UART
// static RINGBUFF_T txring, rxring;
// #define UART_SRB_SIZE 128	/* Send */
// #define UART_RRB_SIZE 32	/* Receive */
// static uint8_t rxbuff[UART_RRB_SIZE];
// static uint8_t txbuff[UART_SRB_SIZE];
// int old_main(void) {
//     // UART
// 	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_6, (IOCON_FUNC1 | IOCON_MODE_INACT));/* RXD */
// 	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_7, (IOCON_FUNC1 | IOCON_MODE_INACT));/* TXD */
//     Chip_UART_Init(LPC_USART);
// 	Chip_UART_SetBaud(LPC_USART, 115200);
// 	Chip_UART_ConfigData(LPC_USART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
// 	Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
// 	Chip_UART_TXEnable(LPC_USART);
//     RingBuffer_Init(&rxring, rxbuff, 1, UART_RRB_SIZE);
// 	RingBuffer_Init(&txring, txbuff, 1, UART_SRB_SIZE);
//     Chip_UART_IntEnable(LPC_USART, (UART_IER_RBRINT | UART_IER_RLSINT));
//     NVIC_SetPriority(UART0_IRQn, 1);
//     NVIC_EnableIRQ(UART0_IRQn);
//     //Chip_UART_SendRB(LPC_USART, &txring, "Hello World!!!!!!", 8);


/* LED0 toggle thread */
static void vLEDTask(void *pvParameters) {
	while (1) {
        // LED点灯
        Chip_GPIO_SetPinState(LPC_GPIO, LED_PORT, LED_PIN, false);
        vTaskDelay(100/portTICK_RATE_MS);
        // LED消灯
        Chip_GPIO_SetPinState(LPC_GPIO, LED_PORT, LED_PIN, true);
        vTaskDelay(900/portTICK_RATE_MS);
	}
}

int main(void)
{
    SystemCoreClockUpdate();
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_GPIO);
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_UART0);
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_CT16B0);

    // GPIO
    Chip_GPIO_Init(LPC_GPIO);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, LED_PORT, LED_PIN);

    xTaskCreate(vLEDTask, "vTaskLed",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(xTaskHandle *) NULL);

    vTaskStartScheduler();
    return 1;
}

extern "C" void SystemInit(void) {
  	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSOSC_PD);
    for (int i = 0; i < 0x100; i++) { __NOP();}
	Chip_Clock_SetSystemPLLSource(SYSCTL_PLLCLKSRC_MAINOSC);
	Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);
	Chip_Clock_SetupSystemPLL(5, 1);// FCLKIN=12MHz, MSEL=4, PSEL=1, 60MHz
    Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSPLL_PD);

	while (!Chip_Clock_IsSystemPLLLocked()) {}

	Chip_Clock_SetSysClockDiv(1);
	Chip_FMC_SetFLASHAccess(FLASHTIM_50MHZ_CPU);
	Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLOUT);
}

extern "C" void vApplicationMallocFailedHook(void)
{
	taskDISABLE_INTERRUPTS();
	for (;; ) {}
}

/* FreeRTOS stack overflow hook */
extern "C" void vApplicationStackOverflowHook( TaskHandle_t xTask, char * pcTaskName )
{
	(void) xTask;
	(void) pcTaskName;

	/* Run time stack overflow checking is performed if
	   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	   function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for (;; ) {}
}