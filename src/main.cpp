#include "chip.h"
#include "string.h"

const uint32_t OscRateIn = 8000000; // 外部オシレータの周波数（8MHz）
const uint32_t ExtRateIn = 0;        // 外部クロックの周波数

#define LED_PORT 0
#define LED_PIN 1

// UART
static RINGBUFF_T txring, rxring;
#define UART_SRB_SIZE 128	/* Send */
#define UART_RRB_SIZE 32	/* Receive */
static uint8_t rxbuff[UART_RRB_SIZE];
static uint8_t txbuff[UART_SRB_SIZE];

void delay(uint32_t count);

extern "C" void SystemInit(void) {
  	Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSOSC_PD);
    for (int i = 0; i < 0x100; i++) { __asm("nop");}
	Chip_Clock_SetSystemPLLSource(SYSCTL_PLLCLKSRC_MAINOSC);
	Chip_SYSCTL_PowerDown(SYSCTL_POWERDOWN_SYSPLL_PD);
	Chip_Clock_SetupSystemPLL(6, 1);// FCLKIN=8MHz, MSEL=6, PSEL=1, 48MHz
    Chip_SYSCTL_PowerUp(SYSCTL_POWERDOWN_SYSPLL_PD);

	while (!Chip_Clock_IsSystemPLLLocked()) {}

	Chip_Clock_SetSysClockDiv(1);
	Chip_FMC_SetFLASHAccess(FLASHTIM_50MHZ_CPU);
	Chip_Clock_SetMainClockSource(SYSCTL_MAINCLKSRC_PLLOUT);
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);
}

int main(void) {
    // システムクロック初期化
    SystemCoreClockUpdate();
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_GPIO);
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_UART0);
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_CT16B0);

    // Timer0 
    // for precious delay, 10us by 1 count
    Chip_TIMER_Init(LPC_TIMER16_0);
    Chip_TIMER_Reset(LPC_TIMER16_0);
    Chip_TIMER_PrescaleSet(LPC_TIMER16_0, (SystemCoreClock / 10000) - 1); // 100us
    Chip_TIMER_Enable(LPC_TIMER16_0);

    // GPIO
    Chip_GPIO_Init(LPC_GPIO);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, LED_PORT, LED_PIN);
    
    // UART
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_6, (IOCON_FUNC1 | IOCON_MODE_INACT));/* RXD */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_7, (IOCON_FUNC1 | IOCON_MODE_INACT));/* TXD */
    Chip_UART_Init(LPC_USART);
	Chip_UART_SetBaud(LPC_USART, 115200);
	Chip_UART_ConfigData(LPC_USART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
	Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
	Chip_UART_TXEnable(LPC_USART);
    RingBuffer_Init(&rxring, rxbuff, 1, UART_RRB_SIZE);
	RingBuffer_Init(&txring, txbuff, 1, UART_SRB_SIZE);
    Chip_UART_IntEnable(LPC_USART, (UART_IER_RBRINT | UART_IER_RLSINT));
    NVIC_SetPriority(UART0_IRQn, 1);
    NVIC_EnableIRQ(UART0_IRQn);

    //Chip_UART_SendRB(LPC_USART, &txring, "Hello World!!!!!!", 8);

    // メインループ
    while (1) {
        // LED消灯
        Chip_GPIO_SetPinState(LPC_GPIO, LED_PORT, LED_PIN, true);
        delay(900);
        
        // LED点灯
        Chip_GPIO_SetPinState(LPC_GPIO, LED_PORT, LED_PIN, false);
        delay(100);
    }
    
    NVIC_DisableIRQ(UART0_IRQn);
	Chip_UART_DeInit(LPC_USART);

    return 0;
}

void delay(uint32_t count) {
    volatile uint32_t t_start = Chip_TIMER_ReadCount(LPC_TIMER16_0);
    //Timer 0 count 100us per count
    for (;;) {
        volatile uint32_t t_now = Chip_TIMER_ReadCount(LPC_TIMER16_0);
        if ((t_now - t_start) >= count*10) break;
    }
    // while (count-- > 0) {
    //     for (int i = 0; i < 1000; i++) {
    //         __asm("nop");__asm("nop");__asm("nop");__asm("nop");__asm("nop");
    //         __asm("nop");__asm("nop");__asm("nop");__asm("nop");__asm("nop");
    //         __asm("nop");__asm("nop");__asm("nop");__asm("nop");__asm("nop");
    //         __asm("nop");__asm("nop");__asm("nop");__asm("nop");__asm("nop");
    //         __asm("nop");__asm("nop");__asm("nop");__asm("nop");__asm("nop");
    //         __asm("nop");__asm("nop");__asm("nop");__asm("nop");__asm("nop");
    //         __asm("nop");__asm("nop");__asm("nop");__asm("nop");__asm("nop");
    //     }
    // }
}

extern "C" void UART_IRQHandler(void)
{
	Chip_UART_IRQRBHandler(LPC_USART, &rxring, &txring);
}
