#include "chip.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include <cstdio>

void delayMicroseconds(uint16_t us);

uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];

const uint32_t OscRateIn = 12000000; // 外部オシレータの周波数（12MHz）
const uint32_t ExtRateIn = 0;        // 外部クロックの周波数

#define LED_PORT 0
#define LED_PIN 1

class Loadcell {
    private:
        int32_t _raw;
        uint8_t _clk_port;
        uint8_t _clk_pin;
        uint8_t _data_port;
        uint8_t _data_pin;
        bool is_ready(void);
        void power_down(void);
        void power_up(void);
    public:
        void init(uint8_t clk_port, uint8_t clk_pin
            , uint8_t data_port, uint8_t data_pin);
        void task(void);
        int16_t read(void);
        int32_t read_i32(void);
};

Loadcell lc[6];

// UART
static RINGBUFF_T txring, rxring;
#define UART_SRB_SIZE 256	/* Send */
#define UART_RRB_SIZE 256	/* Receive */
static uint8_t rxbuff[UART_RRB_SIZE];
static uint8_t txbuff[UART_SRB_SIZE];

static void vLoadcellTask(void *pvParameters) {
    (void) pvParameters;
    Loadcell *lc = static_cast<Loadcell*>(pvParameters);
    lc->task();
}

/* LED0 toggle thread */
static void vLEDTask(void *pvParameters) {
    (void) pvParameters;

	while (1) {
        auto val = lc[0].read();
        Chip_UART_SendRB(LPC_USART, &txring, "0x", 2);

        char temp;
        for (int i = 12; i >= 0; i -= 4) {
            uint8_t nibble = (val >> i) & 0x0F;
            if (nibble < 10)
                temp = '0' + nibble;
            else
                temp = 'A' + (nibble - 10);
            Chip_UART_SendRB(LPC_USART, &txring, &temp, 1);
        }

        Chip_UART_SendRB(LPC_USART, &txring, "\r\n", 2);

        // LED点灯
        Chip_GPIO_SetPinState(LPC_GPIO, LED_PORT, LED_PIN, false);
        vTaskDelay(10/portTICK_PERIOD_MS);
        // LED消灯
        Chip_GPIO_SetPinState(LPC_GPIO, LED_PORT, LED_PIN, true);
        vTaskDelay(990/portTICK_PERIOD_MS);
	}
}

int main(void)
{
    // System Init
    SystemCoreClockUpdate();
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_IOCON);
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_GPIO);
    Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_UART0);

    // GPIO
    Chip_GPIO_Init(LPC_GPIO);
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, LED_PORT, LED_PIN);

    // UART
    Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_6, (IOCON_FUNC1 | IOCON_MODE_INACT));/* RXD */
	Chip_IOCON_PinMuxSet(LPC_IOCON, IOCON_PIO1_7, (IOCON_FUNC1 | IOCON_MODE_INACT));/* TXD */
    Chip_UART_Init(LPC_USART);
    Chip_UART_SetBaud(LPC_USART, 38400);
    Chip_UART_ConfigData(LPC_USART, (UART_LCR_WLEN8 | UART_LCR_SBS_1BIT));
    Chip_UART_SetupFIFOS(LPC_USART, (UART_FCR_FIFO_EN | UART_FCR_TRG_LEV2));
    Chip_UART_TXEnable(LPC_USART);
    RingBuffer_Init(&rxring, rxbuff, 1, UART_RRB_SIZE);
    RingBuffer_Init(&txring, txbuff, 1, UART_SRB_SIZE);
    Chip_UART_IntEnable(LPC_USART, (UART_IER_RBRINT | UART_IER_RLSINT));
    NVIC_SetPriority(UART0_IRQn, 1);
    NVIC_EnableIRQ(UART0_IRQn);


    static StackType_t puxST_Led[ configMINIMAL_STACK_SIZE ];
    static StaticTask_t pxST_Led;
    
    xTaskCreateStatic(vLEDTask, "vTaskLed",
            configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
            &puxST_Led[0], &pxST_Led);

    lc[0].init(0, 9, 0, 8);

    static StackType_t puxST_Loadcell[ configMINIMAL_STACK_SIZE+32 ];
    static StaticTask_t pxST_Loadcell;
    xTaskCreateStatic(vLoadcellTask, "vTaskLoadcell",
            configMINIMAL_STACK_SIZE+32, &lc[0], (tskIDLE_PRIORITY + 1UL),
            &puxST_Loadcell[0], &pxST_Loadcell);
    
    vTaskStartScheduler();

    NVIC_DisableIRQ(UART0_IRQn);
	Chip_UART_DeInit(LPC_USART);

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

extern "C" void UART_IRQHandler(void)
{
	Chip_UART_IRQRBHandler(LPC_USART, &rxring, &txring);
}

void vApplicationMallocFailedHook(void)
{
	taskDISABLE_INTERRUPTS();
	for (;; ) {}
}

/* FreeRTOS stack overflow hook */
void vApplicationStackOverflowHook( TaskHandle_t xTask, char * pcTaskName )
{
	(void) xTask;
	(void) pcTaskName;

	/* Run time stack overflow checking is performed if
	   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	   function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for (;; ) {}
}

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer,
                                    StackType_t **ppxIdleTaskStackBuffer,
                                    uint32_t *pulIdleTaskStackSize )
{
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void delayMicroseconds(uint16_t us) {
    while (us!=0) {
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        __NOP();__NOP();__NOP();__NOP();__NOP();__NOP();
        us -= 1; 
    }
}

bool Loadcell::is_ready(void) {
    return Chip_GPIO_GetPinState(LPC_GPIO, this->_data_port, this->_data_pin)? false : true;
}
void Loadcell::power_down(void) {
    Chip_GPIO_SetPinState(LPC_GPIO, this->_clk_port, this->_clk_pin, true);
    delayMicroseconds(60);
}
void Loadcell::power_up(void) {
    Chip_GPIO_SetPinState(LPC_GPIO, this->_clk_port, this->_clk_pin, false);
    delayMicroseconds(1);
}
void Loadcell::init(uint8_t clk_port, uint8_t clk_pin
            , uint8_t data_port, uint8_t data_pin) {
    this->_clk_port = clk_port;
    this->_clk_pin = clk_pin;
    this->_data_port = data_port;
    this->_data_pin = data_pin;
    this->_raw = 0;
}

void Loadcell::task(void) {
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, this->_clk_port, this->_clk_pin);
    Chip_GPIO_SetPinDIRInput(LPC_GPIO, this->_data_port, this->_data_pin);
    // Data Port is OpenDrain

    this->power_down();
    this->power_up();

    vTaskDelay((80/portTICK_PERIOD_MS)*2);

    while (true){
        while (this->is_ready() == false) {
            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        
        volatile int32_t val = 0;
        for (uint8_t i = 0; i < 24; i++) {
            portDISABLE_INTERRUPTS();
            Chip_GPIO_SetPinState(LPC_GPIO, this->_clk_port, this->_clk_pin, true);
            delayMicroseconds(1);
            val = (val << 1) | (Chip_GPIO_GetPinState(LPC_GPIO, this->_data_port, this->_data_pin) ? 1 : 0);
            Chip_GPIO_SetPinState(LPC_GPIO, this->_clk_port, this->_clk_pin, false);
            delayMicroseconds(1);
            portENABLE_INTERRUPTS();
        }
        // 25th pulse
        portDISABLE_INTERRUPTS();
        Chip_GPIO_SetPinState(LPC_GPIO, this->_clk_port, this->_clk_pin, true);
        delayMicroseconds(1);
        Chip_GPIO_SetPinState(LPC_GPIO, this->_clk_port, this->_clk_pin, false);
        delayMicroseconds(1);
        portENABLE_INTERRUPTS();

        // 2の補数変換
        if (val & 0x00800000) {
            val |= 0xff000000;
        }
        portENTER_CRITICAL();
        this->_raw = val;
        portEXIT_CRITICAL();

        vTaskDelay(70/portTICK_PERIOD_MS);
    }
}

int16_t Loadcell::read(void) {
    int16_t ret = 0;
    portENTER_CRITICAL();
    ret = static_cast<int16_t>(this->_raw >> 8);
    portEXIT_CRITICAL();
    return ret;
}

int32_t Loadcell::read_i32(void) {
    int16_t ret = 0;
    portENTER_CRITICAL();
    ret = this->_raw;
    portEXIT_CRITICAL();
    return ret;
}