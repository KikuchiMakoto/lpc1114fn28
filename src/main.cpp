#include "chip.h"

const uint32_t OscRateIn = 8000000; // 外部オシレータの周波数（12MHz）
const uint32_t ExtRateIn = 0;        // 外部クロックの周波数

// LED接続ピン（例：PIO0_1）
#define LED_PORT 0
#define LED_PIN 1

extern "C" void SystemInit(void) {
    Chip_SystemInit();
}

void delay(uint32_t count) {
    while (count-- > 0) {
        for (int i = 0; i < 1000; i++) {
            __asm("nop");__asm("nop");__asm("nop");__asm("nop");__asm("nop");
            __asm("nop");__asm("nop");__asm("nop");__asm("nop");__asm("nop");
            __asm("nop");__asm("nop");__asm("nop");__asm("nop");__asm("nop");
            __asm("nop");__asm("nop");__asm("nop");__asm("nop");__asm("nop");
            __asm("nop");__asm("nop");__asm("nop");__asm("nop");__asm("nop");
            __asm("nop");__asm("nop");__asm("nop");__asm("nop");__asm("nop");
            __asm("nop");__asm("nop");__asm("nop");__asm("nop");__asm("nop");
        }
    }
}

int main(void) {
    // システムクロック初期化
    SystemCoreClockUpdate();
    
    // GPIOクロック有効化
    Chip_GPIO_Init(LPC_GPIO);
    
    // LEDピンを出力に設定
    Chip_GPIO_SetPinDIROutput(LPC_GPIO, LED_PORT, LED_PIN);
    
    // メインループ
    while (1) {
        // LED点灯
        Chip_GPIO_SetPinState(LPC_GPIO, LED_PORT, LED_PIN, true);
        delay(500);
        
        // LED消灯
        Chip_GPIO_SetPinState(LPC_GPIO, LED_PORT, LED_PIN, false);
        delay(500);
    }
    
    return 0;
}
