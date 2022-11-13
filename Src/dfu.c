// technique is based on this code:
// https://github.com/microready/STM32F4_DFU_Demo

#include <stdint.h>
#include "stm32f4xx_hal.h"

// This is just a random number. It really can be anything (except 0) but it should be unlikely
// to occur in RAM after startup because we will enter DFU if a software reset is triggered
// before dfu_init() and the RAM value matches by accident.
const uint32_t enter_dfu_magic_value = 0xa4d8c938;
uint32_t enter_dfu_flag;
uint32_t rcc_value;

/// call this at the start of main()
/// (before any code that might trigger a software reset)
void dfu_init() {
    // save reset reason (in case we want to use it later)
    rcc_value = RCC->CSR;

    // overwrite our DFU flag
    // This will make sure that we don't enter DFU if some other code triggers a software reset.
    enter_dfu_flag = 0;

    // clear reset reason
    // (Just to be extra sure that we won't trigger DFU if there is an event that corrupts RAM, matches our magic value by accident and triggers some other reset.)
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

/// call this to reboot into DFU, will not return; probably don't call it from interrupt context
void dfu_enter() {
    // Set flag and trigger a software reset. Reset_Handler will see the magic value and jump to DFU.
    enter_dfu_flag = enter_dfu_magic_value;
    HAL_NVIC_SystemReset();
}
