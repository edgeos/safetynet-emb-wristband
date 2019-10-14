#include "nrf.h"
#include "nrf_nvic.h"
#include "vband_common.h"


// clear FPU pending interrupt

void clear_FPU_interrupts(void)
{
    __set_FPSCR(__get_FPSCR() & ~(0x0000009F)); 
    (void) __get_FPSCR();
    //NVIC_ClearPendingIRQ(FPU_IRQn);
    sd_nvic_ClearPendingIRQ(FPU_IRQn);
}