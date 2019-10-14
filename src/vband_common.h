#ifndef VBAND_COMMON_H__
#define VBAND_COMMON_H__

#include <stdint.h>
#include "nrf.h"


#ifdef __cplusplus
extern "C" {
#endif


// clear FPU pending interrupt
void clear_FPU_interrupts(void);


#ifdef __cplusplus
}
#endif

#endif /* VBAND_COMMON_H__ */
