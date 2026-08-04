#ifndef __HYDROX_BOARDS_FMU_V6C_SRC_FMU_V6C_H
#define __HYDROX_BOARDS_FMU_V6C_SRC_FMU_V6C_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void fmu_v6c_inhibit_outputs_early(void);
bool fmu_v6c_outputs_are_inhibited(void);
void fmu_v6c_capture_reset_reason(void);
uint32_t fmu_v6c_reset_reason(void);
int fmu_v6c_bringup(void);

#ifdef __cplusplus
}
#endif
#endif