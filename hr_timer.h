#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void tim_setup(uint32_t);

void delay_usec(uint32_t, uint16_t, void next_step (void));

void prepares_capture(uint32_t);

#ifdef __cplusplus
}
#endif
