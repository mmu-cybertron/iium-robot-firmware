#ifndef FAILSAFE_H
#define FAILSAFE_H

#include <stdint.h>

void failsafe_init(void);
void failsafe_update(void);
void failsafe_background(void);
uint8_t failsafe_is_faulted(void);

#endif /* FAILSAFE_H */
