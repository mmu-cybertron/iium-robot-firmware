#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <stdint.h>

void battery_monitor_init(void);
uint16_t battery_monitor_read_mv(void);

#endif /* BATTERY_MONITOR_H */
