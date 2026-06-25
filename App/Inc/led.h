#ifndef LED_H
#define LED_H

#include <stdint.h>

typedef enum {
    LED_D6 = 0,
    LED_D7,
    LED_D8,
    LED_COUNT
} led_id_t;

#define LED_MASK_D6 (1u << LED_D6)
#define LED_MASK_D7 (1u << LED_D7)
#define LED_MASK_D8 (1u << LED_D8)
#define LED_MASK_ALL (LED_MASK_D6 | LED_MASK_D7 | LED_MASK_D8)

void led_init(void);
void led_on(led_id_t led);
void led_off(led_id_t led);
void led_toggle(led_id_t led);
void led_write(led_id_t led, uint8_t on);
void led_all_on(void);
void led_all_off(void);
void led_write_mask(uint8_t mask);
void led_show_one(led_id_t led);

#endif /* LED_H */
