#include "led.h"

#include "main.h"

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} led_gpio_t;

static const led_gpio_t leds[LED_COUNT] = {
    [LED_D6] = {LED_D6_GPIO_Port, LED_D6_Pin},
    [LED_D7] = {LED_D7_GPIO_Port, LED_D7_Pin},
    [LED_D8] = {LED_D8_GPIO_Port, LED_D8_Pin},
};

static uint8_t led_is_valid(led_id_t led)
{
    return (led >= LED_D6) && (led < LED_COUNT);
}

void led_init(void)
{
    led_all_off();
}

void led_on(led_id_t led)
{
    led_write(led, 1u);
}

void led_off(led_id_t led)
{
    led_write(led, 0u);
}

void led_toggle(led_id_t led)
{
    if (!led_is_valid(led)) {
        return;
    }

    HAL_GPIO_TogglePin(leds[led].port, leds[led].pin);
}

void led_write(led_id_t led, uint8_t on)
{
    if (!led_is_valid(led)) {
        return;
    }

    HAL_GPIO_WritePin(leds[led].port,
                      leds[led].pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void led_all_on(void)
{
    led_write_mask(LED_MASK_ALL);
}

void led_all_off(void)
{
    led_write_mask(0u);
}

void led_write_mask(uint8_t mask)
{
    for (led_id_t led = LED_D6; led < LED_COUNT; led++) {
        led_write(led, (mask & (1u << led)) != 0u);
    }
}

void led_show_one(led_id_t led)
{
    if (!led_is_valid(led)) {
        led_all_off();
        return;
    }

    led_write_mask((uint8_t)(1u << led));
}
