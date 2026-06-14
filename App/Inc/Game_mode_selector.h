#ifndef GAME_MODE_SELECTOR_H
#define GAME_MODE_SELECTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Mode definitions */
typedef enum {
    GAME_MODE_1 = 1,  // Turn left 1s + move forward 1s
    GAME_MODE_2 = 2,  // Turn right 1s + move forward 1s
    GAME_MODE_3 = 3   // Move forward 1s
} game_mode_t;

/* State machine states */
typedef enum {
    MODE_SEL_IDLE = 0,
    MODE_SEL_SELECTING = 1,
    MODE_SEL_CONFIRMING = 2,
    MODE_SEL_LOCKED = 3,
    MODE_SEL_UNLOCK_WAITING = 4,
    MODE_SEL_UNLOCKED = 5
} mode_selector_state_t;

/* Function prototypes */
void game_mode_selector_init(void);
void game_mode_selector_update(void);
uint8_t game_mode_selector_is_locked(void);
game_mode_t game_mode_selector_get_mode(void);
void game_mode_selector_execute_initial_move(void);
void game_mode_selector_reset_for_new_round(void);

#ifdef __cplusplus
}
#endif

#endif /* GAME_MODE_SELECTOR_H */