#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#define ROBOT_UPDATE_PERIOD_MS 5U

#define ROBOT_START_DELAY_MS 5000U

#define MOTOR_PWM_MAX 2250
#define MOTOR_PWM_NEUTRAL 1500
#define MOTOR_PWM_MIN 900

/* Set to 0U to run sensors/state logic without driving the motors. */
#define ROBOT_MOTOR_ENABLE 1U

/* Set to 0U to disable analog edge sensor polling and edge escape logic. */
#define ROBOT_EDGE_SENSOR_ENABLE 0U

#define ROBOT_ATTACK_PWM 2000
#define ROBOT_TRACK_PWM 450
#define ROBOT_SEARCH_PWM 350
#define ROBOT_EDGE_ESCAPE_PWM 600

#define LINE_SENSOR_EDGE_THRESHOLD 2500U
#define OPPONENT_DETECT_DISTANCE_MM 800U

#define ROBOT_MODE_LOGGING_ENABLE 1
#define ROBOT_MODE_RUN        2

#define ROBOT_ACTIVE_MODE ROBOT_MODE_LOGGING_ENABLE

/* Set to 0U to skip game-mode selection and initial-move execution. */
#define ROBOT_GAME_MODE_SELECTOR_ENABLE 0U

#endif /* ROBOT_CONFIG_H */
