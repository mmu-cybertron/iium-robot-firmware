#include "line_sensor.h"
#include "main.h"

#define LINE_DETECTED_STATE GPIO_PIN_RESET

void line_sensor_init(void)
{
}

edge_status_t line_sensor_read_edges(void)
{
    edge_status_t status;

    status.front_left =
        HAL_GPIO_ReadPin(IR1_DO_GPIO_Port, IR1_DO_Pin) == LINE_DETECTED_STATE;

    status.front_right =
        HAL_GPIO_ReadPin(IR2_DO_GPIO_Port, IR2_DO_Pin) == LINE_DETECTED_STATE;

    status.rear_left =
        HAL_GPIO_ReadPin(IR3_DO_GPIO_Port, IR3_DO_Pin) == LINE_DETECTED_STATE;

    status.rear_right =
        HAL_GPIO_ReadPin(IR4_DO_GPIO_Port, IR4_DO_Pin) == LINE_DETECTED_STATE;

    return status;
}