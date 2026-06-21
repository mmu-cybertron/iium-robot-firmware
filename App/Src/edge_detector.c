#include "edge_detector.h"
#include "main.h"
#include "line_sensor.h"
#include "usart1_log.h"

static edge_status_t current_edges;

void edge_detector_init(void)
{
    line_sensor_init();
    current_edges = line_sensor_read_edges();
    LOG_PRINT("Edge detector initialized\r\n");
    
}

void edge_detector_update(void)
{
    current_edges = line_sensor_read_edges();
}

edge_status_t edge_detector_get_status(void)
{
    return current_edges;
}

uint8_t edge_detector_is_edge_detected(void)
{
//    return (uint8_t)(current_edges.front_left ||
//                     current_edges.front_right ||
//                     current_edges.rear_left ||
//                     current_edges.rear_right);

    return (uint8_t)(current_edges.front_left ||
    				 current_edges.front_right);
}
