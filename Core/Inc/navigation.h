#ifndef NAVIGATION_HEADER_G
#define NAVIGATION_HEADER_G
#define UART2_RX_DMA_BUFFER_SIZE 10
#include "custom_typedef.h"
#include "lwgps/lwgps.h"
#include "proximity.h"
#include "landmarks.h"
#include "dijkstra.h"

#include "main.h"

extern volatile lwgps_t gps;
extern volatile coord_t coord;
extern uint8_t nav_rx_data[UART2_RX_DMA_BUFFER_SIZE];
extern uint8_t visited_nodes[LEN_LANDMARKS];
extern float_t path_distances[MAX_PATH_LENGTH];


void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart);
void process_buffer();
int navigation_main_init(uint8_t destination);
void navigation_main_loop();
#endif
