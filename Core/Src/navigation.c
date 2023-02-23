#include "navigation.h"
#include "dijkstra.h"
#include "landmarks.h"
#include "stdio.h"
#include "string.h"
#include "helper_functions.h"
volatile lwgps_t gps;
volatile coord_t coord;
uint8_t nav_rx_data[UART2_RX_DMA_BUFFER_SIZE] = {0};
uint8_t visited_nodes[LEN_LANDMARKS] = {0};
float_t path_distances[MAX_PATH_LENGTH] = {
		MAX_DISTANCE,MAX_DISTANCE,MAX_DISTANCE,MAX_DISTANCE,
		MAX_DISTANCE,MAX_DISTANCE,MAX_DISTANCE,MAX_DISTANCE,
		MAX_DISTANCE,MAX_DISTANCE,MAX_DISTANCE,MAX_DISTANCE,
		MAX_DISTANCE,MAX_DISTANCE,MAX_DISTANCE,MAX_DISTANCE,
		MAX_DISTANCE,MAX_DISTANCE,MAX_DISTANCE,MAX_DISTANCE
};

int navigation_main_init(uint8_t destination) {
	if (gps.fix == 0) {
		//printf("GPS Fix is 0!\n\r");
		return 0;
	} else {
		printf("GPS Fix: %d. Number of satellites initially: %d\n\r", gps.fix, gps.sats_in_use);
	}
    float_t closest_distance;
    coord_t last_coord;
    last_coord.x = coord.x;
    last_coord.y = coord.y;
    printf("Latitude: ");
	print_float(last_coord.y);
	printf(" ");

	printf("Longitude: ");
	print_float(last_coord.x);
	printf("\n\r");
    uint8_t nearest_node = get_nearest_node(&last_coord, &closest_distance);
    printf("Distance to nearest node: ");
    print_float(closest_distance);
    printf("\n\r");
	uint8_t source = nearest_node;
	printf("Source: %d, %s\n\r", source, landmarks[source].name);
	printf("Destination: %d, %s\n\r", destination, landmarks[destination].name);
	dijkstra(source, destination);
	return gps.sats_in_use;
}

void navigation_main_loop() {
	float_t closest_distance;
	coord_t last_coord;
	last_coord.x = coord.x;
	last_coord.y = coord.y;
	uint8_t nearest_node = get_nearest_node(&last_coord, &closest_distance);
	// (50 ft + 5 meters)^2 = 0.0001581697 mi^2
	if (closest_distance < 0.0001581697 + landmarks[nearest_node].buffer_distance) {
		if (visited_nodes[nearest_node] == 0) {
			visited_nodes[nearest_node] = 1;
			// Call Brain function here
			printf("You are now near %s\n\r", landmarks[nearest_node].name);
		}
	}
	for (int i = 0; i < MAX_PATH_LENGTH; i++) {
		if (path[i] == -1) {
			break;
		}
		path_distances[i] = squared_distance_to_node(&last_coord, path[i]);
	}
}

void process_buffer() {
	lwgps_process(&gps, nav_rx_data, UART2_RX_DMA_BUFFER_SIZE);
	//printf("Fix: %d\n\r", gps.fix);
	if (gps.fix == 0) {
		return;
	}
	coord.x = gps.longitude;
	coord.y = gps.latitude;

	return;
	printf("Latitude: ");
	print_float(gps.latitude);
	printf(" ");

	printf("Longitude: ");
	print_float(gps.longitude);
	printf("\n\r");
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
	//print_message();
	process_buffer();
}

