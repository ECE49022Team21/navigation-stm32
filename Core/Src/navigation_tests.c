#include "navigation_tests.h"
#include "custom_typedef.h"
#include "lwgps/lwgps.h"
#include "proximity.h"
#include "landmarks.h"
#include "stdio.h"
#include "string.h"
#include "main.h"
#include "dijkstra.h"
#include "helper_functions.h"
#include "navigation.h"
void parse_test() {
	//<NMEA(GPGGA, time=20:31:15, lat=40.4481621667, NS=N, lon=-86.9424258333, EW=W, quality=2, numSV=11, HDOP=0.9, alt=224.7, altUnit=M, sep=-33.5, sepUnit=M, diffAge=, diffStation=0)>
	//b'$GPGGA,203115.00,4026.88973,N,08656.54555,W,2,11,0.90,224.7,M,-33.5,M,,0000*6F\r\n'
	char gps_rx_test_data[] =
			"$GPGGA,203115.00,4026.88973,N,08656.54555,W,2,11,0.90,224.7,M,-33.5,M,,0000*6F\r\n";
	lwgps_process(&gps, gps_rx_test_data, strlen(gps_rx_test_data));

	printf("Latitude: ");
	print_float(gps.latitude);
	printf(" ");

	printf("Longitude: ");
	print_float(gps.longitude);
	printf("\n\r");
}

void _test_single_run() {
    float_t closest_distance;

    uint8_t nearest_node = get_nearest_node(&coord, &closest_distance);
    printf("Distance to nearest node: ");
    print_float(closest_distance);
    printf("\n\r");
    int source = nearest_node;
    int destination = 1;
    printf("Source: %d, %s\n\r", source, landmarks[source].name);
    printf("Destination: %d, %s\n\r", destination, landmarks[destination].name);
    dijkstra(source, destination);
}

void nav_test_1() {
	// on oval near university: 40.4251986 -86.9149741: 22.69 m
	coord.x = -86.9149741;
	coord.y = 40.4251986;

	_test_single_run(coord);
}

void nav_test_2() {
	// between CL50 and SC: 40.4264039 -86.9147345: 27.83 m
	coord.x = -86.9147345;
	coord.y = 40.4264039;

	_test_single_run(coord);
}


void test_runs() {
	parse_test();
	nav_test_1();
	nav_test_2();
}
