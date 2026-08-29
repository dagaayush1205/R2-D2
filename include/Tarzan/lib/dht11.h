#ifndef DHT11_SENSOR_H
#define DHT11_SENSOR_H 
#include <zephyr/kernel.h> 
#include <zephyr/drivers/gpio.h>

int read_sensor_values(struct gpio_dt_spec device , int arr[5]);

#endif
