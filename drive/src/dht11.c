#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

int read_sensor_values(struct gpio_dt_spec dev , int data[5]){
  int j;
  uint32_t start=0;
  for(int i=0;i<5;i++){
    data[i]=0;
  };
  gpio_pin_configure_dt(&dev,GPIO_OUTPUT);
  gpio_pin_set_dt(&dev,GPIO_OUTPUT_LOW);
  k_msleep(20);
  gpio_pin_set_dt(&dev,GPIO_OUTPUT_HIGH);
  k_busy_wait(30);
  gpio_pin_configure_dt(&dev,GPIO_INPUT | GPIO_PULL_UP);

  start=k_cycle_get_32();
  while(gpio_pin_get_dt(&dev)){
    if (k_cyc_to_us_floor32(k_cycle_get_32() - start) > 100) return -1;
  };
  while(!gpio_pin_get_dt(&dev));
  while (gpio_pin_get_dt(&dev));
  for(int i=0;i<40;i++){
    while(!gpio_pin_get_dt(&dev));
    start=k_cycle_get_32();
    while(gpio_pin_get_dt(&dev));
    uint32_t pulse = k_cyc_to_us_floor32(k_cycle_get_32()-start);
    j=i/8;
    data[j]<<=1;
    if(pulse>50){
      data[j] |=1;
    };
  };
  if (((data[0] + data[1] + data[2] + data[3]) & 0xFF) != data[4]) {
        return -2;
  };
  return 0;

};
