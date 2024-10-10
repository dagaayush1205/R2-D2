#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <kyvernitis/lib/kyvernitis.h>

#include <Tarzan/lib/sbus.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

static const struct device *const uart_dev = DEVICE_DT_GET(DT_ALIAS(mother_uart)); // data from SBUS
static const struct device *const uart_debug = DEVICE_DT_GET(DT_ALIAS(debug_uart)); //debugger

// creating mssgq 
K_MSGQ_DEFINE(uart_msgq, 25*sizeof(uint8_t), 10, 1); 
uint16_t *ch;

void serial_cb(const struct device *dev, void *user_data) {
	
	ARG_UNUSED(user_data);
	uint8_t c,
		packet [25],
		start = 0x0F;
	
	if (!uart_irq_update(uart_dev)) 
		return;
	if (!uart_irq_rx_ready(uart_dev)) 
		return;

	while (uart_fifo_read(uart_dev, &c, 1) == 1) {
		if (c != start) continue; 
		packet[0] = c;
		if(uart_fifo_read(uart_dev, packet+1, 24) == 24) {
			k_msgq_put(&uart_msgq, packet, K_MSEC(10));
		}
	}	
}

int main() {
	int err;
	uint8_t packet[25]; // to store sbus packet
	// device ready chceks
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART device not ready");
	}

	// Interrupt to get sbus data
	err = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
	
	if(err<0) {
		if (err == -ENOTSUP) 
			LOG_ERR("Interrupt-driven UART API support not enabled");
		 
		else if (err == -ENOSYS) 
			LOG_ERR("UART device does not support interrupt-driven API");
		
		else 
			LOG_ERR("Error setting UART callback: %d", err);
	}

	// enable uart device for communication
	uart_irq_rx_enable(uart_dev);
	
	LOG_INF("Initialization completed successfully!");
	
	while(true) {
		k_msgq_get(&uart_msgq, &packet, K_MSEC(20));
		ch = parse_buffer(packet);
		for(int i=0; i<16;i++) 
			printk("%d\t", ch[0]);
		printk("\n");
	}
}