#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gnss.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/stepper.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>
#include <zephyr/toolchain.h>
#include <zephyr/usb/usb_device.h>

#include <Tarzan/lib/arm.h>
#include <Tarzan/lib/cobs.h>
#include <Tarzan/lib/drive.h>
#include <Tarzan/lib/sbus.h>

LOG_MODULE_REGISTER(Tarzan, CONFIG_TARZAN_LOG_LEVEL);

#define STACK_SIZE 4096   // work_q thread stack size
#define PRIORITY 2        // work_q thread priority
#define STEPPER_TIMER 100 // stepper pulse width in microseconds
#define JERK_LIMITER true

/* sbus uart */
static const struct device *const sbus_uart =
    DEVICE_DT_GET(DT_ALIAS(sbus_uart));
/* sbc uart */
static const struct device *const sbc_uart = DEVICE_DT_GET(DT_ALIAS(sbc_uart));
/* gps uart */
#define GNSS_MODEM DEVICE_DT_GET(DT_ALIAS(gps_uart))

/* DT spec for pwm motors */
#define PWM_MOTOR_SETUP(pwm_dev_id)                                            \
  {.dev_spec = PWM_DT_SPEC_GET(pwm_dev_id),                                    \
   .min_pulse = DT_PROP(pwm_dev_id, min_pulse),                                \
   .max_pulse = DT_PROP(pwm_dev_id, max_pulse),                                \
   .channel = DT_PROP_OR(pwm_dev_id, channel, -1)},
const struct pwm_motor pwm_motor[DT_CHILD_NUM_STATUS_OKAY(DT_PATH(pwmmotors))] =
    {DT_FOREACH_CHILD(DT_PATH(pwmmotors), PWM_MOTOR_SETUP)};

/* DT spec for stepper motors */
#define STEPPER_MOTOR_SETUP(stepper_dev_id)                                    \
  {.dir = GPIO_DT_SPEC_GET(stepper_dev_id, dir_gpios),                         \
   .step = GPIO_DT_SPEC_GET(stepper_dev_id, step_gpios),                       \
   .channel = DT_PROP_OR(stepper_dev_id, channel, -1)},
const struct stepper_motor
    stepper_motor[DT_CHILD_NUM_STATUS_OKAY(DT_PATH(steppermotors))] = {
        DT_FOREACH_CHILD(DT_PATH(steppermotors), STEPPER_MOTOR_SETUP)};

/* DT spec for leds */
static const struct gpio_dt_spec init_led =
    GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec sbus_status_led =
    GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
const struct pwm_dt_spec error_led = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));
/* Enum struct to define the diffrent autonomous modes */ 
enum auto_state {
  arm_mode=0,
  drive_mode=1
};
/* msg struct for rx coms */
struct auto_msg {
  uint8_t state;
  struct DiffDriveTwist auto_cmd;
  struct joint arm_cmd[6]; 
  uint32_t crc;
};
/* struct to store gps data */ 
struct gps_data {
  int64_t latitude;
  int64_t longitude;
  int32_t altitude;
  int32_t bearing;

};
struct gps_send_data {
  struct k_work gps_work_item; 
}gps;
/* msg struct for tx coms */
struct base_station_msg {
  struct gps_data data;
  struct joint angles[6];
  uint32_t crc;
};

/* defining sbus message queue*/
K_MSGQ_DEFINE(sbus_msgq, 25 * sizeof(uint8_t), 10, 1);
/* defining cobs message queue */
K_MSGQ_DEFINE(drive_msgq, sizeof(struct auto_msg) + 2, 50, 1);
/* defining cobs message queue for inverse kinematics */ 
K_MSGQ_DEFINE(arm_msgq,sizeof(struct auto_msg)+2,50,1);
/* defining the message queue for tx item arm */ 
K_MSGQ_DEFINE(arm_tx_msgq,sizeof(struct joint),50,1);
/* defining the message queue for tx item gps */ 
K_MSGQ_DEFINE(gps_tx_msgq,sizeof(struct gps_data),50,1);
/* workq dedicated thread */
K_THREAD_STACK_DEFINE(stack_area, STACK_SIZE);

/* semaphore for channels */
struct k_sem ch_sem;
/* mutex for channel reader count */
struct k_mutex ch_reader_cnt_mutex;
/* mutex for channel readers to wait for writer */
struct k_mutex ch_writer_mutex;
/* msgq to store sbus data */
struct k_work_q work_q;
/* sbus work item */
struct k_work sbus_work_item;
/*gps work item */ 

/* struct for drive variables */
struct drive_arg {
  struct k_work drive_work_item;      // drive work item
  struct k_work auto_drive_work_item; // autonomous drive work item
  struct DiffDriveConfig drive_config;
  struct DiffDriveTwist cmd;
  struct DiffDriveCtx *drive_init;
  struct k_work auto_arm_work_item;
  uint8_t arm_work_buffer[sizeof(struct auto_msg)+2];
  uint8_t drive_raw_buffer[sizeof(struct auto_msg) + 2];
} drive;


/* struct for arm variables */

struct arm_arg {
  enum StepperDirection dir[5];
  int pos[5];
  struct k_work channel_work_item;
  struct k_work auto_arm_work_item;
  struct k_work arm_tx_work_item;
  uint8_t arm_work_buffer[sizeof(struct auto_msg)+2];
} arm;
/*struct for communication */ 
struct com_rx_arg {
  struct k_work cobs_rx_work_item;
  struct k_work *work_item;
  void *msg_rx;
  struct k_msgq *msgq_rx;
  int cobs_bytes_read;
  size_t MSG_LEN;
  uint8_t *rx_buf;
};

struct com_rx_arg drive_com; //auto drive struct 
struct com_rx_arg arm_com; //auto arm struct

struct com_tx_arg {
  struct k_work sbc_tx_work_item;
  struct base_station_msg bs_msg_tx; // to store encoded base station mssg
} com_tx;

int ch_reader_cnt;          // no. of readers accessing channels
uint16_t channel[16] = {0}; // to store sbus channels
uint8_t packet[25];         // to store sbus packets
int sbus_bytes_read;        // to store number of sbus bytes read
const int BS_MSG_LEN =
    sizeof(struct base_station_msg) + 2; // len of base station mssg
uint8_t bs_tx_buf[sizeof(struct base_station_msg) + 2] = {0};
/* range variables */
const float linear_velocity_range[] = {-1.5, 1.5};
const float angular_velocity_range[] = {-5.5, 5.5};
const float wheel_velocity_range[] = {-10.0, 10.0};
const uint16_t channel_range[] = {172, 1811};

/* check if received cobs message is valid,
* ret 0 if successfull */
uint32_t check_crc(uint8_t *msg, size_t MSG_LEN) {
  uint32_t valid_crc;
  valid_crc = crc32_ieee((uint8_t *)msg, MSG_LEN - sizeof(uint32_t));
  return valid_crc;
}

/* interrupt to store sbus data */
void sbus_cb(const struct device *dev, void *user_data) {
  ARG_UNUSED(user_data);
  uint8_t c;
  if (!uart_irq_update(sbus_uart))
    return;
  if (!uart_irq_rx_ready(sbus_uart))
    return;
  while (sbus_bytes_read < 25 && uart_fifo_read(sbus_uart, &c, 1)) {
    if (sbus_bytes_read == 0 && c != 0x0f)
      continue;
    packet[sbus_bytes_read++] = c;
  }
  if (sbus_bytes_read == 25) {
    k_msgq_put(&sbus_msgq, &packet, K_NO_WAIT);
    k_work_submit_to_queue(&work_q, &sbus_work_item);
    sbus_bytes_read = 0;
  }
}
/* interrupt to store gps data */
void gps_cb(const struct device *dev, const struct gnss_data *data) {
  struct gps_data dummy_gps; 
  if (data->info.fix_status != GNSS_FIX_STATUS_NO_FIX) {
    /*
    com_tx.bs_msg_tx.data.latitude = data->nav_data.latitude;
    com_tx.bs_msg_tx.data.longitude = data->nav_data.longitude;
    com_tx.bs_msg_tx.data.altitude = data->nav_data.altitude;
    com_tx.bs_msg_tx.data.bearing = data->nav_data.bearing;
    */ 
    k_msgq_put(&gps_tx_msgq,&dummy_gps,K_NO_WAIT);

    dummy_gps.latitude=data->nav_data.latitude;
    dummy_gps.longitude=data->nav_data.longitude;
    dummy_gps.altitude=data->nav_data.altitude;
    dummy_gps.bearing=data->nav_data.bearing;
    k_work_submit_to_queue(&work_q, &gps.gps_work_item);
  } else
    LOG_ERR("GPS: Unable to fix satellite");
}
/* interrupt to send the current joint angles of the arm */ 
/*
struct joint1 {
  float accel[3];
  float gyro[3];
  float mag[3];
  float pitch;
  float roll;
  float yaw;
};
*/ 
void angles_cb(const struct device *dev , struct joint angles){
  // if (!uart_irq_update(sbus_uart))
  //   return;
  // if (!uart_irq_rx_ready(sbus_uart))
  //   return;
  if(k_msgq_put(&arm_tx_msgq,&angles,K_NO_WAIT)!=0){
    return;
  }
  k_work_submit_to_queue(&work_q,&arm.arm_tx_work_item);
};
/* gps work handler */ 
void gps_work_handler(struct k_work *gps_work_item){
  struct gps_data buffer={0};
  if(k_msgq_get(&gps_tx_msgq,&buffer,K_NO_WAIT)!=0){
    return ;
  }
  com_tx.bs_msg_tx.data.latitude=buffer.latitude;
  com_tx.bs_msg_tx.data.longitude=buffer.longitude;
  com_tx.bs_msg_tx.data.altitude=buffer.altitude;
  com_tx.bs_msg_tx.data.bearing=buffer.bearing;
  k_work_submit_to_queue(&work_q,&(com_tx.sbc_tx_work_item));

};
/* work handler for the arms tx message */ 
void arm_tx_work_handler(struct k_work *arm_tx_work_item){
  struct joint latest_angles;

  for(int i=0;i<6;i++){
    if(k_msgq_get(&arm_tx_msgq,&latest_angles,K_NO_WAIT)!=0){
      return ;
    }
    com_tx.bs_msg_tx.angles[i]=latest_angles;
  };
  k_work_submit_to_queue(&work_q,&(com_tx.sbc_tx_work_item));

};
/* interrup to read cobs messages */
void cobs_cb(const struct device *dev, void *user_data) {
  struct com_rx_arg *com_ctx = (struct com_rx_arg *)user_data;
  uint8_t c;

  if (!uart_irq_update(dev)) {
    return;
  }
  if (!uart_irq_rx_ready(dev)) {
    return;
  }
  while (uart_fifo_read(dev, &c, 1)) {
    if (c == 0x00 && com_ctx->cobs_bytes_read > 0) {
      com_ctx->rx_buf[com_ctx->cobs_bytes_read] = 0;
      if (com_ctx->cobs_bytes_read != (com_ctx->MSG_LEN - 1)) {
        com_ctx->cobs_bytes_read = 0;
        continue;
      }
      k_msgq_put(com_ctx->msgq_rx, com_ctx->rx_buf, K_NO_WAIT);
      k_work_submit_to_queue(&work_q, &com_ctx->cobs_rx_work_item);
      com_ctx->cobs_bytes_read = 0;
    } else if (com_ctx->cobs_bytes_read < com_ctx->MSG_LEN) {
      com_ctx->rx_buf[com_ctx->cobs_bytes_read++] = c;
    }
  }
}

/* work handler to form sbus packet and return sbus channels */
void sbus_work_handler(struct k_work *sbus_work_ptr) {
  uint8_t buffer[25] = {0};
  int err;
  k_msgq_get(&sbus_msgq, buffer, K_NO_WAIT);
  err = parity_checker(buffer[23]);

  if (err == 1) {
    gpio_pin_set_dt(&sbus_status_led, 0); // set sbus status led low
    LOG_ERR("Corrupt SBUS Packet\n");
  } else {
    gpio_pin_set_dt(&sbus_status_led, 1); // set sbus status led high
    k_mutex_lock(&ch_writer_mutex, K_FOREVER);
    if (k_sem_take(&ch_sem, K_NO_WAIT) == 0) {
      k_mutex_unlock(&ch_writer_mutex);
      parse_buffer(buffer, channel);
      k_sem_give(&ch_sem);
      k_work_submit_to_queue(&work_q, &(arm.channel_work_item));
      k_work_submit_to_queue(&work_q, &(drive.drive_work_item));
    } else {
      k_mutex_unlock(&ch_writer_mutex);
    }
  }
}

/* received cobs message work handler */
// struct dummy_arm_mode {
//   uint8_t arm_dummy[sizeof(struct auto_msg)+2];
// }dummy_arm;
void cobs_rx_work_handler(struct k_work *cobs_rx_work_ptr) {
  struct com_rx_arg *com_info = CONTAINER_OF(
      cobs_rx_work_ptr, struct com_rx_arg,
      cobs_rx_work_item); // changed type here to check for conflicts

  uint8_t buf[com_info->MSG_LEN];
  k_msgq_get(com_info->msgq_rx, buf, K_MSEC(4));
  struct auto_msg autonomous_state; 
  cobs_decode_result result = cobs_decode(
      &(autonomous_state), sizeof(autonomous_state), buf, com_info->MSG_LEN - 1);
  if (result.status != COBS_DECODE_OK) {
    LOG_ERR("COBS Decode Failed %d\n", result.status);
    return;
  }
  uint8_t *data=(uint8_t *)&autonomous_state;
  if(autonomous_state.state==arm_mode){
     
    for(int i=0;i<sizeof(struct auto_msg);i++){
        drive.arm_work_buffer[i]=data[i];
    };
        
    com_info->work_item=arm_com.work_item;
  }else if(autonomous_state.state==drive_mode){ 
    
    for(int i=0;i<sizeof(struct auto_msg);i++){
      drive.drive_raw_buffer[i]=data[i];
    }; 



    com_info->work_item=drive_com.work_item;
  }
  // submit autonomous work handler
  k_work_submit_to_queue(&work_q, com_info->work_item);
}

void sbc_tx_work_handler(struct k_work *sbc_tx_work_ptr) {

  struct com_tx_arg *com_info =
      CONTAINER_OF(sbc_tx_work_ptr, struct com_tx_arg,
                   sbc_tx_work_item); // changed type from som_arg to
                                      // com_tx_arg to match type
  com_info->bs_msg_tx.crc =
      crc32_ieee((uint8_t *)(&com_info->bs_msg_tx),
                 sizeof(struct base_station_msg) - sizeof(uint32_t));
  cobs_encode_result result =
      cobs_encode(bs_tx_buf, BS_MSG_LEN, (void *)&com_info->bs_msg_tx,
                  sizeof(struct base_station_msg));

  if (result.status != COBS_ENCODE_OK) {
    LOG_ERR("COBS Encoded Failed %d\n", result.status);

    return;
  }
  bs_tx_buf[BS_MSG_LEN - 1] = 0x00;
  for (int i = 0; i < BS_MSG_LEN; i++) {
    uart_poll_out(sbc_uart, bs_tx_buf[i]);
  }

}

int velocity_callback(const float *velocity_buffer, int buffer_len,
                      int wheels_per_side) {
  if (buffer_len < wheels_per_side * 2) {
    return 1;
  }
  if (pwm_motor_write(
          &(pwm_motor[0]),
          LINEAR_INTERPOLATION(*velocity_buffer, wheel_velocity_range[0],
                               wheel_velocity_range[1], pwm_motor[0].min_pulse,
                               pwm_motor[0].max_pulse))) {
    LOG_ERR("Drive: Unable to write pwm pulse to Left");
    return 1;
  }
  if (pwm_motor_write(&(pwm_motor[1]),
                      LINEAR_INTERPOLATION(
                          *(velocity_buffer + wheels_per_side + 1),
                          wheel_velocity_range[0], wheel_velocity_range[1],
                          pwm_motor[1].min_pulse, pwm_motor[1].max_pulse))) {
    LOG_ERR("Drive: Unable to write pwm pulse to Right");
    return 1;
  }
  return 0;
}

/* work handler to write to motors */
void drive_work_handler(struct k_work *drive_work_ptr) {
  struct drive_arg *drive_info =
      CONTAINER_OF(drive_work_ptr, struct drive_arg, drive_work_item);

  if (k_mutex_lock(&ch_writer_mutex, K_NO_WAIT) != 0) {
    return;
  }
  k_mutex_unlock(&ch_writer_mutex);
  k_mutex_lock(&ch_reader_cnt_mutex, K_FOREVER);
  if (ch_reader_cnt == 0) {
    k_sem_take(&ch_sem, K_FOREVER);
  }
  ch_reader_cnt++;
  k_mutex_unlock(&ch_reader_cnt_mutex);

  // drive update
  drive_info->cmd.linear_x = LINEAR_INTERPOLATION(
      channel[pwm_motor[0].channel], channel_range[0], channel_range[1],
      linear_velocity_range[0], linear_velocity_range[1]);
  drive_info->cmd.angular_z = LINEAR_INTERPOLATION(
      channel[pwm_motor[1].channel], channel_range[0], channel_range[1],
      angular_velocity_range[0], angular_velocity_range[1]);
  diffdrive_update(drive_info->drive_init, drive_info->cmd);

  for (size_t i = 2U; i < ARRAY_SIZE(pwm_motor); i++) {
    if (pwm_motor_write(&(pwm_motor[i]),
                        LINEAR_INTERPOLATION(channel[pwm_motor[i].channel],
                                             channel_range[0], channel_range[1],
                                             pwm_motor->min_pulse,
                                             pwm_motor->max_pulse)))
      LOG_ERR("PWM: Motor %s unable to write", pwm_motor[i].dev_spec.dev->name);
  }
  k_mutex_lock(&ch_reader_cnt_mutex, K_FOREVER);
  ch_reader_cnt--;
  if (ch_reader_cnt == 0) {
    k_sem_give(&ch_sem);
  }
  k_mutex_unlock(&ch_reader_cnt_mutex);
  ;
}

/* autonomous drive work handler */
void auto_drive_work_handler(struct k_work *auto_drive_work_ptr) {
  struct drive_arg *drive_info =
      CONTAINER_OF(auto_drive_work_ptr, struct drive_arg, auto_drive_work_item);

  struct auto_msg *msg = (struct auto_msg *)drive_info->drive_raw_buffer;

  // check crc
  if (check_crc(drive_info->drive_raw_buffer, sizeof(struct auto_msg)) !=
      msg->crc)
    return;

  // update drive
  drive_info->cmd.linear_x = msg->auto_cmd.linear_x;
  drive_info->cmd.angular_z = msg->auto_cmd.angular_z;
  diffdrive_update(drive_info->drive_init, drive_info->cmd);


}

/* arm channel work handler */
void arm_channel_work_handler(struct k_work *work_ptr) {
  struct arm_arg *arm_info =
      CONTAINER_OF(work_ptr, struct arm_arg, channel_work_item);
  if (k_mutex_lock(&ch_writer_mutex, K_NO_WAIT) != 0) {
    return;
  }
  k_mutex_unlock(&ch_writer_mutex);
  k_mutex_lock(&ch_reader_cnt_mutex, K_FOREVER);
  if (ch_reader_cnt == 0) {
    k_sem_take(&ch_sem, K_FOREVER);
  }
  ch_reader_cnt++;
  k_mutex_unlock(&ch_reader_cnt_mutex);

  for (int i = 0U; i < ARRAY_SIZE(stepper_motor); i++) {
    if (channel[stepper_motor[i].channel] > 1185)
      arm_info->dir[i] = HIGH_PULSE;
    else if (channel[stepper_motor[i].channel] < 800)
      arm_info->dir[i] = LOW_PULSE;
    else
      arm_info->dir[i] = STOP_PULSE;
  }

  k_mutex_lock(&ch_reader_cnt_mutex, K_FOREVER);
  ch_reader_cnt--;
  if (ch_reader_cnt == 0) {
    k_sem_give(&ch_sem);
  }
  k_mutex_unlock(&ch_reader_cnt_mutex);
}
void auto_arm_work_handler(struct k_work *auto_arm_work_ptr) {
  struct drive_arg *arm_info= CONTAINER_OF(auto_arm_work_ptr,struct drive_arg,auto_arm_work_item);
  struct auto_msg *msg=(struct auto_msg *)arm_info->arm_work_buffer;
  if (check_crc(arm_info->arm_work_buffer, sizeof(struct auto_msg )) !=msg->crc)
      return;
  /* msg should be having x,y,z of target pos or the angles of ik */   
  
    // int target=(msg->arm_cmd[i]);
    // int error=target-arm.pos[i];
    // if(error>1){
    //  arm_info->dir[i]=HIGH_PULSE; 
    // }else if(error<-1){
    //   arm_info->dir[i]=LOW_PULSE;
    // }else {
    //   arm_info->dir[i]=STOP_PULSE;
    // }
  

  
}

/* timer to write to stepper motors*/
void stepper_timer_handler(struct k_timer *stepper_timer_ptr) {
  for (int i = 0U; i < ARRAY_SIZE(stepper_motor); i++) {
    arm.pos[i] = Stepper_motor_write(&stepper_motor[i], arm.dir[i], arm.pos[i]);
  }
}

K_TIMER_DEFINE(stepper_timer, stepper_timer_handler, NULL);

int main() {
  LOG_INF("Tarzan version %s\nFile: %s\n", TARZAN_GIT_VERSION, __FILE__);
  if (usb_enable(NULL))
    LOG_ERR("CDC ACM UART failed");

  /* enable usb for sbc com */



  int err;

  /* initializing work queue */
  k_work_queue_init(&work_q);

  /* initializing channel semaphore */
  k_sem_init(&ch_sem, 1, 1);
  /* initializing mutex for readers to wait */
  k_mutex_init(&ch_writer_mutex);
  /* inittializing mutex for reader count */
  k_mutex_init(&ch_reader_cnt_mutex);

  /* initializing work items */
  k_work_init(&sbus_work_item, sbus_work_handler);
  k_work_init(&(drive.drive_work_item), drive_work_handler);
  k_work_init(&(drive.auto_drive_work_item), auto_drive_work_handler);
  k_work_init(&(arm.channel_work_item), arm_channel_work_handler);
  k_work_init(&(drive.auto_arm_work_item),auto_arm_work_handler);
  k_work_init(&(drive_com.cobs_rx_work_item), cobs_rx_work_handler);
  k_work_init(&(gps.gps_work_item),gps_work_handler);
  k_work_init(&(arm.arm_tx_work_item),arm_tx_work_handler);
  k_work_init(&(com_tx.sbc_tx_work_item), sbc_tx_work_handler);

  /* initializing drive configs */
  const struct DiffDriveConfig tmp_drive_config = {
      .wheel_separation = 0.77f,
      .wheel_separation_multiplier = 1,
      .wheel_radius = 0.15f,
      .wheels_per_side = 2,
      .command_timeout_seconds = 2,
      .left_wheel_radius_multiplier = 1,
      .right_wheel_radius_multiplier = 1,
  };
  drive.drive_config = tmp_drive_config;
  drive.drive_init =
      drive_init(&(drive.drive_config), JERK_LIMITER, velocity_callback);

  /* sbus uart ready check */
  if (!device_is_ready(sbus_uart))
    LOG_ERR("SBUS UART device not ready");
  /* sbc uart ready check */
  if (!device_is_ready(sbc_uart))
    LOG_ERR("SBC UART device not ready");


  /* gps ready check*/
  gnss_systems_t supported, enabled;
  if (gnss_get_supported_systems(GNSS_MODEM, &supported) < 0) 
    LOG_ERR("Failed to query supported systems");
  
  if (gnss_get_enabled_systems(GNSS_MODEM, &enabled) < 0)
    LOG_ERR("Failed to query enabled systems");

  /* set sbus uart for interrupt */
  err = uart_irq_callback_user_data_set(sbus_uart, sbus_cb, NULL);
  if (err < 0) {
    if (err == -ENOTSUP) {
      LOG_ERR("Interrupt-driven UART API support not enabled");
    } else if (err == -ENOSYS) {
      LOG_ERR("UART device does not support interrupt-driven API");
    } else {
      LOG_ERR("Error setting UART callback: %d", err);
    }
  }

  /* set sbc uart for interrupt */
  err = uart_irq_callback_user_data_set(sbc_uart, cobs_cb, &drive_com);
  if (err < 0) {
    if (err == -ENOTSUP) {
      LOG_ERR("Interrupt-driven UART API support not enabled");
    } else if (err == -ENOSYS) {
      LOG_ERR("UART device does not support interrupt-driven API");
    } else {
      LOG_ERR("Error setting UART callback: %d", err);
    }
  }

  /* pwm ready check */
  for (size_t i = 0U; i < ARRAY_SIZE(pwm_motor); i++) {
    if (!pwm_is_ready_dt(&(pwm_motor[i].dev_spec)))
      LOG_ERR("PWM: Motor %s is not ready", pwm_motor[i].dev_spec.dev->name);
    if (pwm_motor_write(&(pwm_motor[i]), 1500000))
      LOG_ERR("Unable to write pwm pulse to PWM Motor : %d\n", i);
  }

  /* stepper motor ready check */
  for (size_t i = 0U; i < ARRAY_SIZE(stepper_motor); i++) {
    if (!gpio_is_ready_dt(&stepper_motor[i].dir) ||
        !gpio_is_ready_dt(&stepper_motor[i].step))
      LOG_ERR("Stepper Motor %d: Pin %d is not ready", i,
              stepper_motor[i].dir.pin);

    if (gpio_pin_configure_dt(&(stepper_motor[i].dir), GPIO_OUTPUT_INACTIVE) ||
        gpio_pin_configure_dt(&(stepper_motor[i].step), GPIO_OUTPUT_INACTIVE))
      LOG_ERR("Stepper motor %d: Pin %d not configured", i,
              stepper_motor[i].dir.pin);
  }

  /* led ready checks */
  if (!gpio_is_ready_dt(&init_led)) {
    LOG_ERR("Initialization led not ready\n");
  }
  if (!gpio_is_ready_dt(&sbus_status_led)) {
    LOG_ERR("SBUS Status led not ready\n");
  }
  if (!pwm_is_ready_dt(&error_led)) {
    LOG_ERR("Error led is not ready");
  }
  if (gpio_pin_configure_dt(&init_led, GPIO_OUTPUT_INACTIVE) < 0) {
    LOG_ERR("Intitialization led not configured\n");
  }
  if (gpio_pin_configure_dt(&sbus_status_led, GPIO_OUTPUT_INACTIVE) < 0) {
    LOG_ERR("SBUS Status led not configured\n");
  }

  LOG_INF("Initialization completed successfully!\n");

  gpio_pin_set_dt(&init_led, 1); // set initialization led high

  /* start running work queue */
  k_work_queue_start(&work_q, stack_area, K_THREAD_STACK_SIZEOF(stack_area),
                     PRIORITY, NULL);

  /* enable interrupt to receive sbus data */
  uart_irq_rx_enable(sbus_uart);
  /* enable interrupt to receive cobs data */
  uart_irq_rx_enable(sbc_uart);
  /* enable interrupt to receive gps data */
  GNSS_DATA_CALLBACK_DEFINE(GNSS_MODEM, gps_cb);

  /* enabling stepper & mssg timer */
  k_timer_start(&stepper_timer, K_SECONDS(1), K_USEC((STEPPER_TIMER) / 2));

  return 0;
}
