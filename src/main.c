#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/mcpwm_cmpr.h"
#include "driver/mcpwm_gen.h"
#include "driver/mcpwm_oper.h"
#include "driver/mcpwm_timer.h"

#include "driver/gpio.h"

#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "esp_timer.h"

#define EPSILON 1e-6

static mcpwm_timer_handle_t timer = NULL;
static mcpwm_oper_handle_t oper = NULL;
static mcpwm_cmpr_handle_t cmpr = NULL;
static mcpwm_gen_handle_t gen = NULL;

static i2c_master_bus_handle_t bus_handler = NULL;
static i2c_master_dev_handle_t as5600_handler = NULL;

const int uart_buffer_size = (1024 * 2);
QueueHandle_t uart_queue;
const uart_port_t uart_num = UART_NUM_1;
const uart_port_t uart_num_pc = UART_NUM_0;

uint8_t AS5600_ADDR = 0x36;
uint8_t CONF_H = 0x07;
uint8_t CONF_L = 0x08;
uint8_t RAW_ANGLE_ADDR = 0x0C;

#define UART_RX 25
#define UART_TX 32
#define DIR_PIN 16
#define STEP_PIN 26
#define GCONF_REG 0x00
#define IHOLD_IRUN_REG 0x10
#define TPOWERDOWN_REG 0x11
#define VACTUAL_REG 0x22
#define CHOPCONF_REG 0x6C

void I2C_Detect() {
  printf("I2C Scanner starting...\n");
  printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
  for (int i = 0; i < 128; i += 16) {
    printf("%02x:", i);
    for (int j = 0; j < 16; j++) {
      fflush(stdout);
      uint8_t address = i + j;

      if (address < 0x03 || address > 0x77) {
        printf("  ");
        continue;
      }

      esp_err_t ret = i2c_master_probe(bus_handler, address, 100);

      if (ret == ESP_OK) {
        printf("%02x ", address);
      } else {
        printf("-- ");
      }
    }
    printf("\n");
  }
  printf("Scan finished.\n");
}

bool GetRawAngle(float *angle) {
  uint8_t recv_buff[2];
  uint16_t raw_angle = 0x00;
  esp_err_t ret = i2c_master_transmit_receive(as5600_handler, &RAW_ANGLE_ADDR,
                                              1, recv_buff, 2, -1);
  if (ret == ESP_OK) {
    raw_angle = raw_angle | ((uint16_t)recv_buff[0] << 8) | recv_buff[1];
    raw_angle = (raw_angle & 0x0FFF);
    *angle = (float)raw_angle / 4095 * 360;
    return false;
  } else {
    return true;
  }
}

void swuart_calcCRC(uint8_t *datagram, uint8_t datagramLength) {
  int i, j;
  uint8_t *crc =
      datagram + (datagramLength - 1); // CRC located in last byte of message
  uint8_t currentByte;
  *crc = 0;
  for (i = 0; i < (datagramLength - 1);
       i++) {                  // Execute for all bytes of a message
    currentByte = datagram[i]; // Retrieve a byte to be sent from Array
    for (j = 0; j < 8; j++) {
      if ((*crc >> 7) ^
          (currentByte & 0x01)) // update CRC based result of XOR operation
      {
        *crc = (*crc << 1) ^ 0x07;
      } else {
        *crc = (*crc << 1);
      }
      currentByte = currentByte >> 1;
    } // for CRC bit
  } // for message byte
}

bool sendConfig(uart_port_t uart_num, uint8_t addr_register, uint32_t *data) {
  uint8_t frame[8];
  // UART
  // From LSB
  frame[0] = 0x05;                 // sync + don't care
  frame[1] = 0x00;                 // node addr(device)
  frame[2] = addr_register | 0x80; // register + rw
  frame[3] = (*data >> 24) & 0xFF; // 32bit data MSB
  frame[4] = (*data >> 16) & 0xFF;
  frame[5] = (*data >> 8) & 0xFF;
  frame[6] = *data & 0xFF;
  frame[7] = 0x00;

  swuart_calcCRC(frame, 8);

  return uart_write_bytes(uart_num, &frame, 8);
}

void communication(int *MODE, float *target_angle, float *target_velocity, int32_t *speed_hz, int32_t *target_hz_m2) {
  char recv_data[128];
  int recv_len =
      uart_read_bytes(uart_num_pc, recv_data, sizeof(recv_data) - 1, 0);
  if (recv_len > 0) {
    recv_data[recv_len] = '\0';
    char *p;
    if ((p = strstr(recv_data, "SETANGLE:")) != NULL) {
      // MODE SERVO
      *MODE = 0;
      *target_angle = atof(p + 9);
      *target_velocity = 0;
      printf("Angle updated: %.2f\n", *target_angle);
    } else if ((p = strstr(recv_data, "SETVELOCITY:")) != NULL) {
      // MODE VELOCITY
      *MODE = 1;
      *target_angle = 0;
      *target_velocity = atof(p + 12);
      if(*target_velocity > 0){
        gpio_set_level(DIR_PIN, 1);
      }else if(*target_velocity < 0){
        gpio_set_level(DIR_PIN, 0);
      }
      printf("Velocity updated: %.2f\n", *target_velocity);
    } else if ((p = strstr(recv_data, "STOP")) != NULL) {
      // MODE STOP
      *MODE = -1;
      *target_angle = 0;
      *target_velocity = 0;
      *speed_hz = 0;
      mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY);
      printf("EMERGENCY STOP\n");
    }else if ((p = strstr(recv_data, "SETM2:")) != NULL) {
        // Set top speed(Hz)
        // For example "SETM2:2000" (CW2000Hz) or "SETM2:-1500" (CCW1500Hz)
        *MODE = 2;
        *target_hz_m2 = atoi(p + 6);
        printf("Mode 2 target Hz updated: %ld\n", *target_hz_m2);
    }
  }
  // vTaskDelay(pdMS_TO_TICKS(1));
}

void app_main() {
  // MCPWM
  // timer
  uint32_t mcpwm_res = 1000000;
  uint32_t tick = 1000;

  mcpwm_timer_config_t timer_conf = {
      .group_id = 0,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = mcpwm_res,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
      .period_ticks = tick,
  };
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_conf, &timer));

  // operator
  mcpwm_operator_config_t oper_conf = {
      .group_id = 0,
  };
  ESP_ERROR_CHECK(mcpwm_new_operator(&oper_conf, &oper));

  // comparator
  mcpwm_comparator_config_t cmp_conf = {
      .flags =
          {
              .update_cmp_on_tez = true,
          },
  };

  ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &cmp_conf, &cmpr));

  // generator
  mcpwm_generator_config_t gen_conf = {
      .gen_gpio_num = STEP_PIN,
      .flags =
          {
              .invert_pwm = 0,
              .io_loop_back = 0,
              .io_od_mode = 0,
              .pull_up = true,
              .pull_down = false,
          },
  };
  ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_conf, &gen));

  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmpr, tick / 2));

  mcpwm_gen_timer_event_action_t action_up = {
      .direction = MCPWM_TIMER_DIRECTION_UP,
      .event = MCPWM_TIMER_EVENT_EMPTY,
      .action = MCPWM_GEN_ACTION_HIGH,
  };

  mcpwm_gen_compare_event_action_t action_down = {
      .direction = MCPWM_TIMER_DIRECTION_UP,
      .comparator = cmpr,
      .action = MCPWM_GEN_ACTION_LOW,
  };

  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));

  ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_timer_event(
      gen, action_up, MCPWM_GEN_TIMER_EVENT_ACTION_END()));

  ESP_ERROR_CHECK(mcpwm_generator_set_actions_on_compare_event(
      gen, action_down, MCPWM_GEN_COMPARE_EVENT_ACTION_END()));

  // GPIO
  gpio_config_t gpio_conf = {
      .pin_bit_mask = (1ULL << DIR_PIN),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&gpio_conf));

  // I2C
  i2c_master_bus_config_t bus_conf = {
      .i2c_port = -1,
      .sda_io_num = 17,
      .scl_io_num = 33,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
  };
  ESP_ERROR_CHECK(i2c_new_master_bus(&bus_conf, &bus_handler));

  i2c_device_config_t as5600_conf = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = AS5600_ADDR,
      .scl_speed_hz = 400000,
  };

  ESP_ERROR_CHECK(
      i2c_master_bus_add_device(bus_handler, &as5600_conf, &as5600_handler));

  // UART
  uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 10,
                      &uart_queue, 0);

  uart_driver_install(uart_num_pc, uart_buffer_size, uart_buffer_size, 10,
                      &uart_queue, 0);
  uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .rx_flow_ctrl_thresh = 122,
  };
  ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
  ESP_ERROR_CHECK(uart_param_config(uart_num_pc, &uart_config));
  ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_RX, UART_TX, -1, -1));
  ESP_ERROR_CHECK(uart_set_pin(uart_num_pc, 1, 3, -1, -1));

  uint8_t current_as5600_conf;
  i2c_master_transmit_receive(as5600_handler, &CONF_L, 1, &current_as5600_conf,
                              1, -1);
  current_as5600_conf |= (0x03 << 2);

  uint8_t write_buf[2] = {CONF_L, current_as5600_conf};
  i2c_master_transmit(as5600_handler, write_buf, 2, -1);

  uint32_t data = 0x00;
  data = data | (0x01 << 6) | (0x01 << 7) | (0x01 << 8) | (0x01 << 2);

  // General Configs
  sendConfig(uart_num, GCONF_REG, &data);

  vTaskDelay(100);

  // IRUN=16 (bit8-12), IHOLD=2 (bit0-4), IHOLDDELAY=4 (bit16-19)
  uint32_t current_data = (16 << 8) | (2 << 0) | (4 << 16);
  sendConfig(uart_num, IHOLD_IRUN_REG, &current_data);

  vTaskDelay(100);

  // Standby transition time: Delay between IHOLD stop detection and the transition to IHOLD
  // value * 2^18 / fCLK ≒ 10 * 21.8ms ≒ 218ms
  uint32_t tpowerdown = 10;
  sendConfig(uart_num, TPOWERDOWN_REG, &tpowerdown);

  vTaskDelay(100);

  // Current Config
  uint32_t chop = 0x04000003;
  sendConfig(uart_num, CHOPCONF_REG, &chop);

  // --- Control Variables ---
  int32_t speed_hz = 500; // Actual pulse freqency[Hz]
  int32_t prev_speed_hz = 500;

  int32_t servo_max_speed_hz = 10000;
  int32_t max_speed_hz = 37000;
  int32_t min_speed_hz = 100;
  int32_t servo_accel_hz = 5000;
  int32_t accel_hz = 500; // Acceleration Limit

  gpio_set_direction(DIR_PIN, GPIO_MODE_OUTPUT);

  float target_angle = 120;
  float target_velocity = 0.0;

  float kp_angle = 100, kd_angle = 0.0, ki_angle = 0.0;
  float kp_velocity = 5.0, kd_velocity = 0.0, ki_velocity = 0.0;

  float angle = 0.0;
  float prev_angle = 0.0;
  float velocity = 0.0;
  float prev_velocity = 0.0;

  float error_angle = 0.0;
  float prev_error_angle = 0.0;
  float integral_error_angle = 0.0;

  float error_velocity = 0.0;
  float prev_error_velocity = 0.0;
  float integral_error_velocity = 0.0;

  float dt = 0.02;

  // MODE  0 = SERVO
  // MODE  1 = VELOCITY
  // MODE -1 = STOP
  // MODE  2 = AUTO ROTATION
  int MODE = -1;

  // MODE2
  int32_t target_hz_m2 = 2000;   // Target Frequency (Positive(+) CW、Negative(-) CCW)
  int32_t current_hz_m2 = 0;  // Current Frequency
  int32_t step_accel = 5;     // Acceleration per loop (Lowering this setting reduces the rate of speed increase)

  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

  uint64_t prev_time = esp_timer_get_time();

  while (true) {
    uint64_t current_time = esp_timer_get_time();
    dt = (float)(current_time - prev_time) / 1000000.0f;
    if(dt <= 0.0f){
      dt = 0.0001f;
    }
    prev_time = current_time;
    GetRawAngle(&angle);
    // MODE SERVO
    if (MODE == 0) {
      error_angle = target_angle - angle;

      if (error_angle > 180.0)
        error_angle -= 360.0;
      else if (error_angle < -180.0)
        error_angle += 360.0;

      // error evaluate
      if (fabs(error_angle) < 3) {
        speed_hz = 0;
        integral_error_angle = 0;
        mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY);
        printf("STOPPED | Angle: %.2f | Error: %.2f\n", angle, error_angle);
        vTaskDelay(5);
        communication(&MODE, &target_angle, &target_velocity, &speed_hz, &target_hz_m2);
      }

      integral_error_angle += error_angle * dt;

      speed_hz = fabs(kp_angle * error_angle +
                      kd_angle * (error_angle - prev_error_angle) / dt +
                      ki_angle * integral_error_angle);

      if (speed_hz > servo_max_speed_hz) {
        speed_hz = servo_max_speed_hz;
      } else if (speed_hz < min_speed_hz) {
        speed_hz = min_speed_hz;
      }

      if ((speed_hz - prev_speed_hz) > servo_accel_hz) {
        speed_hz = prev_speed_hz + servo_accel_hz;
      }

      // dir control
      if (error_angle >= 0) {
        gpio_set_level(DIR_PIN, 1);
      } else if (error_angle < 0) {
        gpio_set_level(DIR_PIN, 0);
      }

      uint32_t period = mcpwm_res / speed_hz;
      mcpwm_timer_set_period(timer, period);
      mcpwm_comparator_set_compare_value(cmpr, period / 2);
      mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

      prev_error_angle = error_angle;
      prev_speed_hz = speed_hz;
      printf("Angle: %.2f | Target: %.2f | Error: %.2f |Freq: %ld Hz\n", angle,
             target_angle, error_angle, speed_hz);
      
      vTaskDelay(5);
    }
    // MODE VELOCITY
    else if (MODE == 1) { 
      // incomplete 
      float delta_angle = angle - prev_angle;
      if(delta_angle >= 180.0){
        delta_angle -= 360.0;
      }else if(delta_angle < -180.0){
        delta_angle += 360.0;
      }

      velocity = delta_angle / dt;
      error_velocity = target_velocity - velocity;

      integral_error_velocity += error_velocity;

      speed_hz = fabs(kp_velocity * error_velocity + kd_velocity * (error_velocity - prev_error_velocity) / dt + ki_velocity * integral_error_velocity);

      if (speed_hz > max_speed_hz) {
        speed_hz = max_speed_hz;
      } else if (speed_hz < min_speed_hz) {
        speed_hz = min_speed_hz;
      }

      if ((speed_hz - prev_speed_hz) > servo_accel_hz) {
        speed_hz = prev_speed_hz + accel_hz;
      }
      
      uint32_t period = mcpwm_res / speed_hz;
      mcpwm_timer_set_period(timer, period);
      mcpwm_comparator_set_compare_value(cmpr, period / 2);
      mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

      prev_error_velocity = error_velocity;
      prev_velocity = velocity;
      prev_speed_hz = speed_hz;

      printf("Velocity: %.2f deg/s | Target: %.2f deg/s | Error: %.2f deg/s | Freq: %ld Hz\n", velocity, target_velocity, error_velocity, speed_hz);
      vTaskDelay(1);
    }
    // MODE STOP
    else if (MODE == -1) {
      //printf("Motor State: Stop\n");
      mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY);
      vTaskDelay(5);
    }
    // MODE AUTO ROTATION
    else if(MODE == 2){
      if (current_hz_m2 < target_hz_m2) {
        current_hz_m2 += accel_hz;
        if (current_hz_m2 > target_hz_m2) current_hz_m2 = target_hz_m2;
      } else if (current_hz_m2 > target_hz_m2) {
        current_hz_m2 -= accel_hz;
        if (current_hz_m2 < target_hz_m2) current_hz_m2 = target_hz_m2;
      }

      int32_t abs_hz = current_hz_m2;
      if (abs_hz >= 0) {
        gpio_set_level(DIR_PIN, 1);
      } else {
        gpio_set_level(DIR_PIN, 0);
        abs_hz = -abs_hz;
      }

      if (abs_hz < min_speed_hz) {
        mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY);
      } else {
        if (abs_hz > max_speed_hz) abs_hz = max_speed_hz;

        uint32_t period = mcpwm_res / abs_hz;
        mcpwm_timer_set_period(timer, period);
        mcpwm_comparator_set_compare_value(cmpr, period / 2);
        mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
      }
      vTaskDelay(5);
    }
    prev_angle = angle;
    communication(&MODE, &target_angle, &target_velocity, &speed_hz, &target_hz_m2);
  }
}
