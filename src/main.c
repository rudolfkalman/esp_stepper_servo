#include <math.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
#define MCPWM_RES_HZ 1000000U
#define DEFAULT_PERIOD_TICKS 1000U
#define CMD_BUF_SIZE 256U
#define MAX_CMD_LENGTH (CMD_BUF_SIZE - 1U)
#define MIN_PWM_PERIOD_TICKS 2U

static mcpwm_timer_handle_t timer = NULL;
static mcpwm_oper_handle_t oper = NULL;
static mcpwm_cmpr_handle_t cmpr = NULL;
static mcpwm_gen_handle_t gen = NULL;

static i2c_master_bus_handle_t bus_handler = NULL;
static i2c_master_dev_handle_t as5600_handler = NULL;

const int uart_buffer_size = (1024 * 2);
QueueHandle_t uart_queue_motor;
QueueHandle_t uart_queue_pc;
const uart_port_t uart_num = UART_NUM_1;
const uart_port_t uart_num_pc = UART_NUM_0;

char data[256] = {0};
char cmd_buf[CMD_BUF_SIZE] = {0};
uint32_t cmd_buf_pos = 0;
bool cmd_is_ready = false;
bool cmd_discarding = false;

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

// --- Control Variables ---
int32_t speed_hz = 500; // Actual pulse freqency[Hz]
int32_t prev_speed_hz = 500;

int32_t servo_max_speed_hz = 10000;
int32_t max_speed_hz = 37000;
int32_t min_speed_hz = 100;
int32_t servo_accel_hz = 5000;
int32_t accel_hz = 500; // Acceleration Limit

float target_angle = 120;
float target_velocity = 0.0;

float kp_angle = 100, kd_angle = 0.0, ki_angle = 0.0;
float kp_velocity = 5.0, kd_velocity = 0.0, ki_velocity = 0.0;

float angle = 0.0;
float prev_angle = 0.0;
float velocity = 0.0;
float prev_velocity = 0.0;
bool velocity_measurement_ready = false;

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
int32_t target_hz_m2 =
    10000;                 // Target Frequency (Positive(+) CW、Negative(-) CCW)
int32_t current_hz_m2 = 0; // Current Frequency
int32_t step_accel = 5;    // Acceleration per loop (Lowering this setting
                           // reduces the rate of speed increase)

static void reset_command_buffer(void) {
  memset(cmd_buf, 0, sizeof(cmd_buf));
  cmd_buf_pos = 0;
}

static bool parse_float_arg(const char *arg, float min_value, float max_value,
                            float *out_value) {
  if (arg[0] == '\0' || arg[0] == ' ') {
    return false;
  }

  char *endptr = NULL;
  errno = 0;
  float value = strtof(arg, &endptr);

  if (endptr == arg || errno == ERANGE || !isfinite(value)) {
    return false;
  }

  if (*endptr != '\0') {
    return false;
  }

  if (value < min_value || value > max_value) {
    return false;
  }

  *out_value = value;
  return true;
}

static bool parse_int32_arg(const char *arg, int32_t min_value,
                            int32_t max_value, int32_t *out_value) {
  if (arg[0] == '\0' || arg[0] == ' ') {
    return false;
  }

  char *endptr = NULL;
  errno = 0;
  long value = strtol(arg, &endptr, 10);

  if (endptr == arg || errno == ERANGE || *endptr != '\0') {
    return false;
  }

  if (value < min_value || value > max_value) {
    return false;
  }

  *out_value = (int32_t)value;
  return true;
}

static void stop_pwm_output(void) {
  if (timer == NULL) {
    return;
  }

  mcpwm_timer_start_stop(timer, MCPWM_TIMER_STOP_EMPTY);
}

static bool update_pwm_frequency(int32_t hz) {
  if (hz < min_speed_hz || hz > max_speed_hz) {
    stop_pwm_output();
    return false;
  }

  uint32_t period = MCPWM_RES_HZ / (uint32_t)hz;
  if (period < MIN_PWM_PERIOD_TICKS) {
    stop_pwm_output();
    return false;
  }

  mcpwm_timer_set_period(timer, period);
  mcpwm_comparator_set_compare_value(cmpr, period / 2);
  mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
  return true;
}

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

void parse() {
  if (cmd_buf[0] == '\0') {
    return;
  }

  if (strncmp(cmd_buf, "SETANGLE:", 9) == 0) {
    float val = 0.0f;
    if (!parse_float_arg(cmd_buf + 9, 0.0f, 360.0f, &val)) {
      printf("Invalid SETANGLE value\n");
      return;
    }

    MODE = 0;
    target_angle = val;
    target_velocity = 0.0f;
    integral_error_angle = 0.0f;
    prev_error_angle = 0.0f;
    velocity_measurement_ready = false;

    printf("Angle updated: %.2f\n", target_angle);
  } else if (strncmp(cmd_buf, "SETVELOCITY:", 12) == 0) {
    float val = 0.0f;
    if (!parse_float_arg(cmd_buf + 12, -5000.0f, 5000.0f, &val)) {
      printf("Invalid SETVELOCITY value\n");
      return;
    }

    MODE = 1;
    target_angle = 0.0f;
    target_velocity = val;
    integral_error_velocity = 0.0f;
    prev_error_velocity = 0.0f;
    velocity_measurement_ready = false;
    gpio_set_level(DIR_PIN, (target_velocity >= 0.0f));

    printf("Velocity updated: %.2f\n", target_velocity);
  } else if (strcmp(cmd_buf, "STOP") == 0) {
    MODE = -1;
    target_angle = 0.0f;
    target_velocity = 0.0f;
    speed_hz = 0;
    current_hz_m2 = 0;
    target_hz_m2 = 0;
    integral_error_angle = 0.0f;
    integral_error_velocity = 0.0f;
    velocity_measurement_ready = false;
    stop_pwm_output();

    printf("EMERGENCY STOP\n");
  } else if (strncmp(cmd_buf, "SETM2:", 6) == 0) {
    int32_t val = 0;
    if (!parse_int32_arg(cmd_buf + 6, -max_speed_hz, max_speed_hz, &val)) {
      printf("Invalid SETM2 value. Allowed range: %ld..%ld Hz\n",
             (long)-max_speed_hz, (long)max_speed_hz);
      return;
    }

    MODE = 2;
    target_hz_m2 = val;
    velocity_measurement_ready = false;

    printf("Mode 2 target Hz updated: %ld\n", (long)target_hz_m2);
  } else {
    printf("Unknown command: [%s]\n", cmd_buf);
  }
}


void receive() {
  int length = uart_read_bytes(uart_num_pc, data, sizeof(data) - 1, 100);
  if (length <= 0) {
    return;
  }

  for (int i = 0; i < length; i++) {
    uint8_t ch = (uint8_t)data[i];

    if (ch == '\n') {
      if (cmd_discarding) {
        printf("Invalid input line discarded\n");
        cmd_discarding = false;
      } else {
        cmd_buf[cmd_buf_pos] = '\0';
        parse();
      }
      reset_command_buffer();
      continue;
    }

    if (ch == '\r') {
      continue;
    }

    if (cmd_discarding) {
      continue;
    }

    if (ch < 0x20 || ch > 0x7e) {
      reset_command_buffer();
      cmd_discarding = true;
      continue;
    }

    if (cmd_buf_pos < MAX_CMD_LENGTH) {
      cmd_buf[cmd_buf_pos++] = (char)ch;
    } else {
      reset_command_buffer();
      cmd_discarding = true;
    }
  }
}

void app_main() {
  // MCPWM
  // timer
  uint32_t tick = DEFAULT_PERIOD_TICKS;

  mcpwm_timer_config_t timer_conf = {
      .group_id = 0,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = MCPWM_RES_HZ,
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
      .sda_io_num = 21,
      .scl_io_num = 22,
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
  uart_driver_install(uart_num, uart_buffer_size, uart_buffer_size, 20,
                      &uart_queue_motor, 0);

  uart_driver_install(uart_num_pc, uart_buffer_size, uart_buffer_size, 20,
                      &uart_queue_pc, 0);
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

  gpio_set_direction(DIR_PIN, GPIO_MODE_OUTPUT);

  // IRUN=16 (bit8-12), IHOLD=2 (bit0-4), IHOLDDELAY=4 (bit16-19)
  uint32_t current_data = (16 << 8) | (2 << 0) | (4 << 16);
  sendConfig(uart_num, IHOLD_IRUN_REG, &current_data);

  vTaskDelay(100);

  // Standby transition time: Delay between IHOLD stop detection and the
  // transition to IHOLD value * 2^18 / fCLK ≒ 10 * 21.8ms ≒ 218ms
  uint32_t tpowerdown = 10;
  sendConfig(uart_num, TPOWERDOWN_REG, &tpowerdown);

  vTaskDelay(100);

  // Current Config
  uint32_t chop = 0x04000003;
  sendConfig(uart_num, CHOPCONF_REG, &chop);

  stop_pwm_output();

  uint64_t prev_time = esp_timer_get_time();

  while (true) {
    uint64_t current_time = esp_timer_get_time();
    dt = (float)(current_time - prev_time) / 1000000.0f;
    if (dt <= 0.0f) {
      dt = 0.0001f;
    }
    prev_time = current_time;
    bool needs_angle = (MODE == 0 || MODE == 1);
    bool angle_read_failed = needs_angle && GetRawAngle(&angle);
    if (needs_angle && angle_read_failed) {
      stop_pwm_output();
      vTaskDelay(pdMS_TO_TICKS(5));
      receive();
      continue;
    }
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
        prev_speed_hz = 0;
        integral_error_angle = 0.0f;
        stop_pwm_output();
        // printf("STOPPED | Angle: %.2f | Error: %.2f\n", angle,
        // error_angle);
        vTaskDelay(pdMS_TO_TICKS(5));
        receive();
        continue;
      }

      integral_error_angle += error_angle * dt;

      float control_hz = fabs(kp_angle * error_angle +
                              kd_angle * (error_angle - prev_error_angle) / dt +
                              ki_angle * integral_error_angle);
      if (!isfinite(control_hz)) {
        MODE = -1;
        speed_hz = 0;
        stop_pwm_output();
        printf("Invalid servo control value, stopped\n");
        vTaskDelay(pdMS_TO_TICKS(5));
        receive();
        continue;
      }
      speed_hz = (int32_t)control_hz;

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

      update_pwm_frequency(speed_hz);

      prev_error_angle = error_angle;
      prev_speed_hz = speed_hz;
      /*printf("Angle: %.2f | Target: %.2f | Error: %.2f |Freq: %ld Hz\n",
         angle, target_angle, error_angle, speed_hz);*/

      vTaskDelay(pdMS_TO_TICKS(5));
    }
    // MODE VELOCITY
    else if (MODE == 1) {
      if (fabs(target_velocity) < EPSILON) {
        speed_hz = 0;
        prev_speed_hz = 0;
        integral_error_velocity = 0.0f;
        stop_pwm_output();
        vTaskDelay(pdMS_TO_TICKS(5));
        receive();
        continue;
      }

      // incomplete
      if (!velocity_measurement_ready) {
        prev_angle = angle;
        prev_velocity = 0.0f;
        prev_error_velocity = 0.0f;
        velocity_measurement_ready = true;
      }

      float delta_angle = angle - prev_angle;
      if (delta_angle >= 180.0) {
        delta_angle -= 360.0;
      } else if (delta_angle < -180.0) {
        delta_angle += 360.0;
      }

      velocity = delta_angle / dt;
      error_velocity = target_velocity - velocity;

      integral_error_velocity += error_velocity;

      float control_hz =
          fabs(kp_velocity * error_velocity +
               kd_velocity * (error_velocity - prev_error_velocity) / dt +
               ki_velocity * integral_error_velocity);
      if (!isfinite(control_hz)) {
        MODE = -1;
        speed_hz = 0;
        stop_pwm_output();
        printf("Invalid velocity control value, stopped\n");
        vTaskDelay(pdMS_TO_TICKS(5));
        receive();
        continue;
      }
      speed_hz = (int32_t)control_hz;

      if (speed_hz > max_speed_hz) {
        speed_hz = max_speed_hz;
      } else if (speed_hz < min_speed_hz) {
        speed_hz = min_speed_hz;
      }

      if ((speed_hz - prev_speed_hz) > servo_accel_hz) {
        speed_hz = prev_speed_hz + accel_hz;
      }

      update_pwm_frequency(speed_hz);

      prev_error_velocity = error_velocity;
      prev_velocity = velocity;
      prev_speed_hz = speed_hz;

      /*printf("Velocity: %.2f deg/s | Target: %.2f deg/s | Error: %.2f deg/s
         | " "Freq: %ld Hz\n", velocity, target_velocity, error_velocity,
         speed_hz);*/
      vTaskDelay(pdMS_TO_TICKS(1));
    }
    // MODE STOP
    else if (MODE == -1) {
      // printf("Motor State: Stop\n");
      stop_pwm_output();
      vTaskDelay(pdMS_TO_TICKS(5));
    }
    // MODE AUTO ROTATION
    else if (MODE == 2) {
      if (current_hz_m2 < target_hz_m2) {
        current_hz_m2 += accel_hz;
        if (current_hz_m2 > target_hz_m2)
          current_hz_m2 = target_hz_m2;
      } else if (current_hz_m2 > target_hz_m2) {
        current_hz_m2 -= accel_hz;
        if (current_hz_m2 < target_hz_m2)
          current_hz_m2 = target_hz_m2;
      }

      int32_t abs_hz = current_hz_m2;
      if (abs_hz >= 0) {
        gpio_set_level(DIR_PIN, 1);
      } else {
        gpio_set_level(DIR_PIN, 0);
        abs_hz = -abs_hz;
      }

      if (abs_hz < min_speed_hz) {
        stop_pwm_output();
      } else {
        if (abs_hz > max_speed_hz)
          abs_hz = max_speed_hz;

        update_pwm_frequency(abs_hz);
      }
      vTaskDelay(pdMS_TO_TICKS(5));
    }
    prev_angle = angle;
    receive();
  }
}
