#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include <fcntl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <termios.h>
#include <unistd.h>

#include <time.h>
#include <errno.h>

#define SERIAL_PORT "/dev/ttyUSB0"

static struct termios old_tio;

void setup_serial(int fd) {
  struct termios tio;

  tcgetattr(fd, &tio);

  cfsetispeed(&tio, B115200);
  cfsetospeed(&tio, B115200);

  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;

  tio.c_cflag |= CLOCAL;
  tio.c_cflag |= CREAD;

  tio.c_cflag &= ~PARENB;
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CRTSCTS;

  // RAW MODE
  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_lflag = 0;

  // non blocking read
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 1;

  tcflush(fd, TCIOFLUSH);

  tcsetattr(fd, TCSANOW, &tio);
}

void flush_serial_input(int fd) {
  tcflush(fd, TCIFLUSH);

  char dump[256];

  while (read(fd, dump, sizeof(dump)) > 0) {
  }
}

void monitor_response(int fd, int seconds) {
  time_t start_time = time(NULL);

  while ((time(NULL) - start_time) < seconds) {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;

    int ret = select(fd + 1, &readfds, NULL, NULL, &tv);

    if (ret > 0 && FD_ISSET(fd, &readfds)) {
      char read_buf[256];

      int len = read(fd, read_buf, sizeof(read_buf) - 1);

      if (len > 0) {
        read_buf[len] = '\0';
        printf("%s", read_buf);
        fflush(stdout);
      }
    }
  }
}

void send_command(int fd, const char *cmd) {
  flush_serial_input(fd);

  write(fd, cmd, strlen(cmd));

  tcdrain(fd);

  usleep(100000);
}

void enable_keyboard_raw_mode() {
  tcgetattr(STDIN_FILENO, &old_tio);

  struct termios new_tio = old_tio;

  new_tio.c_lflag &= ~(ICANON | ECHO);

  tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
}

void disable_keyboard_raw_mode() {
  tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
}

int main() {
  int fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_SYNC);

  if (fd < 0) {
    perror("open");
    return -1;
  }

  setup_serial(fd);

  char input_buf[64];

  while (true) {
    printf("\n");
    printf("SELECT MODE\n");
    printf("0:SERVO\n");
    printf("1:VELOCITY\n");
    printf("-1:STOP\n");
    printf("2:AUTO ROTATION\n");
    printf("3:MONITOR\n");
    printf("MODE: ");

    fflush(stdout);

    if (fgets(input_buf, sizeof(input_buf), stdin) == NULL) {
      break;
    }

    int mode;

    if (sscanf(input_buf, "%d", &mode) != 1) {
      printf("Invalid input\n");
      continue;
    }

    // =========================================================
    // SERVO MODE
    // =========================================================
    if (mode == 0) {
      float angle;

      printf("Selected: SERVO MODE\n");
      printf("Target angle: ");

      fflush(stdout);

      if (fgets(input_buf, sizeof(input_buf), stdin) == NULL)
        continue;

      if (sscanf(input_buf, "%f", &angle) != 1) {
        printf("Invalid angle\n");
        continue;
      }

      char cmd[64];

      snprintf(cmd, sizeof(cmd), "SETANGLE:%.2f\n", angle);

      printf("Sending -> %s", cmd);

      send_command(fd, cmd);

      printf("--- Monitoring response ---\n");

      monitor_response(fd, 2);

      printf("\n--- End ---\n");
    }

    // =========================================================
    // VELOCITY MODE
    // =========================================================
    else if (mode == 1) {
      float velocity;

      printf("Selected: VELOCITY MODE\n");
      printf("Target velocity: ");

      fflush(stdout);

      if (fgets(input_buf, sizeof(input_buf), stdin) == NULL)
        continue;

      if (sscanf(input_buf, "%f", &velocity) != 1) {
        printf("Invalid velocity\n");
        continue;
      }

      char cmd[64];

      snprintf(cmd, sizeof(cmd), "SETVELOCITY:%.2f\n", velocity);

      printf("Sending -> %s", cmd);

      send_command(fd, cmd);

      printf("--- Monitoring response ---\n");

      monitor_response(fd, 2);

      printf("\n--- End ---\n");
    }

    // =========================================================
    // STOP MODE
    // =========================================================
    else if (mode == -1) {
      printf("Selected: STOP\n");

      send_command(fd, "STOP\n");

      monitor_response(fd, 1);
    }

    // =========================================================
    // AUTO ROTATION MODE
    // =========================================================
    else if (mode == 2) {
      int target_hz;

      printf("Selected: AUTO ROTATION MODE\n");
      printf("Target Hz: ");

      fflush(stdout);

      if (fgets(input_buf, sizeof(input_buf), stdin) == NULL)
        continue;

      if (sscanf(input_buf, "%d", &target_hz) != 1) {
        printf("Invalid Hz\n");
        continue;
      }

      char cmd[64];

      snprintf(cmd, sizeof(cmd), "SETM2:%d\n", target_hz);

      printf("Sending -> %s", cmd);

      send_command(fd, cmd);

      monitor_response(fd, 1);
    }

    // =========================================================
    // MONITOR MODE
    // =========================================================
    else if (mode == 3) {
      printf("Selected: MONITOR MODE\n");
      printf("Press q to quit\n");

      enable_keyboard_raw_mode();

      bool running = true;

      while (running) {
        fd_set readfds;

        FD_ZERO(&readfds);

        FD_SET(fd, &readfds);
        FD_SET(STDIN_FILENO, &readfds);

        int maxfd = (fd > STDIN_FILENO) ? fd : STDIN_FILENO;

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000;

        int ret = select(maxfd + 1, &readfds, NULL, NULL, &tv);

        if (ret < 0) {
          break;
        }

        // keyboard
        if (FD_ISSET(STDIN_FILENO, &readfds)) {
          char c;

          if (read(STDIN_FILENO, &c, 1) > 0) {
            if (c == 'q' || c == 'Q') {
              running = false;
            }
          }
        }

        // serial
        if (FD_ISSET(fd, &readfds)) {
          char buf[256];

          int len = read(fd, buf, sizeof(buf) - 1);

          if (len > 0) {
            buf[len] = '\0';

            printf("%s", buf);

            fflush(stdout);
          }
        }
      }

      disable_keyboard_raw_mode();

      flush_serial_input(fd);

      printf("\nExited monitor mode\n");
    }

    else {
      printf("Invalid mode\n");
    }
  }

  close(fd);

  return 0;
}
