#include <stdio.h>
#include <string.h>

#include <fcntl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <termios.h>
#include <unistd.h>

#include <time.h>

#define SERIAL_PORT "/dev/ttyUSB0"

void recv_and_print(int fd) {
  usleep(50000);
  char buf[255];
  int len = read(fd, buf, sizeof(buf));
  if (len > 0) {
    for (int i = 0; i < len; i++) {
      printf("%c", buf[i]);
    }
  }
  printf("\n");
}

int main() {
  int fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY);
  if (fd < 0) {
    perror("Port open error\n");
    return -1;
  }

  struct termios servo_port;
  tcgetattr(fd, &servo_port);

  cfsetispeed(&servo_port, B115200);
  cfsetospeed(&servo_port, B115200);

  servo_port.c_cflag &= ~CSIZE;
  servo_port.c_cflag |= CS8;

  servo_port.c_cflag |= CREAD;
  servo_port.c_cflag |= CLOCAL;

  tcsetattr(fd, TCSANOW, &servo_port);

  char input_buf[64];

  while (true) {
    printf("SELECT MODE\n");
    printf("0:SERVO, 1:VELOCITY, -1:STOP, 2:AUTO ROTATION ,3:MONITOR\n");
    printf("MODE: ");

    if (fgets(input_buf, sizeof(input_buf), stdin) == NULL)
      break;

    int mode;
    if (sscanf(input_buf, "%d", &mode) != 1) {
      printf("Invalid input. Please enter a number.\n\n");
      continue;
    }

    if (mode == 0) {
      float angle;
      printf("Selected: %d (SERVO MODE)\n", mode);
      printf("Please set target angle(degree): ");

      fgets(input_buf, sizeof(input_buf), stdin);
      if (sscanf(input_buf, "%f", &angle) == 1) {
        printf("Target Angle: %.2f\n\n", angle);

        char msg[64];
        int msg_len = sprintf(msg, "SETANGLE:%.2f\n", angle);
        write(fd, msg, msg_len);

        printf("--- Monitoring response for 5 seconds ---\n");

        time_t start_time = time(NULL);

        while (time(NULL) - start_time < 1) {
          char read_buf[255];
          int read_len = read(fd, read_buf, sizeof(read_buf) - 1);
          if (read_len > 0) {
            read_buf[read_len] = '\0';
            printf("%s", read_buf);
            fflush(stdout);
          }
        }
        printf("\n--- End of monitoring ---\n\n");

      } else {
        printf("Invalid angle value.\n\n");
      }

    } else if (mode == 1) {
      float velocity;
      printf("Selected: %d (VELOCITY MODE)\n", mode);
      printf("Please set target velocity(deg/s): ");

      fgets(input_buf, sizeof(input_buf), stdin);
      if (sscanf(input_buf, "%f", &velocity) == 1) {
        printf("Target Velocity: %.2f\n\n", velocity);

        char msg[64];
        int msg_len = sprintf(msg, "SETVELOCITY:%.2f\n", velocity);
        write(fd, msg, msg_len);

        printf("--- Monitoring response for 5 seconds ---\n");

        time_t start_time = time(NULL);

        while (time(NULL) - start_time < 1) {
          char read_buf[255];
          int read_len = read(fd, read_buf, sizeof(read_buf) - 1);
          if (read_len > 0) {
            read_buf[read_len] = '\0';
            printf("%s", read_buf);
            fflush(stdout);
          }
        }
        printf("\n--- End of monitoring ---\n\n");

      } else {
        printf("Invalid velocity value.\n\n");
      }

    } else if (mode == -1) {
      printf("Selected: %d\nSTOP!\n\n", mode);

      char *stop = "STOP\n";
      write(fd, stop, strlen(stop));

      recv_and_print(fd);
    } else if (mode == 2) {
      printf("Selected: %d\nContinuous Rotation Mode\n\n", mode);

      int target_hz = 0;
      printf("Enter target frequency [Hz] (e.g. 2000 for forward, -1500 for "
             "reverse): ");

      if (scanf("%d", &target_hz) == 1) {
        char m2_cmd[64];

        snprintf(m2_cmd, sizeof(m2_cmd), "SETM2:%d\n", target_hz);

        printf("Sending -> %s", m2_cmd); // 確認用の表示

        write(fd, m2_cmd, strlen(m2_cmd));
        recv_and_print(fd);
      } else {
        printf("Invalid input. Please enter an integer.\n");
        while (getchar() != '\n');
      }
    } else if (mode == 3) {
      printf("Selected: %d\n", mode);
      printf("--- Monitoring (Press 'q' to quit) ---\n");
      usleep(50000);

      struct termios old_stdio, new_stdio;
      tcgetattr(STDIN_FILENO, &old_stdio);
      new_stdio = old_stdio;
      new_stdio.c_lflag &= ~(ICANON | ECHO);
      tcsetattr(STDIN_FILENO, TCSANOW, &new_stdio);

      fd_set readfds;
      struct timeval tv;
      int monitor_active = 1;

      while (monitor_active) {
        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);
        FD_SET(fd, &readfds);

        int max_fd = (STDIN_FILENO > fd) ? STDIN_FILENO : fd;

        tv.tv_sec = 0;
        tv.tv_usec = 100000;

        int ret = select(max_fd + 1, &readfds, NULL, NULL, &tv);
        if (ret < 0) {
          break;
        }

        if (FD_ISSET(STDIN_FILENO, &readfds)) {
          char c;
          if (read(STDIN_FILENO, &c, 1) > 0) {
            if (c == 'q' || c == 'Q') {
              monitor_active = 0;
            }
          }
        }

        if (FD_ISSET(fd, &readfds)) {
          char read_buf[255];
          int read_len = read(fd, read_buf, sizeof(read_buf) - 1);
          if (read_len > 0) {
            read_buf[read_len] = '\0';
            printf("%s", read_buf);
            fflush(stdout);
          }
        }
      }

      tcflush(STDIN_FILENO, TCIFLUSH);
      tcsetattr(STDIN_FILENO, TCSANOW, &old_stdio);

      printf("\n--- Exited Monitor Mode ---\n\n");
    } else {
      printf("Selected: %d\nIncorrect mode, please select again\n\n", mode);
    }
  }
  close(fd);
  return 0;
}
