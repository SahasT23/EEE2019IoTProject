#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <time.h>

#define MOTOR_IDLE_VALUE 737  // Value that stops the motor
#define MAX_ADJUSTMENT 600    // Maximum adjustment from idle value
#define TARGET_SPEED 2.0      // Target speed in Hz (rotations per second)

// PID controller parameters - these may need adjustment for your specific setup
#define KP 100.0  // Proportional gain
#define KI 20.0   // Integral gain
#define KD 5.0    // Derivative gain

// Global variables
int running = 1;
int step_mode = 0;
time_t step_start_time = 0;
time_t step_cycle_time = 10;  // 10 seconds for each step phase

// Function to handle SIGINT (Ctrl+C)
void handle_sigint(int sig) {
    running = 0;
}

// Function to set up the serial port for motor control
int setup_serial(const char* device) {
    int fd;
    struct termios options;
    
    // Open the serial port
    fd = open(device, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        perror("Error opening serial port");
        return -1;
    }
    
    // Get the current options
    tcgetattr(fd, &options);
    
    // Set the baud rate to 9600 bps
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    
    // Enable the receiver and set local mode
    options.c_cflag |= (CLOCAL | CREAD);
    
    // Set 8N1 (8 data bits, no parity, 1 stop bit)
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    
    // Set raw input and output
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_oflag &= ~OPOST;
    
    // Set the new options
    tcsetattr(fd, TCSANOW, &options);
    
    return fd;
}

// Function to send a control value to the motor
void control_motor(int fd, int value) {
    char command[32];
    
    // Ensure the value is within the valid range
    if (value < MOTOR_IDLE_VALUE - MAX_ADJUSTMENT) {
        value = MOTOR_IDLE_VALUE - MAX_ADJUSTMENT;
    } else if (value > MOTOR_IDLE_VALUE + MAX_ADJUSTMENT) {
        value = MOTOR_IDLE_VALUE + MAX_ADJUSTMENT;
    }
    
    // Format the command
    sprintf(command, "%d\n", value);
    
    // Send the command
    write(fd, command, strlen(command));
}

int main() {
    int serial_fd;
    char buffer[256];
    float current_speed;
    float error, last_error = 0, integral = 0, derivative;
    int control_value;
    time_t current_time;
    
    // Register signal handler for Ctrl+C
    signal(SIGINT, handle_sigint);
    
    // Set up the serial port for motor control
    serial_fd = setup_serial("/dev/ttyS0");  // Replace with the correct serial port
    if (serial_fd < 0) {
        fprintf(stderr, "Failed to set up serial port\n");
        return 1;
    }
    
    printf("# Controller program starting\n");
    printf("# Target speed: %.1f Hz\n", TARGET_SPEED);
    printf("# PID parameters: Kp=%.1f, Ki=%.1f, Kd=%.1f\n", KP, KI, KD);
    printf("# Time,Speed,TargetSpeed,Control\n");
    fflush(stdout);
    
    // Initialize step mode timing
    step_start_time = time(NULL);
    
    // Main control loop
    while (running) {
        // Read the current speed from stdin (from the encoder program)
        if (fgets(buffer, sizeof(buffer), stdin) != NULL) {
            current_speed = atof(buffer);
            
            // Determine the target speed based on step mode
            float target;
            current_time = time(NULL);
            int cycle_position = (current_time - step_start_time) % (2 * step_cycle_time);
            
            if (step_mode) {
                if (cycle_position < step_cycle_time) {
                    target = TARGET_SPEED;  // First half of cycle: target speed
                } else {
                    target = 0.0;  // Second half of cycle: zero speed
                }
            } else {
                target = TARGET_SPEED;  // Normal mode: constant target
            }
            
            // Calculate the error
            error = target - current_speed;
            
            // Calculate the integral of the error
            integral += error;
            
            // Limit the integral to prevent windup
            if (integral > MAX_ADJUSTMENT / KI) {
                integral = MAX_ADJUSTMENT / KI;
            } else if (integral < -MAX_ADJUSTMENT / KI) {
                integral = -MAX_ADJUSTMENT / KI;
            }
            
            // Calculate the derivative of the error
            derivative = error - last_error;
            
            // Calculate the control value using PID formula
            control_value = MOTOR_IDLE_VALUE + (int)(KP * error + KI * integral + KD * derivative);
            
            // Send the control value to the motor
            control_motor(serial_fd, control_value);
            
            // Print time, current speed, target speed, and control value for data collection
            printf("%ld,%.3f,%.3f,%d\n", current_time, current_speed, target, control_value);
            fflush(stdout);
            
            // Save the current error for the next iteration
            last_error = error;
        }
    }
    
    // Stop the motor before exiting
    control_motor(serial_fd, MOTOR_IDLE_VALUE);
    
    // Close the serial port
    close(serial_fd);
    
    return 0;
}