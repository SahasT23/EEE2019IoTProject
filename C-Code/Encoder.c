#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <poll.h>
#include <fcntl.h>
#include <pthread.h>
#include <time.h>
#include <string.h>

// Global variables
volatile int pulse_count = 0;
pthread_mutex_t pulse_mutex = PTHREAD_MUTEX_INITIALIZER;
int running = 1;

// Function to set up the GPIO pins
void setup_gpio() {
    system("echo 9 > /sys/class/gpio/unexport;\
    echo 9 > /sys/class/gpio/export;\
    echo in > /sys/class/gpio/gpio9/direction;\
    echo rising > /sys/class/gpio/gpio9/edge;\
    echo 10 > /sys/class/gpio/unexport;\
    echo 10 > /sys/class/gpio/export;\
    echo in > /sys/class/gpio/gpio10/direction;\
    echo none > /sys/class/gpio/gpio10/edge");
    
    // Give the system time to set up the GPIO
    usleep(500000);
}

// Thread function to count pulses from the encoder
void* count_pulses(void* arg) {
    struct pollfd pfd;
    char str[4];
    int direction_fd;
    char direction_val[2];

    // Open GPIO9 for the encoder pulses
    pfd.fd = open("/sys/class/gpio/gpio9/value", O_RDONLY | O_NONBLOCK);
    pfd.events = POLLPRI | POLLERR;

    // Open GPIO10 for the direction detection
    direction_fd = open("/sys/class/gpio/gpio10/value", O_RDONLY | O_NONBLOCK);
    
    if (pfd.fd < 0 || direction_fd < 0) {
        perror("Failed to open GPIO");
        return NULL;
    }

    // Initial read to clear any pending interrupts
    lseek(pfd.fd, 0, SEEK_SET);
    read(pfd.fd, str, 1);

    while (running) {
        // Wait for an interrupt on GPIO9
        if (poll(&pfd, 1, 1000) > 0) {
            // Clear the interrupt
            lseek(pfd.fd, 0, SEEK_SET);
            read(pfd.fd, str, 1);
            
            // Read the direction pin (GPIO10)
            lseek(direction_fd, 0, SEEK_SET);
            read(direction_fd, direction_val, 1);
            
            // Increment or decrement pulse count based on direction
            pthread_mutex_lock(&pulse_mutex);
            if (direction_val[0] == '1') {
                // Clockwise
                pulse_count++;
            } else {
                // Counter-clockwise
                pulse_count--;
            }
            pthread_mutex_unlock(&pulse_mutex);
        }
    }

    close(pfd.fd);
    close(direction_fd);
    return NULL;
}

// Thread function to calculate and output the frequency
void* calculate_frequency(void* arg) {
    struct timespec ts;
    int last_count = 0;
    int current_count;
    float frequency;
    float rps; // Rotations per second
    
    // Each rotation of the motor produces 500 pulses
    const float pulses_per_rotation = 500.0;
    
    // Sample rate in milliseconds (40ms = 25Hz sample rate)
    const int sample_rate_ms = 40;
    
    while (running) {
        // Sleep for the sample period
        usleep(sample_rate_ms * 1000);
        
        // Get the current pulse count
        pthread_mutex_lock(&pulse_mutex);
        current_count = pulse_count;
        pthread_mutex_unlock(&pulse_mutex);
        
        // Calculate the frequency in Hz (pulses per second)
        frequency = (float)(current_count - last_count) * (1000.0 / sample_rate_ms);
        
        // Convert frequency to rotations per second
        rps = frequency / pulses_per_rotation;
        
        // Print the rotational speed in Hz (RPS)
        printf("%.3f\n", rps);
        fflush(stdout);
        
        // Update the last count
        last_count = current_count;
    }
    
    return NULL;
}

int main() {
    pthread_t pulse_thread, freq_thread;
    
    printf("# Encoder program starting\n");
    fflush(stdout);
    
    // Set up the GPIO pins
    setup_gpio();
    
    // Create the pulse counting thread
    if (pthread_create(&pulse_thread, NULL, count_pulses, NULL) != 0) {
        perror("Failed to create pulse thread");
        return 1;
    }
    
    // Create the frequency calculation thread
    if (pthread_create(&freq_thread, NULL, calculate_frequency, NULL) != 0) {
        perror("Failed to create frequency thread");
        running = 0;
        pthread_join(pulse_thread, NULL);
        return 1;
    }
    
    // Wait for threads to complete (they won't unless SIGINT is received)
    pthread_join(pulse_thread, NULL);
    pthread_join(freq_thread, NULL);
    
    return 0;
}