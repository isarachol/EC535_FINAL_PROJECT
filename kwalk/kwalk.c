#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>

// --- CONFIGURATION ---
#define PORT 5000
#define KERNEL_DEVICE_PATH "/dev/walker"
#define MAX_COMMAND_LENGTH 32 // e.g., "FORWARD:135"

// Global file descriptors for clean shutdown
int server_fd_g = -1;
int network_socket_g = -1;
int kernel_fd_g = -1;

void clean_shutdown() {
    // Close the kernel device
    if (kernel_fd_g != -1) {
        close(kernel_fd_g);
        kernel_fd_g = -1;
    }
    // Close the active client connection
    if (network_socket_g != -1) {
        close(network_socket_g);
        network_socket_g = -1;
    }
    // Close the listening socket
    if (server_fd_g != -1) {
        close(server_fd_g);
        server_fd_g = -1;
    }
    printf("\nClean shutdown complete.\n");
}

void signal_handler(int signum) {
    printf("\nReceived signal %d. Shutting down bridge...\n", signum);
    clean_shutdown();
    exit(0);
}

int main() {
    struct sockaddr_in address;
    int addrlen = sizeof(address);
    char buffer[MAX_COMMAND_LENGTH] = {0};

    // Explicitly initialize globals to -1
    server_fd_g = -1;
    network_socket_g = -1;
    kernel_fd_g = -1;

    // Set up signal handler for CTRL+C (SIGINT) and termination signals
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // 1. OPEN KERNEL DEVICE FOR WRITING
    // This requires: insmod walker_driver.ko AND mknod /dev/walker c 61 0
    kernel_fd_g = open(KERNEL_DEVICE_PATH, O_WRONLY);
    if (kernel_fd_g < 0) {
        perror("Error opening kernel device. Did you run 'insmod' and 'mknod'?");
        return -1;
    }
    printf("Kernel Servo Device opened successfully: %s\n", KERNEL_DEVICE_PATH);

    // 2. SETUP NETWORK SERVER (Listener for Laptop)
    // Create socket
    server_fd_g = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(server_fd_g, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt));

    // Configure address
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY; // Listen on all IPs
    address.sin_port = htons(PORT);

    // Bind and Listen
    bind(server_fd_g, (struct sockaddr *)&address, sizeof(address));
    listen(server_fd_g, 3);

    printf("Waiting for Laptop Vision Client to connect on Port %d...\n", PORT);

    // --- Primary Loop: Re-accept connections if client disconnects ---
    while(1) {

        // 3. ACCEPT CONNECTION (BLOCKING)
        // Waits for Laptop to call 192.168.7.2:5000
        network_socket_g = accept(server_fd_g, (struct sockaddr *)&address, (socklen_t*)&addrlen);
        if (network_socket_g < 0) {
            perror("Accept failed");
            // If accept fails, try accepting again
            continue;
        }
        printf("Laptop Vision Client Connected. Starting data bridge.\n");

        // 4. MAIN BRIDGE LOOP: READ FROM NETWORK, WRITE TO KERNEL
        while(1) {
            memset(buffer, 0, sizeof(buffer));
            // Read from the network socket
            int valread = read(network_socket_g, buffer, sizeof(buffer) - 1);

            if (valread > 0) {
                // Write the received string directly to the kernel device
                // This triggers 'dev_write' in the kernel module!
                write(kernel_fd_g, buffer, valread);
                // printf("Received: %s -> Forwarded to Kernel.\n", buffer);
            }

            if (valread == 0) {
                // Client initiated a clean disconnect (hang up)
                printf("Laptop disconnected cleanly. Closing active socket.\n");
                close(network_socket_g);
                network_socket_g = -1;
                break; // Return to the outer while(1) to wait for a new accept()
            }

            if (valread < 0) {
                // Error during read (e.g., connection reset)
                perror("Read error");
                close(network_socket_g);
                network_socket_g = -1;
                break; // Return to the outer while(1) to wait for a new accept()
            }
        }
    }

    clean_shutdown();
    return 0;
}
