//=======================================
// Objective
//=======================================
// This program will be run on Beagle Bone and acts as a bridge
// The functionality:
// 1. Receives motor commands from laptop over network (TCP)
// 2. Forwards commands to the kernel driver that controls servos
// Data Flow: Laptop Vision → Network → Userspace → Kernel Driver → Servos

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h> // Socket networking
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h> //Signal handling (CTRL+C)

//=======================================
// Configuration
//=======================================
#define PORT 5000 // Define TCP port to listen 
#define KERNEL_DEVICE_PATH "/dev/walker" // Path to kernel device file
#define MAX_COMMAND_LENGTH 32 // Maximum number of input 

//======================================
// Global File Descriptors -> Used for clean shutdown 
//======================================
// Signal handler will use this to perform a clean shutdown 
int server_fd_g = -1; // For listening socket 
int network_socket_g = -1; // Active client connection socket
int kernel_fd_g = -1; // Kernel device file desciptors 


// Clean shutdown function 
// The function will close all connections and files properly 
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

// Signale Hander Function 
// This function handles Ctrl + C to terminate signals 
// Function ensures a clean shutdown when the program is interrupted 
void signal_handler(int signum) {
    printf("\nReceived signal %d. Shutting down bridge...\n", signum);
    clean_shutdown();
    exit(0);
}

int main() {

    //Set up TCP socket variable  
    struct sockaddr_in address;
    int addrlen = sizeof(address);
    char buffer[MAX_COMMAND_LENGTH] = {0};

    // Initialize Global File Descriptor 
    // Set all descriptors to be -1 
    server_fd_g = -1;
    network_socket_g = -1;
    kernel_fd_g = -1;
    
    // Set up signal handler for CTRL+C (SIGINT) and termination signals
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Open Kernel device for writing 
    kernel_fd_g = open(KERNEL_DEVICE_PATH, O_WRONLY);
    if (kernel_fd_g < 0) {
        perror("Error opening kernel device. Did you run 'insmod' and 'mknod'?");
        return -1;
    }
    printf("Kernel Servo Device opened successfully: %s\n", KERNEL_DEVICE_PATH);

    // ============================================
    // Set up Network server
    //=============================================
    // This will make userspace listen to the laptop 
    // Create TCP socket
    // AF_INET = IPv4, SOCK_STREAM = TCP
    server_fd_g = socket(AF_INET, SOCK_STREAM, 0);

    // The reuse socket logic
    // This will allow us to use the recently opened socket without waiting
    int opt = 1; // This is the flag: 1 = enable, 0 = disble 
    setsockopt(server_fd_g, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)); // Buld-in function to activate reuse option: SO_REUSEADDR = reuse address, SO_REUSEPORT = reuse port

    // Configure server address
    address.sin_family = AF_INET; // IPv4
    address.sin_addr.s_addr = INADDR_ANY; // Listen on ALL network interfaces (0.0.0.0)
    address.sin_port = htons(PORT);  // Port 5000 (converted to network byte order)

    // Bind socket to the address (claim the port)
    bind(server_fd_g, (struct sockaddr *)&address, sizeof(address));

    // Start listening for incoming connections
    // 3 = backlog (max queued connections)
    listen(server_fd_g, 3);

    printf("Waiting for Laptop Vision Client to connect on Port %d...\n", PORT);


    // Client Connection Handler
    // The outer loop allows the server to reconnect to the client if the connection is disconnected
    while(1) {

        // Accept the incoming connection 
        // Waits for Laptop to call 192.168.7.2:5000
        network_socket_g = accept(server_fd_g, (struct sockaddr *)&address, (socklen_t*)&addrlen);
        if (network_socket_g < 0) {
            perror("Accept failed");
            // If connection fail, retry 
            continue;
        }
        printf("Laptop Vision Client Connected. Starting data bridge.\n");

        // Continuously read from the network and forward to the kernel
        while(1) {
            // Set up buffer for the data
            memset(buffer, 0, sizeof(buffer));
            
            // Read data from network socket
            // Waits for laptop to send command like "FORWARD:90"
            int valread = read(network_socket_g, buffer, sizeof(buffer) - 1);

            if (valread > 0) {
                // Write the received string directly to the kernel device
                // This triggers 'dev_write' in the kernel module
                write(kernel_fd_g, buffer, valread);
                printf("Received: %s -> Forwarded to Kernel.\n", buffer);
            }

            if (valread == 0) {
                // Client closed the connection normally (EOF)
                // This happens when the laptop program exits or calls close()
                printf("Laptop disconnected cleanly. Closing active socket.\n");
                close(network_socket_g);
                network_socket_g = -1;
                break; // Return to the outer while(1) to wait for a new accept()
            }

            if (valread < 0) {
                // Network error 
                perror("Read error");
                close(network_socket_g);
                network_socket_g = -1;
                break; // Return to the outer while(1) to wait for a new accept()
            }
        }
        // End of inner loop, it will go back to the outer loop
        // Then trying to accept the new connection
    }
    clean_shutdown();
    return 0;
}
