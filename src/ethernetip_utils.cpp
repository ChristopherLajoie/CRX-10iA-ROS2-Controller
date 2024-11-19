#include <iostream>
#include <csignal>
#include <cstring>
#include <sys/select.h>
#include "OpENerInterface.hpp"
extern "C" {
    #include "opener_api.h"
    #include "networkhandler.h"
    #include "cipcommon.h"
}

bool run_application = true;

// Assembly instance IDs (adjust according to your robot's configuration)
#define ASSEMBLY_INPUT_INSTANCE         100  // Input Assembly (produced by adapter, consumed by scanner)
#define ASSEMBLY_OUTPUT_INSTANCE        150  // Output Assembly (produced by scanner, consumed by adapter)
#define ASSEMBLY_CONFIG_INSTANCE        151  // Configuration Assembly
#define HEARTBEAT_INPUT_ONLY_INSTANCE   152  // Heartbeat for Input-Only connections
#define HEARTBEAT_LISTEN_ONLY_INSTANCE  153  // Heartbeat for Listen-Only connections

// Assembly sizes (adjust sizes based on your data requirements)
#define INPUT_ASSEMBLY_SIZE    16
#define OUTPUT_ASSEMBLY_SIZE   16
#define CONFIG_ASSEMBLY_SIZE   0    // Set to zero if not used

// Global variables for assembly data
EipUint8 g_assembly_input_data[INPUT_ASSEMBLY_SIZE];
EipUint8 g_assembly_output_data[OUTPUT_ASSEMBLY_SIZE];
#if CONFIG_ASSEMBLY_SIZE > 0
EipUint8 g_assembly_config_data[CONFIG_ASSEMBLY_SIZE];
#else
EipUint8* g_assembly_config_data = nullptr;
#endif

// Signal handler for graceful shutdown
void SignalHandler(int signal) {
    (void)signal; // Suppress unused parameter warning
    run_application = false;
}

// Application Initialization
EipStatus ApplicationInitialization() {
    std::cout << "Creating Assembly Objects..." << std::endl;
    std::cout << "Input Assembly Instance: " << ASSEMBLY_INPUT_INSTANCE << ", Size: " << sizeof(g_assembly_input_data) << std::endl;
    std::cout << "Output Assembly Instance: " << ASSEMBLY_OUTPUT_INSTANCE << ", Size: " << sizeof(g_assembly_output_data) << std::endl;
    
    CreateAssemblyObject(ASSEMBLY_INPUT_INSTANCE, g_assembly_input_data, sizeof(g_assembly_input_data));
    CreateAssemblyObject(ASSEMBLY_OUTPUT_INSTANCE, g_assembly_output_data, sizeof(g_assembly_output_data));
    
    #if CONFIG_ASSEMBLY_SIZE > 0
    std::cout << "Config Assembly Instance: " << ASSEMBLY_CONFIG_INSTANCE << ", Size: " << sizeof(g_assembly_config_data) << std::endl;
    CreateAssemblyObject(ASSEMBLY_CONFIG_INSTANCE, g_assembly_config_data, sizeof(g_assembly_config_data));
    #else
    std::cout << "Config Assembly Instance: " << ASSEMBLY_CONFIG_INSTANCE << ", Size: 0" << std::endl;
    static EipUint8 dummy_config_data = 0;
    CreateAssemblyObject(ASSEMBLY_CONFIG_INSTANCE, &dummy_config_data, 1); // Pass 1 instead of 0 to avoid NULL data pointer
    #endif
    
    std::cout << "Creating Heartbeat Instances..." << std::endl;
    CreateAssemblyObject(HEARTBEAT_INPUT_ONLY_INSTANCE, &dummy_config_data, 1); // Pass dummy data for heartbeat instances
    CreateAssemblyObject(HEARTBEAT_LISTEN_ONLY_INSTANCE, &dummy_config_data, 1); // Pass dummy data for heartbeat instances

    // Configure Connection Points
    std::cout << "Configuring Connection Points..." << std::endl;
    ConfigureExclusiveOwnerConnectionPoint(0, ASSEMBLY_OUTPUT_INSTANCE, ASSEMBLY_INPUT_INSTANCE, ASSEMBLY_CONFIG_INSTANCE);
    ConfigureInputOnlyConnectionPoint(0, HEARTBEAT_INPUT_ONLY_INSTANCE, ASSEMBLY_INPUT_INSTANCE, ASSEMBLY_CONFIG_INSTANCE);
    ConfigureListenOnlyConnectionPoint(0, HEARTBEAT_LISTEN_ONLY_INSTANCE, ASSEMBLY_INPUT_INSTANCE, ASSEMBLY_CONFIG_INSTANCE);

    return kEipStatusOk;
}

// Handle received data (After data is received on the output assembly)
EipStatus AfterAssemblyDataReceived(CipInstance* instance) {
    if (instance->instance_number == ASSEMBLY_OUTPUT_INSTANCE) {
        // Process data received from the scanner (Fanuc robot)
        // For example, mirror the data to the input assembly
        memcpy(g_assembly_input_data, g_assembly_output_data, sizeof(g_assembly_input_data));
    }
    return kEipStatusOk;
}

// Prepare data before sending (Before data is sent on the input assembly)
EipBool8 BeforeAssemblyDataSend(CipInstance* instance) {
    if (instance->instance_number == ASSEMBLY_INPUT_INSTANCE) {
        // Update the input assembly data if needed
        // For example, increment a counter or update sensor readings
    }
    return true;  // Return true to indicate data is ready to be sent
}

// Handle application logic
void HandleApplication() {
    // Implement your application logic here
    // For example, update the input assembly data periodically
    static EipUint8 counter = 0;
    g_assembly_input_data[0] = counter++;
}

// Handle I/O connection events
void CheckIoConnectionEvent(unsigned int output_assembly_id, unsigned int input_assembly_id, IoConnectionEvent io_connection_event) {
    (void)output_assembly_id; // Suppress unused parameter warning
    (void)input_assembly_id;  // Suppress unused parameter warning

    if (io_connection_event == kIoConnectionEventOpened) {
        std::cout << "I/O connection established." << std::endl;
    } else if (io_connection_event == kIoConnectionEventClosed) {
        std::cout << "I/O connection closed." << std::endl;
    }
}

// Function to handle network events
void CheckAndHandleSockets() {
    fd_set read_fds;
    int max_fd = 0;

    FD_ZERO(&read_fds);

    // Add your sockets to the set
    // For example:
    // FD_SET(tcp_listener_socket, &read_fds);
    // if (tcp_listener_socket > max_fd) max_fd = tcp_listener_socket;

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000; // 100 ms

    int activity = select(max_fd + 1, &read_fds, NULL, NULL, &timeout);

    if (activity < 0 && errno != EINTR) {
        std::cerr << "Select error." << std::endl;
    }

    // Check for activity on your sockets and handle accordingly
    // Accept new connections, read data, etc.
    // Example:
    // if (FD_ISSET(tcp_listener_socket, &read_fds)) {
    //     // Handle new connection
    // }
    // if (FD_ISSET(existing_socket, &read_fds)) {
    //     // Handle incoming data
    //     EipUint8 buffer[512];
    //     int received_data_length = recv(existing_socket, buffer, sizeof(buffer), 0);
    //     if (received_data_length > 0) {
    //         HandleReceivedConnectedData(buffer, received_data_length, &from_address);
    //     }
    // }
}

// Main function
int main(int argc, char* argv[]) {
    (void)argc;
    (void)argv;

    // Register signal handler
    std::signal(SIGINT, SignalHandler);

    // Initialize OpENer
    EipStatus status = CipStackInit(0);
    if (status != kEipStatusOk) {
        std::cerr << "Failed to initialize OpENer." << std::endl;
        return -1;
    }

    // Platform-specific network initialization
    if (NetworkHandlerInitializePlatform() != kEipStatusOk) {
        std::cerr << "Network handler platform initialization failed." << std::endl;
        ShutdownCipStack();
        return -1;
    }

    // Application-specific initialization
    if (ApplicationInitialization() != kEipStatusOk) {
        std::cerr << "Application initialization failed." << std::endl;
        ShutdownCipStack();
        return -1;
    }

    std::cout << "EtherNet/IP adapter running..." << std::endl;

    // Main loop
    while (run_application) {
        // Process network events
        CheckAndHandleSockets();

        // Manage connections
        ManageConnections(100); // Assuming 100 ms elapsed time

        // Handle application-specific tasks
        HandleApplication();
    }

    // Shutdown OpENer
    ShutdownCipStack();

    std::cout << "EtherNet/IP adapter shutting down." << std::endl;

    return 0;
}