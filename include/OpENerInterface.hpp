#ifndef OPENERINTERFACE_HPP
#define OPENERINTERFACE_HPP

#include <iostream>
#include <csignal>
#include <cstring>
extern "C" {
    #include "opener_api.h"
    #include "networkhandler.h"
    #include "cipcommon.h"
}

// Assembly instance IDs (adjust according to your robot's configuration)
#define ASSEMBLY_INPUT_INSTANCE         100  // Input Assembly (produced by adapter, consumed by scanner)
#define ASSEMBLY_OUTPUT_INSTANCE        150  // Output Assembly (produced by scanner, consumed by adapter)
#define ASSEMBLY_CONFIG_INSTANCE        151  // Configuration Assembly
#define HEARTBEAT_INPUT_ONLY_INSTANCE   152  // Heartbeat for Input-Only connections
#define HEARTBEAT_LISTEN_ONLY_INSTANCE  153  // Heartbeat for Listen-Only connections

// Assembly sizes (adjust sizes based on your data requirements)
#define INPUT_ASSEMBLY_SIZE    32
#define OUTPUT_ASSEMBLY_SIZE   32
#define CONFIG_ASSEMBLY_SIZE   0    // Set to zero if not used

class OpENerInterface {
public:
    OpENerInterface();
    ~OpENerInterface();

    // Initializes the application (e.g., creates assemblies and sets up connections)
    EipStatus ApplicationInitialization();

    // Processes network events for the adapter
    void processNetworkEvents();

    // Handles IO connection events
    void CheckIoConnectionEvent(unsigned int output_assembly_id,
                                unsigned int input_assembly_id,
                                IoConnectionEvent io_connection_event);

    // Handles received data for assemblies
    EipStatus AfterAssemblyDataReceived(CipInstance* instance);

    // Prepares data before sending
    EipBool8 BeforeAssemblyDataSend(CipInstance* instance);

    // Contains the main application logic
    void HandleApplication();

    // Runs the adapter application
    int run(int argc, char* argv[]);

private:
    // Signal handler for graceful shutdown
    static void signalHandler(int signum);

    static bool run_application;  // Flag to control the main loop

    // Assembly data
    static EipUint8 g_input_assembly_data[INPUT_ASSEMBLY_SIZE];
    static EipUint8 g_output_assembly_data[OUTPUT_ASSEMBLY_SIZE];
#if CONFIG_ASSEMBLY_SIZE > 0
    static EipUint8 g_config_assembly_data[CONFIG_ASSEMBLY_SIZE];
#else
    static EipUint8* g_config_assembly_data;
#endif
};

#endif  // OPENERINTERFACE_HPP