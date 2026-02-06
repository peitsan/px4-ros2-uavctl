#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <string>
#include <vector>

void runCommandInNewTab(const std::string &command) {
    std::string fullCommand = "gnome-terminal --tab -- bash -c \"" + command + "; exec bash\"";
    std::system(fullCommand.c_str());  // Run the command in a new terminal tab
}

int main() {
    // List of commands to run
    std::vector<std::string> commands = {
        // Run the Micro XRCE-DDS Agent
        "MicroXRCEAgent udp4 -p 8888",
        
        // Run the PX4 SITL simulation
       "cd /home/ubuntu/PX4-Autopilot && make px4_sitl gz_x500_depth"
        
        // Run QGroundControl (uncomment if needed)
        // "cd ~/QGroundControl && ./QGroundControl.AppImage"
    };

    // Loop through each command in the list and run them in new terminal tabs
    for (const auto &command : commands) {
        runCommandInNewTab(command);
        sleep(1);  // Sleep 1 second between commands
    }

    return 0;
}

