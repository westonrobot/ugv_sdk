#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <iostream>

#include "monitor/scout_monitor.hpp"

using namespace westonrobot;

ScoutMonitor monitor;

void SignalHandler(int s)
{
    printf("Caught signal %d\n", s);
    monitor.Terminate();
    exit(1);
}

int main(int argc, char **argv)
{
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = SignalHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    std::cout << "scout monitor started" << std::endl;
    monitor.Run("can1");

    return 0;
}