// Project Headers
#include "config_handler.hpp"
#include "utils.hpp"
#include "wspr_transmit.hpp"

// Submodule Headers

// Standard C++ Headers
#include <atomic>
#include <array>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <csignal>
#include <ctime>
#include <iostream>
#include <thread>

#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <sys/select.h>

// at file scope
static std::mutex g_end_mtx;
static std::condition_variable g_end_cv;
static bool g_transmission_done = false;

/**
 * @brief Reads a single character from standard input without waiting for Enter.
 *
 * This function configures the terminal for noncanonical mode to read a single
 * character immediately. It then restores the terminal settings.
 *
 * @return The character read from standard input.
 */
int getch()
{
    struct termios oldAttr, newAttr;
    tcgetattr(STDIN_FILENO, &oldAttr); // get terminal attributes
    newAttr = oldAttr;
    newAttr.c_lflag &= ~(ICANON | ECHO);        // disable canonical mode and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newAttr); // set new terminal attributes
    int ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldAttr); // restore old terminal attributes
    return ch;
}

/**
 * @brief Pauses the program until the user presses the spacebar.
 */
void pause_for_space()
{
    extern WsprTransmitter wsprTransmitter;

    while (!wsprTransmitter.isStopping() && getch() != ' ')
        ; // spin until stop or spacebar
}

bool select_wspr()
{
    std::string input;

    std::cout << "Select mode:\n";
    std::cout << "  1) WSPR\n";
    std::cout << "  2) TONE\n";
    std::cout << "Enter choice [1-2, default 1]: ";
    std::getline(std::cin, input);

    int choice = 1;
    try
    {
        if (!input.empty())
        {
            choice = std::stoi(input);
        }
    }
    catch (...)
    {
        choice = 1; // fallback to default
    }
    return (choice == 1);
}

void sig_handler(int sig = SIGTERM)
{
    // Called when exiting or when a signal is received.
    std::cout << "Caught signal: " << sig << " (" << strsignal(sig) << ")." << std::endl;
    std::cout << "Shutting down transmissions." << std::endl;
    wsprTransmitter.shutdownTransmitter();
    exit(EXIT_SUCCESS);
}

void start_cb()
{
    std::cout << "[CALLBACK] Started transmission." << std::endl;
}

void end_cb()
{
    std::cout << "[CALLBACK] Completed transmission." << std::endl;
    {
        std::lock_guard<std::mutex> lk(g_end_mtx);
        g_transmission_done = true;
    }
    g_end_cv.notify_one();
}

int main()
{
    std::array<int, 6> signals = {SIGINT, SIGTERM, SIGHUP, SIGUSR1, SIGUSR2, SIGQUIT};
    for (int sig : signals)
    {
        struct sigaction sa{};
        sa.sa_handler = sig_handler;
        sigaction(sig, &sa, nullptr);
    }
    std::signal(SIGCHLD, SIG_IGN);

    bool isWspr = select_wspr();
    std::cout << "Mode selected: " << (isWspr ? "WSPR" : "TONE") << std::endl;

    // Get adjustments based on PPM
    config.ppm = get_ppm_from_chronyc();

    wsprTransmitter.setThreadScheduling(SCHED_FIFO, 30);

    if (isWspr)
    {
        wsprTransmitter.setTransmissionCallbacks(start_cb, end_cb);
        wsprTransmitter.setupTransmission(7040100.0, 0, config.ppm, "AA0NT", "EM18", 20, true);
    }
    else
    {
        wsprTransmitter.setupTransmission(7040100.0, 0, config.ppm);
    }

    // Print transmission parameters
    wsprTransmitter.printParameters();

    std::cout << "Setup for " << (isWspr ? "WSPR" : "tone") << " complete." << std::endl;
    if (isWspr)
    {
        ;
    }
    else
    {
        std::cout << "Press <spacebar> to begin transmission." << std::endl;
        pause_for_space();
    }

    // Execute transmission
    wsprTransmitter.enableTransmission();

    if (isWspr)
    {
        std::cout << "Waiting for end of WSPR transmission (press space to abort)..." << std::endl;
        std::unique_lock<std::mutex> lk(g_end_mtx);
        // this will block until end_cb() sets g_transmission_done == true
        g_end_cv.wait(lk, []{ return g_transmission_done; });
        std::cout << "WSPR transmission complete." << std::endl;
    }
    else
    {
        std::cout << "Press <spacebar> to end." << std::endl;
        pause_for_space(); // Or some other user action
        std::cout << "Stopping." << std::endl;
    }

    wsprTransmitter.shutdownTransmitter();

    return 0;
}
