// Project headers
#include "config_handler.hpp"
#include "utils.hpp"
#include "wspr_transmit.hpp"

// C++ Standard Library
#include <array>               // std::array for signal list
#include <condition_variable>  // g_end_cv
#include <csignal>             // sigaction, std::signal
#include <cstring>             // strsignal()
#include <cstdio>              // getchar()
#include <iostream>            // std::cout, std::getline
#include <mutex>               // g_end_mtx
#include <string>              // std::string

// POSIX & System-Specific Headers
#include <termios.h>           // tcgetattr(), tcsetattr()
#include <unistd.h>            // STDIN_FILENO

// at file scope
static std::mutex g_end_mtx;
static std::condition_variable g_end_cv;
static bool g_transmission_done = false;
static std::atomic<bool> g_terminate{false};

// A self-pipe so main() can wake up out of select() or cv waits if needed
static int sig_pipe_fds[2] = {-1, -1};

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
    while (getch() != ' ')
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

    // Thread safe write() rather than std::cout
    const char msg1[] = "Caught signal\n";
    write(STDERR_FILENO, msg1, sizeof(msg1) - 1);
    const char msg2[] = "Shutting down transmissions.\n";
    write(STDERR_FILENO, msg2, sizeof(msg2) - 1);
    wsprTransmitter.shutdownTransmitter();

    // Set our “please quit” flag
    g_terminate.store(true, std::memory_order_release);

    // Write a byte to our self-pipe so any select()/poll()/cv_wait can wake
    // up (write() is async-signal-safe; std::cout and strsignal() are not.)
    const char wake = 1;
    ::write(sig_pipe_fds[1], &wake, 1);
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
    // — set up signals —
    std::array<int, 6> signals = {SIGINT, SIGTERM, SIGHUP, SIGUSR1, SIGUSR2, SIGQUIT};
    for (int s : signals)
    {
        struct sigaction sa{};
        sa.sa_handler = sig_handler;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = 0;
        sigaction(s, &sa, nullptr);
    }
    std::signal(SIGCHLD, SIG_IGN);

    // — pick mode —
    bool isWspr = select_wspr();
    std::cout << "Mode selected: " << (isWspr ? "WSPR" : "TONE") << "\n";

    // — get PPM correction and schedule priority —
    config.ppm = get_ppm_from_chronyc();
    wsprTransmitter.setThreadScheduling(SCHED_FIFO, 30);

    // — configure transmitter —
    if (isWspr)
    {
        wsprTransmitter.setTransmissionCallbacks(start_cb, end_cb);
        wsprTransmitter.setupTransmission(
            7040100.0, 0, config.ppm,
            "AA0NT", "EM18", 20, /*use_offset=*/true);
    }
    else
    {
        wsprTransmitter.setupTransmission(7040100.0, 0, config.ppm);
    }

    wsprTransmitter.printParameters();
    std::cout << "Setup for " << (isWspr ? "WSPR" : "tone") << " complete.\n";

    // — for tone mode, wait to start —
    if (!isWspr)
    {
        std::cout << "Press <spacebar> to begin test tone.\n";
        pause_for_space();
    }

    // — kick off the scheduler/transmission thread —
    wsprTransmitter.enableTransmission();

    if (isWspr)
    {
        // block until end_cb() fires
        std::unique_lock<std::mutex> lk(g_end_mtx);
        g_end_cv.wait(lk, []
                      { return g_transmission_done; });
        std::cout << "WSPR transmission complete.\n";
    }
    else
    {
        // tone mode: stop on spacebar
        std::cout << "Press <spacebar> to end test tone.\n";
        pause_for_space();
        std::cout << "Test tone stopped.\n";
    }

    // — teardown —
    wsprTransmitter.shutdownTransmitter();
    return 0;
}
