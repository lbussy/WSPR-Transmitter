// Project headers
#include "config_handler.hpp"
#include "utils.hpp"
#include "wspr_transmit.hpp"

// C++ Standard Library
#include <array>              // std::array for signal list
#include <chrono>             // std::chrono
#include <condition_variable> // g_end_cv
#include <csignal>            // sigaction, std::signal
#include <cstring>            // strsignal()
#include <cstdio>             // getchar()
#include <iomanip>            // std::ostringstream
#include <iostream>           // std::cout, std::getline
#include <mutex>              // g_end_mtx
#include <string>             // std::string

// POSIX & System-Specific Headers
#include <termios.h> // tcgetattr(), tcsetattr()
#include <unistd.h>  // STDIN_FILENO

static constexpr std::string_view CALLSIGN   = "AA0NT";
static constexpr std::string_view GRID= "EM18";
static constexpr uint8_t POWER_DBM = 20;

// Frequency choices
static constexpr double _2200m = 137500.0;
static constexpr double _160m = 1838100.0;
static constexpr double _80m = 3568600.0;
static constexpr double _60m = 5288700.0;
static constexpr double _40m = 7038600.0;
static constexpr double _30m = 10140200.0;
static constexpr double _22m = 13555400.0;
static constexpr double _20m = 14095600.0;
static constexpr double _17m = 18106100.0;
static constexpr double _15m = 21096100.0;
static constexpr double _12m = 24926100.0;
static constexpr double _10m = 28126100.0;
static constexpr double _6m = 50294500.0;
static constexpr double _test = 25000000.0;

static constexpr double WSPR_FREQ = _40m;

// Thread tracking/execution
static std::mutex g_end_mtx;
static std::condition_variable g_end_cv;
static bool g_transmission_done = false;
static std::atomic<bool> g_terminate{false};

// A self-pipe so main() can wake up out of select() or cv waits if needed
static int sig_pipe_fds[2] = {-1, -1};

// RAII helper to disable/restore canonical mode
struct TermiosGuard
{
    termios old_, cur_;
    TermiosGuard()
    {
        tcgetattr(STDIN_FILENO, &old_);
        cur_ = old_;
        cur_.c_lflag &= ~(ICANON | ECHO);
        cur_.c_cc[VMIN] = 1;
        cur_.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &cur_);
    }

    ~TermiosGuard()
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_);
    }
};

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
    tcgetattr(STDIN_FILENO, &oldAttr); // Get terminal attributes
    newAttr = oldAttr;
    newAttr.c_lflag &= ~(ICANON | ECHO);        // Disable canonical mode and echo
    tcsetattr(STDIN_FILENO, TCSANOW, &newAttr); // Set new terminal attributes
    int ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldAttr); // Restore old terminal attributes
    return ch;
}

/**
 * @brief Pauses the program until the user presses the spacebar.
 *
 * Blocks until either the user hits SPACE, or we get signaled on our pipe.
 */
void wait_for_space_or_signal()
{
    TermiosGuard tguard; // Switch to non-canonical mode for the duration

    char c;
    while (!g_terminate.load(std::memory_order_acquire))
    {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);
        FD_SET(sig_pipe_fds[0], &rfds);
        int nf = std::max(STDIN_FILENO, sig_pipe_fds[0]) + 1;

        if (::select(nf, &rfds, nullptr, nullptr, nullptr) < 0)
        {
            if (errno == EINTR)
                continue;
            break;
        }

        if (FD_ISSET(sig_pipe_fds[0], &rfds))
        {
            // Signal arrived — bail out
            break;
        }
        if (FD_ISSET(STDIN_FILENO, &rfds))
        {
            if (::read(STDIN_FILENO, &c, 1) == 1 && c == ' ')
                break;
        }
    }
}

bool select_wspr()
{
    TermiosGuard tg; // Raw input
    std::cout << "Select mode:\n"
              << "  1) WSPR\n"
              << "  2) TONE\n"
              << "Enter [1/2]: " << std::flush;

    fd_set rfds;
    while (!g_terminate.load())
    {
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);
        FD_SET(sig_pipe_fds[0], &rfds);
        int nf = std::max(STDIN_FILENO, sig_pipe_fds[0]) + 1;

        if (::select(nf, &rfds, nullptr, nullptr, nullptr) < 0)
        {
            std::cout << std::endl;
            if (errno == EINTR)
                continue;
            throw std::runtime_error("select failed");
        }
        if (FD_ISSET(sig_pipe_fds[0], &rfds))
        {
            // Got Ctrl-C
            std::cout << std::endl;
            return false; // Or exit early
        }
        if (FD_ISSET(STDIN_FILENO, &rfds))
        {
            char c;
            if (::read(STDIN_FILENO, &c, 1) == 1)
            {
                std::cout << std::endl;
                if (c == '2')
                    return false;
                else
                    return true; // Default to 1/WSPR
            }
        }
    }
    std::cout << std::endl;
    return false;
}

void sig_handler(int)
{
    const char msg[] = "Caught signal\nShutting down transmissions.\n";
    write(STDERR_FILENO, msg, sizeof(msg) - 1);
    wsprTransmitter.stop();
    g_terminate.store(true);
    // Wake any select()/poll() on your self-pipe
    const char wake = 1;
    write(sig_pipe_fds[1], &wake, 1);
}

/**
 * @brief Print a transmission start message.
 *
 * This callback prints to stdout a notice that transmission has begun.
 * If both a descriptive message and frequency are provided, it prints
 * both. Otherwise it prints whichever is available, or a default notice
 * if neither is provided.
 *
 * @param msg          Transmission descriptor string; may be empty.
 * @param frequency    Frequency in Hz; zero indicates no frequency.
 */
void start_cb(const std::string &msg, double frequency)
{
    if (!msg.empty() && frequency != 0.0)
    {
        std::cout << "[Callback] Started transmission (" << msg << ") "
                  << std::setprecision(6)
                  << (frequency / 1e6) << " MHz."
                  << std::endl;
    }
    else if (frequency != 0.0)
    {
        std::cout << "[Callback] Started transmission: "
                  << std::setprecision(6)
                  << (frequency / 1e6) << " MHz."
                  << std::endl;
    }
    else if (!msg.empty())
    {
        std::cout << "[Callback] Started transmission ("
                  << msg << ")."
                  << std::endl;
    }
    else
    {
        std::cout << "[Callback] Started transmission.\n";
    }
}

/**
 * @brief Callback invoked when a transmission finishes.
 *
 * Prints a completion message to stdout that includes an optional
 * descriptor and the elapsed time in seconds (to three decimal places).
 * Then it sets the global flag `g_transmission_done` to true under lock
 * and notifies the condition variable `g_end_cv`.
 *
 * @param msg     Descriptor string for the transmission; may be empty.
 * @param elapsed Duration of the transmission in seconds; zero indicates
 *                no timing information.
 */
void end_cb(const std::string &msg, double elapsed)
{
    if (!msg.empty() && elapsed != 0.0)
    {
        std::cout << "[Callback] Completed transmission (" << msg << ") "
                  << std::setprecision(3)
                  << elapsed << " seconds."
                  << std::endl;
    }
    else if (elapsed != 0.0)
    {
        std::cout << "[Callback] Completed transmission: "
                  << std::setprecision(3)
                  << elapsed << " seconds."
                  << std::endl;
    }
    else if (!msg.empty())
    {
        std::cout << "[Callback] Completed transmission ("
                  << msg << ")."
                  << std::endl;
    }
    else
    {
        std::cout << "[Callback] Completed transmission." << std::endl;
    }

    {
        std::lock_guard<std::mutex> lk(g_end_mtx);
        g_transmission_done = true;
    }
    g_end_cv.notify_one();
}

int main()
{
    if (::pipe(sig_pipe_fds) < 0)
    {
        perror("pipe");
        return 1;
    }

    // Set up signals
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

    // Pick mode
    bool isWspr = select_wspr();
    if (g_terminate.load(std::memory_order_acquire))
    {
        // We saw Ctrl-C in the prompt, so bail out immediately
        return 0;
    }
    std::cout << "Mode selected: " << (isWspr ? "WSPR" : "TONE") << std::endl;

    // Get PPM correction and schedule priority
    config.ppm = get_ppm_from_chronyc();

    // Set transmission server and set priority
    wsprTransmitter.setThreadScheduling(SCHED_FIFO, 50);

    // Configure transmitter
    if (isWspr)
    {
        wsprTransmitter.setTransmissionCallbacks(
            [](const std::string &msg, double frequency)
            {
                start_cb(msg, frequency);
            },

            [](const std::string &msg, double elapsed_secs)
            {
                end_cb(msg, elapsed_secs);
            });

        // Set minimum transmission data
        wsprTransmitter.setupTransmission(
            WSPR_FREQ, 0, config.ppm,
            CALLSIGN, GRID, POWER_DBM, /*use_offset=*/true);
    }
    else
    {
        wsprTransmitter.setupTransmission(WSPR_FREQ, 0, config.ppm);
    }

#ifdef DEBUG_WSPR_TRANSMIT
    wsprTransmitter.printParameters();
#endif
    std::cout << "Setup for " << (isWspr ? "WSPR" : "tone") << " complete." << std::endl;

    // For tone mode, wait to start
    if (isWspr)
    {
        std::cout << "Waiting for next transmission window." << std::endl;
    }
    else
    {
        std::cout << "Press <spacebar> to begin test tone." << std::endl;
        wait_for_space_or_signal();
    }

    // Kick off the scheduler/transmission thread
    wsprTransmitter.enableTransmission();

    if (isWspr)
    {
        std::unique_lock<std::mutex> lk(g_end_mtx);

        // Wake every 100 ms to check for either completion or a Ctrl-C.
        while (!g_transmission_done &&
               !g_terminate.load(std::memory_order_acquire))
        {
            g_end_cv.wait_for(lk, std::chrono::milliseconds(100));
        }

        if (g_transmission_done)
        {
            std::cout << "WSPR transmission complete." << std::endl;
        }
        else
        {
            std::cout << "Interrupted. Aborting WSPR transmission." << std::endl;
        }
    }
    else
    {
        // Tone mode: Stop on spacebar
        std::cout << "Press <spacebar> to end test tone." << std::endl;
        wait_for_space_or_signal();
        std::cout << "Test tone stopped." << std::endl;
    }

    // Teardown
    wsprTransmitter.stop();
    return 0;
}
