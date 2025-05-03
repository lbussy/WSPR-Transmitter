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

// RAII helper to disable/restore canonical mode
struct TermiosGuard
{
    termios old_, cur_;
    TermiosGuard()
    {
        tcgetattr(STDIN_FILENO, &old_);
        cur_ = old_;
        cur_.c_lflag &= ~(ICANON|ECHO);
        cur_.c_cc[VMIN]  = 1;
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
 *
 * Blocks until either the user hits SPACE, or we get signaled on our pipe.
 */
void wait_for_space_or_signal()
{
    TermiosGuard tguard;  // switch to non-canonical mode for the duration

    char c;
    while (!g_terminate.load(std::memory_order_acquire))
    {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);
        FD_SET(sig_pipe_fds[0],   &rfds);
        int nf = std::max(STDIN_FILENO, sig_pipe_fds[0]) + 1;

        if (::select(nf, &rfds, nullptr, nullptr, nullptr) < 0)
        {
            if (errno == EINTR) continue;
            break;
        }

        if (FD_ISSET(sig_pipe_fds[0], &rfds))
        {
            // signal arrived — bail out
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
    TermiosGuard tg;  // raw input
    std::cout << "Select mode:\n"
              << "  1) WSPR\n"
              << "  2) TONE\n"
              << "Enter [1/2]: " << std::flush;

    fd_set rfds;
    while (!g_terminate.load()) {
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);
        FD_SET(sig_pipe_fds[0],   &rfds);
        int nf = std::max(STDIN_FILENO, sig_pipe_fds[0]) + 1;

        if (::select(nf, &rfds, nullptr, nullptr, nullptr) < 0) {
            if (errno==EINTR) continue;
            throw std::runtime_error("select failed");
        }
        if (FD_ISSET(sig_pipe_fds[0], &rfds)) {
            // got Ctrl-C
            return false;  // or exit early
        }
        if (FD_ISSET(STDIN_FILENO, &rfds)) {
            char c;
            if (::read(STDIN_FILENO, &c, 1)==1) {
                if (c=='2') return false;
                else           return true;  // default to 1/WSPR
            }
        }
    }
    return false;
}

void sig_handler(int)
{
    const char msg[] = "Caught signal\nShutting down transmissions.\n";
    write(STDERR_FILENO, msg, sizeof(msg)-1);
    wsprTransmitter.shutdownTransmitter();
    g_terminate.store(true);
    // wake any select()/poll() on your self-pipe
    const char wake = 1;
    write(sig_pipe_fds[1], &wake, 1);
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
    if (::pipe(sig_pipe_fds) < 0) {
        perror("pipe");
        return 1;
    }

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
    if (g_terminate.load(std::memory_order_acquire))
    {
        // we saw Ctrl-C in the prompt, so bail out immediately
        return 0;
    }
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
        wait_for_space_or_signal();
    }

    // — kick off the scheduler/transmission thread —
    wsprTransmitter.enableTransmission();

    if (isWspr)
    {
        std::unique_lock<std::mutex> lk(g_end_mtx);
        // wake every 100 ms to check for either completion or a Ctrl-C
        while (!g_transmission_done && !g_terminate.load(std::memory_order_acquire))
        {
            g_end_cv.wait_for(lk, std::chrono::milliseconds(100));
        }
        if (g_transmission_done)
        {
            std::cout << "WSPR transmission complete.\n";
        }
        else
        {
            std::cout << "Interrupted. Aborting WSPR transmission.\n";
        }
    }
    else
    {
        // tone mode: stop on spacebar
        std::cout << "Press <spacebar> to end test tone.\n";
        wait_for_space_or_signal();
        std::cout << "Test tone stopped.\n";
    }

    // — teardown —
    wsprTransmitter.shutdownTransmitter();
    return 0;
}
