// Project headers
#include "config_handler.hpp"
#include "utils.hpp"
#include "wspr_transmit.hpp"
#include "MorseCodeGenerator.hpp"

// C++ Standard Library
#include <array>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstring>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>

// POSIX & System-Specific Headers
#include <termios.h>
#include <unistd.h>

static constexpr std::string_view CALLSIGN = "AA0NT";
static constexpr std::string_view GRID = "EM18";
static constexpr uint8_t POWER_DBM = 20;
static constexpr std::string_view QRSS_MESSAGE = "AA0NT EM18";
static constexpr uint8_t DIT_LENGTH = 3;

// WSPR Frequency choices
static constexpr double WSPR_80M = 3568600.0;
static constexpr double WSPR_40M = 7038600.0;
static constexpr double WSPR_20M = 14095600.0;
static constexpr double WSPR_FREQ = WSPR_80M;

// QRSS Frequency choices
static constexpr double QRSS_80M = 3569900.0;
static constexpr double QRSS_40M = 7039900.0;
static constexpr double QRSS_20M = 14096900.0;
static constexpr double QRSS_FREQ = QRSS_80M;

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

int getch()
{
    struct termios oldAttr, newAttr;
    tcgetattr(STDIN_FILENO, &oldAttr);
    newAttr = oldAttr;
    newAttr.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newAttr);
    int ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldAttr);
    return ch;
}

void wait_for_space_or_signal()
{
    TermiosGuard tguard;
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
            break;
        if (FD_ISSET(STDIN_FILENO, &rfds))
        {
            if (::read(STDIN_FILENO, &c, 1) == 1 && c == ' ')
                break;
        }
    }
}

int select_mode()
{
    TermiosGuard tg;
    std::cout << "Select mode:\n"
              << "  1) WSPR\n"
              << "  2) TONE\n"
              << "  3) QRSS\n"
              << "Enter [1/2/3]: " << std::flush;

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
            std::cout << std::endl;
            return -1;
        }
        if (FD_ISSET(STDIN_FILENO, &rfds))
        {
            char c;
            if (::read(STDIN_FILENO, &c, 1) == 1)
            {
                std::cout << std::endl;
                if (c == '1')
                    return 1;
                if (c == '2')
                    return 2;
                if (c == '3')
                    return 3;
                return -1;
            }
        }
    }
    std::cout << std::endl;
    return -1;
}

void sig_handler(int)
{
    const char msg[] = "Caught signal\nShutting down transmissions.\n";
    write(STDERR_FILENO, msg, sizeof(msg) - 1);
    wsprTransmitter.stop();
    g_terminate.store(true);
    const char wake = 1;
    write(sig_pipe_fds[1], &wake, 1);
}

void start_cb(const std::string &msg, double frequency)
{
    if (!msg.empty() && frequency != 0.0)
    {
        std::cout << "[Callback] Started transmission (" << msg << ") at "
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
                  << std::setprecision(4)
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

    int mode = select_mode();
    if (mode == -1 || g_terminate.load(std::memory_order_acquire))
        return 0;

    config.ppm = get_ppm_from_chronyc();
    wsprTransmitter.setThreadScheduling(SCHED_FIFO, 50);
    wsprTransmitter.setTransmissionCallbacks(start_cb, end_cb);

    if (mode == 1)
    {
        wsprTransmitter.setupTransmission(
            WSPR_FREQ, 0, config.ppm,
            CALLSIGN, GRID, POWER_DBM, true);
    }
    else if (mode == 2)
    {
        wsprTransmitter.setupTransmission(WSPR_FREQ, 0, config.ppm);
    }
    else if (mode == 3)
    {
        std::cout << "QRSS Payload:      '" << QRSS_MESSAGE << "'" << std::endl;
        wsprTransmitter.setSymbolCallback(
            [](char sym, std::chrono::nanoseconds duration)
            {
                std::cout << "[QRSS Symbol] '" << sym
                        << "' duration: " << std::fixed << std::setprecision(3)
                        << duration.count() / 1e6 << "ms." << std::endl;
            });
        MorseCodeGenerator morseTx;
        morseTx.setMessage(QRSS_MESSAGE);
        std::string morseString = morseTx.getMessage();
        wsprTransmitter.setupTransmissionQRSS(morseString, QRSS_FREQ, DIT_LENGTH);
    }

#ifdef DEBUG_WSPR_TRANSMIT
    wsprTransmitter.printParameters();
#endif
    std::cout << "Setup for ";
    if (mode == 1)
        std::cout << "WSPR";
    else if (mode == 2)
        std::cout << "tone";
    else
        std::cout << "QRSS";
    std::cout << " complete." << std::endl;

    if (mode == 1)
    {
        std::cout << "Waiting for next transmission window." << std::endl;
    }
    else
    {
        std::cout << "Press <spacebar> to begin test." << std::endl;
        wait_for_space_or_signal();
    }

    wsprTransmitter.enableTransmission();

    if (mode == 1)
    {
        std::unique_lock<std::mutex> lk(g_end_mtx);
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
        std::cout << "Press <spacebar> to end test." << std::endl;
        wait_for_space_or_signal();
        std::cout << "Test ended." << std::endl;
    }

    wsprTransmitter.stop();
    return 0;
}
