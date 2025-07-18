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
static constexpr std::string_view QRSS_MESSAGE = "AA0NT";
static constexpr uint8_t DIT_LENGTH = 1;
static constexpr uint8_t GPIO_POWER = 7;

// WSPR Frequency choices
static constexpr double WSPR_80M = 3568600.0;
static constexpr double WSPR_40M = 7038600.0;
static constexpr double WSPR_20M = 14095600.0;
static constexpr double WSPR_FREQ = WSPR_20M;

// QRSS Frequency choices
static constexpr double QRSS_80M = 3569900.0;
static constexpr double QRSS_40M = 7039900.0;
static constexpr double QRSS_20M = 14096900.0;
static constexpr double QRSS_FREQ = QRSS_20M;

// FSKCW/DRCW
static constexpr double OFFSET = 100.0;

// FSKCW/DRCW
static constexpr double OFFSET = 50.0;

// Debug print tag
inline constexpr std::string_view main_debug_tag{"[Test Rig        ] "};

// Thread tracking/execution
static std::mutex g_end_mtx;
static std::condition_variable g_end_cv;
static bool g_transmission_done = false;
static std::atomic<bool> g_terminate{false};

// A self-pipe so main() can wake up out of select() or cv waits if needed
static int sig_pipe_fds[2] = {-1, -1};

#include <termios.h>
#include <unistd.h>

/**
 * @class TermiosGuard
 * @brief RAII class for managing terminal mode settings.
 *
 * @details This class disables canonical mode and echo on the terminal
 *          associated with the specified file descriptor (default is standard
 *          input). It is useful for reading input one character at a time
 *          without requiring the Enter key and without displaying typed
 *          characters.
 *
 *          Upon destruction, the original terminal settings are restored,
 *          ensuring the terminal is returned to its previous state even in
 *          the presence of exceptions or early scope exits.
 *
 * @note This class is intended for use as a local stack object. It does not
 *       throw exceptions. Any terminal errors are silently ignored.
 *
 * @example
 * {
 *     TermiosGuard guard;
 *     char c;
 *     ::read(STDIN_FILENO, &c, 1);  // Reads a single character without echo
 * } // Terminal settings automatically restored here
 */
class TermiosGuard
{
public:
    /**
     * @brief Constructs the guard and sets the terminal to non-canonical mode.
     *
     * @param fd The terminal file descriptor to modify (default: STDIN_FILENO).
     */
    explicit TermiosGuard(int fd = STDIN_FILENO)
        : fd_(fd)
    {
        if (tcgetattr(fd_, &old_) == 0)
        {
            cur_ = old_;
            cur_.c_lflag &= ~(ICANON | ECHO);
            cur_.c_cc[VMIN] = 1;
            cur_.c_cc[VTIME] = 0;
            tcsetattr(fd_, TCSANOW, &cur_);
        }
    }

    /**
     * @brief Restores the original terminal settings.
     */
    ~TermiosGuard() noexcept
    {
        tcsetattr(fd_, TCSANOW, &old_);
    }

private:
    int fd_;        ///< File descriptor for terminal
    termios old_{}; ///< Saved original terminal settings
    termios cur_{}; ///< Modified terminal settings
};

#include <termios.h>
#include <unistd.h>
#include <cstdio>

/**
 * @class Getch
 * @brief Provides a safe, RAII-style interface for reading a single character
 *        from the terminal without waiting for Enter and without echoing it.
 *
 * This class disables canonical mode and echoing on construction and restores
 * the original terminal settings upon destruction. It is useful for interactive
 * terminal applications that require immediate, single-character input.
 *
 * Usage:
 * @code
 * Getch g;
 * int ch = g.read();
 * @endcode
 */
class Getch
{
public:
    /**
     * @brief Constructs the Getch object and configures the terminal.
     *
     * Disables canonical input and echoing. The previous terminal settings
     * are saved and restored when the object is destroyed.
     */
    Getch()
    {
        // Save the current terminal settings
        tcgetattr(STDIN_FILENO, &old_);
        new_ = old_;

        // Disable canonical mode and echo
        new_.c_lflag &= ~(ICANON | ECHO);

        // Apply the new settings immediately
        tcsetattr(STDIN_FILENO, TCSANOW, &new_);
    }

    /**
     * @brief Restores the original terminal settings.
     */
    ~Getch() noexcept
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_);
    }

    /**
     * @brief Reads a single character from the terminal.
     *
     * @return The character read, as an int.
     */
    int read() const
    {
        return getchar();
    }

private:
    struct termios old_; ///< Original terminal settings
    struct termios new_; ///< Modified terminal settings
};

/**
 * @brief Waits for a spacebar press or external signal with timeout.
 * @details Uses select() to wait on STDIN or signal pipe with a timeout. If a
 *          space character is received or a signal pipe event occurs, returns true.
 *
 * @param timeout_ms Maximum number of milliseconds to wait before returning false.
 * @return true if spacebar was pressed or signal received; false on timeout.
 */
bool wait_for_space_or_signal(int timeout_ms)
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

WsprTransmitter::Mode select_mode()
{
    TermiosGuard tg;
    std::cout << "Select mode:" << std::endl;
    std::cout << "\t1) WSPR" << std::endl;
    std::cout << "\t2) QRSS" << std::endl;
    std::cout << "\t3) FSKCW" << std::endl;
    std::cout << "\t4) TONE" << std::endl;
    std::cout << "Enter [1-4]: " << std::endl << std::flush;

    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(STDIN_FILENO, &rfds);
    FD_SET(sig_pipe_fds[0], &rfds);
    int nf = std::max(STDIN_FILENO, sig_pipe_fds[0]) + 1;

    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int result = ::select(nf, &rfds, nullptr, nullptr, &tv);
    if (result < 0)
    {
        if (errno == EINTR)
            return false;
        return true; // Treat unknown error as an exit condition
    }

    if (FD_ISSET(sig_pipe_fds[0], &rfds))
        return true;

    if (FD_ISSET(STDIN_FILENO, &rfds))
    {
        if (::read(STDIN_FILENO, &c, 1) == 1 && c == ' ')
            return true;
    }

    return false; // Timeout or irrelevant key
}

/**
 * @brief Prompts the user to select a transmission mode.
 *
 * Displays a menu and waits for a single keypress (1–5) to select a mode.
 * Also listens for termination signals using a pipe file descriptor.
 *
 * @return Selected WsprTransmitter::Mode value, or UNKNOWN on error or termination.
 */
WsprTransmitter::Mode select_mode()
{
    Getch g;
    std::cout << "Select mode:\n"
              << "\t1) TONE\n"
              << "\t2) WSPR\n"
              << "\t3) QRSS\n"
              << "\t4) FSKCW\n"
              << "\t5) DFCW\n"
              << "Enter [1-5]: " << std::flush;

    fd_set rfds;
    while (!g_terminate.load(std::memory_order_acquire))
    {
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);
        FD_SET(sig_pipe_fds[0], &rfds);
        int nfds = std::max(STDIN_FILENO, sig_pipe_fds[0]) + 1;

        if (::select(nfds, &rfds, nullptr, nullptr, nullptr) < 0)
        {
            if (errno == EINTR)
                continue;
            std::cout << std::endl;
            throw std::runtime_error("select failed");
        }

        // Signal received
        if (FD_ISSET(sig_pipe_fds[0], &rfds))
        {
            std::cout << std::endl;
            return WsprTransmitter::Mode::UNKNOWN;
        }

        // User input available
        if (FD_ISSET(STDIN_FILENO, &rfds))
        {
            char c;
            if (::read(STDIN_FILENO, &c, 1) == 1)
            {
                std::cout << std::endl;
                if (c == '1')
                    return WsprTransmitter::Mode::WSPR;
                if (c == '2')
                    return WsprTransmitter::Mode::QRSS;
                if (c == '3')
                    return WsprTransmitter::Mode::FSKCW;
                if (c == '4')
                    return WsprTransmitter::Mode::TONE;
                return WsprTransmitter::Mode::UNKNOWN;
            }
        }
    }

    std::cout << std::endl;
    return WsprTransmitter::Mode::UNKNOWN;
}

/**
 * @brief Signal handler for graceful shutdown.
 *
 * @details This function is triggered when a registered signal (e.g., SIGINT)
 *          is received. It stops any ongoing transmission, sets the termination
 *          flag, and wakes any thread blocked on `select()` via the signal pipe.
 *
 * @param signal The caught signal (unused but required by the signature).
 *
 * @note This function is safe for use as a signal handler.
 */
void sig_handler(int signal)
{
    // Write directly to STDERR (async-signal-safe)
    const char msg[] =
        "[Signal Handler  ] Caught signal\n"
        "[Signal Handler  ] Shutting down transmissions.\n";
    write(STDERR_FILENO, msg, sizeof(msg) - 1);

    // Stop the transmitter (safe only if .stop() is signal-safe)
    wsprTransmitter.stop();

    // Set termination flag for main loop and threads
    g_terminate.store(true);

    // Wake any thread blocked on select()
    const char wake = 1;
    write(sig_pipe_fds[1], &wake, 1);

    // `signal` is unused, silence potential compiler warnings
    (void)signal;
}

/**
 * @brief Callback to announce the start of a transmission.
 *
 * @details Outputs a message to indicate the start of a transmission,
 *          optionally including a description (`msg`) and a frequency in Hz.
 *          If both are provided, the message will include the transmission
 *          label and the frequency formatted in MHz.
 *
 * @param msg       A description of the transmission (optional).
 * @param frequency Transmission frequency in Hz (optional).
 *
 * @note If both parameters are omitted or zero, a generic message is printed.
 */
void start_cb(const std::string &msg, double frequency)
{
    // Both message and frequency are provided
    if (!msg.empty() && frequency != 0.0)
    {
        std::cout << main_debug_tag
                  << "Started transmission (" << msg << ") at "
                  << std::fixed << std::setprecision(6)
                  << (frequency / 1e6) << " MHz."
                  << std::endl;
    }
    // Only frequency is provided
    else if (frequency != 0.0)
    {
        std::cout << main_debug_tag
                  << "Started transmission: "
                  << std::fixed << std::setprecision(6)
                  << (frequency / 1e6) << " MHz."
                  << std::endl;
    }
    // Only message is provided
    else if (!msg.empty())
    {
        std::cout << main_debug_tag
                  << "Started transmission (" << msg << ")."
                  << std::endl;
    }
    // Neither message nor frequency is provided
    else
    {
        std::cout << main_debug_tag
                  << "Started transmission."
                  << std::endl;
    }
}

/**
 * @brief Callback to mark the end of a transmission.
 *
 * @details Prints a formatted message indicating the completion of a transmission,
 *          including an optional label (`msg`) and the elapsed time in seconds.
 *          Signals any threads waiting on the end-of-transmission condition.
 *
 * @param msg     A string describing the completed transmission (optional).
 * @param elapsed Duration in seconds of the transmission (optional).
 *
 * @note If both `msg` and `elapsed` are empty/zero, a generic message is printed.
 */
void end_cb(const std::string &msg, double elapsed)
{
    // Format the output depending on which values are provided
    if (!msg.empty() && elapsed != 0.0)
    {
        std::cout << main_debug_tag
                  << "Completed transmission (" << msg << ") "
                  << std::fixed << std::setprecision(3)
                  << elapsed << " seconds."
                  << std::endl;
    }
    else if (elapsed != 0.0)
    {
        std::cout << main_debug_tag
                  << "Completed transmission: "
                  << std::fixed << std::setprecision(4)
                  << elapsed << " seconds."
                  << std::endl;
    }
    else if (!msg.empty())
    {
        std::cout << main_debug_tag
                  << "Completed transmission (" << msg << ")."
                  << std::endl;
    }
    else
    {
        std::cout << main_debug_tag
                  << "Completed transmission." << std::endl;
    }

    // Notify waiting threads that the transmission has ended
    {
        std::lock_guard<std::mutex> lk(g_end_mtx);
        g_transmission_done = true;
    }
    g_end_cv.notify_one();
}

/**
 * @brief Symbol callback function for logging Morse symbols and durations
 *
 * @param mode The current transmission mode
 * @param sym The character symbol being transmitted
 * @param duration The duration of the symbol as a std::chrono::nanoseconds value
 */
void sym_cb(char sym, std::chrono::nanoseconds duration, WsprTransmitter::Mode mode)
{
    std::cout << main_debug_tag
              << wsprTransmitter.modeToString(mode)
              << " Symbol: '"
              << sym
              << "' duration: "
              << std::fixed << std::setprecision(3)
              << duration.count() / 1e6
              << "ms." << std::endl;
}

/**
 * @brief Main entry point for WSPR and CW signal transmission modes.
 *
 * @details
 * Initializes signal handlers, prompts for mode selection, configures the
 * `WsprTransmitter` instance, and manages transmission control based on
 * user input or signal interruption.
 *
 * Transmission modes supported:
 * - TONE: Continuous tone output
 * - WSPR: Weak Signal Propagation Reporter
 * - QRSS: Slow Morse code
 * - FSKCW: Frequency-shift keyed CW
 * - DFCW: Dual-frequency CW
 *
 * @return Exit code: 0 on success, 1 on failure
 */
int main()
{
    // Create a pipe to allow signal-safe wake-up of blocking calls
    if (::pipe(sig_pipe_fds) < 0)
    {
        perror("pipe");
        return 1;
    }

    // Register signal handler for graceful shutdown
    std::array<int, 6> signals = {SIGINT, SIGTERM, SIGHUP, SIGUSR1, SIGUSR2, SIGQUIT};
    for (int s : signals)
    {
        struct sigaction sa{};
        sa.sa_handler = sig_handler;
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = 0;
        sigaction(s, &sa, nullptr);
    }

    // Ignore SIGCHLD to avoid zombie children
    std::signal(SIGCHLD, SIG_IGN);

    // Prompt user for transmission mode
    WsprTransmitter::Mode mode = select_mode();
    if (mode == WsprTransmitter::Mode::UNKNOWN || g_terminate.load(std::memory_order_acquire))
        return 0;

    // Apply PPM correction if available
    config.ppm = get_ppm_from_chronyc();

    // Set scheduling and callbacks
    wsprTransmitter.setThreadScheduling(SCHED_FIFO, 50);
    wsprTransmitter.setTransmissionCallbacks(start_cb, end_cb);
    wsprTransmitter.setSymbolCallback(sym_cb);

    // Configure selected mode
    if (mode == WsprTransmitter::Mode::TONE)
    {
        wsprTransmitter.setupToneTransmission(WSPR_FREQ, config.ppm, GPIO_POWER);
    }
    else if (mode == WsprTransmitter::Mode::WSPR)
    {
        wsprTransmitter.setupWSPRTransmission(
            WSPR_FREQ, GPIO_POWER, config.ppm,
            CALLSIGN, GRID, POWER_DBM, true);
    }
    else
    {
        MorseCodeGenerator morseTx;
        morseTx.setMessage(QRSS_MESSAGE);
        std::string morseString = morseTx.getMessage();
        wsprTransmitter.setupQRSSTransmission(morseString, QRSS_FREQ, DIT_LENGTH, config.ppm, GPIO_POWER);
    }
    else if (mode == WsprTransmitter::Mode::FSKCW)
    {
        std::cout << main_debug_tag << "FSKCW Payload:     '" << QRSS_MESSAGE << "'" << std::endl;
        wsprTransmitter.setSymbolCallback(
            [](char sym, std::chrono::nanoseconds duration)
            {
                std::cout << main_debug_tag
                          << "Symbol: '"
                          << sym
                          << "' duration: " << std::fixed << std::setprecision(3)
                          << duration.count() / 1e6 << "ms." << std::endl;
            });
        MorseCodeGenerator morseTx;
        morseTx.setMessage(QRSS_MESSAGE);
        std::string morseString = morseTx.getMessage();
        wsprTransmitter.setupFSKCWTransmission(morseString, QRSS_FREQ, OFFSET, DIT_LENGTH, config.ppm, GPIO_POWER);
    }
    else if (mode == WsprTransmitter::Mode::TONE)
    {
        wsprTransmitter.setupToneTransmission(WSPR_FREQ, GPIO_POWER, config.ppm);
    }

    // Display configuration
    wsprTransmitter.printParameters();
#endif
    std::cout << main_debug_tag << "Setup for ";
    if (mode == WsprTransmitter::Mode::WSPR)
        std::cout << "WSPR";
    else if (mode == WsprTransmitter::Mode::QRSS)
        std::cout << "QRSS";
    else if (mode == WsprTransmitter::Mode::FSKCW)
        std::cout << "FSKCW";
    else
        std::cout << "TONE";
    std::cout << " complete." << std::endl;

    // Wait for user or scheduled start
    if (mode == WsprTransmitter::Mode::WSPR)
    {
        std::cout << main_debug_tag << "Waiting for next transmission window." << std::endl;
    }
    else
    {
        std::cout << main_debug_tag << "Press <spacebar> to begin test." << std::endl;
        while (!wait_for_space_or_signal(100))
        {
        }
    }

    wsprTransmitter.enableTransmission();

    // Transmission control per mode
    if (mode == WsprTransmitter::Mode::TONE)
    {
        if (!g_terminate.load(std::memory_order_acquire))
        {
            std::cout << main_debug_tag << "Press <spacebar> to end test." << std::endl;
            while (!wait_for_space_or_signal(100))
            {
                ;; // Loop and wait
            }
            std::cout << main_debug_tag << "Test ended." << std::endl;
        }
    }
    else
    {
        std::unique_lock<std::mutex> lk(g_end_mtx);
        while (!g_transmission_done &&
               !g_terminate.load(std::memory_order_acquire))
        {
            if (wait_for_space_or_signal(100))
                break;
            g_end_cv.wait_for(lk, std::chrono::milliseconds(100));
        }
    }

    // Final status message
    if (g_transmission_done || mode == WsprTransmitter::Mode::TONE)
    {
        std::cout << main_debug_tag
                  << wsprTransmitter.modeToString(mode)
                  << " transmission complete." << std::endl;
    }
    else if (mode == WsprTransmitter::Mode::FSKCW)
    {
        std::unique_lock<std::mutex> lk(g_end_mtx);
        while (!g_transmission_done &&
               !g_terminate.load(std::memory_order_acquire))
        {
            g_end_cv.wait_for(lk, std::chrono::milliseconds(100));
        }

        if (g_transmission_done)
        {
            std::cout << main_debug_tag << "FSKCW transmission complete." << std::endl;
        }
        else
        {
            std::cout << main_debug_tag << "Interrupted. Aborting FSKCW transmission." << std::endl;
        }
    }
    else if (!g_terminate.load(std::memory_order_acquire))
    {
        std::cout << main_debug_tag
                  << "Terminating " << wsprTransmitter.modeToString(mode)
                  << " transmission." << std::endl;
    }

    // Graceful shutdown
    wsprTransmitter.stop();
    return 0;
}
