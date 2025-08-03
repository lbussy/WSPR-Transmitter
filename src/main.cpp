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

// Edit with your data
static constexpr std::string_view CALLSIGN   = "AA0NT";
static constexpr std::string_view GRID= "EM18";
static constexpr uint8_t POWER_DBM = 20;

// Frequency choices - Leave alone (see below)
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

// Select your frequency constant
static constexpr double WSPR_FREQ = _10m;

/**
 * @brief Mutex used to guard shared shutdown and transmission state
 *
 * This mutex protects access to `g_transmission_done` and is paired
 * with the condition variable `g_end_cv` to coordinate between threads.
 */
static std::mutex g_end_mtx;

/**
 * @brief Condition variable to notify transmission completion or termination
 *
 * This condition variable is used to block or wake threads waiting
 * for the end of a transmission or shutdown signal. It should always
 * be used with `g_end_mtx`.
 */
static std::condition_variable g_end_cv;

/**
 * @brief Flag indicating whether the transmission has completed
 *
 * This boolean tracks the state of the transmitter. It must be accessed
 * only while holding `g_end_mtx`.
 *
 * @note Not thread-safe on its own; synchronize access with `g_end_mtx`.
 */
static bool g_transmission_done = false;

/**
 * @brief Global termination flag shared between signal handler and main loop
 *
 * This atomic flag is set to true when a termination condition (e.g., SIGINT)
 * occurs. It is used by worker threads and the main loop to safely shut down.
 *
 * @note This flag is safe to access from a signal handler.
 */
static std::atomic<bool> g_terminate{false};

/**
 * @brief Pipe file descriptors for self-pipe wake-up mechanism
 *
 * This array holds the read and write ends of a self-pipe used to wake the
 * main thread from blocking system calls like select(), poll(), or condition
 * variable waits. When a signal is received or shutdown is requested,
 * writing a byte to `sig_pipe_fds[1]` will cause the blocking call on
 * `sig_pipe_fds[0]` to return, allowing responsive shutdown.
 *
 * @details The self-pipe mechanism is a common pattern for making signal
 *          handling and inter-thread notifications safe and non-blocking.
 *
 * - `sig_pipe_fds[0]`: Read end (used in select()/poll()).
 * - `sig_pipe_fds[1]`: Write end (used in signal handlers or control logic).
 *
 * @note Must be initialized with `pipe(sig_pipe_fds)` before use.
 */
static int sig_pipe_fds[2] = {-1, -1};

/**
 * @brief RAII class to temporarily modify terminal input settings
 *
 * This guard disables canonical mode and input echo on `STDIN_FILENO`
 * for the duration of its lifetime. When the object is destroyed,
 * it automatically restores the previous terminal settings.
 *
 * @details Canonical mode is disabled using `ICANON`, which allows input
 *          to be read one character at a time. `ECHO` is disabled so that
 *          typed characters are not displayed. The `VMIN` and `VTIME`
 *          fields are also configured to ensure immediate character reads.
 *
 * Example use case:
 * - Reading user input interactively without waiting for newline.
 * - Temporarily hiding user keystrokes (like password prompts).
 *
 * @note This struct modifies `STDIN_FILENO` and uses `tcgetattr()` and
 *       `tcsetattr()` internally. No error checking is performed.
 */
struct TermiosGuard
{
    termios old_; ///< Original terminal settings (restored on destruction)
    termios cur_; ///< Modified settings with canonical mode disabled

    /**
     * @brief Constructor disables canonical mode and echo
     *
     * Captures the current terminal settings, modifies them
     * to disable canonical input and echoing, and applies them.
     */
    TermiosGuard()
    {
        // Get current terminal settings
        tcgetattr(STDIN_FILENO, &old_);
        cur_ = old_;

        // Disable canonical mode and echo
        cur_.c_lflag &= ~(ICANON | ECHO);

        // Require at least one character, no timeout
        cur_.c_cc[VMIN] = 1;
        cur_.c_cc[VTIME] = 0;

        // Apply new settings immediately
        tcsetattr(STDIN_FILENO, TCSANOW, &cur_);
    }

    /**
     * @brief Destructor restores the original terminal settings
     */
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
 * @brief Pauses execution until the user presses the spacebar or a shutdown signal is received
 *
 * This function blocks until either:
 * - The user presses the spacebar (detected using raw terminal input), or
 * - A signal triggers a write to the self-pipe, indicating a termination request.
 *
 * It uses the RAII-based TermiosGuard to temporarily set the terminal to
 * non-canonical, non-echoing mode so input can be read one character at a time.
 * A `select()` call monitors both `STDIN_FILENO` and `sig_pipe_fds[0]`.
 *
 * @details
 * - The function polls with `select()` to wait for input or a signal.
 * - It cleanly handles `EINTR` and uses an atomic flag `g_terminate`
 *   to check for termination requests.
 *
 * @note This function blocks indefinitely unless interrupted or input is received.
 *       It is typically used to pause before transmitting or resuming an operation.
 */
void wait_for_space_or_signal()
{
    TermiosGuard tguard; // Switch to non-canonical mode for the duration

    char c;
    while (!g_terminate.load(std::memory_order_acquire))
    {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);        // Watch for keyboard input
        FD_SET(sig_pipe_fds[0], &rfds);     // Watch for signal-triggered wakeup

        int nf = std::max(STDIN_FILENO, sig_pipe_fds[0]) + 1;

        // Block until input or signal
        if (::select(nf, &rfds, nullptr, nullptr, nullptr) < 0)
        {
            if (errno == EINTR)
                continue;  // Interrupted by signal, retry
            break;         // Other error: exit
        }

        if (FD_ISSET(sig_pipe_fds[0], &rfds))
        {
            // Signal arrived via self-pipe
            break;
        }

        if (FD_ISSET(STDIN_FILENO, &rfds))
        {
            // Read one character
            if (::read(STDIN_FILENO, &c, 1) == 1 && c == ' ')
                break;
        }
    }
}

/**
 * @brief Prompts the user to choose between WSPR and TONE modes
 *
 * This function uses raw terminal input to prompt the user to select
 * one of two transmission modes:
 * - 1) WSPR
 * - 2) TONE
 *
 * It waits for the user to press either '1' or '2', or exits early if a
 * termination signal is received via the self-pipe mechanism.
 *
 * @return true if WSPR mode is selected or implied (default),
 *         false if TONE mode is selected or termination was signaled.
 *
 * @details
 * - Terminal is placed in non-canonical mode using TermiosGuard.
 * - `select()` monitors `STDIN_FILENO` and `sig_pipe_fds[0]`.
 * - If the signal pipe is triggered, the function exits with `false`.
 * - Invalid input defaults to WSPR (true).
 *
 * @note Safe for use in main loops that are responsive to termination.
 */
bool select_wspr()
{
    TermiosGuard tg; // Enable raw input for single-character response

    // Prompt user for input
    std::cout << "Select mode:\n"
              << "  1) WSPR\n"
              << "  2) TONE\n"
              << "Enter [1/2]: " << std::flush;

    fd_set rfds;
    while (!g_terminate.load())
    {
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);         // Monitor keyboard
        FD_SET(sig_pipe_fds[0], &rfds);      // Monitor signal pipe

        int nf = std::max(STDIN_FILENO, sig_pipe_fds[0]) + 1;

        // Block until input or signal
        if (::select(nf, &rfds, nullptr, nullptr, nullptr) < 0)
        {
            std::cout << std::endl;
            if (errno == EINTR)
                continue;  // Retry after interrupt
            throw std::runtime_error("select failed");
        }

        if (FD_ISSET(sig_pipe_fds[0], &rfds))
        {
            // Received signal — exit early
            std::cout << std::endl;
            return false;
        }

        if (FD_ISSET(STDIN_FILENO, &rfds))
        {
            char c;
            if (::read(STDIN_FILENO, &c, 1) == 1)
            {
                std::cout << std::endl;
                if (c == '2')
                    return false;  // User selected TONE mode
                else
                    return true;   // Default or user selected WSPR
            }
        }
    }

    // Exited due to global terminate flag
    std::cout << std::endl;
    return false;
}

/**
 * @brief Signal handler for graceful shutdown
 *
 * This function is invoked when the process receives a termination signal.
 * It writes a message to `stderr`, stops the transmitter (if safe), signals
 * termination, and wakes the main thread if it is blocked in `select()` or
 * `poll()` using a self-pipe trick.
 *
 * @param signal The signal number that triggered the handler (unused).
 *
 * @note Only async-signal-safe functions should be called from within
 *       this handler. Ensure `wsprTransmitter.stop()` is safe, or move
 *       its logic to a safe location based on `g_terminate`.
 */
void sig_handler(int)
{
    // Message to print when signal is received
    const char msg[] = "Caught signal\nShutting down transmissions.\n";
    write(STDERR_FILENO, msg, sizeof(msg) - 1);  // Async-signal-safe

    // Trigger transmitter stop
    wsprTransmitter.stop();

    // Set global termination flag
    g_terminate.store(true);

    // Wake up the main thread if it's blocked on select()/poll()
    const char wake = 1;
    write(sig_pipe_fds[1], &wake, 1);  // Async-signal-safe
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

/**
 * @brief Sets up signal handlers and the self-pipe mechanism
 *
 * Installs signal handlers for termination and user-defined signals.
 * These handlers write to a self-pipe to safely interrupt blocking calls
 * like `select()` or `poll()`. Also ignores `SIGCHLD` to prevent zombies.
 *
 * @throws std::runtime_error if pipe creation fails
 */
void setup_signal_handlers()
{
    if (::pipe(sig_pipe_fds) < 0)
    {
        throw std::runtime_error("Failed to create signal pipe");
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
}

/**
 * @brief Configures the transmitter based on the selected mode
 *
 * Sets real-time scheduling, applies PPM correction, and sets up
 * callbacks and transmission parameters depending on the selected mode.
 *
 * @param isWspr True if WSPR mode is selected, false for TONE mode
 */
void configure_transmitter(bool isWspr)
{
    config.ppm = get_ppm_from_chronyc();

    wsprTransmitter.setThreadScheduling(SCHED_FIFO, 50);

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
}

/**
 * @brief Waits for transmission to complete or user signal
 *
 * In WSPR mode, waits for either transmission completion or shutdown.
 * In TONE mode, waits for spacebar press to end the tone.
 *
 * @param isWspr True if WSPR mode is selected, false for TONE mode
 */
void wait_for_completion(bool isWspr)
{
    if (isWspr)
    {
        std::unique_lock<std::mutex> lk(g_end_mtx);
        while (!g_transmission_done &&
               !g_terminate.load(std::memory_order_acquire))
        {
            g_end_cv.wait_for(lk, std::chrono::milliseconds(100));
        }

        if (g_transmission_done)
            std::cout << "WSPR transmission complete." << std::endl;
        else
            std::cout << "Interrupted. Aborting WSPR transmission." << std::endl;
    }
    else
    {
        std::cout << "Press <spacebar> to end test tone." << std::endl;
        wait_for_space_or_signal();
        std::cout << "Test tone stopped." << std::endl;
    }
}

/**
 * @brief Main entry point for the transmitter application
 *
 * Coordinates signal handling, user interaction, and WSPR or TONE
 * transmission setup and control. Uses cooperative shutdown and
 * precise timing management to support clean signal generation
 * and safe cancellation.
 *
 * @details
 * - Sets up a self-pipe mechanism and signal handlers for safe
 *   interruption of blocking system calls like `select()`.
 * - Prompts the user to select between WSPR and TONE transmission.
 * - Applies PPM correction obtained from chronyd.
 * - Configures the transmitter based on the selected mode.
 * - Waits for the appropriate trigger to begin transmission.
 * - Launches the transmission thread and waits for completion
 *   or user cancellation.
 * - Cleans up and stops the transmitter on exit.
 *
 * @return 0 on normal termination, 1 on unrecoverable failure.
 */
int main()
{
    try
    {
        // Set up signal handling and the self-pipe mechanism
        setup_signal_handlers();

        // Prompt user to choose transmission mode (WSPR or TONE)
        bool isWspr = select_wspr();

        // Exit early if signal (e.g., Ctrl-C) was received during input
        if (g_terminate.load(std::memory_order_acquire))
            return 0;

        std::cout << "Mode selected: " << (isWspr ? "WSPR" : "TONE") << std::endl;

        // Configure transmitter settings based on user selection
        configure_transmitter(isWspr);

        if (isWspr)
        {
            // WSPR mode waits for the next time slot
            std::cout << "Waiting for next transmission window." << std::endl;
        }
        else
        {
            // Tone mode requires spacebar press to begin
            std::cout << "Press <spacebar> to begin test tone." << std::endl;
            wait_for_space_or_signal();
        }

        // Begin scheduled or immediate transmission
        wsprTransmitter.enableTransmission();

        // Wait for transmission to finish or be aborted
        wait_for_completion(isWspr);

        // Clean up and stop transmission
        wsprTransmitter.stop();
        return 0;
    }
    catch (const std::exception &e)
    {
        // Catch and report any fatal errors
        std::cerr << "Fatal error: " << e.what() << std::endl;
        return 1;
    }
}
