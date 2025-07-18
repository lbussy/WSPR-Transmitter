/**
 * @file wspr_transmit.cpp
 * @brief A class to encapsulate configuration and DMA‑driven transmission of
 *        WSPR signals.
 *
 * Copyright (C) 2025 Lee C. Bussy (@LBussy). All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "wspr_transmit.hpp" // Class Declarations

#include "wspr_message.hpp" // WSPR Message Submodule
#include "mailbox.hpp"      // Mailbox Submodule
#include "bcm_model.hpp"    // Enumerates processor types

// C++ Standard Library Headers
#include <algorithm> // std::copy_n, std::clamp
#include <cassert>   // assert()
#include <cerrno>
#include <cmath>     // std::round, std::pow, std::floor
#include <cstdint>   // std::uintptr_t
#include <cstring>   // std::memcpy, std::strerror
#include <cstdlib>   // std::rand, RAND_MAX
#include <fstream>   // std::ifstream
#include <iomanip>   // std::setprecision
#include <iostream>  // std::cout, std::cerr
#include <optional>  // std::optional
#include <random>    // std::random_device, std::mt19937, std::uniform_real_distribution
#include <sstream>   // std::stringstream
#include <stdexcept> // std::runtime_error
#include <string_view>
#include <system_error>

// POSIX & System-Specific Headers
#include <fcntl.h>    // open flags
#include <sys/mman.h> // mmap, munmap, MAP_SHARED, PROT_READ/WRITE
#include <sys/stat.h> // struct stat, stat()
#include <sys/time.h> // gettimeofday(), struct timeval
#include <unistd.h>   // usleep(), close(), unlink()

#ifdef DEBUG_WSPR_TRANSMIT
constexpr const bool debug = true;
#else
constexpr const bool debug = false;
#endif
inline constexpr std::string_view debug_tag{"[WSPR-Transmitter] "};

// Helper classes and functions in anonymous namespace
namespace
{
    static constexpr size_t NUM_PAGES = 4096;

    /**
     * @brief RAII for a pool of DMA-capable pages allocated via mailbox.
     *
     * On construction:
     *   1. Calls `memAlloc()` for `numpages`.
     *   2. Calls `memLock()` to get the bus address.
     *   3. Calls `mapMem()` to map into user space.
     *
     * On destruction: unmaps, unlocks, and frees the pages automatically.
     *
     * @throws std::runtime_error on any mailbox or mapping failure.
     */
    class MailboxMemoryPool
    {
        size_t total_size_;
        uint32_t mem_ref_;
        std::uintptr_t bus_addr_;
        volatile uint8_t *virt_addr_;

    public:
        /**
         * @param numpages  Number of pages to allocate (1 const + N instr).
         */
        MailboxMemoryPool(unsigned numpages)
            : total_size_(numpages * Mailbox::PAGE_SIZE),
              mem_ref_(0), bus_addr_(0), virt_addr_(nullptr)
        {
            try
            {
                // Allocate
                mem_ref_ = mailbox.memAlloc(total_size_, Mailbox::BLOCK_SIZE);

                // Lock
                bus_addr_ = mailbox.memLock(mem_ref_);
                if (bus_addr_ == 0)
                    throw std::runtime_error("MailboxMemoryPool: memLock failed");

                // Map
                auto phys = static_cast<off_t>(Mailbox::busToPhysical(bus_addr_));
                virt_addr_ = mailbox.mapMem(phys, total_size_);
                if (virt_addr_ == nullptr)
                    throw std::runtime_error("MailboxMemoryPool: mapMem failed");
            }
            catch (const std::runtime_error &e)
            {
                // If it was a timeout, mailbox is already closed by memAlloc(),
                // so just re-throw and let caller decide to reopen & retry.
                if (std::string(e.what()).find("timed out") != std::string::npos)
                {
                    throw;
                }

                // Otherwise, clean up any allocation we made, then re-throw
                if (virt_addr_)
                {
                    mailbox.unMapMem(virt_addr_, total_size_);
                    virt_addr_ = nullptr;
                }
                if (bus_addr_)
                {
                    mailbox.memUnlock(mem_ref_);
                    bus_addr_ = 0;
                }
                if (mem_ref_)
                {
                    mailbox.memFree(mem_ref_);
                    mem_ref_ = 0;
                }
                throw; // preserve the original error
            }
        }

        /** Unmap, unlock, and free on destruction. */
        ~MailboxMemoryPool()
        {
            if (virt_addr_)
            {
                mailbox.unMapMem(virt_addr_, total_size_);
            }
            if (bus_addr_)
            {
                mailbox.memUnlock(mem_ref_);
                mailbox.memFree(mem_ref_);
            }
        }

        /** @return Virtual base pointer for the DMA pages. */
        volatile uint8_t *virt() const { return virt_addr_; }
        /** @return Bus address of the first DMA page. */
        std::uintptr_t bus() const { return bus_addr_; }
    };

    // TODO:
    static inline void sleep_until_abs(const struct timespec &ts_target)
    {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts_target, nullptr);
    }

    // TODO:
    // Sleep until absolute monotonic‐clock time `ts_target`
    static inline void sleep_until_abs(const std::chrono::steady_clock::time_point &target)
    {
        while (true)
        {
            auto now = std::chrono::steady_clock::now();
            if (now >= target)
                break;
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    }
} // end anonymous namespace

/**
 * @brief Converts a Mode enum value to its string representation.
 *
 * @details This function maps each WsprTransmitter::Mode enum value
 *          to its corresponding uppercase string, such as "TONE", "WSPR",
 *          "QRSS", etc.
 *
 * @param mode The Mode enum value to convert.
 * @return A std::string representing the name of the mode.
 */
std::string WsprTransmitter::modeToString(Mode mode)
{
    switch (mode)
    {
    case Mode::TONE:
        return "TONE";
    case Mode::WSPR:
        return "WSPR";
    case Mode::QRSS:
        return "QRSS";
    case Mode::FSKCW:
        return "FSKCW";
    case Mode::DFCW:
        return "DFCW";
    case Mode::UNKNOWN:
    default:
        return "UNKNOWN";
    }
}

/**
 * @brief Converts a string name to a Mode enum value.
 *
 * @details This function performs a case-insensitive comparison of the input
 *          string against known mode names ("TONE", "WSPR", "QRSS", etc.)
 *          and returns the corresponding Mode enum value. If the name does
 *          not match any known mode, Mode::UNKNOWN is returned.
 *
 * @param name The input string representing the mode name.
 * @return The corresponding Mode enum value.
 */
WsprTransmitter::Mode WsprTransmitter::stringToMode(const std::string &name)
{
    std::string upper;
    std::transform(name.begin(), name.end(), std::back_inserter(upper),
                   [](unsigned char c)
                   { return std::toupper(c); });

    if (upper == "TONE")
        return Mode::TONE;
    if (upper == "WSPR")
        return Mode::WSPR;
    if (upper == "QRSS")
        return Mode::QRSS;
    if (upper == "FSKCW")
        return Mode::FSKCW;
    if (upper == "DFCW")
        return Mode::DFCW;
    return Mode::UNKNOWN;
}

/**
 * @brief Global instance of the WSPR transmitter.
 *
 * @details This instance provides a globally accessible transmitter object
 * for configuring and initiating WSPR transmissions. It is constructed at
 * program startup and destructed automatically on exit.
 */
WsprTransmitter wsprTransmitter;

/* Public Methods */

/**
 * @brief Constructs a WSPR transmitter with default settings.
 *
 * @details Initializes the WSPR transmitter object. Transmission parameters
 * are configured later via setupWSPRTransmission(). This constructor does not
 * allocate hardware resources or initiate any transmissions.
 */
WsprTransmitter::WsprTransmitter() = default;

/**
 * @brief Destructor for the WSPR transmitter.
 *
 * @details Cleans up DMA, mailbox, and memory-mapped resources associated
 * with the WSPR transmission process. Automatically called at program exit
 * if wsprTransmitter is used as a global object.
 */
WsprTransmitter::~WsprTransmitter()
{
    // Stop scheduling new windows and any current TX
    disableTransmission(); // Stops scheduler + joins any tx_thread_
    dma_cleanup();         // In case anything still mapped
}

/**
 * @brief Install optional callbacks for transmission start/end.
 *
 * @param[in] start_cb
 *   Called on the transmit thread immediately before the first symbol
 *   (or tone) is emitted. If null, no start notification is made.
 *   The callback receives a std::variant containing either a double
 *   (transmit frequency) or a std::string (custom message).
 *
 * @param[in] end_cb
 *   Called on the transmit thread immediately after the last WSPR
 *   symbol is sent (but before DMA/PWM are torn down). If null,
 *   no completion notification is made.
 *   The callback uses the same variant format as the start callback.
 */
void WsprTransmitter::setTransmissionCallbacks(StartCallback start_cb, EndCallback end_cb)
{
    on_transmit_start_ = std::move(start_cb);
    on_transmit_end_ = std::move(end_cb);
}

/**
 * @brief Sets up a continuous tone transmission.
 *
 * @details Configures the transmitter to emit a continuous unmodulated carrier
 *          at the specified frequency. Typically used for audio testing,
 *          signal tracing, or tone-based modes. PPM correction is applied to
 *          improve frequency accuracy.
 *
 * @param frequency The output frequency in Hz.
 * @param ppm The frequency correction in parts per million.
 * @param power The GPIO pin used to enable/disable RF output power.
 */
void WsprTransmitter::setupToneTransmission(
    double frequency,
    double ppm,
    int power)
{
    // Set transmission parameters
    trans_params_.mode = Mode::TONE;
    trans_params_.frequency = frequency;
    trans_params_.power = power;

    if (debug)
    {
        std::cerr << debug_tag
                  << " TONE setup: freq = " << frequency
                  << " Hz, power = " << power
                  << std::endl;
    }

    // Reuse shared CW setup for single-tone mode
    setupCWTransmission(frequency, std::nullopt, ppm);

    // TODO? Update actual center freq if you recompute or adjust it
    if (frequency != 0.0)
        trans_params_.frequency = frequency;
}

/**
 * @brief Sets up a WSPR (Weak Signal Propagation Reporter) transmission.
 *
 * @details Prepares a WSPR message using the provided call sign, grid square,
 *          and power level. Encodes the message, applies optional sub-tone
 *          frequency offset, and sets the corrected transmission frequency.
 *
 * @param frequency The base transmission frequency in Hz.
 * @param power The GPIO pin used to control RF output power.
 * @param ppm Frequency correction in parts per million.
 * @param call_sign The station call sign (maximum 6 characters).
 * @param grid_square The 4-character or 6-character Maidenhead grid locator.
 * @param power_dbm Transmit power level to encode in the WSPR message (in dBm).
 * @param use_offset Whether to use the +1.46 Hz tone offset.
 */
void WsprTransmitter::setupWSPRTransmission(
    double frequency,
    int power,
    double ppm,
    std::string_view call_sign,
    std::string_view grid_square,
    int power_dbm,
    bool use_offset)
{
    // Tear down any existing DMA setup
    if (dma_setup_done_)
    {
        disableTransmission();
        dma_cleanup();
    }

    stop_requested_.store(false);

    // Set WSPR-specific transmission parameters
    trans_params_.mode = Mode::WSPR;
    trans_params_.frequency = frequency;
    trans_params_.ppm = ppm;
    trans_params_.power = power;
    trans_params_.use_offset = use_offset;

    trans_params_.call_sign = call_sign;
    trans_params_.grid_square = grid_square;
    trans_params_.power_dbm = power_dbm;

    trans_params_.symtime = WSPR_SYMTIME;
    trans_params_.tone_spacing = 1.0 / trans_params_.symtime;

    // Encode WSPR message into symbols
    WsprMessage msg(trans_params_.call_sign, trans_params_.grid_square, trans_params_.power_dbm);
    trans_params_.symbols.assign(msg.symbols, msg.symbols + msg.size);

    // Apply random frequency offset if requested
    if (use_offset && frequency != 0.0)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-1.0, 1.0);
        frequency += dis(gen) * WSPR_RAND_OFFSET;
    }

    // Initialize DMA
    setup_dma();

    // Apply PPM correction to clock
    dma_config_.plld_clock_frequency =
        dma_config_.plld_nominal_freq * (1 - ppm / 1e6);

    // Build DMA frequency table
    double center_actual = frequency;
    setup_dma_freq_table(center_actual);

    if (frequency != 0.0)
        trans_params_.frequency = center_actual;
}

/**
 * @brief Sets up a QRSS (very slow CW) transmission.
 *
 * @details Converts the provided CW message to a series of tones
 *          spaced in time using standard Morse code timing. Transmission
 *          uses a fixed dot length (in seconds) and applies PPM correction.
 *
 * @param cw_message The CW message to transmit.
 * @param frequency The output frequency in Hz.
 * @param unit_seconds The dot length in seconds (QRSS speed).
 * @param ppm Frequency correction in parts per million.
 * @param power The GPIO pin used to control RF output power.
 */
void WsprTransmitter::setupQRSSTransmission(
    std::string_view cw_message,
    double frequency,
    int unit_seconds,
    double ppm,
    int power)
{
    // Set transmission parameters
    trans_params_.mode = Mode::QRSS;
    trans_params_.frequency = frequency;
    trans_params_.qrss_unit_length = unit_seconds;
    trans_params_.qrss_message = std::string(cw_message);
    trans_params_.power = power;

    // DMA setup
    setup_dma();

    // Precompute frequency table for QRSS tone (single-tone mode)
    setupCWTransmission(frequency, std::nullopt, ppm);
}

/**
 * @brief Set up an FSKCW transmission with specified parameters.
 *
 * @details This configures a Morse-code message to be sent using Frequency
 * Shift Keyed Continuous Wave (FSKCW) modulation. In this mode:
 * - The carrier is always on.
 * - Dots and dashes are transmitted at the base frequency.
 * - Spacing elements (intra-character and inter-symbol) are sent using a
 *   frequency offset downward from the base.
 *
 * This function must be followed by `enableTransmission()` to start sending.
 *
 * @param cw_message The Morse code message to transmit, consisting only of
 *                   '.', '-', and ' ' characters.
 * @param frequency The base frequency in Hz for tone-on elements (dots/dashes).
 * @param offset_hz Frequency offset in Hz used for space elements (carrier remains on).
 * @param unit_seconds Length in seconds of a single dot element.
 * @param ppm Frequency correction in parts per million.
 * @param power Output drive level (0–7) for GPIO carrier strength.
 */
void WsprTransmitter::setupFSKCWTransmission(
    std::string_view cw_message,
    double frequency,
    double offset_hz,
    int unit_seconds,
    double ppm,
    int power)
{
    // Stop and clean up if DMA was previously configured
    if (dma_setup_done_)
    {
        disableTransmission();
        dma_cleanup();
    }

    stop_requested_.store(false);

    // Store FSKCW transmission parameters
    trans_params_.mode = Mode::FSKCW;
    trans_params_.qrss_message = std::string(cw_message);
    trans_params_.frequency = frequency;
    trans_params_.fskcw_offset = offset_hz;
    trans_params_.qrss_unit_length = unit_seconds;
    trans_params_.ppm = ppm;
    trans_params_.power = power;
    trans_params_.use_offset = false;

    // Set up DMA system (clock registers, mailbox memory, etc.)
    setup_dma();

    // Precompute tone tables for base frequency and spacing frequency
    setupCWTransmission(frequency, frequency - offset_hz, ppm);
}

void WsprTransmitter::setupDFCWTransmission(
    std::string_view cw_message,
    double frequency,
    double offset_hz,
    int unit_seconds,
    double ppm,
    int power)
{
    return;
}

/**
 * @brief Configure POSIX scheduling policy & priority for future transmissions.
 *
 * @details
 *   This must be called _before_ `startTransmission()` if you need real-time
 *   scheduling.  The next call to `startTransmission()` will launch its thread
 *   under the given policy/priority.
 *
 * @param[in] policy
 *   One of the standard POSIX policies (e.g. SCHED_FIFO, SCHED_RR, SCHED_OTHER).
 * @param[in] priority
 *   Thread priority (1–99) for real-time policies; ignored under SCHED_OTHER.
 */
void WsprTransmitter::setThreadScheduling(int policy, int priority)
{
    thread_policy_ = policy;
    thread_priority_ = priority;
}

/**
 * @brief Begin transmission immediately for tone mode, or schedule for the next
 *        WSPR window.
 *
 * @details
 *   – If `trans_params_.mode == Mode::TONE == true`, this will spawn the TX thread
 *     immediately (bypassing the scheduler) so you can start/stop your
 *     test tone on demand.
 *   – Otherwise it launches the scheduler thread, which waits for the next
 *     valid WSPR window and then fires off a transmission.
 */
void WsprTransmitter::enableTransmission()
{
    stop_requested_.store(false, std::memory_order_release);
    // TODO:  Scheduling is bypased here
    if (trans_params_.mode == Mode::QRSS || trans_params_.mode == Mode::FSKCW || trans_params_.mode == Mode::DFCW)
    {
        // Transmit immediately
        tx_thread_ = std::thread(&WsprTransmitter::thread_entry, this);
    }
    else if (trans_params_.mode == Mode::TONE)
    {
        // Transmit immediately
        tx_thread_ = std::thread(&WsprTransmitter::thread_entry, this);
    }
    else if (trans_params_.mode == Mode::WSPR)
    {
        // Allow scheduler to kick in
        scheduler_.start();
    }
    else
    {
        return;
    }
}

/**
 * @brief Cancels the scheduler (and any running transmission).
 *
 * Waits for the scheduler thread to stop, and forces any in‐flight
 * transmission to end.
 */
void WsprTransmitter::disableTransmission()
{
    // Stop scheduling further windows
    scheduler_.stop();

    // if a transmit thread is running, signal & join it too
    stop_requested_.store(true, std::memory_order_release);
    stop_cv_.notify_all();

    if (tx_thread_.joinable() &&
        tx_thread_.get_id() != std::this_thread::get_id())
    {
        tx_thread_.join();
    }
}

/**
 * @brief Request an in‑flight transmission to stop.
 *
 * @details Sets the internal stop flag so that ongoing loops in the transmit
 *          thread will exit at the next interruption point. Notifies any
 *          condition_variable waits to unblock the thread promptly.
 */
void WsprTransmitter::stopTransmission()
{
    // Tell the worker to stop
    stop_requested_.store(true);
    // Unblock any waits
    stop_cv_.notify_all();
}

/**
 * @brief Gracefully stops scheduler and any in-flight transmission, then cleans up.
 *
 * @details
 *   1. Calls disableTransmission() to stop the scheduler thread and join any tx_thread_.
 *   2. Performs DMA/PWM/mailbox cleanup.
 */
void WsprTransmitter::stop()
{
    if (stop_requested_.exchange(true))
        return;

    disableTransmission();
    dma_cleanup();
}

/**
 * @brief Check if the GPIO is bound to the clock.
 *
 * @details Returns a value indicating if the system is transmitting
 * in any way,
 *
 * @return `true` if clock is engaged, `false` otherwise.
 */
bool WsprTransmitter::isTransmitting() const noexcept
{
    return transmit_on_.load(std::memory_order_acquire);
}

/**
 * @brief Prints current transmission parameters and configuration.
 *
 * @details Displays the transmission parameters including frequency,
 * power, mode, tone/test settings, and symbol timing. For WSPR mode, it
 * prints the encoded symbol sequence. For QRSS, it shows the Morse message
 * and unit duration. In tone test mode, message-related fields are shown
 * as "N/A".
 *
 * This function is useful for debugging and verifying that the selected
 * transmission mode and parameters are correctly configured.
 */
void WsprTransmitter::printParameters()
{
    std::cout << debug_tag << "Mode:              ";
    switch (trans_params_.mode)
    {
    case Mode::WSPR:
        std::cout << "WSPR" << std::endl;
        break;
    case Mode::QRSS:
        std::cout << "QRSS" << std::endl;
        break;
    case Mode::FSKCW:
        std::cout << "FSKCW" << std::endl;
        break;
    case Mode::TONE:
        std::cout << "TONE" << std::endl;
        break;
    default:
        std::cout << "Unknown" << std::endl;
        break;
    }

    std::cout << debug_tag << "Frequency:         "
              << std::fixed << std::setprecision(6)
              << (trans_params_.frequency / 1.0e6) << " MHz" << std::endl;

    std::cout << debug_tag << "GPIO Power:        "
              << std::fixed << std::setprecision(1)
              << convert_mw_dbm(get_gpio_power_mw(trans_params_.power)) << " dBm" << std::endl;

    std::cout << debug_tag << "PPM Correction:    "
              << std::fixed << std::setprecision(2)
              << trans_params_.ppm << " ppm" << std::endl;

    std::cout << debug_tag << "DMA Table Size:    "
              << trans_params_.dma_table_freq.size() << std::endl;

    if (trans_params_.mode == Mode::WSPR)
    {
        std::cout << debug_tag << "Call Sign:         " << trans_params_.call_sign << std::endl;
        std::cout << debug_tag << "Grid Square:       " << trans_params_.grid_square << std::endl;
        std::cout << debug_tag << "Power (dBm):       " << trans_params_.power_dbm << std::endl;
        std::cout << debug_tag << "Symbol Time:       " << trans_params_.symtime << " s" << std::endl;
        std::cout << debug_tag << "Tone Spacing:      " << trans_params_.tone_spacing << " Hz" << std::endl;
        std::cout << debug_tag << "Use Offset:        " << (trans_params_.use_offset ? "Yes" : "No") << std::endl;

        std::cout << debug_tag << "WSPR Symbols:" << std::endl;
        const int symbol_count = static_cast<int>(trans_params_.symbols.size());
        for (int i = 0; i < symbol_count; ++i)
        {
            std::cout << static_cast<int>(trans_params_.symbols[i]);
            if (i < symbol_count - 1)
                std::cout << ", ";
            if ((i + 1) % 18 == 0 && i < symbol_count - 1)
                std::cout << std::endl;
        }
        std::cout << std::endl;
    }
    else if (trans_params_.mode == Mode::QRSS)
    {
        std::cout << debug_tag << "QRSS Message:      [" << trans_params_.qrss_message << "]" << std::endl;
        std::cout << debug_tag << "Dot Duration:      " << trans_params_.qrss_unit_length << "s" << std::endl;
    }
    else if (trans_params_.mode == Mode::FSKCW)
    {
        std::cout << debug_tag << "FSKCW Message:     [" << trans_params_.qrss_message << "]" << std::endl;
        std::cout << debug_tag << "Dot Duration:      " << trans_params_.qrss_unit_length << "s" << std::endl;
        std::cout << debug_tag << "FSKCW Offset:      " << trans_params_.fskcw_offset << " Hz" << std::endl;
    }
    else if (trans_params_.mode == Mode::DFCW)
    {
        std::cout << debug_tag << "DFCW Message:     [" << trans_params_.qrss_message << "]" << std::endl;
        std::cout << debug_tag << "Dot Duration:      " << trans_params_.qrss_unit_length << "s" << std::endl;
        std::cout << debug_tag << "DFCW Offset:      " << trans_params_.fskcw_offset << " Hz" << std::endl;
    }
    else if (trans_params_.mode == Mode::TONE)
    {
        std::cout << debug_tag << "Tone Test Mode:    True" << std::endl;
        std::cout << debug_tag << "Message Fields:    N/A" << std::endl;
        std::cout << debug_tag << "Symbol Fields:     N/A" << std::endl;
    }
}

/**
 * @brief Sets the callback for symbol transmission.
 *
 * @details The callback will be called for each symbol with the symbol
 * character, its duration, and the current tone string associated with the
 * transmission (e.g., Morse character context).
 *
 * @param cb A function taking (char symbol, duration, Mode)
 */
void WsprTransmitter::setSymbolCallback(
    std::function<void(char, std::chrono::nanoseconds, Mode)> cb)
{
    symbol_cb_ = std::move(cb);
}

/* Private Methods */

/**
 * @brief Invoke the start‐transmission callback with an empty message.
 *
 * @details Ensures the callback is valid before calling, providing an
 *          empty string to match the `CallbackArg` signature.
 *
 * @param cb  The callback to invoke just before starting transmission.
 */
inline void WsprTransmitter::fire_start_cb(const std::string &msg, const double frequency)
{
    if (on_transmit_start_)
    {
        // Capture msg, frequency, and cb by value
        std::thread([cb = on_transmit_start_, msg, frequency]()
                    { cb(msg, frequency); })
            .detach();
    }
}

/**
 * @brief Invoke the end‐transmission callback with a message.
 *
 * @details Ensures the callback is valid before calling, passing the
 *          provided message string to the user’s callback function.
 *
 * @param cb   The callback to invoke immediately after transmission.
 * @param msg  The message string to pass into the callback.
 */
inline void WsprTransmitter::fire_end_cb(const std::string &msg, double elapsed)
{
    if (on_transmit_end_)
    {
        std::thread([cb = on_transmit_end_, msg, elapsed]()
                    { cb(msg, elapsed); })
            .detach();
    }
}

/**
 * @brief Starts RF transmission based on the current transmission mode.
 *
 * @details This function checks for early termination requests and then
 *          dispatches the transmission to the appropriate method based on
 *          the configured mode in `trans_params_`. Supported modes include
 *          WSPR, QRSS, FSKCW, DFCW, and continuous tone. If the mode is
 *          unknown or unsupported, an end callback is fired with an error
 *          message.
 *
 * @note If `stop_requested_` is already set, the transmission is aborted
 *       and a corresponding callback is issued without starting any RF output.
 */
void WsprTransmitter::transmit()
{
    // Abort early if a stop was requested before entering transmission
    if (stop_requested_.load())
    {
        if (debug)
        {
            std::cerr << debug_tag
                      << "transmit() aborted before start."
                      << std::endl;
        }

        fire_end_cb("Transmission aborted before start", 0.0);
        return;
    }

    // Dispatch to appropriate transmission mode
    switch (trans_params_.mode)
    {
    case Mode::WSPR:
        transmit_wspr();
        break;

    case Mode::QRSS:
        transmit_qrss();
        break;

    case Mode::FSKCW:
        transmit_fskcw();
        break;

    case Mode::DFCW:
        transmit_dfcw();
        break;

    case Mode::TONE:
        transmit_tone();
        break;

    default:
        fire_end_cb("Unsupported mode", 0.0);
        break;
    }
}

/**
 * @brief Begins continuous tone transmission using the precomputed mark frequency.
 *
 * @details This method initiates a tone transmission loop that continuously outputs
 *          a carrier at the base (mark) frequency until a stop request is issued.
 *
 * The tone uses the DMA instruction set precomputed in `dma_pages_mark_`, typically
 * created during `setupCWTransmission()` or `setupToneTransmission()`.
 *
 * This method blocks until `stop_requested_` is set. It is intended for test
 * transmission or carrier-only modes like TONE.
 *
 * @throws None
 */
void WsprTransmitter::transmit_tone()
{
    fire_start_cb("Continuous tone", trans_params_.frequency);

    // Select the precomputed tone table
    dma_table_freq_ = dma_table_freq_base_;
    transmit_on();

    auto t0 = std::chrono::steady_clock::now();

    int dummyBuf = 0;
    while (!stop_requested_.load())
    {
        transmit_symbol(
            0,       // Symbol index (not relevant here)
            0.0,     // No offset
            dummyBuf // DMA buffer index
        );
    }

    transmit_off();

    auto t1 = std::chrono::steady_clock::now();
    double total = std::chrono::duration<double>(t1 - t0).count();
    total = std::round(total * 1000.0) / 1000.0;

    fire_end_cb("Tone transmission ended", total);
}

/**
 * @brief Perform a full WSPR transmission sequence.
 *
 * @details This function transmits a WSPR message using the symbol data previously
 *          encoded into `trans_params_.symbols` and the transmission parameters
 *          (frequency, symbol duration, etc.).
 *
 * Each symbol is transmitted at precise intervals derived from the system
 * monotonic clock to maintain strict timing. The routine supports early termination
 * via `stop_requested_` and uses precomputed DMA buffers for each symbol.
 *
 * @throws None
 */
void WsprTransmitter::transmit_wspr()
{
    // Validate configuration
    if (trans_params_.frequency == 0.0)
    {
        fire_end_cb("Skipping WSPR transmission (frequency = 0.0)", 0.0);
        return;
    }

    fire_start_cb("WSPR", trans_params_.frequency);

    // Capture start time using chrono and raw timespec
    auto t0_chrono = std::chrono::steady_clock::now();
    struct timespec t0_ts;
    clock_gettime(CLOCK_MONOTONIC, &t0_ts);

    const int symbol_count = static_cast<int>(trans_params_.symbols.size());
    const double symtime = trans_params_.symtime;
    int bufPtr = 0;
    bool interrupted = false;

    // Begin RF transmission
    transmit_on();

    // Transmit each symbol with precise timing
    for (int i = 0; i < symbol_count; ++i)
    {
        if (stop_requested_.load())
        {
            interrupted = true;
            break;
        }

        // Calculate the target time for this symbol
        long offset_ns = static_cast<long>(i * symtime * 1e9);
        struct timespec target = t0_ts;
        target.tv_sec += offset_ns / 1000000000;
        target.tv_nsec += offset_ns % 1000000000;
        if (target.tv_nsec >= 1000000000)
        {
            target.tv_sec++;
            target.tv_nsec -= 1000000000;
        }

        // Sleep until the symbol's scheduled time
        sleep_until_abs(target);

        // Transmit this symbol using DMA
        transmit_symbol(
            static_cast<int>(trans_params_.symbols[i]),
            symtime,
            bufPtr);
    }

    // Stop RF output
    transmit_off();

    // Compute total transmission time
    auto t1 = std::chrono::steady_clock::now();
    double total = std::chrono::duration<double>(t1 - t0_chrono).count();
    total = std::round(total * 1000.0) / 1000.0;

    // Report completion or interruption
    if (interrupted)
        fire_end_cb("WSPR transmission interrupted", total);
    else
        fire_end_cb("WSPR transmission complete", total);
}

// TODO
std::pair<int, bool> WsprTransmitter::morse_char_to_units(char c)
{
    switch (c)
    {
    case '.':
        return {1, true};
    case '-':
        return {3, true};
    case ' ':
        return {1, false};
    default:
        return {0, false}; // Skip
    }
}

// TODO
void WsprTransmitter::transmit_qrss()
{
    // Validate frequency
    if (trans_params_.frequency == 0.0)
    {
        fire_end_cb("Skipping QRSS transmission (frequency = 0.0)", 0.0);
        return;
    }

    fire_start_cb("QRSS Message", trans_params_.frequency);

    // Use the precomputed base tone table
    dma_table_freq_ = dma_table_freq_base_;

    auto t0 = std::chrono::steady_clock::now();
    int symbol_index = 0;
    bool interrupted = false;

    for (char c : trans_params_.qrss_message)
    {
        if (stop_requested_.load())
        {
            interrupted = true;
            break;
        }

        auto [units, is_symbol] = morse_char_to_units(c);
        if (units == 0)
            continue;

        std::chrono::steady_clock::time_point symbol_start;
        std::chrono::steady_clock::time_point actual_start;

        if (symbol_index == 0)
        {
            symbol_start = std::chrono::steady_clock::now(); // Emit first symbol immediately
            actual_start = symbol_start;
        }
        else
        {
            symbol_start = t0 + std::chrono::seconds(symbol_index * trans_params_.qrss_unit_length);
            sleep_until_abs(symbol_start);
            actual_start = std::chrono::steady_clock::now();
        }

        auto symbol_end = symbol_start + std::chrono::seconds(units * trans_params_.qrss_unit_length);

        if (is_symbol)
        {
            transmit_on();

            int dummy_buf = 0;
            double duration_s = static_cast<double>(units * trans_params_.qrss_unit_length);
            transmit_symbol(0, duration_s, dummy_buf);

            transmit_off();
        }
        else
        {
            // Space — wait it out
            sleep_until_abs(symbol_end);
        }

        auto actual_end = std::chrono::steady_clock::now();

        if (symbol_cb_)
        {
            auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
                actual_end - actual_start);
            symbol_cb_(c, duration);
        }

        symbol_index += units;
    }

    transmit_off(); // Final safeguard

    auto t1 = std::chrono::steady_clock::now();
    double total = std::chrono::duration<double>(t1 - t0).count();
    total = std::round(total * 10000.0) / 10000.0;

    if (interrupted)
        fire_end_cb("QRSS transmission interrupted", total);
    else
        fire_end_cb("QRSS transmission complete", total);
}

// TODO:
void WsprTransmitter::transmit_fskcw()
{
    if (trans_params_.frequency == 0.0)
    {
        fire_end_cb("Skipping FSKCW transmission (frequency = 0.0)", 0.0);
        return;
    }

    fire_start_cb("FSKCW Message", trans_params_.frequency);
    transmit_on();

    auto t0 = std::chrono::steady_clock::now();
    int symbol_index = 0;
    bool interrupted = false;

    // Iterate through Morse code message
    for (char c : trans_params_.qrss_message)
    {
        // Check for early termination
        if (stop_requested_.load())
        {
            interrupted = true;
            break;
        }

        auto [units, is_symbol] = morse_char_to_units(c);
        if (units == 0)
            continue;

        std::chrono::steady_clock::time_point symbol_start;
        std::chrono::steady_clock::time_point actual_start;

        if (symbol_index == 0)
        {
            symbol_start = std::chrono::steady_clock::now();
            actual_start = symbol_start;
        }
        else
        {
            symbol_start = t0 + std::chrono::seconds(symbol_index * trans_params_.qrss_unit_length);
            sleep_until_abs(symbol_start);
            actual_start = std::chrono::steady_clock::now();
        }

        // Select tone table and copy values into const_page_
        dma_table_freq_ = is_symbol
            ? dma_table_freq_base_
            : dma_table_freq_shifted_;

        reinterpret_cast<uint32_t *>(const_page_.v)[0] = dma_table_freq_[0];
        reinterpret_cast<uint32_t *>(const_page_.v)[1] = dma_table_freq_[1];

        if (debug)
        {
            double freq = is_symbol
                ? trans_params_.frequency
                : trans_params_.frequency - trans_params_.fskcw_offset;

            std::cerr << "[DEBUG] Symbol '" << c << "' using freq "
                      << freq << " Hz → DMA[0] = " << dma_table_freq_[0]
                      << ", DMA[1] = " << dma_table_freq_[1]
                      << std::endl;
        }

        double duration_s = static_cast<double>(units * trans_params_.qrss_unit_length);
        int dummy_buf = 0;
        transmit_symbol(0, duration_s, dummy_buf);

        auto actual_end = std::chrono::steady_clock::now();

        // Invoke symbol callback if registered
        if (symbol_cb_)
        {
            auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
                actual_end - actual_start);
            symbol_cb_(c, duration);
        }

        // Advance index for timing future symbols
        symbol_index += units;
    }

    transmit_off();

    auto t1 = std::chrono::steady_clock::now();
    double total = std::chrono::duration<double>(t1 - t0).count();
    total = std::round(total * 10000.0) / 10000.0;

    if (interrupted)
        fire_end_cb("FSKCW transmission interrupted", total);
    else
        fire_end_cb("FSKCW transmission complete", total);
}

void WsprTransmitter::transmit_dfcw()
{
    return;
}

/**
 * @brief Transmit an FSKCW (Frequency Shift Keyed Continuous Wave) message.
 *
 * @details FSKCW is a Morse-like mode that uses frequency shifts to
 *          distinguish between signal elements and spacing. The tone is
 *          always present — either at the mark (base) or space (offset)
 *          frequency. Symbols (e.g., '.' and '-') use the base frequency,
 *          while spaces use the offset frequency.
 *
 * Timing is derived from `trans_params_.qrss_unit_length`, with durations
 * based on the number of units associated with each character in the
 * `qrss_message`. Frequencies are precomputed into DMA pages.
 *
 * @throws None
 */
void WsprTransmitter::transmit_fskcw()
{
    // Validate frequency
    if (trans_params_.frequency == 0.0)
    {
        fire_end_cb("Skipping FSKCW transmission (frequency = 0.0)", 0.0);
        return;
    }

    fire_start_cb("FSKCW Message", trans_params_.frequency);
    transmit_on();

    auto t0 = std::chrono::steady_clock::now();
    int symbol_index = 0;
    bool interrupted = false;

    // Iterate through Morse message
    for (char c : trans_params_.qrss_message)
    {
        // Early stop check
        if (stop_requested_.load())
        {
            interrupted = true;
            break;
        }

        // Determine how many units and whether it's a symbol or space
        auto [units, is_symbol] = morse_char_to_units(c);
        if (units == 0)
            continue;

        std::chrono::steady_clock::time_point symbol_start;
        std::chrono::steady_clock::time_point actual_start;

        // Calculate absolute time to emit symbol
        if (symbol_index == 0)
        {
            symbol_start = std::chrono::steady_clock::now();
            actual_start = symbol_start;
        }
        else
        {
            symbol_start = t0 + std::chrono::seconds(symbol_index * trans_params_.qrss_unit_length);
            sleep_until_abs(symbol_start);
            actual_start = std::chrono::steady_clock::now();
        }

        // Choose DMA page: base frequency for symbol, offset for space
        const auto &page = is_symbol ? dma_pages_mark_[0] : dma_pages_space_[0];
        std::memcpy(const_page_.v, page.data.data(), page.data.size() * sizeof(uint32_t));

        if (debug)
        {
            double base_freq = trans_params_.frequency;
            double shifted_freq = trans_params_.frequency - trans_params_.offset;

            std::cerr << debug_tag
                      << " Symbol '" << c << "' using freq "
                      << (is_symbol ? base_freq : shifted_freq)
                      << " Hz → DMA[0] = " << page.data[0]
                      << ", DMA[1] = " << page.data[1]
                      << std::endl;
        }

        // Transmit tone for the duration of the symbol or space
        double duration_s = static_cast<double>(units * trans_params_.qrss_unit_length);
        int dummy_buf = 0;
        transmit_symbol(0, duration_s, dummy_buf);

        auto actual_end = std::chrono::steady_clock::now();

        // Call symbol callback if defined
        if (symbol_cb_)
        {
            auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(
                actual_end - actual_start);
            symbol_cb_(c, duration, trans_params_.mode);
        }

        // Advance timing offset
        symbol_index += units;
    }

    transmit_off();

    auto t1 = std::chrono::steady_clock::now();
    double total = std::chrono::duration<double>(t1 - t0).count();
    total = std::round(total * 10000.0) / 10000.0;

    if (interrupted)
        fire_end_cb("FSKCW transmission interrupted", total);
    else
        fire_end_cb("FSKCW transmission complete", total);
}

// TODO:
void WsprTransmitter::transmit_dfcw()
{
    return;
}

/**
 * @brief Converts a Morse code character into its duration and tone status.
 *
 * @details This method translates a Morse code symbol into a pair representing:
 *          - The number of time units it should last.
 *          - Whether it corresponds to an audible tone (`true`) or a silent gap (`false`).
 *
 * Supported characters:
 * - `'.'` (dit): 1 unit with tone.
 * - `'-'` (dah): 3 units with tone.
 * - `' '` (space): 1 unit without tone (used for intra-character or word spacing).
 * - Any other character is ignored (returns {0, false}).
 *
 * @param c The Morse code character to interpret.
 * @return A pair where:
 *         - `first` is the number of timing units.
 *         - `second` is `true` if it produces a tone, `false` for silence.
 */
std::pair<int, bool> WsprTransmitter::morse_char_to_units(char c)
{
    switch (c)
    {
    case '.':
        return {1, true};
    case '-':
        return {3, true};
    case ' ':
        return {1, false};
    default:
        return {0, false}; // Skip
    }
}

/**
 * @brief Waits for the background transmission thread to finish.
 *
 * @details If the transmission thread was launched via
 *          startTransmission(), this call will block until
 *          that thread has completed and joined. After returning,
 *          tx_thread_ is no longer joinable.
 */
void WsprTransmitter::join_transmission()
{
    if (tx_thread_.joinable())
    {
        tx_thread_.join();
    }
}

/**
 * @brief Clean up DMA and mailbox resources.
 *
 * @details Performs teardown in the following order:
 *   1. Prevent multiple invocations.
 *   2. Stop any ongoing DMA transfers and disable the PWM clock.
 *   3. Restore saved clock and PWM register values.
 *   4. Reset the DMA controller.
 *   5. Unmap the peripheral base address region.
 *   6. Deallocate mailbox memory pages.
 *   7. Close the mailbox.
 *   8. Remove the local device file.
 *   9. Reset all configuration data to defaults.
 *
 * @note This function is idempotent; subsequent calls are no‑ops.
 */
void WsprTransmitter::dma_cleanup()
{
    // Only clean up if we have something to tear down
    if (!dma_setup_done_)
    {
        return;
    }
    dma_setup_done_ = false;

    // If we never mapped the peripherals, nothing to tear down:
    if (!dma_config_.peripheral_base_virtual)
    {
        return;
    }

    // Stop DMA transfers and disable PWM clock
    transmit_off();

    // Restore original clock and PWM registers
    access_bus_address(CM_GP0DIV_BUS) = dma_config_.orig_gp0div;
    access_bus_address(CM_GP0CTL_BUS) = dma_config_.orig_gp0ctl;
    access_bus_address(PWM_BUS_BASE + 0x00) = dma_config_.orig_pwm_ctl;
    access_bus_address(PWM_BUS_BASE + 0x04) = dma_config_.orig_pwm_sta;
    access_bus_address(PWM_BUS_BASE + 0x10) = dma_config_.orig_pwm_rng1;
    access_bus_address(PWM_BUS_BASE + 0x20) = dma_config_.orig_pwm_rng2;
    access_bus_address(PWM_BUS_BASE + 0x08) = dma_config_.orig_pwm_fifocfg;

    // Reset DMA controller registers
    clear_dma_setup();

    // Unmap peripheral region if mapped
    if (dma_config_.peripheral_base_virtual)
    {
        ::mailbox.unMapMem(dma_config_.peripheral_base_virtual, Mailbox::PAGE_SIZE * NUM_PAGES);
        dma_config_.peripheral_base_virtual = nullptr;
    }

    // Deallocate mailbox-allocated memory pages
    deallocate_memory_pool();

    // Close mailbox if open
    mailbox.close();

    // Reset global configuration structures to defaults
    dma_config_ = DMAConfig();         // Uses your in-class initializers
    mailbox_struct_ = MailboxStruct(); // Uses your in-class initializers
}

/**
 * @brief Get the GPIO drive strength in milliamps.
 *
 * Maps a drive strength level (0–7) to its corresponding current drive
 * capability in milliamps (mA).
 *
 * @param level Drive strength level (0–7).
 * @return int   Drive strength in mA.
 * @throws std::out_of_range if level is outside the [0,7] range.
 */
constexpr int WsprTransmitter::get_gpio_power_mw(int level)
{
    if (level < 0 || level >= static_cast<int>(DRIVE_STRENGTH_TABLE.size()))
    {
        throw std::out_of_range("WsprTransmitter::get_gpio_power_mw: Drive strength level must be between 0 and 7");
    }
    return DRIVE_STRENGTH_TABLE[level];
}

/**
 * @brief Convert power in milliwatts to decibels referenced to 1 mW (dBm).
 *
 * Uses the formula:
 * PdBm=10*LOG10(mW/1) where mW = power in milliwatts
 *
 * @param mw  Power in milliwatts. Must be greater than 0.
 * @return    Power in dBm.
 * @throws    std::domain_error if mw is not positive.
 */
inline double WsprTransmitter::convert_mw_dbm(double mw)
{
    if (mw <= 0.0)
    {
        throw std::domain_error("WsprTransmitter::convert_mw_dbm: Input power (mW) must be > 0 to compute logarithm");
    }
    return 10.0 * std::log10(mw);
}

/**
 * @brief Entry point for the background transmission thread.
 *
 * @details Applies the configured POSIX scheduling policy and priority
 *          (via set_thread_priority()), then invokes transmit() to carry
 *          out the actual transmission work. This method runs inside the
 *          new thread and returns only when transmit() completes or a
 *          stop request is observed.
 */
void WsprTransmitter::thread_entry()
{
    // Pin this thread to CPU 0
    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    CPU_SET(0, &cpus);
    int aff_ret = pthread_setaffinity_np(pthread_self(),
                                         sizeof(cpus),
                                         &cpus);
    if (aff_ret != 0)
    {
        // Affinity failed; warn but continue
        std::cerr << debug_tag
                  << "thread_entry(): failed to set CPU affinity: "
                  << std::strerror(aff_ret)
                  << std::endl;
    }

    // Bump our own scheduling parameters first:
    try
    {
        set_thread_priority();
    }
    catch (const std::system_error &e)
    {
        throw std::domain_error(
            std::string("WsprTransmitter::thread_entry(): Error setting thread priority: ") + e.what());
    }
    catch (const std::exception &e)
    {
        throw std::domain_error(
            std::string("WsprTransmitter::thread_entry(): Unexpected error: ") + e.what());
    }
    // Actually do the work (blocking until complete or stop_requested_)
    transmit();
}

/**
 * @brief Applies the configured scheduling policy and priority to this thread.
 *
 * @details Builds a sched_param struct using thread_priority_ and invokes
 *          pthread_setschedparam() with thread_policy_ on the current thread.
 *          If the call fails, raise a warning.
 */
void WsprTransmitter::set_thread_priority()
{
    // Only real‑time policies use priority > 0
    sched_param sch{};
    sch.sched_priority = thread_priority_;
    int ret = pthread_setschedparam(pthread_self(), thread_policy_, &sch);

    if (ret != 0)
    {
        throw std::runtime_error(
            std::string("WsprTransmitter::set_thread_priority(): pthread_setschedparam failed: ") +
            std::strerror(ret));
    }
}

/**
 * @brief Calculates the virtual address for a given bus address.
 *
 * This function calculates the appropriate virtual address for accessing the
 * given bus address in the peripheral space. It does so by subtracting 0x7e000000
 * from the supplied bus address to obtain the offset into the peripheral address space,
 * then adds that offset to the virtual base address of the peripherals.
 *
 * @param bus_addr The bus address from which to calculate the corresponding virtual address.
 * @return A reference to a volatile int located at the computed virtual address.
 */
inline volatile int &WsprTransmitter::access_bus_address(std::uintptr_t bus_addr)
{
    // Compute byte‐offset from the bus address
    std::uintptr_t offset = Mailbox::offsetFromBase(bus_addr);

    // Add the offset, then cast to a volatile‐int pointer and dereference.
    return *reinterpret_cast<volatile int *>(dma_config_.peripheral_base_virtual + offset);
}

/**
 * @brief Sets a specified bit at a bus address in the peripheral address space.
 *
 * This function accesses the virtual address corresponding to the given bus address using
 * the `access_bus_address()` function and sets the bit at the provided position.
 *
 * @param base The bus address in the peripheral address space.
 * @param bit The bit number to set (0-indexed).
 */
inline void WsprTransmitter::set_bit_bus_address(std::uintptr_t base, unsigned int bit)
{
    access_bus_address(base) |= 1 << bit;
}

/**
 * @brief Clears a specified bit at a bus address in the peripheral address space.
 *
 * This function accesses the virtual address corresponding to the given bus address using
 * the `access_bus_address()` function and clears the bit at the provided position.
 *
 * @param base The bus address in the peripheral address space.
 * @param bit The bit number to clear (0-indexed).
 */
inline void WsprTransmitter::clear_bit_bus_address(std::uintptr_t base, unsigned int bit)
{
    access_bus_address(base) &= ~(1 << bit);
}

/**
 * @brief Computes the difference between two time values.
 * @details Calculates `t2 - t1` and stores the result in `result`. If `t2 < t1`,
 *          the function returns `1`, otherwise, it returns `0`.
 *
 * @param[out] result Pointer to `timeval` struct to store the difference.
 * @param[in] t2 Pointer to the later `timeval` structure.
 * @param[in] t1 Pointer to the earlier `timeval` structure.
 * @return Returns `1` if the difference is negative (t2 < t1), otherwise `0`.
 */
int WsprTransmitter::symbol_timeval_subtract(struct timeval *result, const struct timeval *t2, const struct timeval *t1)
{
    // Compute the time difference in microseconds
    long int diff_usec = (t2->tv_usec + 1000000 * t2->tv_sec) - (t1->tv_usec + 1000000 * t1->tv_sec);

    // Compute seconds and microseconds for the result
    result->tv_sec = diff_usec / 1000000;
    result->tv_usec = diff_usec % 1000000;

    // Return 1 if t2 < t1 (negative difference), otherwise 0
    return (diff_usec < 0) ? 1 : 0;
}

/**
 * @brief Initialize DMAConfig PLLD frequencies and mailbox memory flag.
 *
 * @details
 *   1. Reads the Raspberry Pi hardware revision from `/proc/cpuinfo` (cached
 *      after first read).
 *   2. Determines the processor ID (BCM2835, BCM2836/37, or BCM2711).
 *   3. Sets `dma_config_.plld_nominal_freq` to the board’s true PLLD base
 *      frequency (500 MHz for Pi 1/2/3, 750 MHz for Pi 4).
 *   4. Initializes `dma_config_.plld_clock_frequency` equal to
 *      `plld_nominal_freq` (zero PPM correction).
 *
 * @throws std::runtime_error if the processor ID is unrecognized.
 */
void WsprTransmitter::get_plld()
{
    // Cache the revision to avoid repeated file I/O
    static std::optional<unsigned> cached_revision;
    if (!cached_revision)
    {
        std::ifstream file("/proc/cpuinfo");
        if (file)
        {
            std::string line;
            unsigned value = 0;
            const std::string pattern = "Revision\t: %x";
            while (std::getline(file, line))
            {
                if (sscanf(line.c_str(), pattern.c_str(), &value) == 1)
                {
                    cached_revision = value;
                    break;
                }
            }
        }
        if (!cached_revision)
        {
            cached_revision = 0; // Fallback if parsing fails
        }
    }

    unsigned rev = *cached_revision;
    BCMChip proc_id;

    if (rev & 0x800000)
    {
        auto raw = (rev & 0xF000) >> 12;
        proc_id = static_cast<BCMChip>(raw);
    }
    else
    {
        proc_id = BCMChip::BCM_HOST_PROCESSOR_BCM2835;
    }

    double base_freq_hz = 500e6;
    switch (proc_id)
    {
    case BCMChip::BCM_HOST_PROCESSOR_BCM2835: // Pi 1
        base_freq_hz = 500e6;
        break;

    case BCMChip::BCM_HOST_PROCESSOR_BCM2836: // Pi 2
    case BCMChip::BCM_HOST_PROCESSOR_BCM2837: // Pi 3
        base_freq_hz = 500e6;
        break;

    case BCMChip::BCM_HOST_PROCESSOR_BCM2711: // Pi 4
        base_freq_hz = 750e6;
        break;

    default:
        throw std::runtime_error(
            std::string("Error: Unknown chipset (") +
            std::string(to_string(proc_id)) + ")");
    }

    // Store nominal and initial (zero‑PPM) working frequency
    dma_config_.plld_nominal_freq = base_freq_hz;
    dma_config_.plld_clock_frequency = base_freq_hz;

    // Sanity check
    if (dma_config_.plld_clock_frequency <= 0)
    {
        std::cerr << "Error: Invalid PLLD frequency; defaulting to 500 MHz" << std::endl;
        dma_config_.plld_nominal_freq = 500e6;
        dma_config_.plld_clock_frequency = 500e6;
    }
}

/**
 * @brief Allocate a pool of DMA‑capable memory pages.
 *
 * @details Uses the Broadcom mailbox interface to:
 *   1. Allocate a contiguous block of physical pages.
 *   2. Lock the block and retrieve its bus address.
 *   3. Map the block into user space for CPU access.
 *   4. Track the pool size and reset the per‑page allocation counter.
 *
 * When called with `numpages = 1025`, one page is reserved for constant data
 * and 1024 pages are used for building the DMA instruction chain.
 *
 * @param numpages Total number of pages to allocate (1 constant + N instruction pages).
 * @throws std::runtime_error if mailbox allocation, locking, or mapping fails.
 */
void WsprTransmitter::allocate_memory_pool(unsigned numpages)
{
    // Allocate a contiguous block of physical pages
    mailbox_struct_.mem_ref = mailbox.memAlloc(
        Mailbox::PAGE_SIZE * numpages,
        Mailbox::BLOCK_SIZE);
    if (mailbox_struct_.mem_ref == 0)
    {
        throw std::runtime_error("Error: memAlloc failed.");
    }

    // Lock the block to obtain its bus address
    mailbox_struct_.bus_addr = mailbox.memLock(mailbox_struct_.mem_ref);
    if (mailbox_struct_.bus_addr == 0)
    {
        mailbox.memFree(mailbox_struct_.mem_ref);
        throw std::runtime_error("Error: memLock failed.");
    }

    // Map the locked pages into user‑space virtual memory
    mailbox_struct_.virt_addr = mailbox.mapMem(mailbox.busToPhysical(mailbox_struct_.bus_addr), Mailbox::PAGE_SIZE * numpages);
    if (mailbox_struct_.virt_addr == nullptr)
    {
        mailbox.memUnlock(mailbox_struct_.mem_ref);
        mailbox.memFree(mailbox_struct_.mem_ref);
        throw std::runtime_error("Error: mapMem failed.");
    }

    // Record pool parameters
    mailbox_struct_.pool_size = numpages; // total pages available
    mailbox_struct_.pool_cnt = 0;         // pages allocated so far
}

/**
 * @brief Retrieves the next available memory page from the allocated pool.
 * @details Provides a virtual and bus address for a memory page in the pool.
 *          If no more pages are available, the function prints an error and exits.
 *
 * @param[out] vAddr Pointer to store the virtual address of the allocated page.
 * @param[out] bAddr Pointer to store the bus address of the allocated page.
 */
void WsprTransmitter::get_real_mem_page_from_pool(void **vAddr, void **bAddr)
{
    // Ensure that we do not exceed the allocated pool size.
    if (mailbox_struct_.pool_cnt >= mailbox_struct_.pool_size)
    {
        throw std::runtime_error("Error: unable to allocate more pages.");
    }

    // Compute the offset for the next available page.
    unsigned offset = mailbox_struct_.pool_cnt * Mailbox::PAGE_SIZE;

    // Retrieve the virtual and bus addresses based on the offset.
    *vAddr = reinterpret_cast<void *>(reinterpret_cast<uintptr_t>(mailbox_struct_.virt_addr) + offset);
    *bAddr = reinterpret_cast<void *>(reinterpret_cast<uintptr_t>(mailbox_struct_.bus_addr) + offset);

    // Increment the count of allocated pages.
    mailbox_struct_.pool_cnt++;
}

/**
 * @brief Deallocates the memory pool.
 * @details Releases the allocated memory by unmapping virtual memory,
 *          unlocking, and freeing the memory via the mailbox interface.
 */
void WsprTransmitter::deallocate_memory_pool()
{
    // Free virtual memory mapping if it was allocated.
    if (mailbox_struct_.virt_addr != nullptr)
    {
        mailbox.unMapMem(mailbox_struct_.virt_addr, mailbox_struct_.pool_size * Mailbox::PAGE_SIZE);
        mailbox_struct_.virt_addr = nullptr; // Prevent dangling pointer usage
    }

    // Free the allocated memory block if it was successfully allocated.
    if (mailbox_struct_.mem_ref != 0)
    {
        mailbox.memUnlock(mailbox_struct_.mem_ref);
        mailbox.memFree(mailbox_struct_.mem_ref);
        mailbox_struct_.mem_ref = 0; // Ensure it does not reference a freed block
    }

    // Reset pool tracking variables
    mailbox_struct_.pool_size = 0;
    mailbox_struct_.pool_cnt = 0;
}

/**
 * @brief Disables the PWM clock.
 * @details Clears the enable bit in the clock control register and waits
 *          until the clock is no longer busy. Ensures proper synchronization.
 */
void WsprTransmitter::disable_clock()
{
    // Set semaphore
    transmit_on_.store(false);

    // Ensure memory-mapped peripherals are initialized before proceeding.
    if (dma_config_.peripheral_base_virtual == nullptr)
    {
        return;
    }

    // Read current clock settings from the clock control register.
    auto settings = access_bus_address(CM_GP0CTL_BUS);

    // Disable the clock: clear the enable bit while preserving other settings.
    // Apply the required password (0x5A000000) to modify the register.
    settings = (settings & 0x7EF) | 0x5A000000;
    access_bus_address(CM_GP0CTL_BUS) = static_cast<int>(settings);

    // Wait until the clock is no longer busy.
    while (access_bus_address(CM_GP0CTL_BUS) & (1 << 7))
    {
        // Busy-wait loop to ensure clock disable is complete.
    }
}

/**
 * @brief Enables TX by configuring GPIO4 and setting the clock source.
 * @details Configures GPIO4 to use alternate function 0 (GPCLK0), sets the drive
 *          strength, disables any active clock, and then enables the clock with PLLD.
 */
void WsprTransmitter::transmit_on()
{
    // Configure GPIO4 function select (Fsel) to alternate function 0 (GPCLK0).
    // This setting follows Section 6.2 of the ARM Peripherals Manual.
    set_bit_bus_address(GPIO_BUS_BASE, 14);   // Set bit 14
    clear_bit_bus_address(GPIO_BUS_BASE, 13); // Clear bit 13
    clear_bit_bus_address(GPIO_BUS_BASE, 12); // Clear bit 12

    // Set GPIO drive strength, values range from 2mA (-3.4dBm) to 16mA (+10.6dBm)
    access_bus_address(PADS_GPIO_0_27_BUS) = 0x5a000018 + trans_params_.power;

    // Define clock control structure and set PLLD as the clock source.
    struct GPCTL setupword = {6 /*SRC*/, 0, 0, 0, 0, 3, 0x5A};

    // Enable the clock by modifying the control word.
    setupword = {6 /*SRC*/, 1, 0, 0, 0, 3, 0x5A};
    int temp;
    std::memcpy(&temp, &setupword, sizeof(int));

    // Apply clock control settings.
    access_bus_address(CM_GP0CTL_BUS) = temp;

    // Set semaphore
    transmit_on_.store(true);
}

/**
 * @brief Disables the transmitter.
 * @details Turns off the transmission by disabling the clock source.
 */
void WsprTransmitter::transmit_off()
{
    // Disable the clock, effectively turning off transmission.
    disable_clock();
}

/**
 * @brief Transmits a symbol for a specified duration using DMA.
 * @details Configures the DMA to transmit a symbol (tone) for the specified
 *          time interval (`tsym`). Uses PWM clocks and adjusts frequency
 *          dynamically based on the desired tone spacing.
 *
 * @param[in] sym_num The symbol number to transmit.
 * @param[in] tsym The duration (seconds) for which the symbol is transmitted.
 * @param[in,out] bufPtr The buffer pointer index for DMA instruction handling.
 */
void WsprTransmitter::transmit_symbol(
    const int &sym_num,
    const double &tsym,
    int &bufPtr)
{
    // Early-exit if a stop was already requested
    if (stop_requested_.load(std::memory_order_acquire))
    {
        if (debug)
            std::cout << debug_tag << "transmit_symbol(" << sym_num
                      << ") aborted before start." << std::endl;
        return;
    }

    if (debug && dma_table_freq_.size() >= 2)
        std::cerr << debug_tag
                  << "DMA f0: " << dma_table_freq_[0]
                  << ", f1: " << dma_table_freq_[1]
                  << std::endl;

    const bool is_tone = (tsym == 0.0);
    const int f0_idx = 0;
    const int f1_idx = 1;

    if (is_tone)
    {
        // Continuous tone
        while (!stop_requested_.load(std::memory_order_acquire))
        {
            const long int n_pwmclk = PWM_CLOCKS_PER_ITER_NOMINAL;

            // SOURCE_AD
            bufPtr = (bufPtr + 1) & 0x3FF;
            {
                std::unique_lock<std::mutex> lk(stop_mutex_);
                stop_cv_.wait_for(lk, std::chrono::milliseconds(1), [&]
                                  { return stop_requested_.load(std::memory_order_acquire) || static_cast<std::uintptr_t>(
                                                                                                  access_bus_address(DMA_BUS_BASE + 0x04)) != reinterpret_cast<std::uintptr_t>(instructions_[bufPtr].b); });
                if (stop_requested_.load(std::memory_order_acquire))
                    return;
            }
            reinterpret_cast<CB *>(instructions_[bufPtr].v)->SOURCE_AD =
                reinterpret_cast<std::uintptr_t>(const_page_.b) + f0_idx * 4;

            // TXFR_LEN
            bufPtr = (bufPtr + 1) & 0x3FF;
            {
                std::unique_lock<std::mutex> lk(stop_mutex_);
                stop_cv_.wait_for(lk, std::chrono::milliseconds(1), [&]
                                  { return stop_requested_.load(std::memory_order_acquire) || static_cast<std::uintptr_t>(
                                                                                                  access_bus_address(DMA_BUS_BASE + 0x04)) != reinterpret_cast<std::uintptr_t>(instructions_[bufPtr].b); });
                if (stop_requested_.load(std::memory_order_acquire))
                    return;
            }
            reinterpret_cast<CB *>(instructions_[bufPtr].v)->TXFR_LEN = n_pwmclk;
        }
    }
    else
    {
        // Calculate PWM freq
        const long int n_pwmclk_per_sym = std::lround(pwm_clock_init_ * tsym);
        long int n_pwmclk_transmitted = 0;
        long int n_f0_transmitted = 0;

        // Precompute interpolation ratio outside the loop
        const double f0_freq = trans_params_.dma_table_freq[f0_idx];
        const double f1_freq = trans_params_.dma_table_freq[f1_idx];
        const double tone_freq =
            trans_params_.frequency - 1.5 * trans_params_.tone_spacing + sym_num * trans_params_.tone_spacing;
        const double f0_ratio =
            1.0 - (tone_freq - f0_freq) / (f1_freq - f0_freq);

        while (n_pwmclk_transmitted < n_pwmclk_per_sym &&
               !stop_requested_.load(std::memory_order_acquire))
        {
            // --- compute clocks for this chunk ---
            long int n_pwmclk =
                PWM_CLOCKS_PER_ITER_NOMINAL;
            n_pwmclk += std::lround(
                (std::rand() / (RAND_MAX + 1.0) - 0.5) * n_pwmclk);
            if (n_pwmclk_transmitted + n_pwmclk > n_pwmclk_per_sym)
                n_pwmclk = n_pwmclk_per_sym - n_pwmclk_transmitted;

            const long int n_f0 = std::lround(f0_ratio * (n_pwmclk_transmitted + n_pwmclk)) - n_f0_transmitted;
            const long int n_f1 = n_pwmclk - n_f0;

            // f0 SOURCE_AD
            bufPtr = (bufPtr + 1) & 0x3FF;
            {
                std::unique_lock<std::mutex> lk(stop_mutex_);
                stop_cv_.wait_for(lk, std::chrono::milliseconds(1), [&]
                                  { return stop_requested_.load(std::memory_order_acquire) || static_cast<std::uintptr_t>(
                                                                                                  access_bus_address(DMA_BUS_BASE + 0x04)) != reinterpret_cast<std::uintptr_t>(instructions_[bufPtr].b); });
                if (stop_requested_.load(std::memory_order_acquire))
                    return;
            }
            reinterpret_cast<CB *>(instructions_[bufPtr].v)->SOURCE_AD =
                reinterpret_cast<std::uintptr_t>(const_page_.b) + f0_idx * 4;

            // f0 TXFR_LEN
            bufPtr = (bufPtr + 1) & 0x3FF;
            {
                std::unique_lock<std::mutex> lk(stop_mutex_);
                stop_cv_.wait_for(lk, std::chrono::milliseconds(1), [&]
                                  { return stop_requested_.load(std::memory_order_acquire) || static_cast<std::uintptr_t>(
                                                                                                  access_bus_address(DMA_BUS_BASE + 0x04)) != reinterpret_cast<std::uintptr_t>(instructions_[bufPtr].b); });
                if (stop_requested_.load(std::memory_order_acquire))
                    return;
            }
            reinterpret_cast<CB *>(instructions_[bufPtr].v)->TXFR_LEN = n_f0;

            // f1 SOURCE_AD
            bufPtr = (bufPtr + 1) & 0x3FF;
            {
                std::unique_lock<std::mutex> lk(stop_mutex_);
                stop_cv_.wait_for(lk, std::chrono::milliseconds(1), [&]
                                  { return stop_requested_.load(std::memory_order_acquire) || static_cast<std::uintptr_t>(
                                                                                                  access_bus_address(DMA_BUS_BASE + 0x04)) != reinterpret_cast<std::uintptr_t>(instructions_[bufPtr].b); });
                if (stop_requested_.load(std::memory_order_acquire))
                    return;
            }
            reinterpret_cast<CB *>(instructions_[bufPtr].v)->SOURCE_AD =
                reinterpret_cast<std::uintptr_t>(const_page_.b) + f1_idx * 4;

            // f1 TXFR_LEN
            bufPtr = (bufPtr + 1) & 0x3FF;
            {
                std::unique_lock<std::mutex> lk(stop_mutex_);
                stop_cv_.wait_for(lk, std::chrono::milliseconds(1), [&]
                                  { return stop_requested_.load(std::memory_order_acquire) || static_cast<std::uintptr_t>(
                                                                                                  access_bus_address(DMA_BUS_BASE + 0x04)) != reinterpret_cast<std::uintptr_t>(instructions_[bufPtr].b); });
                if (stop_requested_.load(std::memory_order_acquire))
                    return;
            }
            reinterpret_cast<CB *>(instructions_[bufPtr].v)->TXFR_LEN = n_f1;

            // Update counters
            n_pwmclk_transmitted += n_pwmclk;
            n_f0_transmitted += n_f0;
        }
    }
}

/**
 * @brief Disables and resets the DMA engine.
 * @details Ensures that the DMA controller is properly reset before exiting.
 *          If the peripheral memory mapping is not set up, the function returns early.
 */
void WsprTransmitter::clear_dma_setup()
{
    // Turn off transmission.
    transmit_off();

    // Ensure memory-mapped peripherals are initialized before proceeding.
    if (dma_config_.peripheral_base_virtual == nullptr)
    {
        return;
    }

    // Obtain a pointer to the DMA control registers.
    volatile DMAregs *DMA0 = reinterpret_cast<volatile DMAregs *>(&(access_bus_address(DMA_BUS_BASE)));

    // Reset the DMA controller by setting the reset bit (bit 31) in the control/status register.
    DMA0->CS = 1 << 31;
}

/**
 * @brief Truncates a floating-point number at a specified bit position.
 * @details Sets all bits less significant than the given LSB to zero.
 *
 * @param d The input floating-point number to be truncated.
 * @param lsb The least significant bit position to retain.
 * @return The truncated value with lower bits set to zero.
 */
double WsprTransmitter::bit_trunc(const double &d, const int &lsb)
{
    // Compute the truncation factor as a power of 2.
    const double factor = std::pow(2.0, lsb);

    // Truncate the number by dividing, flooring, and multiplying back.
    return std::floor(d / factor) * factor;
}

/**
 * @brief Configures and initializes DMA for PWM signal generation.
 * @details Allocates memory pages, creates DMA control blocks, sets up a
 *          circular inked list of DMA instructions, and configures the
 *          PWM clock and registers.
 *
 * @param[out] const_page_ PageInfo structure for storing constant data.
 * @param[out] instr_page_ PageInfo structure for the initial DMA instruction
 *                       page.
 * @param[out] instructions_ Array of PageInfo structures for DMA instructions.
 */
void WsprTransmitter::create_dma_pages(
    struct PageInfo &const_page_,
    struct PageInfo &instr_page_,
    struct PageInfo instructions_[])
{
    // Allocate memory pool for DMA operation
    allocate_memory_pool(1025);

    // Allocate a memory page for storing constants
    {
        // Allocate a real memory page for constants
        void *tmp_v, *tmp_b;
        get_real_mem_page_from_pool(&tmp_v, &tmp_b);
        const_page_.v = tmp_v;
        const_page_.b = reinterpret_cast<std::uintptr_t>(tmp_b);
    }

    // Initialize instruction counter
    int instrCnt = 0;

    // Allocate memory pages and create DMA instructions
    while (instrCnt < 1024)
    {
        // Allocate a memory page for instructions
        {
            // Allocate a real memory page for this CB page
            void *tmp_v, *tmp_b;
            get_real_mem_page_from_pool(&tmp_v, &tmp_b);
            instr_page_.v = tmp_v;
            instr_page_.b = reinterpret_cast<std::uintptr_t>(tmp_b);
        }

        // Create DMA control blocks (CBs)
        struct CB *instr0 = reinterpret_cast<struct CB *>(instr_page_.v);

        for (int i = 0; i < static_cast<int>(Mailbox::PAGE_SIZE / sizeof(struct CB)); i++)
        {
            // Assign virtual and bus addresses for each instruction
            instructions_[instrCnt].v = static_cast<void *>(static_cast<char *>(instr_page_.v) + sizeof(struct CB) * i);
            instructions_[instrCnt].b = instr_page_.b + static_cast<std::uintptr_t>(sizeof(struct CB) * i);

            // Configure DMA transfer: Source = constant memory page, Destination = PWM FI
            // On 64-bit, const_page_.b is already a uintptr_t; truncate to 32 bits for the register.
            instr0->SOURCE_AD = static_cast<uint32_t>(const_page_.b + 2048);
            instr0->DEST_AD = PWM_BUS_BASE + 0x18; // FIFO1
            instr0->TXFR_LEN = 4;
            instr0->STRIDE = 0;
            instr0->TI = (1 << 6) | (5 << 16) | (1 << 26); // DREQ = 1, PWM = 5, No wide mode
            instr0->RES1 = 0;
            instr0->RES2 = 0;

            // Odd instructions modify the GP0 clock divider instead of PWM FIFO
            if (i % 2)
            {
                instr0->DEST_AD = CM_GP0DIV_BUS;
                instr0->STRIDE = 4;
                instr0->TI = (1 << 26); // No wide mode
            }

            // Link previous instruction to the next in the DMA sequence
            if (instrCnt != 0)
            {
                // On 64-bit, truncate the bus address to 32 bits for the DMA engine:
                reinterpret_cast<volatile CB *>(instructions_[instrCnt - 1].v)
                    ->NEXTCONBK = static_cast<uint32_t>(instructions_[instrCnt].b);
            }

            instr0++;
            instrCnt++;
        }
    }

    // Create a circular linked list of DMA instructions (64-bit safe)
    reinterpret_cast<volatile CB *>(instructions_[1023].v)
        ->NEXTCONBK = static_cast<uint32_t>(instructions_[0].b);
    // Create a circular linked list of DMA instructions (64-bit safe)
    reinterpret_cast<volatile CB *>(instructions_[1023].v)
        ->NEXTCONBK = static_cast<uint32_t>(instructions_[0].b);

    // Configure the PWM clock (disable, set divisor, enable)
    access_bus_address(CLK_BUS_BASE + 40 * 4) = 0x5A000026; // Source = PLLD, disable
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    access_bus_address(CLK_BUS_BASE + 41 * 4) = 0x5A002000; // Set PWM divider to 2 (250MHz)
    access_bus_address(CLK_BUS_BASE + 40 * 4) = 0x5A000016; // Source = PLLD, enable
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // Configure PWM registers
    access_bus_address(PWM_BUS_BASE + 0x0) = 0; // Disable PWM
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    access_bus_address(PWM_BUS_BASE + 0x4) = -1; // Clear status errors
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    access_bus_address(PWM_BUS_BASE + 0x10) = 32; // Set default range
    access_bus_address(PWM_BUS_BASE + 0x20) = 32;
    access_bus_address(PWM_BUS_BASE + 0x0) = -1; // Enable FIFO mode, repeat, serializer, and channel
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    access_bus_address(PWM_BUS_BASE + 0x8) = (1 << 31) | 0x0707; // Enable DMA

    // Obtain the base address as an integer pointer
    //
    // Compute the byte‐offset from the bus base (0x7E000000) to your desired
    // DMA register block (DMA_BUS_BASE).
    std::uintptr_t delta = DMA_BUS_BASE - Mailbox::PERIPH_BUS_BASE;

    // Add the offset, then cast to your register pointer
    volatile uint8_t *dma_base = dma_config_.peripheral_base_virtual + delta;

    // Cast to DMAregs pointer to activate DMA
    volatile struct DMAregs *DMA0 = reinterpret_cast<volatile struct DMAregs *>(dma_base);
    DMA0->CS = 1 << 31; // Reset DMA
    DMA0->CONBLK_AD = 0;
    DMA0->TI = 0;
    // on a 64-bit build, DMA regs are only 32 bits wide
    DMA0->CONBLK_AD = static_cast<uint32_t>(instr_page_.b);
    DMA0->CS = (1 << 0) | (255 << 16); // Enable DMA, priority level 255
}

/**
 * @brief Create a vector of DMA pages for a specific frequency.
 *
 * @details This function generates the packed frequency divisor table
 *          using `create_dma_freq_table()` for the given frequency,
 *          wraps it in a `DmaPage` struct, and returns it inside a vector.
 *          This structure is used to represent DMA-friendly waveform data
 *          for a specific output frequency.
 *
 * @param freq The desired output frequency in Hz.
 *
 * @return A vector of `DmaPage` structures, each containing the DMA waveform
 *         data for the specified frequency.
 */
std::vector<WsprTransmitter::DmaPage> WsprTransmitter::create_dma_pages(double freq)
{
    if (debug)
        std::cerr << debug_tag
                  << " Creating DMA pages for freq = "
                  << freq
                  << " Hz"
                  << std::endl;

    std::vector<DmaPage> result;

    // Generate the packed frequency divisor table
    std::vector<uint32_t> packed_table = create_dma_freq_table(freq);

    // Wrap the packed table into a DmaPage
    DmaPage page;
    page.data = std::move(packed_table);

    // Store the page in the result vector
    result.push_back(std::move(page));

    return result;
}

/**
 * @brief Configure and initialize the DMA system for WSPR transmission.
 *
 * @details Performs the following steps in order:
 *   1. Retrieve and configure the PLLD clock frequency and DMA memory flag.
 *   2. Map the peripheral base address into user space.
 *   3. Save the original clock and PWM register values for later restoration.
 *   4. Open the Broadcom mailbox interface for DMA memory allocation.
 *   5. Allocate and set up DMA control blocks for constants and instruction pages.
 *
 * @throws std::runtime_error if the PLLD clock cannot be determined.
 * @throws std::runtime_error if peripheral memory mapping fails.
 * @throws std::runtime_error if mailbox opening fails.
 */
void WsprTransmitter::setup_dma()
{
    // Open the mailbox
    mailbox.open();

    // PLLD & mem-flag
    get_plld();

    // Map peripherals via mailbox.mapMem()
    uint32_t base = Mailbox::discoverPeripheralBase();
    dma_config_.peripheral_base_virtual = ::mailbox.mapMem(base, Mailbox::PAGE_SIZE * NUM_PAGES /*=0x01000000*/);

    // Snapshot regs
    dma_config_.orig_gp0ctl = access_bus_address(CM_GP0CTL_BUS);
    dma_config_.orig_gp0div = access_bus_address(CM_GP0DIV_BUS);
    dma_config_.orig_pwm_ctl = access_bus_address(PWM_BUS_BASE + 0x00);
    dma_config_.orig_pwm_sta = access_bus_address(PWM_BUS_BASE + 0x04);
    dma_config_.orig_pwm_rng1 = access_bus_address(PWM_BUS_BASE + 0x10);
    dma_config_.orig_pwm_rng2 = access_bus_address(PWM_BUS_BASE + 0x20);
    dma_config_.orig_pwm_fifocfg = access_bus_address(PWM_BUS_BASE + 0x08);

    constexpr int kMaxAttempts = 3;
    int attempts = 0;
    while (true)
    {
        try
        {
            // This may throw on timeout or other failure
            MailboxMemoryPool pool(1025);
            // Success!  Use 'pool' here…
            break;
        }
        catch (const std::system_error &e)
        {
            if (e.code().value() == ETIMEDOUT)
            {
                if (debug)
                    std::cerr << debug_tag << "Timeout (attempt " << attempts << ") allocating memory pool, retrying.";

                // A timeout, let's retry
                if (++attempts >= kMaxAttempts)
                    throw std::runtime_error("Mailbox::setup_dma() Too many mailbox timeouts, giving up");

                // Cleanly close and reopen the mailbox
                try
                {
                    ::mailbox.close();
                }
                catch (...)
                { /* Swallow */
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                ::mailbox.open();

                // And loop to retry
            }
        }
        catch (...)
        {
            // Some other error—propagate
            throw;
        }
    }

    // Build DMA pages
    create_dma_pages(const_page_, instr_page_, instructions_);

    // Done
    dma_setup_done_ = true;

    // Read back the divisor you just wrote
    uint32_t div_reg = static_cast<uint32_t>(
        access_bus_address(CLK_BUS_BASE + 41 * 4));
    uint32_t divisor = (div_reg >> 12) & 0xFFF; // bits 23–12
    pwm_clock_init_ = dma_config_.plld_clock_frequency / double(divisor);

    if (debug)
        std::cerr
            << debug_tag
            << "Actual PWM clock = "
            << std::fixed << std::setprecision(0)
            << pwm_clock_init_
            << " Hz"
            << std::endl;
}

/**
 * @brief Configures the DMA frequency table for signal generation.
 * @details Generates a tuning word table based on the desired center frequency
 *          and tone spacing, adjusting for hardware limitations if necessary.
 *
 * @param[in] center_freq_desired The desired center frequency in Hz.
 * @param[in] tone_spacing The spacing between frequency tones in Hz.
 * @param[in] plld_actual_freq The actual PLLD clock frequency in Hz.
 * @param[out] center_freq_actual The actual center frequency, which may be adjusted.
 * @param[in,out] const_page_ The PageInfo structure for storing tuning words.
 */
void WsprTransmitter::setup_dma_freq_table(double &center_freq_actual)
{
    // Compute the divider values for the lowest and highest WSPR tones.
    double div_lo = bit_trunc(dma_config_.plld_clock_frequency / (trans_params_.frequency - 1.5 * trans_params_.tone_spacing), -12) + std::pow(2.0, -12);
    double div_hi = bit_trunc(dma_config_.plld_clock_frequency / (trans_params_.frequency + 1.5 * trans_params_.tone_spacing), -12);

    // If the integer portion of dividers differ, adjust the center frequency.
    if (std::floor(div_lo) != std::floor(div_hi))
    {
        center_freq_actual = dma_config_.plld_clock_frequency / std::floor(div_lo) - 1.6 * trans_params_.tone_spacing;
        if (debug && trans_params_.frequency != 0.0)
        {
            std::stringstream temp;
            temp << std::fixed << std::setprecision(6)
                 << debug_tag
                 << "Center frequency has been changed to "
                 << center_freq_actual / 1e6 << " MHz";
            std::cerr << temp.str() << " because of hardware limitations." << std::endl;
        }
    }

    // Initialize tuning word table.
    double tone0_freq = center_freq_actual - 1.5 * trans_params_.tone_spacing;
    std::vector<long int> tuning_word(1024);

    // Generate tuning words for WSPR tones.
    for (int i = 0; i < 8; i++)
    {
        double tone_freq = tone0_freq + (i >> 1) * trans_params_.tone_spacing;
        double div = bit_trunc(dma_config_.plld_clock_frequency / tone_freq, -12);

        // Apply rounding for even indices.
        if (i % 2 == 0)
        {
            div += std::pow(2.0, -12);
        }

        tuning_word[i] = static_cast<int>(div * std::pow(2.0, 12));
    }

    // Fill the remaining table with default values.
    for (int i = 8; i < 1024; i++)
    {
        double div = 500 + i;
        tuning_word[i] = static_cast<int>(div * std::pow(2.0, 12));
    }

    // Program the DMA table.
    for (int i = 0; i < 1024; i++)
    {
        trans_params_.dma_table_freq[i] = dma_config_.plld_clock_frequency / (tuning_word[i] / std::pow(2.0, 12));

        // Store values in the memory-mapped page.
        reinterpret_cast<int *>(const_page_.v)[i] = (0x5A << 24) + tuning_word[i];

        // Ensure adjacent tuning words have the same integer portion for valid tone generation.
        if ((i % 2 == 0) && (i < 8))
        {
            assert((tuning_word[i] & (~0xFFF)) == (tuning_word[i + 1] & (~0xFFF)));
        }
    }
}

void WsprTransmitter::setupCWTransmission(
    double base_freq,
    std::optional<double> alt_freq,
    double ppm)
{
    // Apply PPM correction to PLLD
    dma_config_.plld_clock_frequency =
        dma_config_.plld_nominal_freq * (1.0 - ppm / 1e6);

    if (debug)
        std::cerr
            << "[DEBUG] PLLD clock adjusted to "
            << dma_config_.plld_clock_frequency
            << " Hz (PPM: " << ppm << ")"
            << std::endl;

    // Generate primary tone table
    if (debug)
    std::cerr << "[DEBUG] setupCWTransmission: base = "
              << base_freq << " Hz, alt = "
              << (alt_freq.has_value() ? std::to_string(alt_freq.value()) : "n/a")
              << std::endl;
    dma_table_freq_base_ = create_dma_freq_table(base_freq);

    if (debug)
        std::cerr
            << "[DEBUG] Base freq " << base_freq
            << " Hz → DMA[0] = " << dma_table_freq_base_[0]
            << ", DMA[1] = " << dma_table_freq_base_[1]
            << std::endl;

    // Optional alternate tone (e.g. for spacing or dash in DFCW)
    if (alt_freq.has_value())
    {
        dma_table_freq_shifted_ = create_dma_freq_table(alt_freq.value());

        if (debug)
            std::cerr
                << "[DEBUG] Shifted freq " << alt_freq.value()
                << " Hz → DMA[0] = " << dma_table_freq_shifted_[0]
                << ", DMA[1] = " << dma_table_freq_shifted_[1]
                << std::endl;
    }
    else
    {
        dma_table_freq_shifted_.clear();
    }

    // Clear default table to ensure transmit_symbol uses current one
    dma_table_freq_.clear();
}

std::vector<uint32_t> WsprTransmitter::create_dma_freq_table(double freq)
{
    if (debug)
        std::cerr << "[DEBUG] Creating DMA table for freq = " << freq << " Hz" << std::endl;

    std::vector<uint32_t> table;

    uint32_t plld_clock = dma_config_.plld_clock_frequency;
    double div = static_cast<double>(plld_clock) / freq;
    uint32_t divi = static_cast<uint32_t>(div);
    uint32_t divf = static_cast<uint32_t>((div - divi) * 4096.0);
    uint32_t packed_div = (divi << 12) | (divf & 0xFFF);

    if (debug)
        std::cerr
            << "[DEBUG] Calculated divisor for " << freq << " Hz is "
            << divi << " + " << divf << "/4096"
            << " → packed = 0x" << std::hex << packed_div << std::dec
            << std::endl;

    for (std::size_t i = 0; i < dma_config_.freq_table_length; ++i)
        table.push_back(packed_div);

    return table;
}
