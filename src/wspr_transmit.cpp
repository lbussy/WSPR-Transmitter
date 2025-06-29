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

} // end anonymous namespace

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
 * are configured later via setupTransmission(). This constructor does not
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
void WsprTransmitter::setTransmissionCallbacks(Callback start_cb,
                                               Callback end_cb)
{
    on_transmit_start_ = std::move(start_cb);
    on_transmit_end_ = std::move(end_cb);
}

/**
 * @brief Configure and start a WSPR transmission.
 *
 * @details Performs the following sequence:
 *   1. Set the desired RF frequency and power level.
 *   2. Populate WSPR symbol data if transmitting a message.
 *   3. Determine WSPR mode (2‑tone or 15‑tone) and symbol timing.
 *   4. Optionally apply a random frequency offset to spread spectral load.
 *   5. Initialize DMA and mailbox resources.
 *   6. Apply the specified PPM calibration to the PLLD clock.
 *   7. Rebuild the DMA frequency table with the new PPM‑corrected clock.
 *   8. Update the actual center frequency after any hardware adjustments.
 *
 * @param[in] frequency    Target RF frequency in Hz.
 * @param[in] power        Transmit power index (0‑n).
 * @param[in] ppm          Parts‑per‑million correction to apply (e.g. +11.135).
 * @param[in] callsign     Optional callsign for WSPR message.
 * @param[in] grid_square  Optional Maidenhead grid locator.
 * @param[in] power_dbm    dBm value for WSPR message (ignored if tone).
 * @param[in] use_offset   True to apply a small random offset within band.
 *
 * @throws std::runtime_error if DMA setup or mailbox operations fail.
 */
void WsprTransmitter::setupTransmission(
    double frequency,
    int power,
    double ppm,
    std::string call_sign,
    std::string grid_square,
    int power_dbm,
    bool use_offset)
{
    // If we’ve already got DMA up, tear down the old run
    if (dma_setup_done_)
    {
        disableTransmission(); // Stop the trandmission and scheduler
        dma_cleanup();         // Now unmap/free everything
    }

    // Clear the stop flag so the next thread can run
    stop_requested_.store(false);

    // Set transmission parameters
    trans_params_.call_sign = call_sign;
    trans_params_.grid_square = grid_square;
    trans_params_.power_dbm = power_dbm;
    trans_params_.frequency = frequency;
    trans_params_.ppm = ppm;
    trans_params_.power = power;
    trans_params_.use_offset = use_offset;

    // Default to tone‑only; load WSPR message if provided
    if (!trans_params_.call_sign.empty() && !trans_params_.grid_square.empty() && trans_params_.power_dbm != 0)
    {
        trans_params_.is_tone = false;
        WsprMessage msg(trans_params_.call_sign, trans_params_.grid_square, trans_params_.power_dbm);
        std::copy_n(msg.symbols, msg.size, trans_params_.symbols.begin());
    }
    else
    {
        trans_params_.is_tone = true;
    }

    // Choose WSPR mode and symbol timing
    int offset_freq = 0;
    // WSPR‑2 mode
    trans_params_.symtime = WSPR_SYMTIME;
    if (trans_params_.use_offset)
        offset_freq = WSPR_RAND_OFFSET;
    trans_params_.tone_spacing = 1.0 / trans_params_.symtime;

    // Apply random offset if requested
    if (trans_params_.use_offset)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-1.0, 1.0);
        // Skip offset if frequency is 0.0
        if (trans_params_.frequency != 0.0)
            trans_params_.frequency += dis(gen) * offset_freq;
    }

    // Initialize DMA, mapping, and control blocks
    setup_dma();

    // Apply PPM correction to the PLLD clock
    // (use a stored nominal base frequency for repeatable resets)
    dma_config_.plld_clock_frequency =
        dma_config_.plld_nominal_freq * (1 - ppm / 1e6);

    // Build the DMA frequency lookup table with new clock
    // Sets center_actual according to hardware limitations
    double center_actual = trans_params_.frequency;
    setup_dma_freq_table(center_actual);

    // Update actual frequency after any hardware adjustments
    // Do not update if frequency = 0 (skip)
    if (trans_params_.frequency != 0.0)
        trans_params_.frequency = center_actual;
}

/**
 * @brief Rebuild the DMA tuning‐word table with a fresh PPM correction.
 *
 * @param ppm_new The new parts‑per‑million offset (e.g. +11.135).
 * @throws std::runtime_error if peripherals aren’t mapped.
 */
void WsprTransmitter::updateDMAForPPM(double ppm_new)
{
    // Apply the PPM correction to your working PLLD clock.
    dma_config_.plld_clock_frequency =
        dma_config_.plld_nominal_freq * (1.0 - ppm_new / 1e6);

    // Recompute the DMA frequency table in place.
    // Pass in your current center frequency so it can adjust if needed.
    double center_actual = trans_params_.frequency;
    setup_dma_freq_table(center_actual);
    if (trans_params_.frequency != 0.0)
        trans_params_.frequency = center_actual;
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
 *   – If `trans_params_.is_tone == true`, this will spawn the TX thread
 *     immediately (bypassing the scheduler) so you can start/stop your
 *     test tone on demand.
 *   – Otherwise it launches the scheduler thread, which waits for the next
 *     valid WSPR window and then fires off a transmission.
 */
void WsprTransmitter::enableTransmission()
{
    stop_requested_.store(false, std::memory_order_release);
    if (trans_params_.is_tone)
    {
        // immediate start for tone tests
        tx_thread_ = std::thread(&WsprTransmitter::thread_entry, this);
    }
    else
    {
        // Scheduled WSPR window
        scheduler_.start();
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
    disableTransmission(); // stops scheduler, signals & joins tx_thread_
    dma_cleanup();         // unmaps peripherals, frees mailbox memory
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
 * @brief Prints current transmission parameters and encoded WSPR symbols.
 *
 * @details Displays the configured WSPR parameters including frequency,
 * power, mode, tone/test settings, and symbol timing. If tone mode is
 * enabled (`trans_params_.is_tone == true`), message-related fields like
 * call sign and symbols are shown as "N/A".
 *
 * This function is useful for debugging and verifying that all transmission
 * settings and symbol sequences are correctly populated before transmission.
 */
void WsprTransmitter::printParameters()
{
    // General transmission metadata
    std::cout << "Call Sign:         "
              << (trans_params_.is_tone ? "N/A" : trans_params_.call_sign) << std::endl;

    std::cout << "Grid Square:       "
              << (trans_params_.is_tone ? "N/A" : trans_params_.grid_square) << std::endl;

    std::cout << "WSPR Frequency:    "
              << std::fixed << std::setprecision(6)
              << (trans_params_.frequency / 1.0e6) << " MHz" << std::endl;

    std::cout << "GPIO Power:        "
              << std::fixed << std::setprecision(1)
              << convert_mw_dbm(get_gpio_power_mw(trans_params_.power)) << " dBm" << std::endl;

    std::cout << "Test Tone:         "
              << (trans_params_.is_tone ? "True" : "False") << std::endl;

    std::cout << "WSPR Symbol Time:  "
              << (trans_params_.is_tone ? "N/A" : std::to_string(trans_params_.symtime) + " s") << std::endl;

    std::cout << "WSPR Tone Spacing: "
              << (trans_params_.is_tone ? "N/A" : std::to_string(trans_params_.tone_spacing) + " Hz") << std::endl;

    std::cout << "DMA Table Size:    "
              << trans_params_.dma_table_freq.size() << std::endl;

    // Print symbols unless in tone mode
    if (trans_params_.is_tone)
    {
        std::cout << "WSPR Symbols:      N/A" << std::endl;
    }
    else
    {
        std::cout << "WSPR Symbols:" << std::endl;
        const int symbol_count = static_cast<int>(trans_params_.symbols.size());
        for (int i = 0; i < symbol_count; ++i)
        {
            std::cout << static_cast<int>(trans_params_.symbols[i]);

            if (i < symbol_count - 1)
            {
                std::cout << ", ";
            }

            // Insert newline every 18 symbols for readability
            if ((i + 1) % 18 == 0 && i < symbol_count - 1)
            {
                std::cout << std::endl;
            }
        }
        std::cout << std::endl;
    }
}

/* Private Methods */

/**
 * @brief Invoke the start‐transmission callback with an empty message.
 *
 * @details Ensures the callback is valid before calling, providing an
 *          empty string to match the `Callback` signature.
 *
 * @param cb  The callback to invoke just before starting transmission.
 */
inline void WsprTransmitter::fire_start_cb(const double frequency)
{
    if (on_transmit_start_)
    {
        // Launch callback on a detached thread:
        std::thread([cb = on_transmit_start_, frequency]()
                    { cb(frequency); })
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
inline void WsprTransmitter::fire_end_cb(const std::string &msg)
{
    if (on_transmit_end_)
    {
        // Launch callback on a detached thread:
        std::thread([cb = on_transmit_end_, msg]()
                    { cb(msg); })
            .detach();
    }
}

/**
 * @brief Perform DMA-driven RF transmission.
 *
 * @details
 *   1. Records the start time as a reference for symbol scheduling.
 *   2. Enables the PWM clock and DMA engine for transmission.
 *   3. If in tone mode, continuously transmits a fixed‑frequency test tone.
 *   4. Otherwise, transmits each WSPR symbol in sequence, using gettimeofday()
 *      and `timeval_subtract()` to schedule precise symbol timing.
 *   5. Disables transmission when complete.
 *
 * @note In tone mode (`trans_params_.is_tone == true`), this function only
 *       returns via SIGINT.
 */
void WsprTransmitter::transmit()
{
    // If we're not in tone‐test mode and frequency was zeroed out, skip.
    if (!trans_params_.is_tone && trans_params_.frequency == 0.0)
    {
        fire_end_cb("Skipping transmission (frequency = 0.0).");
        return;
    }

    // Existing stop‐requested check
    if (stop_requested_.load())
    {
        if (debug)
        {
            std::cerr << debug_tag << "transmit(): transmit() aborted before start." << std::endl;
        }
        fire_end_cb("Transmission aborted before start.");
        return;
    }

    // Record reference time for scheduling
    struct timeval tv_begin{};
    gettimeofday(&tv_begin, nullptr);

    // Initialize DMA buffer index
    int bufPtr = 0;

    // Enable PWM clock and DMA transmission
    transmit_on();

    // Choose tone vs WSPR
    if (trans_params_.is_tone)
    {
        // Continuous tone loop — exit as soon as stop_requested_ is true
        while (!stop_requested_.load())
        {
            transmit_symbol(
                0,     // symbol number
                0.0,   // test‐tone
                bufPtr // DMA buffer index
            );
        }
    }
    else
    {
        // Fire callback with frequency as an argument
        fire_start_cb(trans_params_.frequency);
        // Transmit each symbol in the WSPR message
        const int symbol_count =
            static_cast<int>(trans_params_.symbols.size());
        struct timeval sym_start{}, diff{};

        for (int i = 0; i < symbol_count; ++i)
        {
            // Check for stop between symbols
            if (stop_requested_.load())
            {
                break;
            }

            // Compute elapsed time since tv_begin
            gettimeofday(&sym_start, nullptr);
            symbol_timeval_subtract(&diff, &sym_start, &tv_begin);
            double elapsed = diff.tv_sec + diff.tv_usec / 1e6;
            double sched_end = (i + 1) * trans_params_.symtime;
            double time_symbol = sched_end - elapsed;
            time_symbol = std::clamp(
                time_symbol,
                0.2,
                2.0 * trans_params_.symtime);

            // Transmit the current symbol
            int symbol = static_cast<int>(trans_params_.symbols[i]);
            transmit_symbol(
                symbol,      // symbol index
                time_symbol, // scheduled duration
                bufPtr       // DMA buffer index
            );
        }

        // Invoke the completion callback if set
        fire_end_cb();
    }

    // Disable PWM clock and stop transmission
    transmit_off();
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

    const bool is_tone = (tsym == 0.0);
    const int f0_idx = sym_num * 2;
    const int f1_idx = f0_idx + 1;

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
