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

#include "wspr_message.hpp" // WsprMessage Submodule

extern "C"
{
#include "mailbox.h" // Broadcom Mailbox Submodule
}

// C++ Standard Library Headers
#include <algorithm> // std::copy_n, std::clamp
#include <cassert>   // assert()
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

/* Public Methods */

/**
 * @brief Global instance of the WSPR transmitter.
 *
 * @details This instance provides a globally accessible transmitter object
 * for configuring and initiating WSPR transmissions. It is constructed at
 * program startup and destructed automatically on exit.
 */
WsprTransmitter wsprTransmitter;

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
    shutdownTransmitter();
}

/**
 * @brief Install optional callbacks for transmission start/end.
 *
 * @param[in] start_cb
 *   Called on the transmit thread immediately before the first symbol
 *   (or tone) is emitted.  If null, no start notification is made.
 * @param[in] end_cb
 *   Called on the transmit thread immediately after the last WSPR
 *   symbol is sent (but before DMA/PWM are torn down).  If null,
 *   no completion notification is made.
 */
void WsprTransmitter::setTransmissionCallbacks(CompletionCallback start_cb,
                                               CompletionCallback end_cb)
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
        stopTransmission();        // tell the worker to stop
        if (tx_thread_.joinable()) // wait for it to actually exit
            tx_thread_.join();
        dma_cleanup(); // now unmap/free everything
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
    trans_params_.is_tone = true;
    if (!trans_params_.call_sign.empty() && !trans_params_.grid_square.empty() && trans_params_.power_dbm != 0)
    {
        trans_params_.is_tone = false;
        WsprMessage msg(trans_params_.call_sign, trans_params_.grid_square, trans_params_.power_dbm);
        std::copy_n(msg.symbols, msg.size, trans_params_.symbols.begin());
    }

    // Choose WSPR mode and symbol timing
    int offset_freq = 0;
    if (!trans_params_.is_tone &&
        ((trans_params_.frequency > 137600 && trans_params_.frequency < 137625) ||
         (trans_params_.frequency > 475800 && trans_params_.frequency < 475825) ||
         (trans_params_.frequency > 1838200 && trans_params_.frequency < 1838225)))
    {
        // WSPR‑15 mode
        trans_params_.wspr_mode = WsprMode::WSPR15;
        trans_params_.symtime = 8.0 * WSPR_SYMTIME;
        if (trans_params_.use_offset)
            offset_freq = WSPR15_RAND_OFFSET;
    }
    else
    {
        // WSPR‑2 mode
        trans_params_.wspr_mode = WsprMode::WSPR2;
        trans_params_.symtime = WSPR_SYMTIME;
        if (trans_params_.use_offset)
            offset_freq = WSPR_RAND_OFFSET;
    }
    trans_params_.tone_spacing = 1.0 / trans_params_.symtime;

    // Apply random offset if requested
    if (trans_params_.use_offset)
    {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(-1.0, 1.0);
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
    trans_params_.frequency = center_actual;

    // Optional debug output
    if (debug)
    {
        std::cout << std::setprecision(30)
                  << "DEBUG: dma_table_freq[0] = " << trans_params_.dma_table_freq[0] << std::endl
                  << "DEBUG: dma_table_freq[1] = " << trans_params_.dma_table_freq[1] << std::endl
                  << "DEBUG: dma_table_freq[2] = " << trans_params_.dma_table_freq[2] << std::endl
                  << "DEBUG: dma_table_freq[3] = " << trans_params_.dma_table_freq[3] << std::endl;
    }
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
 * @brief Start the background scheduler that will fire at the next
 *        WSPR window and launch the transmit thread.
 *
 * @note This is non‐blocking: returns immediately, scheduler runs in
 *       its own thread.
 */
void WsprTransmitter::enableTransmission()
{
    // Any in-flight tx should be clear
    stop_requested_.store(false, std::memory_order_release);
    scheduler_.start();
}

/**
 * @brief Cancels the scheduler (and any running transmission).
 *
 * Waits for the scheduler thread to stop, and forces any in‐flight
 * transmission to end.
 */
void WsprTransmitter::disableTransmission()
{
    // Atop scheduling further windows
    scheduler_.stop();
    // if a transmit thread is running, signal & join it too
    stop_requested_.store(true, std::memory_order_release);
    stop_cv_.notify_all();
    if (tx_thread_.joinable())
        tx_thread_.join();
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
 * @brief Gracefully stops and waits for the transmission thread.
 *
 * @details Combines stopTransmission() to signal the worker thread to
 *          exit, and join_transmission() to block until that thread has
 *          fully terminated. After this call returns, no transmission
 *          thread remains running.
 */
void WsprTransmitter::shutdownTransmitter()
{
    stopTransmission();
    join_transmission();
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

    std::cout << "WSPR Mode:         "
              << (trans_params_.is_tone ? "N/A" : (trans_params_.wspr_mode == WsprMode::WSPR2 ? "WSPR-2" : "WSPR-15")) << std::endl;

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
    // Early exit if a stop was already requested
    if (stop_requested_.load())
    {
        if (debug)
        {
            std::cerr << "transmit() aborted before start." << std::endl;
        }
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
        /// Do callback if set
        if (on_transmit_start_)
            on_transmit_start_();
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
        if (on_transmit_end_)
            on_transmit_end_();
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
 *   7. Close the mailbox handle.
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
        munmap(dma_config_.peripheral_base_virtual, 0x01000000);
        dma_config_.peripheral_base_virtual = nullptr;
    }

    // Deallocate mailbox-allocated memory pages
    deallocate_memory_pool();

    // Close mailbox handle if open
    if (mailbox_.handle >= 0)
    {
        mbox_close(mailbox_.handle);
        mailbox_.handle = -1;
    }

    // Remove the local device file if it exists
    safe_remove();

    // Reset global configuration structures to defaults
    dma_config_ = DMAConfig(); // Uses your in-class initializers
    mailbox_ = Mailbox();      // Uses your in-class initializers
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
        throw std::out_of_range("Drive strength level must be between 0 and 7");
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
        throw std::domain_error("Input power (mW) must be > 0 to compute logarithm");
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
    // bump our own scheduling parameters first:
    set_thread_priority();
    // actually do the work (blocking until complete or stop_requested_)
    transmit();
}

/**
 * @brief Applies the configured scheduling policy and priority to this thread.
 *
 * @details Builds a sched_param struct using thread_priority_ and invokes
 *          pthread_setschedparam() with thread_policy_ on the current thread.
 *          If the call fails, writes a warning to stderr with the error message.
 */
void WsprTransmitter::set_thread_priority()
{
    // Only real‑time policies use priority > 0
    sched_param sch{};
    sch.sched_priority = thread_priority_;
    int ret = pthread_setschedparam(pthread_self(), thread_policy_, &sch);
    if (ret != 0)
    {
        std::cerr << "Warning: pthread_setschedparam failed: "
                  << std::strerror(ret) << std::endl;
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
    std::uintptr_t offset = bus_addr - 0x7E000000UL;

    // Treat the void* as a char* (1‐byte pointer) so pointer arithmetic
    // is in bytes.
    auto base = static_cast<char *>(dma_config_.peripheral_base_virtual);

    // Add the offset, then cast to a volatile‐int pointer and dereference.
    return *reinterpret_cast<volatile int *>(base + offset);
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
 * @brief Converts a bus address to a physical address.
 *
 * This function converts a given bus address into the corresponding physical address by
 * masking out the upper bits (0xC0000000) that are not part of the physical address.
 *
 * @param x The bus address.
 * @return The physical address obtained from the bus address.
 */
inline std::uintptr_t WsprTransmitter::bus_to_physical(std::uintptr_t x)
{
    return x & ~0xC0000000UL;
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
 *   3. Sets `dma_config_.mem_flag` to the correct mailbox allocation flag.
 *   4. Sets `dma_config_.plld_nominal_freq` to the board’s true PLLD base
 *      frequency (500 MHz for Pi 1/2/3, 750 MHz for Pi 4).
 *   5. Initializes `dma_config_.plld_clock_frequency` equal to
 *      `plld_nominal_freq` (zero PPM correction).
 *
 * @throws std::runtime_error if the processor ID is unrecognized.
 */
void WsprTransmitter::get_plld_and_memflag()
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
            cached_revision = 0; // fallback if parsing fails
        }
    }

    const unsigned rev = *cached_revision;

    // Extract processor ID (high‑bit indicates new format)
    const int proc_id = (rev & 0x800000)
                            ? ((rev & 0xF000) >> 12)
                            : BCM_HOST_PROCESSOR_BCM2835;

    // Determine base PLLD frequency and mailbox flag
    double base_freq_hz = 500e6;
    switch (proc_id)
    {
    case BCM_HOST_PROCESSOR_BCM2835: // Pi 1
        dma_config_.mem_flag = 0x0C;
        base_freq_hz = 500e6;
        break;
    case BCM_HOST_PROCESSOR_BCM2836: // Pi 2
    case BCM_HOST_PROCESSOR_BCM2837: // Pi 3
        dma_config_.mem_flag = 0x04;
        base_freq_hz = 500e6;
        break;
    case BCM_HOST_PROCESSOR_BCM2711: // Pi 4
        dma_config_.mem_flag = 0x04;
        base_freq_hz = 750e6;
        break;
    default:
        throw std::runtime_error(
            "Error: Unknown chipset (" + std::to_string(proc_id) + ")");
    }

    // Store nominal and initial (zero‑PPM) working frequency
    dma_config_.plld_nominal_freq = base_freq_hz;
    dma_config_.plld_clock_frequency = base_freq_hz;

    // Sanity check
    if (dma_config_.plld_clock_frequency <= 0)
    {
        std::cerr << "Error: Invalid PLLD frequency; defaulting to 500 MHz\n";
        dma_config_.plld_nominal_freq = 500e6;
        dma_config_.plld_clock_frequency = 500e6;
    }
}

/**
 * @brief Maps peripheral base address to virtual memory.
 *
 * Reads the Raspberry Pi's device tree to determine the peripheral base
 * address, then memory-maps that region for access via virtual memory.
 *
 * This is used for low-level register access to GPIO, clocks, DMA, etc.
 *
 * @param[out] dma_config_.peripheral_base_virtual Reference to a pointer that will
 *             be set to the mapped virtual memory address.
 *
 * @throws Terminates the program if the peripheral base cannot be determined,
 *         `/dev/mem` cannot be opened, or `mmap` fails.
 */
void WsprTransmitter::setup_peripheral_base_virtual()
{
    auto read_dt_range = [](const std::string &filename, unsigned offset) -> std::optional<unsigned>
    {
        std::ifstream file(filename, std::ios::binary);
        if (!file)
            return std::nullopt;

        file.seekg(offset);
        if (!file.good())
            return std::nullopt;

        unsigned char buf[4] = {};
        file.read(reinterpret_cast<char *>(buf), sizeof(buf));
        if (file.gcount() != sizeof(buf))
            return std::nullopt;

        // Big‑endian to host‑endian conversion
        return (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    };

    unsigned peripheral_base = 0x20000000; // Default fallback
    if (auto addr = read_dt_range("/proc/device-tree/soc/ranges", 4); addr && *addr != 0)
    {
        peripheral_base = *addr;
    }
    else if (auto addr = read_dt_range("/proc/device-tree/soc/ranges", 8); addr)
    {
        peripheral_base = *addr;
    }

    int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mem_fd < 0)
    {
        throw std::runtime_error("Error: Cannot open /dev/mem.");
    }

    // mmap returns void*, so assign directly
    dma_config_.peripheral_base_virtual = mmap(
        nullptr,
        0x01000000,             // 16 MB
        PROT_READ | PROT_WRITE, // read/write
        MAP_SHARED,
        mem_fd,
        peripheral_base);
    close(mem_fd);

    if (!dma_config_.peripheral_base_virtual)
        throw std::runtime_error("peripheral_base_virtual not initialized");

    // Check against MAP_FAILED, not –1
    if (dma_config_.peripheral_base_virtual == MAP_FAILED)
    {
        throw std::runtime_error("Error: peripheral_base_virtual mmap failed.");
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
    mailbox_.mem_ref = mem_alloc(
        mailbox_.handle,
        PAGE_SIZE * numpages,
        BLOCK_SIZE,
        dma_config_.mem_flag);
    if (mailbox_.mem_ref == 0)
    {
        throw std::runtime_error("Error: mem_alloc failed.");
    }

    // Lock the block to obtain its bus address
    mailbox_.bus_addr = mem_lock(mailbox_.handle, mailbox_.mem_ref);
    if (mailbox_.bus_addr == 0)
    {
        mem_free(mailbox_.handle, mailbox_.mem_ref);
        throw std::runtime_error("Error: mem_lock failed.");
    }

    // Map the locked pages into user‑space virtual memory
    mailbox_.virt_addr = static_cast<unsigned char *>(
        mapmem(bus_to_physical(mailbox_.bus_addr), PAGE_SIZE * numpages));
    if (mailbox_.virt_addr == nullptr)
    {
        mem_unlock(mailbox_.handle, mailbox_.mem_ref);
        mem_free(mailbox_.handle, mailbox_.mem_ref);
        throw std::runtime_error("Error: mapmem failed.");
    }

    // Record pool parameters
    mailbox_.pool_size = numpages; // total pages available
    mailbox_.pool_cnt = 0;         // pages allocated so far

    if (debug)
    {
        // Debug output: Show allocated bus & virtual addresses
        std::cout << "DEBUG: allocate_memory_pool bus_addr=0x"
                  << std::hex << mailbox_.bus_addr
                  << " virt_addr=0x" << reinterpret_cast<unsigned long>(mailbox_.virt_addr)
                  << " mem_ref=0x" << mailbox_.mem_ref
                  << std::dec << std::endl;
    }
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
    if (mailbox_.pool_cnt >= mailbox_.pool_size)
    {
        throw std::runtime_error("Error: unable to allocate more pages.");
    }

    // Compute the offset for the next available page.
    unsigned offset = mailbox_.pool_cnt * PAGE_SIZE;

    // Retrieve the virtual and bus addresses based on the offset.
    *vAddr = reinterpret_cast<void *>(reinterpret_cast<uintptr_t>(mailbox_.virt_addr) + offset);
    *bAddr = reinterpret_cast<void *>(reinterpret_cast<uintptr_t>(mailbox_.bus_addr) + offset);

    if (debug)
    {
        // Debug print: Displays allocated memory details.
        std::cout << "DEBUG: get_real_mem_page_from_pool bus_addr=0x"
                  << std::hex << reinterpret_cast<uintptr_t>(*bAddr)
                  << " virt_addr=0x" << reinterpret_cast<uintptr_t>(*vAddr)
                  << std::dec << std::endl;
    }

    // Increment the count of allocated pages.
    mailbox_.pool_cnt++;
}

/**
 * @brief Deallocates the memory pool.
 * @details Releases the allocated memory by unmapping virtual memory,
 *          unlocking, and freeing the memory via the mailbox interface.
 */
void WsprTransmitter::deallocate_memory_pool()
{
    // Free virtual memory mapping if it was allocated.
    if (mailbox_.virt_addr != nullptr)
    {
        unmapmem(mailbox_.virt_addr, mailbox_.pool_size * PAGE_SIZE);
        mailbox_.virt_addr = nullptr; // Prevent dangling pointer usage
    }

    // Free the allocated memory block if it was successfully allocated.
    if (mailbox_.mem_ref != 0)
    {
        mem_unlock(mailbox_.handle, mailbox_.mem_ref);
        mem_free(mailbox_.handle, mailbox_.mem_ref);
        mailbox_.mem_ref = 0; // Ensure it does not reference a freed block
    }

    // Reset pool tracking variables
    mailbox_.pool_size = 0;
    mailbox_.pool_cnt = 0;
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
    // Early‑exit if a stop was already requested
    if (stop_requested_.load())
    {
        if (debug)
            std::cout << "DEBUG: transmit_symbol(" << sym_num
                      << ") aborted before start.\n";
        return;
    }

    // Tone vs WSPR symbol?
    const bool is_tone = (tsym == 0.0);

    // DMA table indices
    const int f0_idx = sym_num * 2;
    const int f1_idx = f0_idx + 1;

    // Frequency bounds
    const double f0_freq = trans_params_.dma_table_freq[f0_idx];
    const double f1_freq = trans_params_.dma_table_freq[f1_idx];

    // Desired tone frequency
    const double tone_freq =
        trans_params_.frequency - 1.5 * trans_params_.tone_spacing + sym_num * trans_params_.tone_spacing;
    assert((tone_freq >= f0_freq) && (tone_freq <= f1_freq));

    // Interpolation ratio
    const double f0_ratio =
        1.0 - (tone_freq - f0_freq) / (f1_freq - f0_freq);
    if (debug)
        std::cout << "DEBUG: f0_ratio = " << f0_ratio << std::endl;
    assert((f0_ratio >= 0.0) && (f0_ratio <= 1.0));

    // Total PWM clocks for this symbol
    const long int n_pwmclk_per_sym =
        std::round(F_PWM_CLK_INIT * tsym);
    long int n_pwmclk_transmitted = 0;
    long int n_f0_transmitted = 0;

    if (debug)
        std::cout << "DEBUG: <instructions_[bufPtr] begin=0x"
                  << std::hex
                  << reinterpret_cast<unsigned long>(&instructions_[bufPtr])
                  << std::dec << ">\n";

    // Transmit
    if (is_tone)
    {
        // Infinite tone until stopped
        while (!stop_requested_.load())
        {
            const long int n_pwmclk =
                PWM_CLOCKS_PER_ITER_NOMINAL;

            // Point DMA to waveform
            bufPtr = (bufPtr + 1) & 0x3FF;
            while (access_bus_address(DMA_BUS_BASE + 0x04) ==
                   reinterpret_cast<long>(instructions_[bufPtr].b))
            {
                if (stop_requested_.load())
                    return;
                usleep(100);
            }
            reinterpret_cast<CB *>(instructions_[bufPtr].v)->SOURCE_AD =
                reinterpret_cast<long>(const_page_.b) + f0_idx * 4;

            // Set transfer length
            bufPtr = (bufPtr + 1) & 0x3FF;
            while (access_bus_address(DMA_BUS_BASE + 0x04) ==
                   reinterpret_cast<long>(instructions_[bufPtr].b))
            {
                if (stop_requested_.load())
                    return;
                usleep(100);
            }
            reinterpret_cast<CB *>(instructions_[bufPtr].v)->TXFR_LEN =
                n_pwmclk;
        }
    }
    else
    {
        // Finite symbol
        while (n_pwmclk_transmitted < n_pwmclk_per_sym &&
               !stop_requested_.load())
        {
            // Compute clocks
            long int n_pwmclk =
                PWM_CLOCKS_PER_ITER_NOMINAL;
            n_pwmclk += std::round(
                (std::rand() / (RAND_MAX + 1.0) - 0.5) * n_pwmclk);
            if (n_pwmclk_transmitted + n_pwmclk >
                n_pwmclk_per_sym)
            {
                n_pwmclk =
                    n_pwmclk_per_sym - n_pwmclk_transmitted;
            }
            const long int n_f0 =
                std::round(f0_ratio * (n_pwmclk_transmitted + n_pwmclk)) - n_f0_transmitted;
            const long int n_f1 = n_pwmclk - n_f0;

            // f0 SOURCE_AD
            bufPtr = (bufPtr + 1) & 0x3FF;
            while (access_bus_address(DMA_BUS_BASE + 0x04) ==
                   reinterpret_cast<long>(instructions_[bufPtr].b))
            {
                if (stop_requested_.load())
                    return;
                usleep(100);
            }
            reinterpret_cast<CB *>(instructions_[bufPtr].v)->SOURCE_AD =
                reinterpret_cast<long>(const_page_.b) + f0_idx * 4;

            // f0 TXFR_LEN
            bufPtr = (bufPtr + 1) & 0x3FF;
            while (access_bus_address(DMA_BUS_BASE + 0x04) ==
                   reinterpret_cast<long>(instructions_[bufPtr].b))
            {
                if (stop_requested_.load())
                    return;
                usleep(100);
            }
            reinterpret_cast<CB *>(instructions_[bufPtr].v)->TXFR_LEN =
                n_f0;

            // f1 SOURCE_AD
            bufPtr = (bufPtr + 1) & 0x3FF;
            while (access_bus_address(DMA_BUS_BASE + 0x04) ==
                   reinterpret_cast<long>(instructions_[bufPtr].b))
            {
                if (stop_requested_.load())
                    return;
                usleep(100);
            }
            reinterpret_cast<CB *>(instructions_[bufPtr].v)->SOURCE_AD =
                reinterpret_cast<long>(const_page_.b) + f1_idx * 4;

            // f1 TXFR_LEN
            bufPtr = (bufPtr + 1) & 0x3FF;
            while (access_bus_address(DMA_BUS_BASE + 0x04) ==
                   reinterpret_cast<long>(instructions_[bufPtr].b))
            {
                if (stop_requested_.load())
                    return;
                usleep(100);
            }
            reinterpret_cast<CB *>(instructions_[bufPtr].v)->TXFR_LEN =
                n_f1;

            // Update counters
            n_pwmclk_transmitted += n_pwmclk;
            n_f0_transmitted += n_f0;
        }
    }

    // Final debug
    if (debug)
        std::cout << "DEBUG: <instructions_[bufPtr]=0x"
                  << std::hex
                  << reinterpret_cast<unsigned long>(instructions_[bufPtr].v)
                  << " 0x"
                  << reinterpret_cast<unsigned long>(instructions_[bufPtr].b)
                  << std::dec << ">\n";
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
 * @brief Opens the mailbox device.
 * @details Creates the mailbox special files and attempts to open the mailbox.
 *          If opening the mailbox fails, an error message is printed, and the program exits.
 */
void WsprTransmitter::open_mbox()
{
    // Attempt to open the mailbox
    mailbox_.handle = mbox_open();

    // Check for failure and handle the error
    if (mailbox_.handle < 0)
    {
        throw std::runtime_error("Error: Failed to open mailbox.");
    }
}

/**
 * @brief Safely removes a file if it exists.
 * @details Checks whether the specified file exists before attempting to remove it.
 *          If the file exists but removal fails, a warning is displayed.
 *
 * @param[in] filename Pointer to a null-terminated string containing the file path.
 */
void WsprTransmitter::safe_remove()
{
    const char *filename = LOCAL_DEVICE_FILE_NAME;
    struct stat buffer;

    // Check if the file exists before attempting to remove it
    if (stat(filename, &buffer) == 0)
    {
        // Attempt to remove the file
        if (unlink(filename) != 0)
        {
            std::cerr << "Warning: Failed to remove " << filename << std::endl;
        }
    }
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
    get_real_mem_page_from_pool(&const_page_.v, &const_page_.b);

    // Initialize instruction counter
    int instrCnt = 0;

    // Allocate memory pages and create DMA instructions
    while (instrCnt < 1024)
    {
        // Allocate a memory page for instructions
        get_real_mem_page_from_pool(&instr_page_.v, &instr_page_.b);

        // Create DMA control blocks (CBs)
        struct CB *instr0 = reinterpret_cast<struct CB *>(instr_page_.v);

        for (int i = 0; i < static_cast<int>(PAGE_SIZE / sizeof(struct CB)); i++)
        {
            // Assign virtual and bus addresses for each instruction
            instructions_[instrCnt].v = reinterpret_cast<void *>(reinterpret_cast<long int>(instr_page_.v) + sizeof(struct CB) * i);
            instructions_[instrCnt].b = reinterpret_cast<void *>(reinterpret_cast<long int>(instr_page_.b) + sizeof(struct CB) * i);

            // Configure DMA transfer: Source = constant memory page, Destination = PWM FIFO
            instr0->SOURCE_AD = reinterpret_cast<unsigned long int>(const_page_.b) + 2048;
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
                reinterpret_cast<struct CB *>(instructions_[instrCnt - 1].v)->NEXTCONBK = reinterpret_cast<long int>(instructions_[instrCnt].b);
            }

            instr0++;
            instrCnt++;
        }
    }

    // Create a circular linked list of DMA instructions
    reinterpret_cast<struct CB *>(instructions_[1023].v)->NEXTCONBK = reinterpret_cast<long int>(instructions_[0].b);

    // Configure the PWM clock (disable, set divisor, enable)
    access_bus_address(CLK_BUS_BASE + 40 * 4) = 0x5A000026; // Source = PLLD, disable
    usleep(1000);
    access_bus_address(CLK_BUS_BASE + 41 * 4) = 0x5A002000; // Set PWM divider to 2 (250MHz)
    access_bus_address(CLK_BUS_BASE + 40 * 4) = 0x5A000016; // Source = PLLD, enable
    usleep(1000);

    // Configure PWM registers
    access_bus_address(PWM_BUS_BASE + 0x0) = 0; // Disable PWM
    usleep(1000);
    access_bus_address(PWM_BUS_BASE + 0x4) = -1; // Clear status errors
    usleep(1000);
    access_bus_address(PWM_BUS_BASE + 0x10) = 32; // Set default range
    access_bus_address(PWM_BUS_BASE + 0x20) = 32;
    access_bus_address(PWM_BUS_BASE + 0x0) = -1; // Enable FIFO mode, repeat, serializer, and channel
    usleep(1000);
    access_bus_address(PWM_BUS_BASE + 0x8) = (1 << 31) | 0x0707; // Enable DMA

    // Obtain the base address as an integer pointer
    //
    // Compute the byte‐offset from the bus base (0x7E000000) to your desired
    // DMA register block (DMA_BUS_BASE).
    std::uintptr_t delta = DMA_BUS_BASE - 0x7E000000UL;
    //
    // Treat your void* as a char* so arithmetic is in bytes
    auto base = static_cast<char *>(dma_config_.peripheral_base_virtual);
    //
    // Add the offset, then cast to your register pointer
    volatile int *dma_base = reinterpret_cast<volatile int *>(base + delta);

    // Cast to DMAregs pointer to activate DMA
    volatile struct DMAregs *DMA0 = reinterpret_cast<volatile struct DMAregs *>(dma_base);
    DMA0->CS = 1 << 31; // Reset DMA
    DMA0->CONBLK_AD = 0;
    DMA0->TI = 0;
    DMA0->CONBLK_AD = reinterpret_cast<unsigned long int>(instr_page_.b);
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
    // Retrieve PLLD frequency and DMA memory flag
    get_plld_and_memflag();

    // Map the peripheral base address into virtual memory
    setup_peripheral_base_virtual();

    // Save original register states for cleanup
    dma_config_.orig_gp0ctl = access_bus_address(CM_GP0CTL_BUS);
    dma_config_.orig_gp0div = access_bus_address(CM_GP0DIV_BUS);
    dma_config_.orig_pwm_ctl = access_bus_address(PWM_BUS_BASE + 0x00);
    dma_config_.orig_pwm_sta = access_bus_address(PWM_BUS_BASE + 0x04);
    dma_config_.orig_pwm_rng1 = access_bus_address(PWM_BUS_BASE + 0x10);
    dma_config_.orig_pwm_rng2 = access_bus_address(PWM_BUS_BASE + 0x20);
    dma_config_.orig_pwm_fifocfg = access_bus_address(PWM_BUS_BASE + 0x08);

    // Open the mailbox for DMA memory allocation
    open_mbox();

    // Allocate memory pages and build DMA control blocks
    create_dma_pages(const_page_, instr_page_, instructions_);

    // Flag setup as done
    dma_setup_done_ = true;
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
        if (debug)
        {
            std::stringstream temp;
            temp << std::fixed << std::setprecision(6)
                 << "Warning: center frequency has been changed to "
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
