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

#ifndef _WSPR_TRANSMIT_HPP
#define _WSPR_TRANSMIT_HPP

// Project header
#include "wspr_message.hpp"

// C++ Standard Library
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint> // for std::uint32_t, etc.
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <variant>
#include <vector>

// POSIX/System headers
#include <sys/time.h> // for struct timeval

/**
 * @class WsprTransmitter
 * @brief Encapsulates configuration and DMA‑driven transmission of WSPR signals.
 *
 * @details
 *   The WsprTransmitter class provides a full interface for setting up and
 *   executing Weak Signal Propagation Reporter (WSPR) transmissions on a
 *   Raspberry Pi. It handles:
 *     - Configuration of RF frequency, power level, PPM calibration, and message
 *       parameters via setupTransmission().
 *     - Low‑level mailbox allocation, peripheral memory mapping, and DMA/PWM
 *       initialization for precise symbol timing.
 *     - Start/stop of transmission loops (tone mode or symbol mode) with
 *       gettimeofday() scheduling.
 *     - Dynamic updates of PPM correction and DMA frequency tables.
 *     - Safe teardown of DMA, PWM, and mailbox resources (idempotent).
 *
 *   Designed for global instantiation and thread‑safe operation, this class
 *   abstracts the complexities of hardware interaction, allowing higher‑level
 *   code to transmit WSPR messages with minimal boilerplate.
 */
class WsprTransmitter
{
public:
    /**
     * @brief Constructs a WSPR transmitter with default settings.
     *
     * This is for global constructions, parameters are set via
     * setupTransmission().
     */
    WsprTransmitter();

    /**
     * @brief Destroys the WSPR transmitter.
     *
     * Cleans up any allocated resources and stops
     * any running transmission threads.
     */
    ~WsprTransmitter();

    // non‐copyable, non‐movable
    WsprTransmitter(WsprTransmitter const &) = delete;
    WsprTransmitter &operator=(WsprTransmitter const &) = delete;
    WsprTransmitter(WsprTransmitter &&) = delete;
    WsprTransmitter &operator=(WsprTransmitter &&) = delete;

    /**
     * @brief Signature for user-provided transmission callbacks.
     *
     * This callback receives either a message string or a frequency value,
     * allowing the user to handle both human-readable messages and numeric data.
     *
     * @param arg A variant containing either a std::string or a double value.
     *            The string may carry a descriptive message, while the double
     *            represents a frequency in Hz (or another unit depending on context).
     */
    using CallbackArg = std::variant<std::string, double>;
    using Callback = std::function<void(const CallbackArg &)>;

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
    void setTransmissionCallbacks(Callback start_cb = {},
                                  Callback end_cb = {});

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
    void setupTransmission(
        double frequency,
        int power = 0,
        double ppm = 0.0,
        std::string call_sign = "",
        std::string grid_square = "",
        int power_dbm = 0,
        bool use_offset = false);

    /**
     * @brief Rebuild the DMA tuning‐word table with a fresh PPM correction.
     *
     * @param ppm_new The new parts‑per‑million offset (e.g. +11.135).
     * @throws std::runtime_error if peripherals aren’t mapped.
     */
    void updateDMAForPPM(double ppm_new);

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
    void setThreadScheduling(int policy, int priority);

    /**
     * @brief Start transmission, either immediately or via the scheduler.
     *
     * @details
     *   If `trans_params_.is_tone == true`, this will spawn the transmit
     *   thread right away (bypassing the scheduler). Otherwise it launches
     *   the background scheduler, which will fire at the next WSPR window
     *   and then spawn the transmit thread.
     *
     * @note This call is non-blocking. In tone mode it returns immediately
     *       after spawning the thread; in WSPR mode it returns immediately
     *       after starting the scheduler thread.
     */
    void enableTransmission();

    /**
     * @brief Cancels the scheduler (and any running transmission).
     *
     * Waits for the scheduler thread to stop, and forces any in‐flight
     * transmission to end.
     */
    void disableTransmission();

    /**
     * @brief Request an in‑flight transmission to stop.
     *
     * @details Sets the internal stop flag so that ongoing loops in the transmit
     *          thread will exit at the next interruption point. Notifies any
     *          condition_variable waits to unblock the thread promptly.
     */
    void stopTransmission();

    /**
     * @brief Gracefully stops and waits for the transmission thread.
     *
     * @details Combines stopTransmission() to signal the worker thread to
     *          exit, and join_transmission() to block until that thread has
     *          fully terminated. After this call returns, no transmission
     *          thread remains running.
     */
    void stop();

    /**
     * @brief Check if the GPIO is bound to the clock.
     *
     * @details Returns a value indicating if the system is transmitting
     * in any way,
     *
     * @return `true` if clock is engaged, `false` otherwise.
     */
    bool isTransmitting() const noexcept;

    /**
     * @brief Prints current transmission parameters and encoded WSPR symbols.
     *
     * @details Displays the configured WSPR parameters including frequency,
     * power, mode, tone/test settings, and symbol timing. Also prints all
     * WSPR symbols as integer values, grouped for readability.
     *
     * This function is useful for debugging and verifying that all transmission
     * settings and symbol sequences are correctly populated before transmission.
     */
    void printParameters();

private:
    /**
     * @brief Invoked just before each transmission begins.
     *
     * This callback is fired on the transmit thread immediately before
     * starting either a continuous tone or a WSPR symbol sequence.
     * Users can assign a function via `setTransmissionCallbacks()` to
     * perform any setup or logging when transmission is about to start.
     */
    Callback on_transmit_start_{};

    /**
     * @brief Invoked immediately after all WSPR symbols have been sent.
     *
     * This callback is fired on the transmit thread right after the last
     * WSPR symbol is transmitted and before the hardware is torn down.
     * Users can assign a function via `setTransmissionCallbacks()` to
     * perform any cleanup or notification when a WSPR transmission completes.
     */
    Callback on_transmit_end_{};

    /**
     * @brief Background thread for carrying out the transmission.
     *
     * Launched by startTransmission() and joined by
     * join_transmission().
     */
    std::thread tx_thread_;

    /**
     * @brief POSIX scheduling policy for the transmission thread.
     *
     * One of SCHED_FIFO, SCHED_RR, or SCHED_OTHER.
     */
    int thread_policy_ = SCHED_OTHER;

    /**
     * @brief Scheduling priority for the transmission thread.
     *
     * Valid range is 1–99 for real‑time policies; ignored by
     * SCHED_OTHER.
     */
    int thread_priority_ = 0;

    /**
     * @brief Flag indicating that a stop request has been issued.
     *
     * When true, loops in transmit() and transmit_symbol() will
     * exit at the next interruption point.
     */
    std::atomic<bool> stop_requested_{false};

    /**
     * @brief Condition variable used to wake the transmission thread.
     *
     * stopTransmission() calls notify_all() on this to unblock
     * any waits so the thread can observe stop_requested_.
     */
    std::condition_variable stop_cv_;

    /**
     * @brief Bool used to store transmission state.
     *
     * True when transmit_on() is called, false when transmit_off() or
     * disable_clock() is called.
     */
    std::atomic<bool> transmit_on_{false};

    /**
     * @brief Mutex accompanying stop_cv_ for coordinated waits.
     */
    std::mutex stop_mutex_;

    /**
     * @brief Global dma setup semaphore.
     *
     * Shows if setup_dma() been run and not yet torn down.
     */
    bool dma_setup_done_{false};

    /**
     * @brief Holds the bus and virtual addresses for a physical memory page.
     *
     * This structure is used to store the mapping between the bus address (used for DMA
     * and peripheral accesses) and the virtual address (used by the application) of a single
     * page of physical memory.
     *
     * @var PageInfo::b
     *      The bus address of the physical memory page.
     * @var PageInfo::v
     *      The virtual address mapped to the physical memory page.
     */
    struct PageInfo
    {
        std::uintptr_t b; ///< Bus address.
        void *v;          ///< Virtual address.
    };

    /**
     * @brief Page information for the constant memory page.
     *
     * This global variable holds the bus and virtual addresses of the constant memory page,
     * which is used to store fixed data required for DMA operations, such as the tuning words
     * for frequency generation.
     */
    struct PageInfo const_page_;

    /**
     * @brief Page information for the DMA instruction page.
     *
     * This global variable holds the bus and virtual addresses of the DMA instruction page,
     * where DMA control blocks (CBs) are stored. This page is used during the setup and
     * operation of DMA transfers.
     */
    struct PageInfo instr_page_;

    /**
     * @brief Array of page information structures for DMA control blocks.
     *
     * This global array contains the bus and virtual addresses for each page used in the DMA
     * instruction chain. It holds 1024 entries, corresponding to the 1024 DMA control blocks used
     * for managing data transfers.
     */
    struct PageInfo instructions_[1024];

    /**
     * @brief Random frequency offset for standard WSPR transmissions.
     *
     * This constant defines the range, in Hertz, for random frequency offsets
     * applied to standard WSPR transmissions. The offset is applied symmetrically
     * around the target frequency, resulting in a random variation of ±80 Hz.
     *
     * This helps distribute transmissions within the WSPR band, reducing the
     * likelihood of overlapping signals.
     *
     * @note This offset is applicable for standard WSPR transmissions (2-minute cycles).
     *
     * @see WSPR15_RAND_OFFSET
     */
    static constexpr int WSPR_RAND_OFFSET = 80;

    /**
     * @brief Nominal symbol duration for WSPR transmissions.
     *
     * This constant represents the nominal time duration of a WSPR symbol,
     * calculated as 8192 samples divided by a sample rate of 12000 Hz.
     *
     * @details This duration is a key parameter in WSPR transmissions,
     * ensuring the correct timing for symbol generation and encoding.
     *
     * @note Any deviation in sample rate or processing latency could affect
     *       the actual symbol duration.
     */
    static constexpr double WSPR_SYMTIME = 8192.0 / 12000.0;

    /**
     * @brief Actual PWM clock frequency used for symbol timing.
     *
     * This field holds the measured PWM clock rate (in Hz) read back from the
     * hardware after configuring the clock divisor. It ensures precise symbol
     * durations across platforms.
     *
     * @details Calculated as:
     *   pwm_clock_init_ = plld_clock_frequency / divisor
     * where `divisor` is read from the PWM clock divider register. Used in
     * `transmit_symbol()` to convert the desired symbol time (`tsym`) into
     * clock ticks.
     */
    double pwm_clock_init_{0};

    /**
     * @brief Base bus address for the General-Purpose Input/Output (GPIO)
     * registers.
     *
     * The GPIO peripheral bus base address is used to access GPIO-related
     * registers for pin configuration and control.
     */
    static constexpr uint32_t GPIO_BUS_BASE = 0x7E200000;

    /**
     * @brief Bus address for the General-Purpose Clock 0 Control Register.
     *
     * This register controls the clock settings for General-Purpose Clock 0 (GP0).
     */
    static constexpr uint32_t CM_GP0CTL_BUS = 0x7E101070;

    /**
     * @brief Bus address for the General-Purpose Clock 0 Divider Register.
     *
     * This register sets the frequency division for General-Purpose Clock 0.
     */
    static constexpr uint32_t CM_GP0DIV_BUS = 0x7E101074;

    /**
     * @brief Bus address for the GPIO pads control register (GPIO 0-27).
     *
     * This register configures drive strength, pull-up, and pull-down settings
     * for GPIO pins 0 through 27.
     */
    static constexpr uint32_t PADS_GPIO_0_27_BUS = 0x7E10002C;

    /**
     * @brief Base bus address for the clock management registers.
     *
     * The clock management unit controls various clock sources and divisors
     * for different peripherals on the Raspberry Pi.
     */
    static constexpr uint32_t CLK_BUS_BASE = 0x7E101000;

    /**
     * @brief Base bus address for the Direct Memory Access (DMA) controller.
     *
     * The DMA controller allows high-speed data transfer between peripherals
     * and memory without CPU intervention.
     */
    static constexpr uint32_t DMA_BUS_BASE = 0x7E007000;

    /**
     * @brief Base bus address for the Pulse Width Modulation (PWM) controller.
     *
     * The PWM controller is responsible for generating PWM signals used in
     * applications like audio output, LED dimming, and motor control.
     */
    static constexpr uint32_t PWM_BUS_BASE = 0x7E20C000;

    /**
     * @brief The nominal number of PWM clock cycles per iteration.
     *
     * This constant defines the expected number of PWM clock cycles required for
     * a single iteration of the waveform generation process. It serves as a reference
     * value to maintain precise timing in signal generation.
     */
    static constexpr std::uint32_t PWM_CLOCKS_PER_ITER_NOMINAL = 1000;

    /**
     * @brief Drive strength lookup table for GPIO output levels.
     *
     * @details Maps drive strength levels 0 through 7 to their corresponding
     * electrical output current in milliamps (mA). This table is indexed using
     * a numeric strength level and returns the standard BCM283x drive strength
     * setting for GPIO pins.
     *
     * The values are:
     *   - 0 →  2 mA
     *   - 1 →  4 mA
     *   - 2 →  6 mA
     *   - 3 →  8 mA
     *   - 4 → 10 mA
     *   - 5 → 12 mA
     *   - 6 → 14 mA
     *   - 7 → 16 mA
     *
     * @note This table is used internally by get_gpio_power_mw() and is not exposed
     *       in the public interface unless declared in a header.
     */
    static inline constexpr std::array<int, 8> DRIVE_STRENGTH_TABLE = {
        2, 4, 6, 8, 10, 12, 14, 16};

    /**
     * @brief Structure containing parameters for a WSPR transmission.
     *
     * This structure encapsulates all the necessary parameters required to configure
     * and execute a WSPR transmission, including the message, transmission frequency,
     * symbol time, tone spacing, and the DMA frequency lookup table.
     */
    struct WsprTransmissionParams
    {
        static const std::size_t symbol_count = MSG_SIZE;
        std::array<uint8_t, symbol_count> symbols;

        std::string call_sign;              ///< Callsign of operator
        std::string grid_square;            ///< Maidenhead grid square of origination
        int power_dbm;                      ///< Reported transmission power in dBm
        double frequency;                   ///< Transmission frequency in Hz.
        double ppm;                         ///< Current system PPM adjustment
        bool is_tone;                       ///< Is test tone
        int power;                          ///< GPIO power level 0-7
        double symtime;                     ///< Duration of each symbol in seconds.
        double tone_spacing;                ///< Frequency spacing between adjacent tones in Hz.
        std::vector<double> dma_table_freq; ///< DMA frequency lookup table.
        bool use_offset;                    ///< Use random offset on transmissions

        /**
         * @brief Default constructor for WsprTransmissionParams.
         *
         * Initializes the transmission parameters with default values.
         */
        WsprTransmissionParams()
            : symbols{},
              frequency(0.0),
              is_tone(false),
              power(0),
              symtime(0.0),
              tone_spacing(0.0),
              dma_table_freq(1024, 0.0),
              use_offset(false)
        {
        }
    };

    /**
     * @brief Global instance of transmission parameters for WSPR.
     *
     * This global variable holds the current settings used for a WSPR transmission,
     * including the WSPR message, transmission frequency, symbol time, tone spacing,
     * and the DMA frequency lookup table.
     */
    struct WsprTransmissionParams trans_params_;

    /**
     * @brief DMA configuration and saved state for transmission setup/cleanup.
     *
     * @details Stores both the nominal and (PPM‑corrected) PLLD clock frequencies,
     *          mailbox memory flags, the virtual base address for peripheral access,
     *          and the original register values that must be restored when
     *          tearing down DMA/PWM configuration.
     */
    struct DMAConfig
    {
        double plld_nominal_freq;                  ///< PLLD clock frequency in Hz before any PPM correction.
        double plld_clock_frequency;               ///< PLLD clock frequency in Hz after PPM correction.
        volatile uint8_t *peripheral_base_virtual; ///< Virtual base pointer for /dev/mem mapping of peripherals.
        uint32_t orig_gp0ctl;                      ///< Saved GP0CTL register (clock control).
        uint32_t orig_gp0div;                      ///< Saved GP0DIV register (clock divider).
        uint32_t orig_pwm_ctl;                     ///< Saved PWM control register.
        uint32_t orig_pwm_sta;                     ///< Saved PWM status register.
        uint32_t orig_pwm_rng1;                    ///< Saved PWM range register 1.
        uint32_t orig_pwm_rng2;                    ///< Saved PWM range register 2.
        uint32_t orig_pwm_fifocfg;                 ///< Saved PWM FIFO configuration register.

        /**
         * @brief Construct a new DMAConfig with default (nominal) settings.
         *
         * @details Initializes:
         *   - `plld_nominal_freq` to 500 MHz with the built‑in 2.5 ppm correction.
         *   - `plld_clock_frequency` equal to `plld_nominal_freq`.
         *   - All pointers and saved‑register fields to zero or nullptr.
         */
        DMAConfig()
            : plld_nominal_freq(500000000.0 * (1 - 2.500e-6)),
              plld_clock_frequency(plld_nominal_freq),
              peripheral_base_virtual(nullptr),
              orig_gp0ctl(0),
              orig_gp0div(0),
              orig_pwm_ctl(0),
              orig_pwm_sta(0),
              orig_pwm_rng1(0),
              orig_pwm_rng2(0),
              orig_pwm_fifocfg(0)
        {
        }
    };

    /**
     * @brief Global configuration object.
     *
     * This DMAConfig instance holds the transmission functionality global objects.
     */
    struct DMAConfig dma_config_;

    struct MailboxStruct
    {
        uint32_t mem_ref = 0;                  ///< memAlloc()
        std::uintptr_t bus_addr = 0;           ///< memLock()
        volatile uint8_t *virt_addr = nullptr; ///< mapMem()
        unsigned pool_size = 0;                ///< total DMA pages
        unsigned pool_cnt = 0;                 ///< pages handed out
    };

    /**
     * @brief MailboxStruct state for DMA memory management.
     *
     * This member holds the memory reference ID (from memAlloc()), the bus
     * address (from memLock()), the virtual address pointer (from mapMem()),
     * and the pool parameters for page allocation.
     */
    MailboxStruct mailbox_struct_;

    /**
     * @brief Control Block (CB) structure for DMA engine commands.
     *
     * This struct defines the fields the DMA engine reads to perform a
     * transfer: control bits, source/destination addresses, transfer length,
     * optional 2D stride, and chaining to the next block. Reserved fields
     * must be zero.
     */
    struct CB
    {
        volatile unsigned int TI;        ///< Transfer Information field for DMA control.
        volatile unsigned int SOURCE_AD; ///< Source bus address for the DMA transfer.
        volatile unsigned int DEST_AD;   ///< Destination bus address for the DMA transfer.
        volatile unsigned int TXFR_LEN;  ///< Length (in bytes) of the transfer.
        volatile unsigned int STRIDE;    ///< 2D stride value (unused if single block).
        volatile unsigned int NEXTCONBK; ///< Bus address of the next CB in the chain.
        volatile unsigned int RES1;      ///< Reserved, must be zero.
        volatile unsigned int RES2;      ///< Reserved, must be zero.
    };

    /**
     * @brief Control structure for the clock generator.
     *
     * This structure is used to configure the clock generator on the Raspberry Pi,
     * which is essential for transmitting radio signals by manipulating the PLLD clock
     * and routing the output to a GPIO pin.
     *
     * The bit-fields in this structure allow you to set the clock source, enable or disable
     * the clock, force the clock off (kill), check the busy status, flip the output phase,
     * set the MASH filter (which affects clock stability), and provide the required password
     * to modify the control registers.
     *
     * Bit-field breakdown:
     * - **SRC (4 bits):** Clock source selection.
     * - **ENAB (1 bit):** Enable bit. Set to 1 to enable the clock.
     * - **KILL (1 bit):** Kill bit. Set to 1 to force the clock off.
     * - **(1 bit reserved):** Unused.
     * - **BUSY (1 bit):** Busy status. Indicates if the clock generator is active.
     * - **FLIP (1 bit):** Flip bit. May be used for phase inversion.
     * - **MASH (2 bits):** MASH filter setting for noise shaping.
     * - **(13 bits reserved):** Unused/reserved.
     * - **PASSWD (8 bits):** Password field required to modify the clock control registers.
     */
    struct GPCTL
    {
        uint32_t SRC : 4;    ///< Clock source selection.
        uint32_t ENAB : 1;   ///< Enable bit.
        uint32_t KILL : 1;   ///< Kill bit.
        uint32_t : 1;        ///< Reserved.
        uint32_t BUSY : 1;   ///< Busy flag.
        uint32_t FLIP : 1;   ///< Phase inversion flag.
        uint32_t MASH : 2;   ///< MASH filter.
        uint32_t : 13;       ///< Reserved.
        uint32_t PASSWD : 8; ///< Password field.
    };
    static_assert(sizeof(GPCTL) == 4, "GPCTL must be exactly 32 bits.");

    /**
     * @brief DMA Engine Status Registers.
     *
     * This structure represents the status and control registers for the DMA engine.
     * It is used to monitor and control DMA operations such as data transfers between
     * memory and peripherals. These registers are critical when setting up and debugging
     * DMA transfers, such as those involved in transmitting radio signals by manipulating
     * the PLLD and routing output to GPIO.
     *
     * @var DMAregs::CS
     *      Control/Status register: Holds flags for starting, resetting, and error states.
     * @var DMAregs::CONBLK_AD
     *      Current control block address: Points to the next DMA control block in the chain.
     * @var DMAregs::TI
     *      Transfer Information register: Contains configuration flags and settings for the current transfer.
     * @var DMAregs::SOURCE_AD
     *      Source Address register: Specifies the memory address to read data from.
     * @var DMAregs::DEST_AD
     *      Destination Address register: Specifies the memory address to write data to.
     * @var DMAregs::TXFR_LEN
     *      Transfer Length register: Indicates the number of bytes to transfer.
     * @var DMAregs::STRIDE
     *      Stride register: Determines the address increment between consecutive transfers.
     * @var DMAregs::NEXTCONBK
     *      Next Control Block register: Contains the address of the next control block, enabling chained transfers.
     * @var DMAregs::DEBUG
     *      Debug register: Provides information useful for debugging DMA operations.
     */
    struct DMAregs
    {
        volatile unsigned int CS;        ///< Control/Status register.
        volatile unsigned int CONBLK_AD; ///< Address of the current control block.
        volatile unsigned int TI;        ///< Transfer Information register.
        volatile unsigned int SOURCE_AD; ///< Source address for data transfer.
        volatile unsigned int DEST_AD;   ///< Destination address for data transfer.
        volatile unsigned int TXFR_LEN;  ///< Transfer length (in bytes).
        volatile unsigned int STRIDE;    ///< Stride for address increment.
        volatile unsigned int NEXTCONBK; ///< Address of the next control block.
        volatile unsigned int DEBUG;     ///< Debug register for diagnostics.
    };

    /**
     * @brief Safely invoke the user’s start‐transmission callback.
     *
     * @details Checks whether a start callback has been set, and if so,
     *          calls it with an empty message string to satisfy the
     *          `Callback` signature.
     *
     * @param frequency  A double frequency to pass into the callback.
     */
    void fire_start_cb(const double frequency);

    /**
     * @brief Safely invoke the user’s end‐transmission callback with a message.
     *
     * @details Checks whether an end callback has been set, and if so,
     *          calls it with the provided message. Useful for reporting
     *          completion status or diagnostics.
     *
     * @param msg  A string message to pass into the callback.
     */
    void fire_end_cb(const std::string &msg = {});

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
    void transmit();

    /**
     * @brief Waits for the background transmission thread to finish.
     *
     * @details If the transmission thread was launched via
     *          startTransmission(), this call will block until
     *          that thread has completed and joined. After returning,
     *          tx_thread_ is no longer joinable.
     */
    void join_transmission();

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
    void dma_cleanup();

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
    constexpr int get_gpio_power_mw(int level);

    /**
     * @brief Convert power in milliwatts to decibels referenced to 1 mW (dBm).
     *
     * @param mw  Power in milliwatts. Must be greater than 0.
     * @return    Power in dBm.
     * @throws    std::domain_error if mw is not positive.
     */
    inline double convert_mw_dbm(double mw);

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
    inline volatile int &access_bus_address(std::uintptr_t bus_addr);

    /**
     * @brief Sets a specified bit at a bus address in the peripheral address space.
     *
     * This function accesses the virtual address corresponding to the given bus address using
     * the `access_bus_address()` function and sets the bit at the provided position.
     *
     * @param base The bus address in the peripheral address space.
     * @param bit The bit number to set (0-indexed).
     */
    inline void set_bit_bus_address(std::uintptr_t base, unsigned int bit);

    /**
     * @brief Clears a specified bit at a bus address in the peripheral address space.
     *
     * This function accesses the virtual address corresponding to the given bus address using
     * the `access_bus_address()` function and clears the bit at the provided position.
     *
     * @param base The bus address in the peripheral address space.
     * @param bit The bit number to clear (0-indexed).
     */
    inline void clear_bit_bus_address(std::uintptr_t base, unsigned int bit);

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
    int symbol_timeval_subtract(struct timeval *result, const struct timeval *t2, const struct timeval *t1);

    /**
     * @brief Initialize DMAConfig PLLD frequencies
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
    void get_plld();

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
    void allocate_memory_pool(unsigned numpages);

    /**
     * @brief Retrieves the next available memory page from the allocated pool.
     * @details Provides a virtual and bus address for a memory page in the pool.
     *          If no more pages are available, the function prints an error and exits.
     *
     * @param[out] vAddr Pointer to store the virtual address of the allocated page.
     * @param[out] bAddr Pointer to store the bus address of the allocated page.
     */
    void get_real_mem_page_from_pool(void **vAddr, void **bAddr);

    /**
     * @brief Deallocates the memory pool.
     * @details Releases the allocated memory by unmapping virtual memory,
     *          unlocking, and freeing the memory via the mailbox interface.
     */
    void deallocate_memory_pool();

    /**
     * @brief Disables the PWM clock.
     * @details Clears the enable bit in the clock control register and waits
     *          until the clock is no longer busy. Ensures proper synchronization.
     */
    void disable_clock();

    /**
     * @brief Enables TX by configuring GPIO4 and setting the clock source.
     * @details Configures GPIO4 to use alternate function 0 (GPCLK0), sets the drive
     *          strength, disables any active clock, and then enables the clock with PLLD.
     */
    void transmit_on();

    /**
     * @brief Disables the transmitter.
     * @details Turns off the transmission by disabling the clock source.
     */
    void transmit_off();

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
    void transmit_symbol(
        const int &sym_num,
        const double &tsym,
        int &bufPtr);

    /**
     * @brief Disables and resets the DMA engine.
     * @details Ensures that the DMA controller is properly reset before exiting.
     *          If the peripheral memory mapping is not set up, the function returns early.
     */
    void clear_dma_setup();

    /**
     * @brief Truncates a floating-point number at a specified bit position.
     * @details Sets all bits less significant than the given LSB to zero.
     *
     * @param d The input floating-point number to be truncated.
     * @param lsb The least significant bit position to retain.
     * @return The truncated value with lower bits set to zero.
     */
    double bit_trunc(const double &d, const int &lsb);

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
    void create_dma_pages(
        struct PageInfo &const_page_,
        struct PageInfo &instr_page_,
        struct PageInfo instructions_[]);

    /**
     * @brief Configure and initialize the DMA system for WSPR transmission.
     *
     * @details Performs the following steps in order:
     *   1. Retrieve and configure the PLLD clock frequency.
     *   2. Map the peripheral base address into user space.
     *   3. Save the original clock and PWM register values for later restoration.
     *   4. Open the Broadcom mailbox interface for DMA memory allocation.
     *   5. Allocate and set up DMA control blocks for constants and instruction pages.
     *
     * @throws std::runtime_error if the PLLD clock cannot be determined.
     * @throws std::runtime_error if peripheral memory mapping fails.
     * @throws std::runtime_error if mailbox opening fails.
     */
    void setup_dma();

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
    void setup_dma_freq_table(double &center_freq_actual);

    /**
     * @brief Entry point for the background transmission thread.
     *
     * @details Applies the configured POSIX scheduling policy and priority
     *          (via set_thread_priority()), then invokes transmit() to carry
     *          out the actual transmission work. This method runs inside the
     *          new thread and returns only when transmit() completes or a
     *          stop request is observed.
     */
    void thread_entry();

    /**
     * @brief Applies the configured scheduling policy and priority to this thread.
     *
     * @details Builds a sched_param struct using thread_priority_ and invokes
     *          pthread_setschedparam() with thread_policy_ on the current thread.
     *          If the call fails, writes a warning to stderr with the error message.
     */
    void set_thread_priority();

    class TransmissionScheduler
    {
    public:
        TransmissionScheduler(WsprTransmitter *parent)
            : parent_{parent}
        {
        }

        ~TransmissionScheduler()
        {
            stop();
        }

        /// Start the scheduler thread
        void start()
        {
            // don’t spawn a second scheduler if one is already running
            if (thread_.joinable())
                return;

            stop_requested_.store(false, std::memory_order_release);
            thread_ = std::thread(&TransmissionScheduler::run, this);
        }

        /// Signal the scheduler to exit, join the thread
        void stop()
        {
            stop_requested_.store(true, std::memory_order_release);
            cv_.notify_all();

            // Only join if it's a different thread than the one calling stop()
            if (thread_.joinable() &&
                thread_.get_id() != std::this_thread::get_id())
            {
                thread_.join();
            }
        }

    private:
        WsprTransmitter *parent_;
        std::thread thread_;
        std::atomic<bool> stop_requested_{false};
        std::mutex mtx_;
        std::condition_variable cv_;

        /// Compute “1 s past next even‐minute or quarter‐hour”
        std::chrono::system_clock::time_point nextEvent() const
        {
            using namespace std::chrono;

            // “now” and its epoch‐seconds
            auto now = system_clock::now();
            auto secs = duration_cast<seconds>(now.time_since_epoch()).count();

            // Choose cycle length: 2 min = 120 s
            const int cycle = 2 * 60;

            // Integer‐divide to find current cycle index
            auto idx = secs / cycle;

            // If we’re already within the first second of this window, fire now+1s,
            // otherwise bump to the next cycle:
            auto base = idx * cycle;
            seconds target_secs;
            if (secs < base + 1)
            {
                // still before “:XX:01” of this cycle
                target_secs = seconds{base + 1};
            }
            else
            {
                // past “:XX:01”, schedule at next cycle’s +1s
                target_secs = seconds{(idx + 1) * cycle + 1};
            }

            return system_clock::time_point{target_secs};
        }

        /// The scheduler loop
        void run()
        {
            while (!stop_requested_.load(std::memory_order_acquire))
            {
                auto when = nextEvent();
                std::unique_lock<std::mutex> lk(mtx_);
                cv_.wait_until(lk, when, [this]
                               { return stop_requested_.load(std::memory_order_acquire); });
                if (stop_requested_.load(std::memory_order_acquire))
                    break;

                parent_->stop_requested_.store(false, std::memory_order_release);

                // If there is an existing transmit thread, wait for it to finish:
                if (parent_->tx_thread_.joinable())
                {
                    parent_->tx_thread_.join();
                }
                // Now it's safe to start a new transmit
                parent_->tx_thread_ = std::thread(&WsprTransmitter::thread_entry, parent_);
            }
        }
    };

    TransmissionScheduler scheduler_{this};
};

extern WsprTransmitter wsprTransmitter;

#endif // _WSPR_TRANSMIT_HPP
