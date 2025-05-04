<!-- omit in toc -->
# WsprTransmitter

A self-contained C++ class for DMA-driven WSPR (Weak Signal Propagation Reporter) transmission on Raspberry Pi or other Linux systems.  This library can be used standalone (via `main.cpp` demo) or incorporated into other projects by simply including `wspr_transmit.hpp` and `wspr_transmit.cpp`.

<!-- omit in toc -->
## Table of Contents
- [Repository Layout](#repository-layout)
- [Dependencies](#dependencies)
- [Building](#building)
- [Public API](#public-api)
  - [Class: `WsprTransmitter`](#class-wsprtransmitter)
    - [Construction \& Destruction](#construction--destruction)
    - [Callbacks](#callbacks)
    - [Configuration](#configuration)
    - [Transmission Control](#transmission-control)
    - [Status \& Debug](#status--debug)
- [Demo](#demo)
- [License](#license)

---

## Repository Layout

```
/src
  ├── main.cpp             # Example/demo application
  ├── Makefile             # Build script (assumes dependencies at peer level)
  ├── wspr_transmit.hpp    # Public header
  └── wspr_transmit.cpp    # Implementation

/external
  ├── config_handler.hpp   # (Optional) shared config struct
  ├── config_handler.cpp
  ├── utils.hpp
  └── utils.cpp            # Helper: PPM from chrony
```

## Dependencies

* **WSPR-Message** (symbol generation) — expected at `../../WSPR-Message/src`
* **Broadcom-Mailbox** (DMA/mailbox interface) — expected at `../../Broadcom-Mailbox/src`

The current Makefile assumes the dependencies are at the same folder level as this repo in a larger project.

> **Note:** This is where the `Makefile` includes the dependencies:
>
> ```make
> SUBMODULE_SRCDIRS := $(wildcard ../../WSPR-Message/src)
> SUBMODULE_SRCDIRS += $(wildcard ../../Broadcom-Mailbox/src)
> ```
>
> If your configuration is different, edit `Makefile` accordingly.

---

## Building

To build as a stand-alone demo:

```bash
cd src
make debug
sudo ./build/bin/wspr-transmitter_test
```

* Requires linking against pthreads (`-pthread`).
* Must be run as root (for `/dev/mem` access).

The `Makefile` is quite comprehensive and includes a `help` argument:

```bash
$ make help

Available targets:
  release    Build optimized binary
  debug      Build debug binary
  test       Run tests
  gdb        Debug with gdb
  lint       Static analysis
  macros     Show macros
  clean      Remove build artifacts
  help       This message
```

---

## Public API

### Class: `WsprTransmitter`

#### Construction & Destruction

```cpp
WsprTransmitter();        // default ctor
~WsprTransmitter();       // stops and cleans up
```

#### Callbacks

```cpp
using Callback = std::function<void(const std::string &msg)>;
void setTransmissionCallbacks(
    Callback on_start = {},
    Callback on_end   = {});
```

* `on_start` fires just before transmission
* `on_end` fires immediately after symbols/tone finish

#### Configuration

* Fully initialize frequency, power, PPM, callsign/grid, offset:

```cpp
void setupTransmission(
    double frequency,
    int    power_dbm,
    double ppm,
    std::string callsign = "",
    std::string grid     = "",
    bool    use_offset   = false
);
```
* Rebuild DMA frequency table when PPM changes at runtime:

```cpp
void updateDMAForPPM(double ppm_new);
```

* POSIX scheduling for the future transmit thread (SCHED\_FIFO/RR):

```cpp
void setThreadScheduling(int policy, int priority);
```

#### Transmission Control

```cpp
void enableTransmission();   // non-blocking: tone or scheduler
void disableTransmission();  // cancel scheduler + any active transmit
void stopTransmission();     // request in-flight stop
void shutdownTransmitter();  // disable + cleanup
```

#### Status & Debug

```cpp
bool isTransmitting() const noexcept;
void printParameters();      // dumps current config & symbols
```

---

## Demo

The provided `main.cpp` shows a minimal example:

1. Pipe-based signal handling to catch `SIGINT`/`SIGTERM`.
2. Choose WSPR vs tone mode.
3. Configure PPM manually via `setupTransmission()`.
4. Spawn transmission with `enableTransmission()`.
5. Wait on condition variable or spacebar before shutdown.

Compile (as shown above) with:

```bash
cd src
make debug
sudo ./build/bin/wspr-transmitter_test
```

---

## License

MIT — see [LICENSE.md](../LICENSE.md).

---

*2025 © Lee C. Bussy (@LBussy)*
