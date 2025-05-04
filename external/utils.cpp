#include "utils.hpp"

#include <array>
#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <cstdio>
#include <cerrno>
#include <cstring>
#include <sys/wait.h>

/**
 * @brief Get the PPM (frequency) value from `chronyc tracking`
 *
 * Executes the `chronyc tracking` command and parses the "Frequency" line,
 * which reflects the estimated drift rate in parts per million (PPM).
 *
 * @return The frequency in PPM as a double
 * @throws std::runtime_error if the command fails or parsing fails
 */
double get_ppm_from_chronyc()
{
    const char *cmd = "chronyc tracking 2>&1";
    std::array<char, 256> buffer{};
    std::string result;

    FILE *pipe = popen(cmd, "r");
    if (!pipe)
    {
        throw std::runtime_error("Failed to run chronyc tracking.");
    }

    while (fgets(buffer.data(), buffer.size(), pipe) != nullptr)
    {
        result += buffer.data();
    }

    int status = pclose(pipe);

    if (status == -1)
    {
        if (errno == ECHILD)
        {
            // Treat as success
            ;
            ;
        }
        else
        {
            std::ostringstream err;
            err << "pclose() failed while closing pipe to chronyc.\n";
            err << "errno = " << errno << " (" << std::strerror(errno) << ")\n";
            err << "Command output:\n"
                << result;
            throw std::runtime_error(err.str());
        }
    }
    else if (WIFEXITED(status))
    {
        int exit_code = WEXITSTATUS(status);
        if (exit_code != 0)
        {
            std::ostringstream err;
            err << "chronyc tracking exited with status " << exit_code << ".\n";
            err << "Command output:\n"
                << result;
            throw std::runtime_error(err.str());
        }
    }
    else if (WIFSIGNALED(status))
    {
        int signal = WTERMSIG(status);
        std::ostringstream err;
        err << "chronyc tracking terminated by signal " << signal << ".\n";
        err << "Command output:\n"
            << result;
        throw std::runtime_error(err.str());
    }

    // Parse "Frequency" line
    std::istringstream iss(result);
    std::string line;
    while (std::getline(iss, line))
    {
        if (line.find("Frequency") != std::string::npos)
        {
            std::istringstream linestream(line);
            std::string label, colon;
            double ppm;
            std::string units;

            linestream >> label >> colon >> ppm >> units;
            std::cout << "PPM set to: " << std::fixed << std::setprecision(3) << ppm << std::endl;
            return ppm;
        }
    }

    throw std::runtime_error("Frequency line not found in chronyc tracking output.");
}
