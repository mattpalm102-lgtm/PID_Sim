/******************************************************************************
 * @file        CLI.hpp
 * @brief       Command-line interface for the PID simulation application.
 *
 * @details     The CLI class is responsible for ALL terminal I/O:
 *
 *                - Printing the application banner and help text
 *                - Interactively prompting the user for PID, Plant, and
 *                  Simulation parameters with full validation and retry loops
 *                - Displaying simulation results as formatted tables
 *                - Rendering an ASCII step-response chart
 *                - Exporting results as CSV to stdout
 *
 *              All user input is validated against the safety bounds defined
 *              in Types.hpp. Invalid entries produce a descriptive error
 *              message and re-prompt the user rather than crashing or
 *              silently accepting bad data — making this suitable for use by
 *              operators with no PID background.
 *
 * @author      Matt Palmer
 * @date        2026-05-01
 * @version     0.1.0
 *
 * @copyright
 * Copyright (c) 2026 Matt Palmer
 *
 * This file is part of the PID Controller Project.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 ******************************************************************************/

#ifndef PIDSIM_CLI_H
#define PIDSIM_CLI_H

#include "Types.hpp"
#include <iostream>
#include <string>

/**
 * @class CLI
 * @brief Interactive terminal interface for configuring and displaying
 *        PID simulation sessions.
 *
 * Construct with references to input/output streams (defaulting to std::cin /
 * std::cout) to allow easy unit testing with string streams.
 */
class CLI
{
public:
    // -------------------------------------------------------------------------
    // ----- CONSTRUCTOR / DESTRUCTOR ------------------------------------------
    // -------------------------------------------------------------------------

    /**
     * @brief  Constructs the CLI with injectable I/O streams.
     *
     * @param  in   Input stream (default: std::cin).
     * @param  out  Output stream (default: std::cout).
     */
    explicit CLI(std::istream& in  = std::cin,
                 std::ostream& out = std::cout);

    /// Default destructor.
    ~CLI() = default;

    // -------------------------------------------------------------------------
    // ----- PUBLIC METHODS ----------------------------------------------------
    // -------------------------------------------------------------------------

    /** @brief Prints the application banner (name, version, author). */
    void PrintBanner() const;

    /** @brief Prints detailed help text explaining PID concepts and valid ranges. */
    void PrintHelp() const;

    /**
     * @brief  Interactively collects and validates PID controller parameters.
     * @return Fully validated PIDConfig ready for use.
     */
    PIDConfig   PromptPIDConfig();

    /**
     * @brief  Interactively collects and validates plant model parameters.
     * @return Fully validated PlantConfig ready for use.
     */
    PlantConfig PromptPlantConfig();

    /**
     * @brief  Interactively collects and validates simulation run parameters.
     * @return Fully validated SimConfig ready for use.
     */
    SimConfig   PromptSimConfig();

    /**
     * @brief  Renders a formatted summary table of simulation results.
     * @param  result  Completed simulation result to display.
     */
    void DisplayResults(const SimulationResult& result) const;

    /**
     * @brief  Renders an ASCII step-response chart to the output stream.
     * @param  result  Completed simulation result to visualise.
     */
    void DisplayAsciiPlot(const SimulationResult& result) const;

    /**
     * @brief  Emits the simulation time-series as RFC 4180-compatible CSV.
     * @param  result  Completed simulation result to export.
     */
    void ExportCSV(const SimulationResult& result) const;

    /**
     * @brief  Asks the user if they want to run another simulation.
     * @return true if the user enters 'y' or 'Y', false otherwise.
     */
    bool AskRunAgain();

    // -------------------------------------------------------------------------
    // ----- STATIC VALIDATION HELPERS (public for unit testing) ---------------
    // -------------------------------------------------------------------------

    /**
     * @brief  Parses and validates a double from a raw string.
     *
     * @param  raw    String to parse.
     * @param  lo     Inclusive lower bound.
     * @param  hi     Inclusive upper bound.
     * @param  out    Destination for parsed value on success.
     *
     * @return true if parsing succeeded and value is within [lo, hi].
     */
    static bool ParseDouble(const std::string& raw,
                            double lo, double hi,
                            double& out);

    /**
     * @brief  Parses and validates an integer from a raw string.
     *
     * @param  raw    String to parse.
     * @param  lo     Inclusive lower bound.
     * @param  hi     Inclusive upper bound.
     * @param  out    Destination for parsed value on success.
     *
     * @return true if parsing succeeded and value is within [lo, hi].
     */
    static bool ParseInt(const std::string& raw,
                         int lo, int hi,
                         int& out);

private:
    // -------------------------------------------------------------------------
    // ----- PRIVATE HELPERS ---------------------------------------------------
    // -------------------------------------------------------------------------

    /**
     * @brief  Prompts the user with a message and returns the trimmed input line.
     * @param  prompt  Text to display before waiting for input.
     * @return Trimmed string entered by the user.
     */
    std::string PromptLine(const std::string& prompt) const;

    /**
     * @brief  Prompts repeatedly until a valid double in [lo, hi] is entered.
     *
     * @param  prompt   Display prompt.
     * @param  lo       Inclusive minimum.
     * @param  hi       Inclusive maximum.
     * @param  default_val  Value returned if the user presses Enter without input.
     *
     * @return Validated double value.
     */
    double PromptDouble(const std::string& prompt,
                        double lo, double hi,
                        double default_val);

    /**
     * @brief  Prompts repeatedly until a valid integer in [lo, hi] is entered.
     *
     * @param  prompt       Display prompt.
     * @param  lo           Inclusive minimum.
     * @param  hi           Inclusive maximum.
     * @param  default_val  Default if user presses Enter without input.
     *
     * @return Validated integer.
     */
    int PromptInt(const std::string& prompt,
                  int lo, int hi,
                  int default_val);

    /**
     * @brief  Prompts for a boolean yes/no response.
     *
     * @param  prompt       Display prompt (should suggest [y/n]).
     * @param  default_val  Default when user presses Enter.
     *
     * @return true for 'y'/'Y', false for 'n'/'N'.
     */
    bool PromptBool(const std::string& prompt, bool default_val);

    // -------------------------------------------------------------------------
    // ----- PRIVATE DATA MEMBERS ----------------------------------------------
    // -------------------------------------------------------------------------

    std::istream& in_;   ///< Input stream (injected for testability)
    std::ostream& out_;  ///< Output stream (injected for testability)
};

#endif // PIDSIM_CLI_H