/******************************************************************************
 * @file        CLI.cpp
 * @brief       Command-line interface implementation.
 *
 * @details     Implements all terminal I/O for the PID simulation application:
 *                - Validated prompting for every configuration parameter
 *                - Descriptive error messages and retry loops
 *                - Formatted results table with performance metrics
 *                - ASCII step-response chart (72-column × 24-row terminal graph)
 *                - CSV export with RFC 4180 formatting
 *
 *              Design philosophy: every user-facing message is written to be
 *              understood by an operator with NO knowledge of PID control.
 *              Input ranges, units, and context are always shown.
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

#include "CLI.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

// =============================================================================
// ANONYMOUS NAMESPACE - file-local helpers
// =============================================================================
namespace
{

/// Width of the ASCII chart in characters.
constexpr int CHART_WIDTH  = 70;

/// Height of the ASCII chart in rows.
constexpr int CHART_HEIGHT = 20;

// -------------------------------------------------------------------------
/// Trims leading and trailing whitespace from a string in-place.
// -------------------------------------------------------------------------
void TrimString(std::string& s)
{
    const auto not_space = [](unsigned char c){ return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
    s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
}

} // anonymous namespace

// =============================================================================
// CONSTRUCTOR
// =============================================================================
CLI::CLI(std::istream& in, std::ostream& out)
    : in_(in), out_(out)
{
}

// =============================================================================
// PUBLIC METHODS
// =============================================================================

// -----------------------------------------------------------------------------
// PrintBanner
// -----------------------------------------------------------------------------
void CLI::PrintBanner() const
{
    out_ << "\n";
    out_ << "================================================================\n";
    out_ << "|          " << APP_NAME << "                           |\n";
    out_ << "|                   Version " << APP_VERSION << "                              |\n";
    out_ << "|                   " << APP_AUTHOR << "                                |\n";
    out_ << "================================================================\n";
    out_ << "\n";
    out_ << "  This tool lets you configure and simulate a PID controller\n";
    out_ << "  against a variety of process plant models.\n";
    out_ << "  Type 'help' at any prompt for guidance. Press Enter to accept\n";
    out_ << "  the default value shown in [brackets].\n";
    out_ << "\n";
}

// -----------------------------------------------------------------------------
// PrintHelp
// -----------------------------------------------------------------------------
void CLI::PrintHelp() const
{
    out_ << "\n";
    out_ << "================================================================\n";
    out_ << "  PID CONTROLLER - HELP & PARAMETER GUIDE\n";
    out_ << "================================================================\n";
    out_ << "\n";
    out_ << "  A PID controller calculates how much corrective action to apply\n";
    out_ << "  to a process in order to reach and maintain a desired value.\n";
    out_ << "\n";
    out_ << "  THREE GAINS:\n";
    out_ << "    Kp (Proportional) - Reacts to current error.\n";
    out_ << "      Too low  -> slow, sluggish response.\n";
    out_ << "      Too high -> oscillation or instability.\n";
    out_ << "      Safe start: 0.5 - 5.0\n";
    out_ << "\n";
    out_ << "    Ki (Integral)     - Eliminates steady-state error over time.\n";
    out_ << "      Too low  -> persistent offset from setpoint.\n";
    out_ << "      Too high -> overshoot and integral wind-up.\n";
    out_ << "      Safe start: 0.0 - 1.0\n";
    out_ << "\n";
    out_ << "    Kd (Derivative)   - Dampens rapid changes (predictive).\n";
    out_ << "      Too low  -> overshoot.\n";
    out_ << "      Too high -> amplifies sensor noise, instability.\n";
    out_ << "      Safe start: 0.0 - 0.5\n";
    out_ << "\n";
    out_ << "  PLANT MODELS:\n";
    out_ << "    1 - First Order  (t*dy/dt = -y + K*u)\n";
    out_ << "        Most common industrial process model.\n";
    out_ << "    2 - Second Order (mass-spring-damper)\n";
    out_ << "        Mechanical/electromechanical systems.\n";
    out_ << "    3 - Integrating  (dy/dt = K*u)\n";
    out_ << "        Tank level, position control.\n";
    out_ << "    4 - Dead Time    (FOPDT - first order + transport delay)\n";
    out_ << "        Processes with measurement/actuator delays.\n";
    out_ << "\n";
    out_ << "  OUTPUT CLAMP: Limits how hard the controller can drive the\n";
    out_ << "  actuator. E.g., a valve can only open 0-100%.\n";
    out_ << "\n";
    out_ << "  ANTI-WINDUP: Prevents the integral term from accumulating\n";
    out_ << "  during actuator saturation, which would cause severe overshoot.\n";
    out_ << "  Recommended: ON.\n";
    out_ << "\n";
    out_ << "----------------------------------------------------------------\n";
    out_ << "\n";
}

// -----------------------------------------------------------------------------
// PromptPIDConfig
// -----------------------------------------------------------------------------
PIDConfig CLI::PromptPIDConfig()
{
    out_ << "\n--- PID Controller Configuration ------------------------------\n";
    out_ << "  Enter PID gains. Press Enter to accept defaults. Type 'help'\n";
    out_ << "  for an explanation of each parameter.\n\n";

    PIDConfig cfg;

    cfg.kp = PromptDouble("  Proportional gain Kp [1.0] (range 0 - 1e6): ",
                          GAIN_MIN, GAIN_MAX, 1.0);

    cfg.ki = PromptDouble("  Integral gain     Ki [0.1] (range 0 - 1e6): ",
                          GAIN_MIN, GAIN_MAX, 0.1);

    cfg.kd = PromptDouble("  Derivative gain   Kd [0.05] (range 0 - 1e6): ",
                          GAIN_MIN, GAIN_MAX, 0.05);

    out_ << "\n  Output clamping limits (constrains actuator range):\n";
    cfg.output_min = PromptDouble("  Output minimum [-100.0]: ",
                                  -OUTPUT_CLAMP_MAX, 0.0, -100.0);

    cfg.output_max = PromptDouble("  Output maximum [100.0]:  ",
                                  0.0, OUTPUT_CLAMP_MAX, 100.0);

    // Ensure min < max
    if (cfg.output_min >= cfg.output_max)
    {
        out_ << "  [!] output_min must be less than output_max. "
                "Resetting to defaults (-100, 100).\n";
        cfg.output_min = -100.0;
        cfg.output_max =  100.0;
    }

    cfg.anti_windup = PromptBool("  Enable anti-windup? [y]: ", true);

    return cfg;
}
// -----------------------------------------------------------------------------
// PromptPlantConfig
// -----------------------------------------------------------------------------
PlantConfig CLI::PromptPlantConfig()
{
    out_ << "\n---- Plant Model Configuration -----------------------------------\n";
    out_ << "  Select the process model to simulate.\n\n";
    out_ << "    1 - First Order  (t*dy/dt = -y + K*u)        [most common]\n";
    out_ << "    2 - Second Order (mass-spring-damper)\n";
    out_ << "    3 - Integrating  (dy/dt = K*u)\n";
    out_ << "    4 - Dead Time    (FOPDT - first order + transport delay)\n\n";

    PlantConfig cfg;

    int model_sel = PromptInt("  Plant model [1]: ", 1, 4, 1);
    switch (model_sel)
    {
        case 1: cfg.model = PlantModel::FIRST_ORDER;  break;
        case 2: cfg.model = PlantModel::SECOND_ORDER; break;
        case 3: cfg.model = PlantModel::INTEGRATING;  break;
        case 4: cfg.model = PlantModel::DEAD_TIME;    break;
        default: cfg.model = PlantModel::FIRST_ORDER; break;
    }

    cfg.gain = PromptDouble("  Process gain K [1.0] (range 0 - 1e6): ",
                            GAIN_MIN, GAIN_MAX, 1.0);

    if (cfg.model == PlantModel::FIRST_ORDER ||
        cfg.model == PlantModel::DEAD_TIME)
    {
        cfg.time_constant = PromptDouble(
            "  Time constant t [1.0 s] (range 1e-6 - 1e6 s): ",
            1e-6, 1e6, 1.0);
    }

    if (cfg.model == PlantModel::SECOND_ORDER)
    {
        cfg.natural_freq = PromptDouble(
            "  Natural frequency ωₙ [1.0 rad/s] (range 0.001 - 1000): ",
            0.001, 1000.0, 1.0);

        cfg.damping_ratio = PromptDouble(
            "  Damping ratio ζ [0.7] (range 0 - 5.0): ",
            0.0, 5.0, 0.7);
    }

    if (cfg.model == PlantModel::DEAD_TIME)
    {
        cfg.dead_time = PromptDouble(
            "  Dead time θ [0.5 s] (range 0 - 100 s): ",
            0.0, 100.0, 0.5);
    }

    cfg.initial_value = PromptDouble(
        "  Initial process value y0 [0.0]: ",
        SETPOINT_MIN, SETPOINT_MAX, 0.0);

    return cfg;
}

// -----------------------------------------------------------------------------
// PromptSimConfig
// -----------------------------------------------------------------------------
SimConfig CLI::PromptSimConfig()
{
    out_ << "\n---- Simulation Parameters ---------------------------------------\n\n";

    SimConfig cfg;

    cfg.setpoint = PromptDouble(
        "  Setpoint (desired value) [10.0]: ",
        SETPOINT_MIN, SETPOINT_MAX, 10.0);

    cfg.duration = PromptDouble(
        "  Simulation duration [20.0 s] (range 0.001 - 3600 s): ",
        SIM_DURATION_MIN, SIM_DURATION_MAX, 20.0);

    cfg.dt = PromptDouble(
        "  Time step dt [0.01 s] (range 1e-6 - 10 s): ",
        DT_MIN, DT_MAX, 0.01);

    // Warn if dt is large relative to duration.
    if (cfg.dt > cfg.duration / 10.0)
    {
        out_ << "  [!] Warning: dt is large relative to duration. "
                "Simulation accuracy may be reduced.\n";
    }

    cfg.show_plot  = PromptBool("  Show ASCII step-response chart? [y]: ", true);
    cfg.print_csv  = PromptBool("  Export raw CSV data?              [n]: ", false);

    return cfg;
}

// -----------------------------------------------------------------------------
// DisplayResults
// -----------------------------------------------------------------------------
void CLI::DisplayResults(const SimulationResult& result) const
{
    if (!result.success)
    {
        out_ << "\n[ERROR] Simulation failed: " << result.error_message << "\n\n";
        return;
    }

    const SimulationMetrics& m = result.metrics;

    out_ << "\n";
    out_ << "================================================================\n";
    out_ << "  SIMULATION RESULTS\n";
    out_ << "================================================================\n";
    out_ << std::fixed << std::setprecision(4);

    auto inf_or = [](double v) -> std::string {
        if (v == std::numeric_limits<double>::infinity()) return "  N/A (not reached)";
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(4) << v;
        return ss.str();
    };

    out_ << "  Data points collected : " << result.data.size()                 << "\n";
    out_ << "  Rise time   (10->90%) : " << inf_or(m.rise_time)       << " s\n";
    out_ << "  Settling time (+/-2%)  : " << inf_or(m.settling_time)   << " s\n";
    out_ << "  Overshoot             : " << std::setprecision(2)
         << m.overshoot_pct                                             << " %\n";
    out_ << "  Peak process value    : " << std::setprecision(4)
         << m.peak_value                                                << "\n";
    out_ << "  Steady-state error    : " << m.steady_state_error       << "\n";
    out_ << "================================================================\n";
    out_ << "\n";

    // Print a condensed sample of the time-series (every ~10% interval).
    if (!result.data.empty())
    {
        const std::size_t n = result.data.size();
        const std::size_t stride = std::max<std::size_t>(1, n / 10);

        out_ << "  " << std::setw(8)  << "Time(s)"
             << "  " << std::setw(10) << "Setpoint"
             << "  " << std::setw(12) << "ProcessVal"
             << "  " << std::setw(12) << "Error"
             << "  " << std::setw(12) << "CtrlOutput"
             << "\n";
        out_ << "  " << std::string(60, '-') << "\n";

        for (std::size_t i = 0; i < n; i += stride)
        {
            const auto& pt = result.data[i];
            out_ << "  " << std::setw(8)  << std::setprecision(3) << pt.time
                 << "  " << std::setw(10) << std::setprecision(4) << pt.setpoint
                 << "  " << std::setw(12) << pt.process_value
                 << "  " << std::setw(12) << pt.error
                 << "  " << std::setw(12) << pt.control_output
                 << "\n";
        }
        // Always print the final row.
        if ((n - 1) % stride != 0)
        {
            const auto& pt = result.data.back();
            out_ << "  " << std::setw(8)  << std::setprecision(3) << pt.time
                 << "  " << std::setw(10) << std::setprecision(4) << pt.setpoint
                 << "  " << std::setw(12) << pt.process_value
                 << "  " << std::setw(12) << pt.error
                 << "  " << std::setw(12) << pt.control_output
                 << "\n";
        }
        out_ << "\n";
    }
}

// -----------------------------------------------------------------------------
// DisplayAsciiPlot
// -----------------------------------------------------------------------------
void CLI::DisplayAsciiPlot(const SimulationResult& result) const
{
    if (!result.success || result.data.empty())
    {
        return;
    }

    // ── Determine Y-axis range ────────────────────────────────────────────────
    double y_min = result.data.front().process_value;
    double y_max = y_min;
    for (const auto& pt : result.data)
    {
        y_min = std::min(y_min, pt.process_value);
        y_max = std::max(y_max, pt.process_value);
    }
    // Include setpoint in range.
    const double sp = result.data.front().setpoint;
    y_min = std::min(y_min, sp);
    y_max = std::max(y_max, sp);

    // Add 10% padding.
    const double y_range = y_max - y_min;
    const double pad     = (y_range > 1e-9) ? y_range * 0.1 : 1.0;
    y_min -= pad;
    y_max += pad;

    // ── Build the character grid ──────────────────────────────────────────────
    // grid[row][col]: row 0 = top of chart, row CHART_HEIGHT-1 = bottom.
    std::vector<std::string> grid(
        static_cast<std::size_t>(CHART_HEIGHT),
        std::string(static_cast<std::size_t>(CHART_WIDTH), ' '));

    auto to_row = [&](double y) -> int {
        if (std::abs(y_max - y_min) < 1e-12) return CHART_HEIGHT / 2;
        int r = static_cast<int>(
            (1.0 - (y - y_min) / (y_max - y_min)) * (CHART_HEIGHT - 1));
        return std::clamp(r, 0, CHART_HEIGHT - 1);
    };

    const std::size_t n = result.data.size();

    // Draw setpoint line ('·')
    {
        const int sp_row = to_row(sp);
        for (int col = 0; col < CHART_WIDTH; ++col)
        {
            grid[static_cast<std::size_t>(sp_row)][static_cast<std::size_t>(col)] = '.';
        }
    }

    // Draw process variable trace ('*')
    for (std::size_t i = 0; i < n; ++i)
    {
        const int col = static_cast<int>(
            static_cast<double>(i) / static_cast<double>(n - 1) * (CHART_WIDTH - 1));
        const int row = to_row(result.data[i].process_value);
        const int safe_col = std::clamp(col, 0, CHART_WIDTH - 1);
        const int safe_row = std::clamp(row, 0, CHART_HEIGHT - 1);
        grid[static_cast<std::size_t>(safe_row)][static_cast<std::size_t>(safe_col)] = '*';
    }

    // ── Render ────────────────────────────────────────────────────────────────
    out_ << "\n  ASCII Step Response Chart\n";
    out_ << "  Legend:  * = Process Value   . = Setpoint\n";
    out_ << "          " << std::string(CHART_WIDTH, '-') << "\n";

    for (int row = 0; row < CHART_HEIGHT; ++row)
    {
        const double y_label = y_max - (static_cast<double>(row) / (CHART_HEIGHT - 1)) * (y_max - y_min);
        out_ << std::setw(8) << std::fixed << std::setprecision(2) << y_label
             << " |" << grid[static_cast<std::size_t>(row)] << "|\n";
    }

    out_ << "          " << std::string(static_cast<std::size_t>(CHART_WIDTH), '-') << "\n";
    out_ << "           t=0" << std::string(CHART_WIDTH - 10, ' ')
         << "t=" << std::fixed << std::setprecision(1)
         << result.data.back().time << "s\n\n";
}

// -----------------------------------------------------------------------------
// ExportCSV
// -----------------------------------------------------------------------------
void CLI::ExportCSV(const SimulationResult& result) const
{
    if (!result.success)
    {
        out_ << "# Simulation failed: " << result.error_message << "\n";
        return;
    }

    out_ << "time_s,setpoint,process_value,error,control_output\n";
    out_ << std::fixed << std::setprecision(6);
    for (const auto& pt : result.data)
    {
        out_ << pt.time          << ","
             << pt.setpoint      << ","
             << pt.process_value << ","
             << pt.error         << ","
             << pt.control_output << "\n";
    }
}

// -----------------------------------------------------------------------------
// AskRunAgain
// -----------------------------------------------------------------------------
bool CLI::AskRunAgain()
{
    return PromptBool("\n  Run another simulation? [n]: ", false);
}

// =============================================================================
// STATIC VALIDATION HELPERS
// =============================================================================

// -----------------------------------------------------------------------------
// ParseDouble
// -----------------------------------------------------------------------------
bool CLI::ParseDouble(const std::string& raw, double lo, double hi, double& out)
{
    if (raw.empty()) return false;

    std::size_t consumed = 0;
    double value = 0.0;

    try
    {
        value = std::stod(raw, &consumed);
    }
    catch (const std::invalid_argument&)
    {
        return false;
    }
    catch (const std::out_of_range&)
    {
        return false;
    }

    // Reject input with trailing non-whitespace characters.
    const std::string remainder = raw.substr(consumed);
    for (char c : remainder)
    {
        if (!std::isspace(static_cast<unsigned char>(c))) return false;
    }

    if (value < lo || value > hi) return false;

    out = value;
    return true;
}

// -----------------------------------------------------------------------------
// ParseInt
// -----------------------------------------------------------------------------
bool CLI::ParseInt(const std::string& raw, int lo, int hi, int& out)
{
    double dval = 0.0;
    if (!ParseDouble(raw, static_cast<double>(lo), static_cast<double>(hi), dval))
    {
        return false;
    }
    // Reject non-integer values (e.g., "1.5").
    if (dval != std::floor(dval)) return false;

    out = static_cast<int>(dval);
    return true;
}

// =============================================================================
// PRIVATE HELPERS
// =============================================================================

// -----------------------------------------------------------------------------
// PromptLine
// -----------------------------------------------------------------------------
std::string CLI::PromptLine(const std::string& prompt) const
{
    out_ << prompt;
    std::string line;
    std::getline(in_, line);
    TrimString(line);
    return line;
}

// -----------------------------------------------------------------------------
// PromptDouble
// -----------------------------------------------------------------------------
double CLI::PromptDouble(const std::string& prompt,
                         double lo, double hi,
                         double default_val)
{
    while (true)
    {
        const std::string raw = PromptLine(prompt);

        // Empty input -> use default.
        if (raw.empty()) return default_val;

        // User asked for help.
        if (raw == "help" || raw == "?")
        {
            PrintHelp();
            continue;
        }

        double value = 0.0;
        if (ParseDouble(raw, lo, hi, value))
        {
            return value;
        }

        out_ << "  [!] Invalid input '" << raw << "'. "
             << "Expected a number in [" << lo << ", " << hi << "]. "
             << "Please try again.\n";
    }
}

// -----------------------------------------------------------------------------
// PromptInt
// -----------------------------------------------------------------------------
int CLI::PromptInt(const std::string& prompt,
                   int lo, int hi,
                   int default_val)
{
    while (true)
    {
        const std::string raw = PromptLine(prompt);

        if (raw.empty()) return default_val;

        if (raw == "help" || raw == "?")
        {
            PrintHelp();
            continue;
        }

        int value = 0;
        if (ParseInt(raw, lo, hi, value))
        {
            return value;
        }

        out_ << "  [!] Invalid input '" << raw << "'. "
             << "Expected an integer in [" << lo << ", " << hi << "]. "
             << "Please try again.\n";
    }
}

// -----------------------------------------------------------------------------
// PromptBool
// -----------------------------------------------------------------------------
bool CLI::PromptBool(const std::string& prompt, bool default_val)
{
    while (true)
    {
        const std::string raw = PromptLine(prompt);

        if (raw.empty()) return default_val;

        if (raw == "y" || raw == "Y" || raw == "yes" || raw == "YES") return true;
        if (raw == "n" || raw == "N" || raw == "no"  || raw == "NO")  return false;

        out_ << "  [!] Please enter 'y' or 'n'.\n";
    }
}