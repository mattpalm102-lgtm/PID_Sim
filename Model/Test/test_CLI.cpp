/******************************************************************************
 * @file        test_CLI.cpp
 * @brief       Unit tests for the CLI class (CLI.hpp / CLI.cpp).
 *
 * @details     Tests all CLI behaviour using injected std::stringstream I/O
 *              to avoid requiring a real terminal. Covers:
 *
 *                ParseDouble / ParseInt static validators:
 *                  - Valid values within range
 *                  - Out-of-range rejection (above and below)
 *                  - Non-numeric input rejection
 *                  - Trailing garbage rejection
 *                  - Empty string rejection
 *                  - Boundary exact-match acceptance
 *                  - Floating-point non-integer rejection in ParseInt
 *
 *                PromptPIDConfig / PromptPlantConfig / PromptSimConfig:
 *                  - Defaults accepted on empty input
 *                  - Invalid-then-valid retry loop
 *                  - 'help' input loops correctly and continues
 *                  - output_min >= output_max reset to defaults
 *
 *                DisplayResults:
 *                  - Failed result prints error message
 *                  - Successful result contains expected labels
 *
 *                DisplayAsciiPlot:
 *                  - Empty/failed result does not crash or emit chart
 *                  - Valid result emits chart markers
 *
 *                ExportCSV:
 *                  - Header row is first line
 *                  - Correct number of data rows
 *                  - Failed result emits comment line
 *
 *                AskRunAgain:
 *                  - 'y' → true, 'n' → false, default → false
 *
 * @author      Matt Palmer
 * @date        2026-05-01
 * @version     0.1.0
 ******************************************************************************/

#include <gtest/gtest.h>
#include "CLI.hpp"
#include "Types.hpp"
#include "Simulator.hpp"
#include <sstream>
#include <string>
#include <limits>

// =============================================================================
// Helper: build a successful SimulationResult with N data points
// =============================================================================
static SimulationResult MakeResult(std::size_t n = 10, bool success = true,
                                    double sp = 10.0)
{
    SimulationResult r;
    r.success = success;
    if (!success)
    {
        r.error_message = "Simulated failure";
        return r;
    }
    for (std::size_t i = 0; i < n; ++i)
    {
        SimulationPoint pt;
        pt.time           = static_cast<double>(i) * 0.1;
        pt.setpoint       = sp;
        pt.process_value  = sp * (1.0 - std::exp(-static_cast<double>(i) * 0.1));
        pt.error          = sp - pt.process_value;
        pt.control_output = pt.error * 2.0;
        r.data.push_back(pt);
    }
    r.metrics.rise_time          = 0.5;
    r.metrics.settling_time      = 1.2;
    r.metrics.overshoot_pct      = 5.0;
    r.metrics.peak_value         = sp * 1.05;
    r.metrics.steady_state_error = 0.01;
    return r;
}

// =============================================================================
// ParseDouble
// =============================================================================
TEST(CLIParseDoubleTest, ValidValueInRange)
{
    double out = 0.0;
    EXPECT_TRUE(CLI::ParseDouble("5.0", 0.0, 10.0, out));
    EXPECT_DOUBLE_EQ(out, 5.0);
}

TEST(CLIParseDoubleTest, ExactLowerBound)
{
    double out = 0.0;
    EXPECT_TRUE(CLI::ParseDouble("0.0", 0.0, 10.0, out));
    EXPECT_DOUBLE_EQ(out, 0.0);
}

TEST(CLIParseDoubleTest, ExactUpperBound)
{
    double out = 0.0;
    EXPECT_TRUE(CLI::ParseDouble("10.0", 0.0, 10.0, out));
    EXPECT_DOUBLE_EQ(out, 10.0);
}

TEST(CLIParseDoubleTest, BelowLowerBound)
{
    double out = 0.0;
    EXPECT_FALSE(CLI::ParseDouble("-1.0", 0.0, 10.0, out));
}

TEST(CLIParseDoubleTest, AboveUpperBound)
{
    double out = 0.0;
    EXPECT_FALSE(CLI::ParseDouble("11.0", 0.0, 10.0, out));
}

TEST(CLIParseDoubleTest, NonNumericInput)
{
    double out = 0.0;
    EXPECT_FALSE(CLI::ParseDouble("abc", 0.0, 10.0, out));
}

TEST(CLIParseDoubleTest, EmptyString)
{
    double out = 0.0;
    EXPECT_FALSE(CLI::ParseDouble("", 0.0, 10.0, out));
}

TEST(CLIParseDoubleTest, TrailingGarbage)
{
    double out = 0.0;
    EXPECT_FALSE(CLI::ParseDouble("5.0abc", 0.0, 10.0, out));
}

TEST(CLIParseDoubleTest, LeadingWhitespaceAccepted)
{
    // stod accepts leading whitespace; we accept it too.
    double out = 0.0;
    EXPECT_TRUE(CLI::ParseDouble("  3.5", 0.0, 10.0, out));
    EXPECT_DOUBLE_EQ(out, 3.5);
}

TEST(CLIParseDoubleTest, TrailingWhitespaceAccepted)
{
    double out = 0.0;
    EXPECT_TRUE(CLI::ParseDouble("3.5  ", 0.0, 10.0, out));
    EXPECT_DOUBLE_EQ(out, 3.5);
}

TEST(CLIParseDoubleTest, NegativeValueAllowed)
{
    double out = 0.0;
    EXPECT_TRUE(CLI::ParseDouble("-5.0", -10.0, 0.0, out));
    EXPECT_DOUBLE_EQ(out, -5.0);
}

// =============================================================================
// ParseInt
// =============================================================================
TEST(CLIParseIntTest, ValidInteger)
{
    int out = 0;
    EXPECT_TRUE(CLI::ParseInt("3", 1, 4, out));
    EXPECT_EQ(out, 3);
}

TEST(CLIParseIntTest, ExactBoundary)
{
    int out = 0;
    EXPECT_TRUE(CLI::ParseInt("1", 1, 4, out));
    EXPECT_EQ(out, 1);
    EXPECT_TRUE(CLI::ParseInt("4", 1, 4, out));
    EXPECT_EQ(out, 4);
}

TEST(CLIParseIntTest, FloatingPointRejected)
{
    int out = 0;
    EXPECT_FALSE(CLI::ParseInt("2.5", 1, 4, out));
}

TEST(CLIParseIntTest, OutOfRange)
{
    int out = 0;
    EXPECT_FALSE(CLI::ParseInt("0", 1, 4, out));
    EXPECT_FALSE(CLI::ParseInt("5", 1, 4, out));
}

TEST(CLIParseIntTest, NonNumericRejected)
{
    int out = 0;
    EXPECT_FALSE(CLI::ParseInt("xyz", 1, 4, out));
}

TEST(CLIParseIntTest, EmptyStringRejected)
{
    int out = 0;
    EXPECT_FALSE(CLI::ParseInt("", 1, 4, out));
}

// =============================================================================
// PrintBanner — smoke test (does not crash, produces output)
// =============================================================================
TEST(CLIBannerTest, PrintsBannerWithAppName)
{
    std::istringstream in("");
    std::ostringstream out;
    CLI cli(in, out);
    cli.PrintBanner();
    EXPECT_NE(out.str().find(APP_NAME), std::string::npos);
}

TEST(CLIBannerTest, PrintsBannerWithVersion)
{
    std::istringstream in("");
    std::ostringstream out;
    CLI cli(in, out);
    cli.PrintBanner();
    EXPECT_NE(out.str().find(APP_VERSION), std::string::npos);
}

// =============================================================================
// PrintHelp — smoke test
// =============================================================================
TEST(CLIHelpTest, PrintsHelpContainsKp)
{
    std::istringstream in("");
    std::ostringstream out;
    CLI cli(in, out);
    cli.PrintHelp();
    EXPECT_NE(out.str().find("Kp"), std::string::npos);
}

// =============================================================================
// PromptPIDConfig — defaults
// =============================================================================
TEST(CLIPromptPIDTest, AllDefaultsAccepted)
{
    // Ten empty lines → all defaults accepted.
    std::istringstream in("\n\n\n\n\n\n\n\n\n\n");
    std::ostringstream out;
    CLI cli(in, out);
    const PIDConfig cfg = cli.PromptPIDConfig();
    EXPECT_DOUBLE_EQ(cfg.kp, 1.0);
    EXPECT_DOUBLE_EQ(cfg.ki, 0.1);
    EXPECT_DOUBLE_EQ(cfg.kd, 0.05);
    EXPECT_DOUBLE_EQ(cfg.output_min, -100.0);
    EXPECT_DOUBLE_EQ(cfg.output_max,  100.0);
    EXPECT_TRUE(cfg.anti_windup);
}

TEST(CLIPromptPIDTest, CustomGainsAccepted)
{
    // Provide values: Kp=3, Ki=0.5, Kd=0.2, min=-50, max=50, antiwindup=n
    std::istringstream in("3\n0.5\n0.2\n-50\n50\nn\n");
    std::ostringstream out;
    CLI cli(in, out);
    const PIDConfig cfg = cli.PromptPIDConfig();
    EXPECT_DOUBLE_EQ(cfg.kp, 3.0);
    EXPECT_DOUBLE_EQ(cfg.ki, 0.5);
    EXPECT_DOUBLE_EQ(cfg.kd, 0.2);
    EXPECT_DOUBLE_EQ(cfg.output_min, -50.0);
    EXPECT_DOUBLE_EQ(cfg.output_max,  50.0);
    EXPECT_FALSE(cfg.anti_windup);
}

TEST(CLIPromptPIDTest, InvalidKpTriggersRetry)
{
    // First Kp is invalid ("abc"), second is valid ("2.0")
    std::istringstream in("abc\n2.0\n\n\n\n\n\n\n\n");
    std::ostringstream out;
    CLI cli(in, out);
    const PIDConfig cfg = cli.PromptPIDConfig();
    EXPECT_DOUBLE_EQ(cfg.kp, 2.0);
}

TEST(CLIPromptPIDTest, OutputMinGeMaxResetsToDefaults)
{
    // output_min=50 and output_max=10 → min >= max, should reset
    std::istringstream in("1\n0.1\n0.05\n50\n10\n\n");
    std::ostringstream out;
    CLI cli(in, out);
    const PIDConfig cfg = cli.PromptPIDConfig();
    EXPECT_DOUBLE_EQ(cfg.output_min, -100.0);
    EXPECT_DOUBLE_EQ(cfg.output_max,  100.0);
}

TEST(CLIPromptPIDTest, HelpInputLoopsWithoutCrash)
{
    // 'help' followed by valid value
    std::istringstream in("help\n1.0\n\n\n\n\n\n\n\n");
    std::ostringstream out;
    CLI cli(in, out);
    const PIDConfig cfg = cli.PromptPIDConfig();
    EXPECT_DOUBLE_EQ(cfg.kp, 1.0);
}

// =============================================================================
// PromptPlantConfig — all models
// =============================================================================
TEST(CLIPromptPlantTest, DefaultsFirstOrder)
{
    std::istringstream in("\n\n\n\n\n\n");
    std::ostringstream out;
    CLI cli(in, out);
    const PlantConfig cfg = cli.PromptPlantConfig();
    EXPECT_EQ(cfg.model, PlantModel::FIRST_ORDER);
    EXPECT_DOUBLE_EQ(cfg.gain, 1.0);
}

TEST(CLIPromptPlantTest, SecondOrderSelected)
{
    // model=2, gain=1, wn=2, zeta=0.5, y0=0
    std::istringstream in("2\n1\n2\n0.5\n0\n");
    std::ostringstream out;
    CLI cli(in, out);
    const PlantConfig cfg = cli.PromptPlantConfig();
    EXPECT_EQ(cfg.model, PlantModel::SECOND_ORDER);
    EXPECT_DOUBLE_EQ(cfg.natural_freq, 2.0);
    EXPECT_DOUBLE_EQ(cfg.damping_ratio, 0.5);
}

TEST(CLIPromptPlantTest, IntegratingSelected)
{
    std::istringstream in("3\n1\n0\n");
    std::ostringstream out;
    CLI cli(in, out);
    const PlantConfig cfg = cli.PromptPlantConfig();
    EXPECT_EQ(cfg.model, PlantModel::INTEGRATING);
}

TEST(CLIPromptPlantTest, DeadTimeSelected)
{
    // model=4, gain=1, tau=2, dead_time=0.5, y0=0
    std::istringstream in("4\n1\n2\n0.5\n0\n");
    std::ostringstream out;
    CLI cli(in, out);
    const PlantConfig cfg = cli.PromptPlantConfig();
    EXPECT_EQ(cfg.model, PlantModel::DEAD_TIME);
    EXPECT_DOUBLE_EQ(cfg.dead_time, 0.5);
}

// =============================================================================
// PromptSimConfig
// =============================================================================
TEST(CLIPromptSimTest, DefaultsAccepted)
{
    std::istringstream in("\n\n\n\n\n");
    std::ostringstream out;
    CLI cli(in, out);
    const SimConfig cfg = cli.PromptSimConfig();
    EXPECT_DOUBLE_EQ(cfg.setpoint, 10.0);
    EXPECT_DOUBLE_EQ(cfg.duration, 20.0);
    EXPECT_DOUBLE_EQ(cfg.dt,       0.01);
    EXPECT_TRUE(cfg.show_plot);
    EXPECT_FALSE(cfg.print_csv);
}

TEST(CLIPromptSimTest, CustomValuesAccepted)
{
    std::istringstream in("25\n15\n0.05\nn\ny\n");
    std::ostringstream out;
    CLI cli(in, out);
    const SimConfig cfg = cli.PromptSimConfig();
    EXPECT_DOUBLE_EQ(cfg.setpoint, 25.0);
    EXPECT_DOUBLE_EQ(cfg.duration, 15.0);
    EXPECT_DOUBLE_EQ(cfg.dt,       0.05);
    EXPECT_FALSE(cfg.show_plot);
    EXPECT_TRUE(cfg.print_csv);
}

TEST(CLIPromptSimTest, LargeDtWarningIssued)
{
    // dt=5, duration=6 → dt > duration/10; warning expected in output
    std::istringstream in("10\n6\n5\n\n\n");
    std::ostringstream out;
    CLI cli(in, out);
    cli.PromptSimConfig();
    EXPECT_NE(out.str().find("Warning"), std::string::npos);
}

// =============================================================================
// DisplayResults
// =============================================================================
TEST(CLIDisplayTest, FailedResultShowsError)
{
    std::istringstream in("");
    std::ostringstream out;
    CLI cli(in, out);
    cli.DisplayResults(MakeResult(0, false));
    EXPECT_NE(out.str().find("ERROR"), std::string::npos);
}

TEST(CLIDisplayTest, SuccessfulResultShowsMetricLabels)
{
    std::istringstream in("");
    std::ostringstream out;
    CLI cli(in, out);
    cli.DisplayResults(MakeResult(50, true));
    const std::string s = out.str();
    EXPECT_NE(s.find("Rise time"), std::string::npos);
    EXPECT_NE(s.find("Settling"), std::string::npos);
    EXPECT_NE(s.find("Overshoot"), std::string::npos);
    EXPECT_NE(s.find("Steady-state"), std::string::npos);
}

TEST(CLIDisplayTest, SuccessfulResultShowsDataPointCount)
{
    std::istringstream in("");
    std::ostringstream out;
    CLI cli(in, out);
    cli.DisplayResults(MakeResult(20, true));
    EXPECT_NE(out.str().find("20"), std::string::npos);
}

TEST(CLIDisplayTest, InfiniteRiseTimeShownAsNA)
{
    SimulationResult r = MakeResult(10, true);
    r.metrics.rise_time = std::numeric_limits<double>::infinity();
    std::istringstream in("");
    std::ostringstream out;
    CLI cli(in, out);
    cli.DisplayResults(r);
    EXPECT_NE(out.str().find("N/A"), std::string::npos);
}

// =============================================================================
// DisplayAsciiPlot
// =============================================================================
TEST(CLIAsciiPlotTest, FailedResultProducesNoOutput)
{
    std::istringstream in("");
    std::ostringstream out;
    CLI cli(in, out);
    cli.DisplayAsciiPlot(MakeResult(0, false));
    EXPECT_TRUE(out.str().empty());
}

TEST(CLIAsciiPlotTest, EmptyDataProducesNoOutput)
{
    SimulationResult r;
    r.success = true;
    // data is empty
    std::istringstream in("");
    std::ostringstream out;
    CLI cli(in, out);
    cli.DisplayAsciiPlot(r);
    EXPECT_TRUE(out.str().empty());
}

TEST(CLIAsciiPlotTest, ValidResultContainsAsterisk)
{
    std::istringstream in("");
    std::ostringstream out;
    CLI cli(in, out);
    cli.DisplayAsciiPlot(MakeResult(100, true));
    EXPECT_NE(out.str().find('*'), std::string::npos);
}

TEST(CLIAsciiPlotTest, ValidResultContainsDotForSetpoint)
{
    std::istringstream in("");
    std::ostringstream out;
    CLI cli(in, out);
    cli.DisplayAsciiPlot(MakeResult(100, true));
    EXPECT_NE(out.str().find('.'), std::string::npos);
}

TEST(CLIAsciiPlotTest, FlatDataDoesNotCrash)
{
    // All process values identical → y_range = 0 → padding kicks in.
    SimulationResult r;
    r.success = true;
    for (int i = 0; i < 20; ++i)
    {
        SimulationPoint pt;
        pt.time = static_cast<double>(i) * 0.1;
        pt.setpoint = 5.0;
        pt.process_value = 5.0;
        pt.error = 0.0;
        pt.control_output = 0.0;
        r.data.push_back(pt);
    }
    std::istringstream in("");
    std::ostringstream out;
    CLI cli(in, out);
    EXPECT_NO_THROW(cli.DisplayAsciiPlot(r));
}

// =============================================================================
// ExportCSV
// =============================================================================
TEST(CLICSVTest, HeaderRowIsFirstLine)
{
    std::istringstream in("");
    std::ostringstream out;
    CLI cli(in, out);
    cli.ExportCSV(MakeResult(5, true));
    const std::string s = out.str();
    EXPECT_EQ(s.substr(0, 4), "time");
}

TEST(CLICSVTest, CorrectNumberOfDataRows)
{
    std::istringstream in("");
    std::ostringstream out;
    CLI cli(in, out);
    cli.ExportCSV(MakeResult(5, true));

    std::istringstream ss(out.str());
    std::string line;
    int rows = 0;
    while (std::getline(ss, line)) ++rows;
    // 1 header + 5 data rows = 6
    EXPECT_EQ(rows, 6);
}

TEST(CLICSVTest, FailedResultEmitsCommentLine)
{
    std::istringstream in("");
    std::ostringstream out;
    CLI cli(in, out);
    cli.ExportCSV(MakeResult(0, false));
    EXPECT_EQ(out.str()[0], '#');
}

// =============================================================================
// AskRunAgain
// =============================================================================
TEST(CLIRunAgainTest, YesReturnsTrue)
{
    std::istringstream in("y\n");
    std::ostringstream out;
    CLI cli(in, out);
    EXPECT_TRUE(cli.AskRunAgain());
}

TEST(CLIRunAgainTest, UppercaseYReturnsTrue)
{
    std::istringstream in("Y\n");
    std::ostringstream out;
    CLI cli(in, out);
    EXPECT_TRUE(cli.AskRunAgain());
}

TEST(CLIRunAgainTest, NoReturnsFalse)
{
    std::istringstream in("n\n");
    std::ostringstream out;
    CLI cli(in, out);
    EXPECT_FALSE(cli.AskRunAgain());
}

TEST(CLIRunAgainTest, DefaultReturnsFalse)
{
    std::istringstream in("\n");
    std::ostringstream out;
    CLI cli(in, out);
    EXPECT_FALSE(cli.AskRunAgain());
}

TEST(CLIRunAgainTest, InvalidThenValidInput)
{
    std::istringstream in("maybe\ny\n");
    std::ostringstream out;
    CLI cli(in, out);
    EXPECT_TRUE(cli.AskRunAgain());
}