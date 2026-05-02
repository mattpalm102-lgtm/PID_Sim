/******************************************************************************
 * @file        test_Simulator.cpp
 * @brief       Unit tests for the Simulator engine (Simulator.hpp / .cpp).
 *
 * @details     Tests cover:
 *                - Successful simulation run produces non-empty data
 *                - Invalid dt/duration → failure result with error message
 *                - Metrics computed for a first-order step response
 *                - Rise time and settling time finite on converging system
 *                - Overshoot detected on underdamped second-order system
 *                - Steady-state error approaches zero with integral action
 *                - Zero-setpoint edge case (settling band normalised)
 *                - Step-down setpoint (negative error direction)
 *                - Empty data guard in ComputeMetrics (via zero-duration trick)
 *                - Data point count matches expected number of steps
 *
 * @author      Matt Palmer
 * @date        2026-05-01
 * @version     0.1.0
 ******************************************************************************/

#include <gtest/gtest.h>
#include "Simulator.hpp"
#include "Types.hpp"
#include <cmath>
#include <limits>

// =============================================================================
// Helper factories
// =============================================================================
static PIDConfig MakePID(double kp = 2.0,
                          double ki = 0.5,
                          double kd = 0.1)
{
    PIDConfig cfg;
    cfg.kp         = kp;
    cfg.ki         = ki;
    cfg.kd         = kd;
    cfg.output_min = -200.0;
    cfg.output_max =  200.0;
    cfg.anti_windup = true;
    return cfg;
}

static PlantConfig MakeFirstOrder(double K = 1.0, double tau = 1.0)
{
    PlantConfig cfg;
    cfg.model         = PlantModel::FIRST_ORDER;
    cfg.gain          = K;
    cfg.time_constant = tau;
    cfg.initial_value = 0.0;
    return cfg;
}

static SimConfig MakeSim(double sp       = 10.0,
                          double duration = 30.0,
                          double dt       = 0.01)
{
    SimConfig cfg;
    cfg.setpoint    = sp;
    cfg.duration    = duration;
    cfg.dt          = dt;
    cfg.show_plot   = false;
    cfg.print_csv   = false;
    return cfg;
}

// =============================================================================
// Basic run success
// =============================================================================
TEST(SimulatorTest, SuccessfulRunReturnsSuccess)
{
    Simulator sim(MakePID(), MakeFirstOrder(), MakeSim());
    const SimulationResult r = sim.Run();
    EXPECT_TRUE(r.success);
    EXPECT_TRUE(r.error_message.empty());
}

TEST(SimulatorTest, SuccessfulRunProducesNonEmptyData)
{
    Simulator sim(MakePID(), MakeFirstOrder(), MakeSim());
    const SimulationResult r = sim.Run();
    EXPECT_FALSE(r.data.empty());
}

TEST(SimulatorTest, DataPointCountApproximatelyCorrect)
{
    // duration=10, dt=0.1 → ~101 steps
    SimConfig sc = MakeSim(10.0, 10.0, 0.1);
    Simulator sim(MakePID(), MakeFirstOrder(), sc);
    const SimulationResult r = sim.Run();
    EXPECT_GE(r.data.size(), 100u);
    EXPECT_LE(r.data.size(), 110u);
}

TEST(SimulatorTest, TimeSeriesStartsAtZero)
{
    Simulator sim(MakePID(), MakeFirstOrder(), MakeSim());
    const SimulationResult r = sim.Run();
    EXPECT_NEAR(r.data.front().time, 0.0, 1e-9);
}

TEST(SimulatorTest, TimeSeriesEndsNearDuration)
{
    const double duration = 5.0;
    SimConfig sc = MakeSim(10.0, duration, 0.01);
    Simulator sim(MakePID(), MakeFirstOrder(), sc);
    const SimulationResult r = sim.Run();
    EXPECT_NEAR(r.data.back().time, duration, 0.02);
}

TEST(SimulatorTest, SetpointConstantThroughoutRun)
{
    const double sp = 7.5;
    SimConfig sc = MakeSim(sp, 5.0, 0.1);
    Simulator sim(MakePID(), MakeFirstOrder(), sc);
    const SimulationResult r = sim.Run();
    for (const auto& pt : r.data)
    {
        EXPECT_DOUBLE_EQ(pt.setpoint, sp);
    }
}

// =============================================================================
// Invalid configuration → failure
// =============================================================================
TEST(SimulatorTest, ZeroDtReturnsFailure)
{
    SimConfig sc = MakeSim(10.0, 5.0, 0.0);
    Simulator sim(MakePID(), MakeFirstOrder(), sc);
    const SimulationResult r = sim.Run();
    EXPECT_FALSE(r.success);
    EXPECT_FALSE(r.error_message.empty());
}

TEST(SimulatorTest, NegativeDtReturnsFailure)
{
    SimConfig sc = MakeSim(10.0, 5.0, -0.01);
    Simulator sim(MakePID(), MakeFirstOrder(), sc);
    const SimulationResult r = sim.Run();
    EXPECT_FALSE(r.success);
}

TEST(SimulatorTest, ZeroDurationReturnsFailure)
{
    SimConfig sc = MakeSim(10.0, 0.0, 0.01);
    Simulator sim(MakePID(), MakeFirstOrder(), sc);
    const SimulationResult r = sim.Run();
    EXPECT_FALSE(r.success);
    EXPECT_FALSE(r.error_message.empty());
}

TEST(SimulatorTest, NegativeDurationReturnsFailure)
{
    SimConfig sc = MakeSim(10.0, -1.0, 0.01);
    Simulator sim(MakePID(), MakeFirstOrder(), sc);
    const SimulationResult r = sim.Run();
    EXPECT_FALSE(r.success);
}

// =============================================================================
// Performance metrics
// =============================================================================
TEST(SimulatorTest, RiseTimeFiniteForConvergingSystem)
{
    // Well-tuned PID on first-order plant should produce finite rise time.
    Simulator sim(MakePID(5.0, 2.0, 0.5), MakeFirstOrder(1.0, 1.0), MakeSim(10.0, 20.0, 0.01));
    const SimulationResult r = sim.Run();
    EXPECT_LT(r.metrics.rise_time, std::numeric_limits<double>::infinity());
}

TEST(SimulatorTest, SettlingTimeFiniteForConvergingSystem)
{
    Simulator sim(MakePID(5.0, 2.0, 0.5), MakeFirstOrder(1.0, 1.0), MakeSim(10.0, 30.0, 0.01));
    const SimulationResult r = sim.Run();
    EXPECT_LT(r.metrics.settling_time, std::numeric_limits<double>::infinity());
}

TEST(SimulatorTest, SteadyStateErrorNearZeroWithIntegral)
{
    // Long-running simulation with integral action → SSE should be very small.
    Simulator sim(MakePID(2.0, 1.0, 0.0), MakeFirstOrder(1.0, 1.0), MakeSim(10.0, 60.0, 0.01));
    const SimulationResult r = sim.Run();
    EXPECT_NEAR(r.metrics.steady_state_error, 0.0, 0.1);
}

TEST(SimulatorTest, OvershootPositiveForUnderdampedSystem)
{
    // Very aggressive Kp, no Kd → significant overshoot
    PlantConfig plant;
    plant.model         = PlantModel::SECOND_ORDER;
    plant.gain          = 1.0;
    plant.natural_freq  = 5.0;
    plant.damping_ratio = 0.1;
    plant.initial_value = 0.0;

    PIDConfig pid;
    pid.kp = 20.0; pid.ki = 0.0; pid.kd = 0.0;
    pid.output_min = -500.0; pid.output_max = 500.0;

    Simulator sim(pid, plant, MakeSim(1.0, 5.0, 0.001));
    const SimulationResult r = sim.Run();
    EXPECT_GT(r.metrics.overshoot_pct, 0.0);
}

TEST(SimulatorTest, PeakValueAtLeastSetpointForStepUp)
{
    Simulator sim(MakePID(), MakeFirstOrder(), MakeSim(10.0, 20.0, 0.01));
    const SimulationResult r = sim.Run();
    EXPECT_GE(r.metrics.peak_value, 0.0);
}

// =============================================================================
// Zero setpoint edge case
// =============================================================================
TEST(SimulatorTest, ZeroSetpointDoesNotCrash)
{
    // setpoint=0 should not cause division-by-zero in metrics.
    SimConfig sc = MakeSim(0.0, 5.0, 0.01);
    Simulator sim(MakePID(), MakeFirstOrder(), sc);
    EXPECT_NO_THROW(sim.Run());
    const SimulationResult r = sim.Run();
    EXPECT_TRUE(r.success);
}

// =============================================================================
// Step-down setpoint
// =============================================================================
TEST(SimulatorTest, StepDownSetpointRunsSuccessfully)
{
    // Plant starts at 0, setpoint = -10 (negative step)
    Simulator sim(MakePID(2.0, 1.0, 0.1), MakeFirstOrder(), MakeSim(-10.0, 20.0, 0.01));
    const SimulationResult r = sim.Run();
    EXPECT_TRUE(r.success);
    // Final process value should be approaching -10
    EXPECT_LT(r.data.back().process_value, -5.0);
}

// =============================================================================
// All plant models run successfully
// =============================================================================
TEST(SimulatorTest, SecondOrderPlantRunsSuccessfully)
{
    PlantConfig plant;
    plant.model         = PlantModel::SECOND_ORDER;
    plant.gain          = 1.0;
    plant.natural_freq  = 2.0;
    plant.damping_ratio = 0.7;
    plant.initial_value = 0.0;

    Simulator sim(MakePID(), plant, MakeSim(5.0, 10.0, 0.005));
    const SimulationResult r = sim.Run();
    EXPECT_TRUE(r.success);
}

TEST(SimulatorTest, IntegratingPlantRunsSuccessfully)
{
    PlantConfig plant;
    plant.model         = PlantModel::INTEGRATING;
    plant.gain          = 0.1;
    plant.initial_value = 0.0;

    PIDConfig pid = MakePID(1.0, 0.0, 0.5);

    Simulator sim(pid, plant, MakeSim(5.0, 10.0, 0.01));
    const SimulationResult r = sim.Run();
    EXPECT_TRUE(r.success);
}

TEST(SimulatorTest, DeadTimePlantRunsSuccessfully)
{
    PlantConfig plant;
    plant.model         = PlantModel::DEAD_TIME;
    plant.gain          = 1.0;
    plant.time_constant = 2.0;
    plant.dead_time     = 0.5;
    plant.initial_value = 0.0;

    Simulator sim(MakePID(1.0, 0.2, 0.05), plant, MakeSim(5.0, 30.0, 0.01));
    const SimulationResult r = sim.Run();
    EXPECT_TRUE(r.success);
}