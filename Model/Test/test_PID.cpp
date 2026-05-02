/******************************************************************************
 * @file        test_PID.cpp
 * @brief       Unit tests for the PID controller (PID.hpp / PID.cpp).
 *
 * @details     Exercises every code path in the PID controller including:
 *                - Standard proportional, integral, and derivative responses
 *                - Output clamping (saturation)
 *                - Anti-windup prevention and bypass
 *                - Non-positive dt guard path
 *                - Zero-gain degenerate configurations
 *                - State reset correctness
 *                - Numerical precision and boundary values
 *
 *              Coverage target: 100% of PID.cpp lines/branches.
 *
 * @author      Matt Palmer
 * @date        2026-05-01
 * @version     0.1.0
 ******************************************************************************/

#include <gtest/gtest.h>
#include "PID.hpp"
#include "Types.hpp"
#include <cmath>
#include <limits>

// =============================================================================
// Helper: build a minimal PIDConfig
// =============================================================================
static PIDConfig MakeConfig(double kp = 1.0,
                             double ki = 0.0,
                             double kd = 0.0,
                             double out_min = -100.0,
                             double out_max =  100.0,
                             bool   anti_windup = true)
{
    PIDConfig cfg;
    cfg.kp          = kp;
    cfg.ki          = ki;
    cfg.kd          = kd;
    cfg.output_min  = out_min;
    cfg.output_max  = out_max;
    cfg.anti_windup = anti_windup;
    return cfg;
}

// =============================================================================
// Construction & Accessors
// =============================================================================
TEST(PIDTest, ConstructionStoresConfig)
{
    const PIDConfig cfg = MakeConfig(2.0, 0.5, 0.1);
    PID pid(cfg);
    EXPECT_DOUBLE_EQ(pid.GetConfig().kp, 2.0);
    EXPECT_DOUBLE_EQ(pid.GetConfig().ki, 0.5);
    EXPECT_DOUBLE_EQ(pid.GetConfig().kd, 0.1);
}

TEST(PIDTest, InitialIntegralIsZero)
{
    PID pid(MakeConfig());
    EXPECT_DOUBLE_EQ(pid.GetIntegral(), 0.0);
}

TEST(PIDTest, InitialPrevErrorIsZero)
{
    PID pid(MakeConfig());
    EXPECT_DOUBLE_EQ(pid.GetPreviousError(), 0.0);
}

// =============================================================================
// Proportional-only behaviour
// =============================================================================
TEST(PIDTest, ProportionalOnlyPositiveError)
{
    // Kp=2, Ki=0, Kd=0  → output = 2 * (sp - meas) = 2 * 5 = 10
    PID pid(MakeConfig(2.0, 0.0, 0.0));
    const double out = pid.Update(10.0, 5.0, 0.1);
    EXPECT_DOUBLE_EQ(out, 10.0);
}

TEST(PIDTest, ProportionalOnlyNegativeError)
{
    PID pid(MakeConfig(3.0, 0.0, 0.0));
    const double out = pid.Update(5.0, 10.0, 0.1);
    EXPECT_DOUBLE_EQ(out, -15.0);
}

TEST(PIDTest, ProportionalOnlyZeroError)
{
    PID pid(MakeConfig(5.0, 0.0, 0.0));
    const double out = pid.Update(7.0, 7.0, 0.1);
    EXPECT_DOUBLE_EQ(out, 0.0);
}

// =============================================================================
// Integral-only behaviour
// =============================================================================
TEST(PIDTest, IntegralAccumulates)
{
    // Ki=1, error=5, dt=0.1 → integral after 1 step = 0.5, output = 0.5
    PID pid(MakeConfig(0.0, 1.0, 0.0));
    const double out = pid.Update(5.0, 0.0, 0.1);
    EXPECT_NEAR(out, 0.5, 1e-9);
    EXPECT_NEAR(pid.GetIntegral(), 0.5, 1e-9);
}

TEST(PIDTest, IntegralAccumulatesOverMultipleSteps)
{
    // Ki=1, error=5, dt=0.1, 3 steps → integral = 1.5, output = 1.5
    PID pid(MakeConfig(0.0, 1.0, 0.0));
    for (int i = 0; i < 3; ++i)
    {
        pid.Update(5.0, 0.0, 0.1);
    }
    EXPECT_NEAR(pid.GetIntegral(), 1.5, 1e-9);
}

// =============================================================================
// Derivative-only behaviour
// =============================================================================
TEST(PIDTest, DerivativeFirstStepUsesZeroPrevError)
{
    // Kd=1, first call: prev_error=0, error=5, dt=0.1
    // derivative = (5 - 0) / 0.1 = 50 → output = 50
    PID pid(MakeConfig(0.0, 0.0, 1.0));
    const double out = pid.Update(5.0, 0.0, 0.1);
    EXPECT_NEAR(out, 50.0, 1e-9);
}

TEST(PIDTest, DerivativeZeroOnConstantError)
{
    // Same error twice → derivative = 0 on second call
    PID pid(MakeConfig(0.0, 0.0, 1.0));
    pid.Update(5.0, 0.0, 0.1);   // primes prev_error = 5
    const double out = pid.Update(5.0, 0.0, 0.1);
    EXPECT_NEAR(out, 0.0, 1e-9);
}

TEST(PIDTest, DerivativeNegativeSlope)
{
    // error goes from 5 to 2; derivative = (2-5)/0.1 = -30, Kd=1 → -30
    PID pid(MakeConfig(0.0, 0.0, 1.0));
    pid.Update(5.0, 0.0, 0.1);   // primes prev_error = 5
    const double out = pid.Update(2.0, 0.0, 0.1);
    EXPECT_NEAR(out, -30.0, 1e-9);
}

// =============================================================================
// Output clamping
// =============================================================================
TEST(PIDTest, ClampUpperBound)
{
    // Kp=1000, error=1 → raw 1000, clamped to 100
    PID pid(MakeConfig(1000.0, 0.0, 0.0, -100.0, 100.0));
    const double out = pid.Update(1.0, 0.0, 0.1);
    EXPECT_DOUBLE_EQ(out, 100.0);
}

TEST(PIDTest, ClampLowerBound)
{
    PID pid(MakeConfig(1000.0, 0.0, 0.0, -100.0, 100.0));
    const double out = pid.Update(-1.0, 0.0, 0.1);
    EXPECT_DOUBLE_EQ(out, -100.0);
}

TEST(PIDTest, OutputWithinBoundsNotClamped)
{
    PID pid(MakeConfig(1.0, 0.0, 0.0, -100.0, 100.0));
    const double out = pid.Update(10.0, 0.0, 0.1);
    EXPECT_DOUBLE_EQ(out, 10.0);
}

TEST(PIDTest, ClampAtExactBoundary)
{
    PID pid(MakeConfig(1.0, 0.0, 0.0, -50.0, 50.0));
    EXPECT_DOUBLE_EQ(pid.Update(50.0, 0.0, 0.1), 50.0);
    pid.Reset();
    EXPECT_DOUBLE_EQ(pid.Update(-50.0, 0.0, 0.1), -50.0);
}

// =============================================================================
// Anti-windup
// =============================================================================
TEST(PIDTest, AntiWindupPreventsIntegralGrowthWhenSaturatedPositive)
{
    // Kp=0, Ki=1, large positive error → saturates at output_max
    // With anti-windup, integral should NOT grow indefinitely.
    PID pid(MakeConfig(0.0, 1.0, 0.0, -10.0, 10.0, true));

    for (int i = 0; i < 100; ++i)
    {
        pid.Update(100.0, 0.0, 0.1);  // error=100, saturated high
    }
    // Integral should be frozen at the point of saturation, not 100*100*0.1=1000
    EXPECT_LT(pid.GetIntegral(), 200.0);
}

TEST(PIDTest, AntiWindupPreventsIntegralGrowthWhenSaturatedNegative)
{
    PID pid(MakeConfig(0.0, 1.0, 0.0, -10.0, 10.0, true));

    for (int i = 0; i < 100; ++i)
    {
        pid.Update(-100.0, 0.0, 0.1);  // error=-100, saturated low
    }
    EXPECT_GT(pid.GetIntegral(), -200.0);
}

TEST(PIDTest, AntiWindupDisabledAllowsWindup)
{
    // Without anti-windup integral keeps accumulating.
    PID pid(MakeConfig(0.0, 1.0, 0.0, -10.0, 10.0, false));

    for (int i = 0; i < 20; ++i)
    {
        pid.Update(100.0, 0.0, 0.1);  // error=100
    }
    // integral = 100 * 0.1 * 20 = 200 — well beyond output bounds
    EXPECT_NEAR(pid.GetIntegral(), 200.0, 1e-6);
}

TEST(PIDTest, AntiWindupAllowsIntegralWhenNotSaturated)
{
    // Small error that does not saturate; integral must still accumulate.
    PID pid(MakeConfig(0.0, 1.0, 0.0, -100.0, 100.0, true));
    for (int i = 0; i < 5; ++i)
    {
        pid.Update(1.0, 0.0, 0.1);
    }
    EXPECT_NEAR(pid.GetIntegral(), 0.5, 1e-9);
}

// =============================================================================
// Non-positive dt guard
// =============================================================================
TEST(PIDTest, ZeroDtDoesNotCrash)
{
    // dt=0 should not cause UB; derivative term should be zeroed.
    PID pid(MakeConfig(1.0, 1.0, 1.0));
    // This should not crash or throw.
    EXPECT_NO_THROW(pid.Update(5.0, 0.0, 0.0));
}

TEST(PIDTest, ZeroDtDerivativeIsZero)
{
    // With dt=0 the derivative contribution must be zero.
    // Kp=0, Ki=0, Kd=1 → if derivative computed, output would be non-zero.
    PID pid(MakeConfig(0.0, 0.0, 1.0));
    // dt=0: derivative zeroed → output = 0 (no P or I contribution either)
    // Integral update: integral_ += error * 0 → no change; output = 0 + 0 + 0
    const double out = pid.Update(5.0, 0.0, 0.0);
    EXPECT_NEAR(out, 0.0, 1e-9);
}

TEST(PIDTest, NegativeDtDoesNotCrash)
{
    PID pid(MakeConfig(1.0, 0.0, 0.0));
    EXPECT_NO_THROW(pid.Update(5.0, 0.0, -0.1));
}

// =============================================================================
// Reset behaviour
// =============================================================================
TEST(PIDTest, ResetClearsIntegral)
{
    PID pid(MakeConfig(0.0, 1.0, 0.0));
    pid.Update(5.0, 0.0, 0.1);
    EXPECT_GT(pid.GetIntegral(), 0.0);
    pid.Reset();
    EXPECT_DOUBLE_EQ(pid.GetIntegral(), 0.0);
}

TEST(PIDTest, ResetClearsPrevError)
{
    PID pid(MakeConfig(0.0, 0.0, 1.0));
    pid.Update(5.0, 0.0, 0.1);
    EXPECT_NE(pid.GetPreviousError(), 0.0);
    pid.Reset();
    EXPECT_DOUBLE_EQ(pid.GetPreviousError(), 0.0);
}

TEST(PIDTest, ResetAllowsCleanRestart)
{
    PID pid(MakeConfig(1.0, 1.0, 0.0));
    for (int i = 0; i < 10; ++i) pid.Update(10.0, 0.0, 0.1);

    pid.Reset();

    // After reset, first call should give same result as a fresh PID.
    const double out_reset = pid.Update(10.0, 0.0, 0.1);

    PID fresh(MakeConfig(1.0, 1.0, 0.0));
    const double out_fresh = fresh.Update(10.0, 0.0, 0.1);

    EXPECT_DOUBLE_EQ(out_reset, out_fresh);
}

// =============================================================================
// Combined PID response
// =============================================================================
TEST(PIDTest, CombinedPIDResponse)
{
    // Manual calculation:
    // Kp=1, Ki=0.5, Kd=0.2  setpoint=10, meas=0, dt=0.1
    // error=10, p=10, integral=1.0, i=0.5, derivative=(10-0)/0.1=100, d=20
    // raw = 30.5, within bounds → output = 30.5
    PID pid(MakeConfig(1.0, 0.5, 0.2));
    const double out = pid.Update(10.0, 0.0, 0.1);
    EXPECT_NEAR(out, 30.5, 1e-9);
}

// =============================================================================
// Zero-gain degenerate cases
// =============================================================================
TEST(PIDTest, AllZeroGainsAlwaysOutputZero)
{
    PID pid(MakeConfig(0.0, 0.0, 0.0));
    for (int i = 0; i < 5; ++i)
    {
        EXPECT_DOUBLE_EQ(pid.Update(100.0, 0.0, 0.1), 0.0);
    }
}

// =============================================================================
// Setpoint == measurement (zero error)
// =============================================================================
TEST(PIDTest, ZeroErrorProducesZeroOutputWithNoIntegralHistory)
{
    PID pid(MakeConfig(2.0, 0.5, 0.3));
    const double out = pid.Update(5.0, 5.0, 0.1);
    EXPECT_DOUBLE_EQ(out, 0.0);
}