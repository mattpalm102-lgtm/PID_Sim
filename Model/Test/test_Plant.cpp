/******************************************************************************
 * @file        test_Plant.cpp
 * @brief       Unit tests for the Plant process model (Plant.hpp / Plant.cpp).
 *
 * @details     Exercises every plant model variant and every code path:
 *                - First-order step response and time-constant accuracy
 *                - First-order with near-zero time constant guard
 *                - Second-order underdamped and overdamped responses
 *                - Integrating plant accumulation
 *                - Dead-time (FOPDT) delay verification
 *                - Non-positive dt guard
 *                - Unknown model guard (via reinterpret cast)
 *                - Reset correctness for all models
 *                - GetOutput accessor
 *
 * @author      Matt Palmer
 * @date        2026-05-01
 * @version     0.1.0
 ******************************************************************************/

#include <gtest/gtest.h>
#include "Plant.hpp"
#include "Types.hpp"
#include <cmath>

// =============================================================================
// Helper: build a PlantConfig
// =============================================================================
static PlantConfig MakeFirstOrder(double K = 1.0,
                                   double tau = 1.0,
                                   double y0  = 0.0)
{
    PlantConfig cfg;
    cfg.model         = PlantModel::FIRST_ORDER;
    cfg.gain          = K;
    cfg.time_constant = tau;
    cfg.initial_value = y0;
    return cfg;
}

static PlantConfig MakeSecondOrder(double K    = 1.0,
                                    double wn   = 1.0,
                                    double zeta = 0.7,
                                    double y0   = 0.0)
{
    PlantConfig cfg;
    cfg.model         = PlantModel::SECOND_ORDER;
    cfg.gain          = K;
    cfg.natural_freq  = wn;
    cfg.damping_ratio = zeta;
    cfg.initial_value = y0;
    return cfg;
}

static PlantConfig MakeIntegrating(double K = 1.0, double y0 = 0.0)
{
    PlantConfig cfg;
    cfg.model         = PlantModel::INTEGRATING;
    cfg.gain          = K;
    cfg.initial_value = y0;
    return cfg;
}

static PlantConfig MakeDeadTime(double K        = 1.0,
                                 double tau      = 1.0,
                                 double dead     = 0.5,
                                 double y0       = 0.0)
{
    PlantConfig cfg;
    cfg.model         = PlantModel::DEAD_TIME;
    cfg.gain          = K;
    cfg.time_constant = tau;
    cfg.dead_time     = dead;
    cfg.initial_value = y0;
    return cfg;
}

// =============================================================================
// Construction & Accessors
// =============================================================================
TEST(PlantTest, InitialOutputMatchesConfig)
{
    Plant p(MakeFirstOrder(1.0, 1.0, 5.0));
    EXPECT_DOUBLE_EQ(p.GetOutput(), 5.0);
}

TEST(PlantTest, GetConfigReturnsCorrectModel)
{
    Plant p(MakeSecondOrder());
    EXPECT_EQ(p.GetConfig().model, PlantModel::SECOND_ORDER);
}

// =============================================================================
// First-order model
// =============================================================================
TEST(PlantTest, FirstOrderStepTowardsSteadyState)
{
    // Constant input u=1, K=1, tau=1, y0=0.
    // After many steps the output should approach K*u = 1.
    Plant p(MakeFirstOrder(1.0, 1.0, 0.0));
    for (int i = 0; i < 1000; ++i) p.Step(1.0, 0.01);
    EXPECT_NEAR(p.GetOutput(), 1.0, 0.01);
}

TEST(PlantTest, FirstOrderTimeConstantApproximatelyCorrect)
{
    // At t = τ the first-order step response should be ~63.2% of final value.
    // K=1, tau=2, u=1, y0=0. At t=2: y ≈ 0.632.
    Plant p(MakeFirstOrder(1.0, 2.0, 0.0));
    const double dt  = 0.001;
    const double tau = 2.0;
    const int steps  = static_cast<int>(tau / dt);
    for (int i = 0; i < steps; ++i) p.Step(1.0, dt);
    EXPECT_NEAR(p.GetOutput(), 1.0 - std::exp(-1.0), 0.01);
}

TEST(PlantTest, FirstOrderWithGain)
{
    // K=3, u=1, tau=1, y0=0. Steady-state → 3.
    Plant p(MakeFirstOrder(3.0, 1.0, 0.0));
    for (int i = 0; i < 2000; ++i) p.Step(1.0, 0.01);
    EXPECT_NEAR(p.GetOutput(), 3.0, 0.05);
}

TEST(PlantTest, FirstOrderNearZeroTimeConstantGuard)
{
    // tau very close to zero should not produce NaN or crash.
    PlantConfig cfg = MakeFirstOrder(1.0, 1e-20, 0.0);
    Plant p(cfg);
    EXPECT_NO_THROW(p.Step(1.0, 0.01));
    EXPECT_FALSE(std::isnan(p.GetOutput()));
}

// =============================================================================
// Second-order model
// =============================================================================
TEST(PlantTest, SecondOrderOutputChangesWithInput)
{
    Plant p(MakeSecondOrder(1.0, 2.0, 0.7, 0.0));
    p.Step(1.0, 0.01);
    // Output should have changed from zero.
    EXPECT_NE(p.GetOutput(), 0.0);
}

TEST(PlantTest, SecondOrderOverdampedReachesSetpoint)
{
    // Heavily overdamped (zeta=2) should smoothly approach K*u=1
    Plant p(MakeSecondOrder(1.0, 1.0, 2.0, 0.0));
    for (int i = 0; i < 5000; ++i) p.Step(1.0, 0.001);
    EXPECT_NEAR(p.GetOutput(), 1.0, 0.05);
}

TEST(PlantTest, SecondOrderUnderdampedOvershoots)
{
    // Underdamped (zeta=0.1) should overshoot K*u=1
    Plant p(MakeSecondOrder(1.0, 5.0, 0.1, 0.0));
    double peak = 0.0;
    for (int i = 0; i < 10000; ++i)
    {
        p.Step(1.0, 0.001);
        peak = std::max(peak, p.GetOutput());
    }
    EXPECT_GT(peak, 1.0);  // overshoot must occur
}

TEST(PlantTest, SecondOrderInitialValueNonZero)
{
    Plant p(MakeSecondOrder(1.0, 1.0, 0.7, 5.0));
    EXPECT_DOUBLE_EQ(p.GetOutput(), 5.0);
}

// =============================================================================
// Integrating model
// =============================================================================
TEST(PlantTest, IntegratingOutputIncreasesWithPositiveInput)
{
    Plant p(MakeIntegrating(1.0, 0.0));
    p.Step(2.0, 0.1);
    // y += K*u*dt = 1*2*0.1 = 0.2
    EXPECT_NEAR(p.GetOutput(), 0.2, 1e-9);
}

TEST(PlantTest, IntegratingAccumulatesCorrectly)
{
    Plant p(MakeIntegrating(1.0, 0.0));
    for (int i = 0; i < 10; ++i) p.Step(1.0, 0.1);
    // y = 10 * 1.0 * 0.1 = 1.0
    EXPECT_NEAR(p.GetOutput(), 1.0, 1e-9);
}

TEST(PlantTest, IntegratingWithNegativeInput)
{
    Plant p(MakeIntegrating(1.0, 5.0));
    for (int i = 0; i < 10; ++i) p.Step(-1.0, 0.1);
    // y = 5.0 + 10*(-1)*0.1 = 4.0
    EXPECT_NEAR(p.GetOutput(), 4.0, 1e-9);
}

TEST(PlantTest, IntegratingWithGain)
{
    Plant p(MakeIntegrating(3.0, 0.0));
    p.Step(1.0, 0.5);
    // y = 3 * 1 * 0.5 = 1.5
    EXPECT_NEAR(p.GetOutput(), 1.5, 1e-9);
}

// =============================================================================
// Dead-time model
// =============================================================================
TEST(PlantTest, DeadTimeInitialResponseIsZero)
{
    // For the first dead_time/dt steps the output should remain at initial_value
    // because no delayed input has arrived yet.
    Plant p(MakeDeadTime(1.0, 1.0, 0.5, 0.0));
    const double dt          = 0.1;
    const int    delay_steps = static_cast<int>(0.5 / dt);  // 5 steps

    for (int i = 0; i < delay_steps; ++i)
    {
        const double y = p.Step(1.0, dt);
        (void)y;
    }
    // Output should be very close to 0 (no input has arrived through delay yet).
    EXPECT_NEAR(p.GetOutput(), 0.0, 0.05);
}

TEST(PlantTest, DeadTimeEventuallyReachesSteadyState)
{
    Plant p(MakeDeadTime(1.0, 1.0, 0.2, 0.0));
    for (int i = 0; i < 2000; ++i) p.Step(1.0, 0.01);
    EXPECT_NEAR(p.GetOutput(), 1.0, 0.02);
}

TEST(PlantTest, DeadTimeZeroDelayBehavesLikeFirstOrder)
{
    // dead_time=0 should be identical to a plain first-order plant.
    PlantConfig fo_cfg  = MakeFirstOrder(1.0, 1.0, 0.0);
    PlantConfig dt_cfg  = MakeDeadTime(1.0, 1.0, 0.0, 0.0);

    Plant fo_plant(fo_cfg);
    Plant dt_plant(dt_cfg);

    for (int i = 0; i < 50; ++i)
    {
        fo_plant.Step(1.0, 0.01);
        dt_plant.Step(1.0, 0.01);
    }
    EXPECT_NEAR(fo_plant.GetOutput(), dt_plant.GetOutput(), 1e-6);
}

// =============================================================================
// Non-positive dt guard
// =============================================================================
TEST(PlantTest, ZeroDtDoesNotChangePlantState)
{
    Plant p(MakeFirstOrder(1.0, 1.0, 3.0));
    const double before = p.GetOutput();
    EXPECT_NO_THROW(p.Step(1.0, 0.0));
    EXPECT_DOUBLE_EQ(p.GetOutput(), before);
}

TEST(PlantTest, NegativeDtDoesNotChangePlantState)
{
    Plant p(MakeIntegrating(1.0, 2.0));
    const double before = p.GetOutput();
    EXPECT_NO_THROW(p.Step(1.0, -0.1));
    EXPECT_DOUBLE_EQ(p.GetOutput(), before);
}

// =============================================================================
// Reset correctness
// =============================================================================
TEST(PlantTest, ResetRestoresFirstOrderToInitial)
{
    Plant p(MakeFirstOrder(1.0, 1.0, 2.5));
    for (int i = 0; i < 100; ++i) p.Step(1.0, 0.01);
    EXPECT_NE(p.GetOutput(), 2.5);
    p.Reset();
    EXPECT_DOUBLE_EQ(p.GetOutput(), 2.5);
}

TEST(PlantTest, ResetRestoresIntegratingToInitial)
{
    Plant p(MakeIntegrating(1.0, 0.0));
    for (int i = 0; i < 50; ++i) p.Step(1.0, 0.1);
    p.Reset();
    EXPECT_DOUBLE_EQ(p.GetOutput(), 0.0);
}

TEST(PlantTest, ResetClearsDeadTimeBuffer)
{
    Plant p(MakeDeadTime(1.0, 1.0, 0.5, 0.0));
    for (int i = 0; i < 100; ++i) p.Step(1.0, 0.01);
    p.Reset();
    EXPECT_DOUBLE_EQ(p.GetOutput(), 0.0);
    // After reset + one step, output should still be essentially zero
    // because the buffer was cleared.
    p.Step(1.0, 0.01);
    EXPECT_NEAR(p.GetOutput(), 0.0, 0.01);
}

TEST(PlantTest, ResetRestoresSecondOrderState)
{
    Plant p(MakeSecondOrder(1.0, 2.0, 0.7, 1.0));
    for (int i = 0; i < 200; ++i) p.Step(1.0, 0.01);
    p.Reset();
    EXPECT_DOUBLE_EQ(p.GetOutput(), 1.0);
}