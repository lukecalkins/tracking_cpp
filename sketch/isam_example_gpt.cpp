#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/inference/Symbol.h>

#include <iostream>
#include <cmath>
#include <random>

using namespace gtsam;
using symbol_shorthand::X;

// ---------------------------------------------------------------------
// Custom Motion Factor: x_k = f(x_{k-1})
// ---------------------------------------------------------------------
struct MotionFactor : public NoiseModelFactor2<Vector, Vector> {
    double dt_;
    MotionFactor(Key k1, Key k2, double dt, const SharedNoiseModel& model)
        : NoiseModelFactor2<Vector, Vector>(model, k1, k2), dt_(dt) {}

    Vector evaluateError(
        const Vector& x1, const Vector& x2,
        boost::optional<Matrix&> H1 = boost::none,
        boost::optional<Matrix&> H2 = boost::none) const override
    {
        Vector pred(4);
        pred(0) = x1(0) + x1(2) * dt_;
        pred(1) = x1(1) + x1(3) * dt_;
        pred(2) = x1(2);
        pred(3) = x1(3);

        if (H1) {
            Matrix& J = *H1;
            J = Matrix::Zero(4,4);
            J(0,0) = -1.0;  J(0,2) = -dt_;
            J(1,1) = -1.0;  J(1,3) = -dt_;
            J(2,2) = -1.0;
            J(3,3) = -1.0;
        }
        if (H2) {
            Matrix& J = *H2;
            J = Matrix::Identity(4,4);
        }

        return x2 - pred;
    }
};

// ---------------------------------------------------------------------
// Bearing-only measurement factor
// ---------------------------------------------------------------------
struct BearingFactor : public NoiseModelFactor1<Vector> {
    double meas_;
    BearingFactor(Key k, double m, const SharedNoiseModel& model)
        : NoiseModelFactor1<Vector>(model, k), meas_(m) {}

    Vector evaluateError(
        const Vector& x,
        boost::optional<Matrix&> H = boost::none) const override
    {
        double px = x(0), py = x(1);
        double pred = std::atan2(py, px);

        double err = pred - meas_;
        while (err >  M_PI) err -= 2*M_PI;
        while (err < -M_PI) err += 2*M_PI;

        if (H) {
            Matrix& J = *H;
            J = Matrix::Zero(1,4);
            double r2 = px*px + py*py;
            if (r2 < 1e-9) r2 = 1e-9;
            J(0,0) = -py / r2;
            J(0,1) =  px / r2;
        }

        Vector e(1);
        e(0) = err;
        return e;
    }
};

// ---------------------------------------------------------------------
// MAIN PROGRAM
// ---------------------------------------------------------------------
int main() {
    const double dt = 1.0;

    ISAM2 isam;
    NonlinearFactorGraph newFactors;
    Values initialEstimate;

    // Noise settings
    auto motionNoise  = noiseModel::Diagonal::Sigmas((Vector(4) << 0.1, 0.1, 0.1, 0.1).finished());
    auto bearingNoise = noiseModel::Isotropic::Sigma(1, 0.05);

    // -----------------------------
    // TRUE STATE (sim only)
    // -----------------------------
    Vector true_state(4);
    true_state << 20.0, 5.0, -1.0, 0.5;

    // -----------------------------
    // ORIGINAL PRIOR
    // -----------------------------
    Vector x0 = true_state;  // your best guess; does not need to be exact

    auto priorNoise = noiseModel::Diagonal::Sigmas(
        (Vector(4) << 5.0, 5.0, 2.0, 2.0).finished());

    newFactors.add(PriorFactor<Vector>(X(0), x0, priorNoise));
    initialEstimate.insert(X(0), x0);

    // -----------------------------
    // EXTRA FIXING PRIORS (patch)
    // -----------------------------
    // 1. Constrain only position strongly
    auto posTight_velLoose = noiseModel::Diagonal::Sigmas(
        (Vector(4) << 5.0, 5.0, 1000.0, 1000.0).finished());
    Vector posOnly(4);
    posOnly << x0(0), x0(1), 0, 0;
    newFactors.add(PriorFactor<Vector>(X(0), posOnly, posTight_velLoose));

    // 2. Constrain only velocity strongly
    auto posLoose_velTight = noiseModel::Diagonal::Sigmas(
        (Vector(4) << 1000.0, 1000.0, 2.0, 2.0).finished());
    Vector velOnly(4);
    velOnly << 0, 0, x0(2), x0(3);
    newFactors.add(PriorFactor<Vector>(X(0), velOnly, posLoose_velTight));

    // Initialize ISAM with priors
    isam.update(newFactors, initialEstimate);
    newFactors.resize(0);
    initialEstimate.clear();

    // Random noise generator for measurements
    std::default_random_engine rng(0);
    std::normal_distribution<double> bearing_noise(0.0, 0.03);

    // ---------------------------------------------
    // MAIN LOOP
    // ---------------------------------------------
    for (int k = 1; k <= 30; ++k) {
        // ---- Simulate truth ----
        true_state(0) += true_state(2) * dt;
        true_state(1) += true_state(3) * dt;

        double trueBearing = std::atan2(true_state(1), true_state(0));
        double measurement = trueBearing + bearing_noise(rng);

        // ---- Predict next state (initial guess) ----
        Vector last_est = isam.calculateEstimate<Vector>(X(k-1));
        Vector predicted = last_est;
        predicted(0) += last_est(2) * dt;
        predicted(1) += last_est(3) * dt;

        // ---- Add motion + bearing factors ----
        newFactors.add(boost::make_shared<MotionFactor>(
            X(k-1), X(k), dt, motionNoise));

        newFactors.add(boost::make_shared<BearingFactor>(
            X(k), measurement, bearingNoise));

        // ---- Insert predicted state ----
        initialEstimate.insert(X(k), predicted);

        // ---- Update ISAM ----
        isam.update(newFactors, initialEstimate);

        newFactors.resize(0);
        initialEstimate.clear();

        // ---- Print estimate ----
        Vector est = isam.calculateEstimate<Vector>(X(k));
        std::cout << "k=" << k
                  << "  est_pos=(" << est(0) << ", " << est(1) << ")"
                  << "  est_vel=(" << est(2) << ", " << est(3) << ")\n";
    }

    return 0;
}
