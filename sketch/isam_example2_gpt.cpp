// tracking.cpp
// Bearing-only 4D tracking with iSAM2 — Option 1 (strong single prior)

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
//#include <gtsam/noiseModel/Diagonal.h>

#include <iostream>
#include <cmath>
#include <random>

using namespace gtsam;
inline Symbol X(size_t t) { return Symbol('x', t); }

// ----------------- Motion factor -----------------
// State: Vector of length 4: [px, py, vx, vy]
class MotionFactor : public NoiseModelFactor2<Vector, Vector> {
    double dt_;
public:
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

        // error = x2 - pred
        if (H1) {
            Matrix &J1 = *H1;
            J1 = Matrix::Zero(4,4);
            J1(0,0) = -1.0; J1(0,2) = -dt_;
            J1(1,1) = -1.0; J1(1,3) = -dt_;
            J1(2,2) = -1.0; J1(3,3) = -1.0;
        }
        if (H2) {
            Matrix &J2 = *H2;
            J2 = Matrix::Identity(4,4);
        }

        return x2 - pred;
    }
};

// ----------------- Bearing-only factor -----------------
class BearingFactor : public NoiseModelFactor1<Vector> {
    double measured_;
    Vector sensor_pos_;
public:
    BearingFactor(Key k, double measured, const SharedNoiseModel& model, const Vector& sensor_pos)
        : NoiseModelFactor1<Vector>(model, k), measured_(measured), sensor_pos_(sensor_pos) {}

    Vector evaluateError(
        const Vector& x,
        boost::optional<Matrix&> H = boost::none) const override
    {
        double dx = x(0) - sensor_pos_(0);
        double dy = x(1) - sensor_pos_(1);

        double pred = std::atan2(dy, dx);
        double err = pred - measured_;
        // wrap to [-pi,pi]
        while (err >  M_PI) err -= 2.0*M_PI;
        while (err < -M_PI) err += 2.0*M_PI;

        if (H) {
            Matrix &J = *H;
            J = Matrix::Zero(1,4);
            double r2 = dx*dx + dy*dy;
            if (r2 < 1e-12) r2 = 1e-12;
            J(0,0) = -dy / r2;
            J(0,1) =  dx / r2;
            // vel entries remain zero
        }

        Vector out(1);
        out(0) = err;
        return out;
    }
};

void print_trajectory(const ISAM2& isam, int k) {
    std::cout << "Trajectory at step: " << k << "\n";
    for (int i = 0; i <= k; i++) {
        auto est  = isam.calculateEstimate<Vector>(X(i));
        std::cout <<i <<  ": (" << est(0) << ", " << est(1) << ", " << est(2) << ", " << est (3) << ")\n";
    }
}

// ----------------- Main -----------------
int main() {
    const double dt = 1.0;
    ISAM2 isam;

    NonlinearFactorGraph newFactors;
    Values initialEstimates;

    // Process / measurement noises
    auto motionNoise  = noiseModel::Diagonal::Sigmas((Vector(4) << 0.1, 0.1, 0.1, 0.1).finished());
    auto bearingNoise = noiseModel::Isotropic::Sigma(1, 0.02);

    // -------- Option A: single, stronger prior on X(0) ----------
    // Tightened sigmas to make the problem observable & stable.
    Vector priorSigmas(4);
    priorSigmas << 10.0, 10.0, 1.0, 1.0; // px_sigma, py_sigma, vx_sigma, vy_sigma
    auto priorNoise = noiseModel::Diagonal::Sigmas(priorSigmas);

    // Initial belief (not ground truth). Choose plausible values.
    Vector x0(4);
    x0 << 10.0, 5.0, 0.5, 0.0;

    // Insert prior
    newFactors.add(PriorFactor<Vector>(X(0), x0, priorNoise));
    initialEstimates.insert(X(0), x0);

    // Commit initial priors into iSAM
    isam.update(newFactors, initialEstimates);
    newFactors.resize(0);
    initialEstimates.clear();

    // Simulated true state (ONLY to generate measurements)
    Vector true_state(4);
    true_state << 10.0, 5.0, 1.0, 0.0;
    std::default_random_engine rng(123);
    std::normal_distribution<double> bearing_noise(0.0, 0.02);

    // Set sensor positions
    Vector sensor_1(2);
    Vector sensor_2(2);
    sensor_1 << 0.0, 0.0;
    sensor_2 << 50.0, 0.0;

    // Run filter loop
    for (int k = 1; k <= 50; ++k) {
        // Propagate true state for measurement simulation
        true_state(0) += true_state(2) * dt;
        true_state(1) += true_state(3) * dt;
        double trueBearing_1 = std::atan2(true_state(1) - sensor_1(1), true_state(0) - sensor_1(0));
        double measurement_1 = trueBearing_1 + bearing_noise(rng);
        double trueBearing_2 = std::atan2(true_state(1) - sensor_2(1), true_state(0) - sensor_2(0));
        double measurement_2 = trueBearing_2 + bearing_noise(rng);

        // Predict from last estimate (use isam current estimate)
        Vector last_est = isam.calculateEstimate<Vector>(X(k-1));
        Vector predicted(4);
        predicted = last_est; // copy
        predicted(0) += last_est(2) * dt;
        predicted(1) += last_est(3) * dt;

        // Add motion and bearing factors for this timestep
        newFactors.add(boost::make_shared<MotionFactor>(X(k-1), X(k), dt, motionNoise));
        newFactors.add(boost::make_shared<BearingFactor>(X(k), measurement_1, bearingNoise, sensor_1));
        newFactors.add(boost::make_shared<BearingFactor>(X(k), measurement_2, bearingNoise, sensor_2));

        // Insert concrete initial guess for X(k)
        initialEstimates.insert(X(k), predicted);

        // Update isam2
        isam.update(newFactors, initialEstimates);

        // Clear temporaries
        newFactors.resize(0);
        initialEstimates.clear();

        // Retrieve and print estimate
        Vector est = isam.calculateEstimate<Vector>(X(k));
        print_trajectory(isam, k);
        std::cout << "time step: " << k <<  ", true state=(" << true_state(0) << ", " << true_state(1) << ", " << true_state(2) <<
                        ", " << true_state(3) << ")\n";



        /*
        std::cout << "k=" << k
                  << " est_pos=(" << est(0) << ", " << est(1) << est(2) << ", " << est(3) << ")\n"
                    << "true state=(" << true_state(0) << ", " << true_state(1) << ", " << true_state(2) <<
                        ", " << true_state(3) << ")\n";
        */
    }

    return 0;
}
