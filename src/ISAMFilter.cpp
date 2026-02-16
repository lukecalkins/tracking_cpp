//
// Created by Luke Calkins on 12/8/25.
//
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>

#include <iostream>
#include <cmath>
#include <random>

#include "tracker.h"
#include "sensor.h"
#include "utils.h"

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



/********************************************************************
 *                    Bearing-Only Measurement Factor
 *    Sensor at known (sx, sy), measures theta = atan2(py - sy, px - sx)
 ********************************************************************/
class BearingOnlyFactor : public NoiseModelFactor1<Vector>
{
    Point2 sensor_;
    double measured_;

public:
    BearingOnlyFactor(Key key,
                      const Point2& sensor,
                      double measured,
                      const SharedNoiseModel& model)
        : NoiseModelFactor1<Vector>(model, key),
          sensor_(sensor),
          measured_(measured)
    {}

    // CORRECT OVERRIDE SIGNATURE
    Vector evaluateError(
        const Vector& x,
        boost::optional<Matrix&> H = boost::none) const override
    {
        double px = x(0), py = x(1);
        double dx = px - sensor_.x();
        double dy = py - sensor_.y();

        double pred = std::atan2(dy, dx);

        double error = pred - measured_;
        while (error >  M_PI) error -= 2*M_PI;
        while (error < -M_PI) error += 2*M_PI;

        if (H) {
            Matrix& J = *H;
            J = Matrix::Zero(1,4);
            double r2 = dx*dx + dy*dy;
            // protect against divide-by-zero
            if (r2 < 1e-12) r2 = 1e-12;
            J(0,0) = -dy / r2;
            J(0,1) =  dx / r2;
            // velocities derivatives are zero
        }

        Vector e(1);
        e(0) = error;
        return e;
    }
};

ISAMFilter::ISAMFilter(TargetLinear2DBelief init_target_belief, std::shared_ptr<Sensor> sensor)
                        :Tracker(init_target_belief, sensor) {

    /*
    ISAM2 isam;
    NonlinearFactorGraph newFactors;
    Values initialEstimates;
    */

    // Convert process noise covariance from arma to eign before defining
    arma::mat process_noise_arma = init_target_belief.get_cov();
    Matrix process_noise(process_noise_arma.n_rows, process_noise_arma.n_cols);
    arma_mat_to_gtsam_mat(process_noise_arma, process_noise);
    auto motionNoise = noiseModel::Gaussian::Covariance(process_noise);

    auto bearingNoise = noiseModel::Isotropic::Sigma(1, sensor->get_measurement_noise_double());

    Vector priorSigmas(init_target_belief.get_x_dim());
    arma::mat init_cov = init_target_belief.get_cov();
    for (int i = 0; i < priorSigmas.size(); i++) {
        priorSigmas[i] = init_cov(i ,i);
    }
    auto priorNoise = noiseModel::Diagonal::Sigmas(priorSigmas);

    //Convert initial belief to gtsam vector
    arma::vec x0_arma = init_target_belief.get_state();
    Vector x0(x0_arma.n_elem);
    arma_vec_to_gtsam_vec(x0_arma, x0);

    // Insert prior
    newFactors.add(PriorFactor<Vector>(X(0), x0, priorNoise));
    initialEstimates.insert(X(0), x0);

    //Commit initial priors into ISAM
    isam.update(newFactors, initialEstimates);
    newFactors.resize(0);
    initialEstimates.clear();
};

void ISAMFilter::update_belief(std::vector<arma::vec> measurements, std::vector<arma::vec> ownships) {};

