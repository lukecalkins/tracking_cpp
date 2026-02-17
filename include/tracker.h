//
// Created by Luke Calkins on 7/19/25.
//

#ifndef TRACKER_H
#define TRACKER_H

#include "target.h"
#include "targetBelief.h"
#include "sensor.h"
#include <gtsam/nonlinear/ExtendedKalmanFilter.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/ISAM2.h>

class Tracker {

protected:
    TargetLinear2DBelief infoTarget;
    std::shared_ptr<Sensor> sensor;
    unsigned int curr_time_step;
    double sample_interval;

public:

    Tracker(TargetLinear2DBelief init_target_estimate, std::shared_ptr<Sensor> sensor, double sample_interval)
                : infoTarget((init_target_estimate)),
                sensor(sensor),
                curr_time_step(0),
                sample_interval(sample_interval){};

    virtual void update_belief(std::vector<arma::vec> measurements, std::vector<arma::vec> ownships) = 0;

    arma::vec get_target_belief_state() {
        return infoTarget.get_state();
    }

    arma::mat get_target_belief_cov() {
        return infoTarget.get_cov();
    }

    virtual ~Tracker() = default;

};


class KalmanFilter final : public Tracker {

public:

    KalmanFilter(TargetLinear2DBelief init_target_belief, std::shared_ptr<Sensor> sensor, double sample_interval)
                : Tracker(init_target_belief, sensor, sample_interval){};

    void update_belief(std::vector<arma::vec> measurements, std::vector<arma::vec> ownships) override;

    ~KalmanFilter() override = default;
};


class UnscentedKalmanFilter final : public Tracker {

    double alpha;
    double beta;
    double kappa;

public:

    UnscentedKalmanFilter(TargetLinear2DBelief init_target_belief,
                            std::shared_ptr<Sensor> sensor,
                            double alpha,
                            double beta,
                            double kappa,
                            double sample_interval);

    void update_belief(std::vector<arma::vec> measurements, std::vector<arma::vec> ownships) override;

    ~UnscentedKalmanFilter() override = default;
};


class ISAMFilter final : public Tracker {

    gtsam::ISAM2 isam;
    gtsam::NonlinearFactorGraph newFactors;
    gtsam::Values initialEstimates;
    gtsam::noiseModel::Gaussian::shared_ptr motionNoise;
    gtsam::noiseModel::Isotropic::shared_ptr sensorNoise;

public:
    ISAMFilter(TargetLinear2DBelief init_target_belief, std::shared_ptr<Sensor> sensor, double sample_interval);

    void update_belief(std::vector<arma::vec> measurements, std::vector<arma::vec> ownships) override;

    ~ISAMFilter() override = default;
};

#endif //TRACKER_H
