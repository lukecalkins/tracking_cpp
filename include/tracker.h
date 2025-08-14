//
// Created by Luke Calkins on 7/19/25.
//

#ifndef TRACKER_H
#define TRACKER_H

#include "target.h"
#include "targetBelief.h"
#include "sensor.h"


class Tracker {

protected:
    TargetLinear2DBelief infoTarget;
    std::shared_ptr<Sensor> sensor;

public:

    Tracker(TargetLinear2DBelief init_target_estimate, std::shared_ptr<Sensor> sensor) : infoTarget((init_target_estimate)), sensor(sensor){};

    virtual void update_belief(arma::vec measurement, arma::vec ownship) = 0;

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

    KalmanFilter(TargetLinear2DBelief init_target_belief, std::shared_ptr<Sensor> sensor): Tracker(init_target_belief, sensor){};

    void update_belief(arma::vec measurement, arma::vec ownship) override;

    ~KalmanFilter() override = default;
};

class UnscentedKalmanFilter final : public Tracker {

    double alpha;
    double beta;
    double kappa;

public:

    UnscentedKalmanFilter(TargetLinear2DBelief init_target_belief, std::shared_ptr<Sensor> sensor, double alpha, double beta, double kappa) :
                                                                Tracker(init_target_belief, sensor),
                                                                alpha(alpha),
                                                                beta(beta),
                                                                kappa(kappa){};

    void update_belief(arma::vec measurement, arma::vec ownship) override;

    ~UnscentedKalmanFilter() override = default;
};
#endif //TRACKER_H
