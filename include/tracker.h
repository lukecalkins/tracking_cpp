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

    void update_belief(arma::vec measurement, arma::vec ownship) override {
        std::cout << "Inside update belief" << std::endl;

        const arma::mat A = infoTarget.get_jacobian();
        const arma::mat W = infoTarget.get_process_noise_covariance();
        const arma::mat Sigma_curr = infoTarget.get_cov();

        const unsigned int target_dim = infoTarget.get_x_dim();
        const unsigned int z_dim = sensor->get_z_dim();
        arma::mat H(z_dim, target_dim, arma::fill::zeros);
        arma::mat V(z_dim, z_dim, arma::fill::zeros);


        //Predictions step
        infoTarget.forward_simulate(1);
        arma::vec x_bar_t = infoTarget.get_state();
        arma::mat Sigma_bar_t = A * Sigma_curr * A.t() + W;

        sensor->get_jacobian(H, V, x_bar_t, ownship);

        arma::mat R = H * Sigma_bar_t * H.t() + V;
        arma::mat K = Sigma_bar_t * H.t() * R.i();
        arma::vec x_t_update = x_bar_t + K * (measurement - sensor->observation_model(x_bar_t, ownship));
        arma::mat Sigma_t_update = (arma::eye(target_dim, target_dim) - K * H) * Sigma_bar_t;

        infoTarget.update_belief(x_t_update, Sigma_t_update);
    };

    ~KalmanFilter() override = default;
};

class UnscentedKalmanFilter final : public Tracker {

public:

    UnscentedKalmanFilter(TargetLinear2DBelief init_target_belief, std::shared_ptr<Sensor> sensor) : Tracker(init_target_belief, sensor){};

    void update_belief(arma::vec measurement, arma::vec ownship) override;

    ~UnscentedKalmanFilter() override = default;
};
#endif //TRACKER_H
