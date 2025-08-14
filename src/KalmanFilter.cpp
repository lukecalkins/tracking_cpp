//
// Created by Luke Calkins on 5/30/25.
//
#include "tracker.h"


void KalmanFilter::update_belief(arma::vec measurement, arma::vec ownship) {

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

}