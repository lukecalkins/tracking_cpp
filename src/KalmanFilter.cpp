//
// Created by Luke Calkins on 5/30/25.
//
#include "tracker.h"


void KalmanFilter::update_belief(std::vector<arma::vec> measurements, std::vector<arma::vec> ownships) {
    std::cout << "Inside update belief" << std::endl;

    const arma::mat A = infoTarget.get_jacobian();
    const arma::mat W = infoTarget.get_process_noise_covariance();
    const arma::mat Sigma_curr = infoTarget.get_cov();

    const unsigned int target_dim = infoTarget.get_x_dim();
    const unsigned int z_dim = sensor->get_z_dim();
    const unsigned int num_sensors = ownships.size();
    arma::mat H(z_dim * num_sensors, target_dim, arma::fill::zeros);
    arma::mat V(z_dim * num_sensors, z_dim * num_sensors, arma::fill::zeros);

    //Predictions step
    infoTarget.forward_simulate(1);
    arma::vec x_bar_t = infoTarget.get_state();
    arma::mat Sigma_bar_t = A * Sigma_curr * A.t() + W;

    // Build Jacobian and multi sensor noise matrix
    for (int i = 0; i < ownships.size(); i++) {
        arma::mat H_temp(z_dim, target_dim, arma::fill::zeros);
        arma::mat V_temp(z_dim, z_dim, arma::fill::zeros);
        sensor->get_jacobian(H_temp, V_temp, x_bar_t, ownships[i]);
        H.submat(i, 0, i, target_dim - 1) = H_temp;
        V.submat(i, i, i, i) = V_temp;
    }

    //build multi sensor innovation
    arma::vec innovation(z_dim * num_sensors, arma::fill::zeros);
    for (int i=0; i<measurements.size(); i++) {
        innovation.subvec(i * z_dim, i * z_dim + z_dim - 1) = measurements[i] - sensor->observation_model(x_bar_t, ownships[i]);
    }

    arma::mat R = H * Sigma_bar_t * H.t() + V;
    arma::mat K = Sigma_bar_t * H.t() * R.i();
    arma::mat x_t_update = x_bar_t + K * innovation;
    //arma::vec x_t_update = x_bar_t + K * (measurement - sensor->observation_model(x_bar_t, ownship));
    arma::mat Sigma_t_update = (arma::eye(target_dim, target_dim) - K * H) * Sigma_bar_t;

    infoTarget.update_belief(x_t_update, Sigma_t_update);
    curr_time_step += 1;
}