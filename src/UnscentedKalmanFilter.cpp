//
// Created by Luke Calkins on 8/2/25.
//
#include "tracker.h"

void UnscentedKalmanFilter::update_belief(arma::vec measurement, arma::vec ownship) {
    /*
     L = length of state
     N = number of sigma points
     total sigma points = 2L + 1,
     W_0_a = W_0_c
     W_i_a = W_i_c, i = 1,...,2L
     */
    //compute sigma points
    arma::vec s_0 = infoTarget.get_state();
    double L = infoTarget.get_x_dim();
    double w_0_a = (alpha * alpha * kappa - L) / (alpha * alpha * kappa);
    double w_0_c = w_0_a + 1 - alpha * alpha + beta;
    arma::mat P_0 = infoTarget.get_cov();
    arma::mat Q = infoTarget.get_process_noise_covariance();
    unsigned int z_dim = sensor->get_z_dim();
    arma::mat R = sensor->get_measurement_covariance();

    //Compute square root of prior covariance mat
    arma::mat A = arma::sqrtmat_sympd(P_0);

    //Create Sigma points
    arma::mat sigmaVecs(L, 2*L + 1, arma::fill::zeros);
    arma::vec W_a(2 * L + 1, arma::fill::zeros);
    arma::vec W_c(2 * L + 1, arma::fill::zeros);
    sigmaVecs.col(0) = s_0;
    W_a(0) = w_0_a;
    W_c(0) = w_0_c;
    for (int i = 0; i < L; i++) {
        sigmaVecs.col(i + 1) = s_0 + alpha * std::sqrt(kappa) * A.col(i);
        sigmaVecs.col(L + i + 1) = s_0 - alpha * std::sqrt(kappa) * A.col(i);
        W_a(i + 1) = 1 / (2 * alpha * alpha * kappa);
        W_a(L + i + 1) = 1 / (2 * alpha * alpha * kappa);
        W_c(i + 1) = 1 / (2 * alpha * alpha * kappa);
        W_c(L + i + 1) = 1 / (2 * alpha * alpha * kappa);
    }

    // ***** Predict ***** //
    arma::vec x_k_predict(L, arma::fill::zeros);
    arma::mat cov_predict(L, L, arma::fill::zeros);
    arma::mat sigmaVecsPredict(L, 2*L + 1, arma::fill::zeros);
    for (int i = 0; i < 2*L + 1; i++) {
        sigmaVecsPredict.col(i) = infoTarget.predict_state(1, sigmaVecs.col(i));
        x_k_predict += + W_a(i) * sigmaVecsPredict.col(i);
    }
    for (int i = 0; i < 2*L + 1; i++) {
        cov_predict += + W_c(i) * (sigmaVecsPredict.col(i) - x_k_predict) * (sigmaVecsPredict.col(i) - x_k_predict).t();
    }
    cov_predict += Q;

    // ***** Update ****** //
    //Compute 2L + 1 measurements of sigma points
    arma::vec z_predict(z_dim, arma::fill::zeros);
    arma::vec sigmaMeasurementsPredict(2*L + 1, arma::fill::zeros);
    arma::mat S_k(z_dim, z_dim, arma::fill::zeros);
    for (int i = 0; i < 2*L + 1; i++) {
        arma::vec sigmaMeasurement = sensor->observation_model(sigmaVecsPredict.col(i), ownship);
        sigmaMeasurementsPredict(i) = sigmaMeasurement(0);
        z_predict += W_a(i) * sigmaMeasurementsPredict(i);
    }
    for (int i = 0; i < 2*L + 1; i++) {
        S_k += W_c(i) * (sigmaMeasurementsPredict(i) - z_predict) * (sigmaMeasurementsPredict(i) - z_predict).t();
    }
    S_k += R;

    //Compute Cross covariance matrix
    arma::mat C_xz(L, z_dim, arma::fill::zeros);
    for (int i = 0; i < 2*L + 1; i++) {
        C_xz += W_c(i) * (sigmaVecsPredict.col(i) - x_k_predict) * (sigmaMeasurementsPredict(i) - z_predict).t();
    }

    //Kalman Gain
    arma::mat K_k = C_xz * arma::inv(S_k);

    // final update
    arma::vec x_k_update = x_k_predict + K_k *(measurement - z_predict);
    arma::mat P_k_update = cov_predict + - K_k * S_k * K_k.t();
}